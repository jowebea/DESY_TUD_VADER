# -*- coding: utf-8 -*-
"""
maxi.py
Finaler synchroner MAXI-Treiber:
- SET senden, genau EIN Status-Frame empfangen
- Robust gegenüber Heartbeats (optional: settle_ms-Sammelfenster)
- Sauberes DEBUG-Logging aller RX-Chunks und Frames
"""

import logging
import time
import struct
import argparse
from typing import Dict, Any, Union

# Erwartet Hilfsfunktionen/Klassen aus deinem Projekt:
# utilities._hexdump: bytes -> "AA BB ..."
# utilities.SerialDevice: Abstraktion für realen/mocked Serial-Port
from utilities import MODULE_LOGGER_NAME, _hexdump, SerialDevice


class MAXI:
    """
    Synchroner MAXI-Treiber (ein Gerät pro Port).
    Workflow: _tx_set_once() -> _rx_status_once() -> parsed Dict

    Parameter:
      port:       z.B. '/dev/tty.usbmodem1133201'
      timeout:    Gesamttimeout für das Warten auf Status (Sekunden)
      settle_ms:  optionales Mini-Fenster (ms): liefere das *letzte* gültige Frame,
                  das innerhalb des Fensters nach dem ersten Treffer eintrifft.
                  Nützlich, wenn auf der Leitung zusätzlich ein Heartbeat läuft.
    """
    # Status-Frame
    SYNC0, SYNC1 = 0xAA, 0x55
    STATUS_LEN = 25  # 25 oder 29 je nach FW – Parser ist tolerant

    # SET-Frame
    H0, H1, CMD_SET, SET_LEN = 0xA5, 0x5A, 0x01, 0x04

    # ADDR6
    A_RequestStatus = 0x00
    A_V1, A_V2, A_V3 = 0x01, 0x02, 0x03
    A_FanIn, A_FanOut = 0x04, 0x05
    A_FlowCO2, A_TotCO2 = 0x10, 0x11
    A_FlowN2,  A_TotN2  = 0x12, 0x13
    A_FlowBut, A_TotBut = 0x14, 0x15

    def __init__(self, port: str, timeout: float = 8.0, settle_ms: int = 0, boot_delay: float = 2.2):
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.MAXI")
        self.dev = SerialDevice(port, 115200, name="MAXI")
        self.timeout = timeout
        self.settle_ms = max(0, int(settle_ms))

        # USB/Boot-Reset abwarten + Eingangsbuffer leeren
        time.sleep(boot_delay)
        t0 = time.time()
        while time.time() - t0 < 0.25:
            if not self.dev.read_bytes(256):
                time.sleep(0.01)
        self.logger.info("MAXI (sync) ready on %s", port)

    # ---------- Utilities ----------
    @staticmethod
    def _u26_from_float100(x: Union[int, float]) -> int:
        v = int(round(float(x) * 100.0))
        return max(0, min(v, 0x03FF_FFFF))

    @staticmethod
    def _checksum_sum(data: bytes) -> int:
        return sum(data) & 0xFF

    @staticmethod
    def _le16(b: bytes, off: int) -> int:
        return b[off] | (b[off + 1] << 8)

    @staticmethod
    def _le32_val(b: bytes, off: int) -> int:
        return b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24)

    @staticmethod
    def _pack_le32(word: int) -> bytes:
        return struct.pack("<I", word & 0xFFFFFFFF)

    # ---------- Low-level TX/RX ----------
    def _tx_set_once(self, addr6: int, data26: int) -> None:
        word = ((data26 & 0x03FF_FFFF) << 6) | (addr6 & 0x3F)
        payload = self._pack_le32(word)
        frame = bytes([self.H0, self.H1, self.CMD_SET, self.SET_LEN]) + payload + bytes([self._checksum_sum(payload)])
        self.logger.debug("TX SET addr=0x%02X data=%d -> %s", addr6, data26, _hexdump(frame))
        self.dev.write_bytes(frame)
        # winziger Delay, damit USB/CDC garantiert sendet
        time.sleep(0.004)

    def _rx_status_once(self) -> Dict[str, Any]:
        """
        Liest bis timeout und extrahiert Status-Frames.
        Wenn settle_ms > 0: nach dem ersten gültigen Frame noch 'settle_ms' weiter sammeln
        und das *letzte* gültige Frame zurückgeben (Heartbeat-resistent).
        """
        t_end = time.time() + self.timeout
        buf = bytearray()
        raw = bytearray()
        have_first = False
        last_parsed = None
        t_settle_end = 0.0

        while time.time() < t_end:
            chunk = self.dev.read_bytes(256)
            if chunk:
                # Rohlog für Diagnose
                self.logger.debug("RX chunk (%d B): %s", len(chunk), _hexdump(chunk))
                raw.extend(chunk)
                buf.extend(chunk)

                # Frames im Puffer suchen
                i = 0
                while i + 1 < len(buf):
                    if buf[i] == self.SYNC0 and buf[i+1] == self.SYNC1:
                        if i > 0:
                            self.logger.warning("Discarding %d B before sync: %s", i, _hexdump(bytes(buf[:i])))
                            del buf[:i]
                            i = 0
                            if len(buf) < 3:
                                break
                        if len(buf) < 3:
                            break
                        length = buf[2]
                        total = 3 + length + 1
                        if len(buf) < total:
                            break
                        payload = bytes(buf[3:3+length])
                        cs = buf[3+length]
                        del buf[:total]

                        self.logger.debug("Found frame: LEN=%d, CS=0x%02X, payload=%s",
                                          length, cs, _hexdump(payload))
                        if length not in (self.STATUS_LEN, 23, 29):
                            self.logger.warning("Unexpected length=%d", length)
                            continue
                        calc = self._checksum_sum(payload)
                        if calc != cs:
                            self.logger.warning("Checksum mismatch: calc=%02X got=%02X", calc, cs)
                            continue

                        # gültig → parsen
                        last_parsed = self._parse_status(payload)
                        self.logger.debug("Parsed status: %s", last_parsed)

                        if self.settle_ms <= 0:
                            return last_parsed

                        if not have_first:
                            have_first = True
                            t_settle_end = time.time() + (self.settle_ms / 1000.0)
                        continue
                    i += 1

            else:
                if have_first and time.time() >= t_settle_end:
                    # Sammelfenster vorbei → letztes gültiges Frame zurückgeben
                    return last_parsed if last_parsed is not None else self._parse_status(bytes([0]*self.STATUS_LEN))
                time.sleep(0.002)

        # Timeout:
        if last_parsed is not None:
            return last_parsed
        if raw:
            self.logger.error("Timeout. Raw RX during wait (%d B): %s", len(raw), _hexdump(bytes(raw)))
        else:
            self.logger.error("Timeout. No data received at all.")
        raise TimeoutError("No status frame within timeout")

    def _txrx_set(self, addr6: int, data26: int, retries: int = 2) -> Dict[str, Any]:
        last_exc = None
        for k in range(retries + 1):
            try:
                self._tx_set_once(addr6, data26)
                return self._rx_status_once()
            except Exception as e:
                last_exc = e
                self.logger.warning("TX/RX try %d/%d failed: %s", k+1, retries+1, e)
                time.sleep(0.05)
        assert last_exc is not None
        raise last_exc

    # ---------- Parsing ----------
    def _parse_status(self, p: bytes) -> Dict[str, Any]:
        """
        Standardlayout (25..29 bytes). Unterstützt optionale Setpoint-Felder,
        falls die Firmware sie mitsendet. (Deine Logs zeigen Setpoints.)
        """
        flags = p[0]
        P11 = self._le16(p, 1)
        P31 = self._le16(p, 3)

        flowCO2 = self._le16(p, 5) / 100.0
        totCO2  = self._le32_val(p, 7) / 100.0

        flowN2  = self._le16(p, 11) / 100.0
        totN2   = self._le32_val(p, 13) / 100.0

        flowBut = self._le16(p, 17) / 100.0
        totBut  = self._le32_val(p, 19) / 100.0

        # Optionale Setpoints am Ende (je 2 Bytes, 0.01 Auflösung) – defensiv prüfen
        sp_co2 = sp_n2 = sp_but = None
        # Bekannte Varianten: 25 (ohne), 29 (mit 2 Bytes? + padding), ggf. 31 (mit 3×U16)
        # Wir lesen, was da ist, in Reihenfolge: CO2, N2, Butan
        try:
            # Rest ab Offset 23
            off = 23
            remain = len(p) - off
            if remain >= 2:
                sp_co2 = self._le16(p, off) / 100.0
                off += 2; remain -= 2
            if remain >= 2:
                sp_n2 = self._le16(p, off) / 100.0
                off += 2; remain -= 2
            if remain >= 2:
                sp_but = self._le16(p, off) / 100.0
        except Exception:
            pass

        return {
            "io": {
                "GasLeak": bool(flags & (1 << 0)),
                "V1":      bool(flags & (1 << 1)),
                "V2":      bool(flags & (1 << 2)),
                "V3":      bool(flags & (1 << 3)),
                "FanIn":   bool(flags & (1 << 4)),
                "FanOut":  bool(flags & (1 << 5)),
            },
            "adc": {"P11": P11, "P31": P31},
            "mfc": {
                "co2":   {"flow": flowCO2, "total": totCO2, "setpoint": sp_co2},
                "n2":    {"flow": flowN2,  "total": totN2,  "setpoint": sp_n2},
                "butan": {"flow": flowBut, "total": totBut, "setpoint": sp_but},
            }
        }

    # ---------- Öffentliche API ----------
    def get_status(self) -> Dict[str, Any]:
        return self._txrx_set(self.A_RequestStatus, 1)

    # --- High-Level Shims (für VaderDeviceDriver) ---------------------------
    def request_status(self) -> None:
        """Nicht-blockierende Status-Anfrage: sendet nur das Request-Frame."""
        try:
            self._tx_set_once(self.A_RequestStatus, 1)
        except Exception as e:
            self.logger.warning("request_status() TX failed: %s", e)

    def stop(self) -> None:
        """Alias für close() – vom High-Level-Driver erwartet."""
        self.close()

    # Standard-API
    def set_v1(self, open_: bool) -> Dict[str, Any]:
        return self._txrx_set(self.A_V1, 1 if open_ else 0)

    def set_v2(self, open_: bool) -> Dict[str, Any]:
        return self._txrx_set(self.A_V2, 1 if open_ else 0)

    def set_v3(self, open_: bool) -> Dict[str, Any]:
        return self._txrx_set(self.A_V3, 1 if open_ else 0)

    def set_fan_in(self, on: bool) -> Dict[str, Any]:
        return self._txrx_set(self.A_FanIn, 1 if on else 0)

    def set_fan_out(self, on: bool) -> Dict[str, Any]:
        return self._txrx_set(self.A_FanOut, 1 if on else 0)

    def set_flow_co2(self, flow: float) -> Dict[str, Any]:
        return self._txrx_set(self.A_FlowCO2, self._u26_from_float100(flow))

    def set_flow_n2(self, flow: float) -> Dict[str, Any]:
        return self._txrx_set(self.A_FlowN2, self._u26_from_float100(flow))

    def set_flow_butan(self, flow: float) -> Dict[str, Any]:
        return self._txrx_set(self.A_FlowBut, self._u26_from_float100(flow))

    def set_total_co2(self, value: float = 0.0) -> Dict[str, Any]:
        return self._txrx_set(self.A_TotCO2, self._u26_from_float100(value))

    def set_total_n2(self, value: float = 0.0) -> Dict[str, Any]:
        return self._txrx_set(self.A_TotN2,  self._u26_from_float100(value))

    def set_total_butan(self, value: float = 0.0) -> Dict[str, Any]:
        return self._txrx_set(self.A_TotBut, self._u26_from_float100(value))

    def close(self) -> None:
        self.dev.close()


# parser.add_argument("--port", default="/dev/tty.usbmodem1133201", help="Serieller Port (z.B. /dev/tty.usbmodemXXXXXXXX)")
# ----------------------- CLI / Test -----------------------
# ----------------------- CLI / Test -----------------------

import math
from contextlib import suppress

# ---- neue/ersetzte Helpers ----
def _nearly(a: float, b: float, tol: float = 0.02) -> bool:
    try:
        return abs(float(a) - float(b)) <= tol
    except Exception:
        return False

def _require(cond: bool, msg: str):
    if not cond:
        raise AssertionError(msg)

def _keys_ok(s: Dict[str, Any]) -> None:
    _require(isinstance(s, dict), "Status ist kein Dict")
    for k in ("io", "adc", "mfc"):
        _require(k in s, f"Key '{k}' fehlt im Status")
    for k in ("P11", "P31"):
        _require(k in s["adc"], f"ADC '{k}' fehlt")
    for g in ("co2", "n2", "butan"):
        _require(g in s["mfc"], f"MFC '{g}' fehlt")
        for f in ("flow", "total"):
            _require(f in s["mfc"][g], f"MFC '{g}.{f}' fehlt")

def _check_toggle(mx: "MAXI", func_name: str, io_key: str):
    func = getattr(mx, func_name)
    s1 = func(True)
    _keys_ok(s1); _require(s1["io"][io_key] is True, f"{io_key} sollte True sein nach {func_name}(True)")
    s2 = func(False)
    _keys_ok(s2); _require(s2["io"][io_key] is False, f"{io_key} sollte False sein nach {func_name}(False)")

def _read_sp(s: Dict[str, Any], mfc_key: str):
    sp = s["mfc"][mfc_key].get("setpoint")
    return None if sp is None else float(sp)

def _check_set_flow(mx: "MAXI", func_name: str, mfc_key: str, value: float,
                    maxflow: float, strict: bool):
    """
    Erwartung:
      - Wenn 0 <= value <= maxflow: Setpoint ~== value (±0.02)
      - Wenn value > maxflow: Gerät darf clippen/ignorieren -> WARN (oder FAIL in strict)
    """
    func = getattr(mx, func_name)

    # Vorherigen SP merken
    s_prev = mx.get_status()
    _keys_ok(s_prev)
    sp_prev = _read_sp(s_prev, mfc_key)

    # Setzen
    s = func(value)
    _keys_ok(s)
    sp_now = _read_sp(s, mfc_key)

    _require(sp_now is not None, f"Setpoint {mfc_key} wurde nicht mitgesendet")

    if 0.0 <= value <= maxflow:
        _require(_nearly(sp_now, value),
                 f"{mfc_key} Setpoint ~={value} erwartet, got {sp_now}")
    else:
        # Out-of-range: akzeptiere 'no change' oder 'clip ≤ maxflow'
        if strict:
            _require(_nearly(sp_now, value) or
                     (sp_prev is not None and _nearly(sp_now, sp_prev)) or
                     (sp_now <= maxflow + 1e-9),
                     f"{mfc_key} Out-of-range Verhalten unerwartet: wanted>{maxflow}, got {sp_now}, prev {sp_prev}")
        else:
            logging.getLogger(f"{MODULE_LOGGER_NAME}.Selftest").warning(
                "%s Setpoint %.3f außerhalb Range (max %.3f). Gerät antwortete mit %.3f (prev %.3f).",
                mfc_key.upper(), value, maxflow, sp_now, sp_prev if sp_prev is not None else float("nan")
            )

def _check_set_total(mx: "MAXI", func_name: str, mfc_key: str, value: float):
    func = getattr(mx, func_name)
    s = func(value)
    _keys_ok(s)
    tot = s["mfc"][mfc_key]["total"]
    _require(_nearly(tot, value), f"{mfc_key} Total ~={value} erwartet, got {tot}")

def run_selftest(mx: "MAXI", *, maxflow: float, strict: bool):
    print("▶︎ Selbsttest startet …")

    # 0) Smoke
    print("  • Frage Status ab …")
    s0 = mx.get_status()
    _keys_ok(s0)

    # 1) IO
    print("  • Toggle V1/V2/V3 …")
    _check_toggle(mx, "set_v1", "V1")
    _check_toggle(mx, "set_v2", "V2")
    _check_toggle(mx, "set_v3", "V3")

    print("  • Toggle FanIn/FanOut …")
    _check_toggle(mx, "set_fan_in", "FanIn")
    _check_toggle(mx, "set_fan_out", "FanOut")

    # 2) MFC Setpoints – „sichere“ Werte im Bereich
    print("  • Setze MFC-Setpoints (CO2/N2/Butan) …")
    _check_set_flow(mx, "set_flow_co2",  "co2",   5.00, maxflow, strict)
    _check_set_flow(mx, "set_flow_n2",   "n2",    3.25, maxflow, strict)
    _check_set_flow(mx, "set_flow_butan","butan", 1.00, maxflow, strict)

    # 3) Totals
    print("  • Schreibe/prüfe Totalisatoren …")
    _check_set_total(mx, "set_total_co2",  "co2",   0.00)
    _check_set_total(mx, "set_total_n2",   "n2",    0.00)
    _check_set_total(mx, "set_total_butan","butan", 0.00)

    # 4) Expliziter Request-Status
    print("  • Expliziter Request-Status und direktes RX …")
    mx.request_status()
    s_req = mx._rx_status_once()
    _keys_ok(s_req)

    # 5) Sammelfenster
    print("  • Test Sammelfenster (settle_ms) …")
    old_settle = mx.settle_ms
    try:
        mx.settle_ms = 150
        s_settle = mx.get_status()
        _keys_ok(s_settle)
    finally:
        mx.settle_ms = old_settle

    # 6) Roundtrip & Out-of-range Probe (keine harte Assertion, außer --strict-setpoint)
    print("  • Roundtrip & Out-of-range Probe …")
    _check_set_flow(mx, "set_flow_co2", "co2", 0.00, maxflow, strict)
    _check_set_flow(mx, "set_flow_co2", "co2", 0.01, maxflow, strict)
    _check_set_flow(mx, "set_flow_co2", "co2", maxflow * 1.5, maxflow, strict)  # ggf. clip/ignore

    print("✅ Selbsttest abgeschlossen – alle Checks OK (unter den gewählten Grenzen).")

def _demo_sequence(mx: "MAXI") -> None:
    print("Frage Status ab …")
    print(mx.get_status())

    print("\nV1 = OPEN …")
    print(mx.set_v1(True))

    print("\nV1 = CLOSE …")
    print(mx.set_v1(False))

    print("\nCO2 Flow = 5.0 sccm …")
    print(mx.set_flow_co2(5.0))

    print("\nNochmals Status …")
    print(mx.get_status())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MAXI Treiber – synchrones Testtool")
    parser.add_argument("--port", default="/dev/cu.usbmodem1133201")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--settle", type=int, default=0)
    parser.add_argument("--log", default="DEBUG")
    parser.add_argument("--selftest", action="store_true", default=True)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--maxflow", type=float, default=10.0, help="erwartete Maximalrange für Flow-Setpoints (sccm)")
    parser.add_argument("--strict-setpoint", action="store_true", help="Out-of-range Verhalten als Fehler werten")
    args = parser.parse_args()

    level = getattr(logging, args.log.upper(), logging.DEBUG)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    mx = None
    try:
        mx = MAXI(args.port, timeout=args.timeout, settle_ms=args.settle)
        if args.demo and not args.selftest:
            _demo_sequence(mx)
        else:
            run_selftest(mx, maxflow=args.maxflow, strict=args.strict_setpoint)
    except Exception as e:
        print(f"Fehler: {e}")
        raise
    finally:
        try:
            if mx: mx.close()
        except Exception:
            pass