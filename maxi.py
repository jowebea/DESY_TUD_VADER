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
    STATUS_LEN = 29

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
                        if length not in (self.STATUS_LEN, 25, 23):
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
        flags = p[0]
        P11 = self._le16(p, 1)
        P31 = self._le16(p, 3)
        flowCO2 = self._le16(p, 5) / 100.0
        totCO2  = self._le32_val(p, 7) / 100.0
        flowN2  = self._le16(p, 11) / 100.0
        totN2   = self._le32_val(p, 13) / 100.0
        flowBut = self._le16(p, 17) / 100.0
        totBut  = self._le32_val(p, 19) / 100.0
        spCO2 = self._le16(p, 23) / 100.0 if len(p) >= 29 else None
        spN2  = self._le16(p, 25) / 100.0 if len(p) >= 29 else None
        spBut = self._le16(p, 27) / 100.0 if len(p) >= 29 else None
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
                "co2":   {"flow": flowCO2, "total": totCO2, "setpoint": spCO2},
                "n2":    {"flow": flowN2,  "total": totN2,  "setpoint": spN2},
                "butan": {"flow": flowBut, "total": totBut, "setpoint": spBut},
             }
         }

    # ---------- Öffentliche API ----------
    def get_status(self) -> Dict[str, Any]:
        return self._txrx_set(self.A_RequestStatus, 1)

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


# ----------------------- CLI / Test -----------------------

def _demo_sequence(mx: MAXI) -> None:
    print("Frage Status ab ...")
    print(mx.get_status())

    print("\nV1 = OPEN ...")
    print(mx.set_v1(True))

    print("\nV1 = CLOSE ...")
    print(mx.set_v1(False))

    print("\nCO2 Flow = 5.0 sccm ...")
    print(mx.set_flow_co2(5.0))

    print("\nNochmals Status ...")
    print(mx.get_status())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MAXI Treiber – synchrones Testtool")
    parser.add_argument("--port", default="/dev/tty.usbmodem1133201", help="Serieller Port (z.B. /dev/tty.usbmodemXXXXXXXX)")
    parser.add_argument("--timeout", type=float, default=8.0, help="RX Timeout in Sekunden")
    parser.add_argument("--settle", type=int, default=0, help="Sammelfenster in ms (0 zum Deaktivieren)")
    parser.add_argument("--log", default="DEBUG", help="Loglevel (DEBUG/INFO/WARN/ERROR)")
    args = parser.parse_args()

    level = getattr(logging, args.log.upper(), logging.DEBUG)
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    try:
        mx = MAXI(args.port, timeout=args.timeout, settle_ms=args.settle)
        _demo_sequence(mx)
    except Exception as e:
        print(f"Fehler: {e}")
    finally:
        try:
            mx.close()
        except Exception:
            pass