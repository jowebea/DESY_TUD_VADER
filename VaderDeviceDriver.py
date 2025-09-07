import logging
import threading
import time
import re
from collections import deque
from datetime import datetime, timezone
from typing import Optional, Tuple, Dict, Any, List, Union
import serial

# ====================== Serial wrapper ======================
class SerialDevice:
    """
    Dünne Serial-Wrapper-Klasse mit Lock. Unterstützt Byte-IO (neu) und
    weiterhin einfache Line-IO (für MINI1).
    """
    def __init__(self, port: str, baudrate: int = 115200, name: str = ""):
        self.name = name
        self._lock = threading.Lock()
        self.ser = serial.Serial(port, baudrate, timeout=0.1)

    # --- Byte-IO ---
    def write_bytes(self, data: bytes) -> None:
        with self._lock:
            self.ser.write(data)

    def read_bytes(self, n: int) -> bytes:
        try:
            return self.ser.read(n)
        except Exception:
            return b""

    # --- Line-IO (nur noch von MINI1 genutzt) ---
    def write_line(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        with self._lock:
            self.ser.write(line.encode())

    def readline(self) -> Optional[str]:
        try:
            line = self.ser.readline()
            if not line:
                return None
            return line.decode(errors="ignore").strip()
        except Exception:
            return None

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


# ====================== MINI1 ======================
class MINI1:
    """
    MINI1 liefert weiterhin ASCII-Zahlenzeilen (Druck). Wir puffern eine Zeitreihe.
    """
    def __init__(self, port: str, max_samples: int = 300_000):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.dev = SerialDevice(port, 115200, name="MINI1")
        self._series: deque[Tuple[datetime, float]] = deque(maxlen=max_samples)
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

    @staticmethod
    def _compute_pressure(p20: float, p21: float) -> float:
        return p20 if (p21 < 150 and abs(p21 - p20) < 5) else p21

    def _reader_loop(self) -> None:
        while self._running:
            line = self.dev.readline()
            if not line:
                time.sleep(0.0005)
                continue
            try:
                pressure = float(line)
            except ValueError:
                continue
            with self._lock:
                self._series.append((datetime.now(timezone.utc), pressure))

    def get_last_n(self, n: Optional[int] = None) -> List[Tuple[datetime, float]]:
        with self._lock:
            data = list(self._series)
        return data if n is None or n >= len(data) else data[-n:]

    @property
    def pressure(self) -> Optional[float]:
        with self._lock:
            return self._series[-1][1] if self._series else None

    def set_max_samples(self, max_samples: int) -> None:
        with self._lock:
            self._series = deque(self._series, maxlen=max_samples)

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()

    def __enter__(self): return self
    def __exit__(self, *exc): self.stop()


# ====================== MINI2 (aus deiner Vorlage) ======================
class MINI2:
    """
    MINI2 – Binärprotokoll (32-­Bit Words, Big-Endian)

    Frames (addr = BASE_ADDR + x):
      SET (Python->Arduino):
        BASE+0: MODE(2)@25..24 (ignoriert), VP1(8)@23..16, VP2(8)@15..8
        BASE+1: SETPOINT(16, 0.1 kPa)@25..10
        BASE+5: RAMP_SPEED(16, 0.01 kPa/s)@25..10, RAMP_END(10, kPa)@9..0
        BASE+6: COMMANDS: Bit0=SEND_STATUS

      STATUS (Arduino->Python):
        BASE+2: P20(16, 0.1 kPa)@25..10
        BASE+3: P21(16, 0.1 kPa)@25..10
        BASE+4: MODE(2)@25..24 (spiegelt STATE), VP1(8)@23..16, VP2(8)@15..8

    STATE-Ableitung in der Firmware:
      - SETPOINT empfangen  -> STATE = AUTOMATIC
      - VP1/VP2 empfangen   -> STATE = MANUAL
      - RAMP_* empfangen    -> STATE = RAMP
    """
    def __init__(self, port: str, base_addr: int = 0x05):
        self.dev = SerialDevice(port, 115200, name="MINI2")
        self.base = int(base_addr) & 0x3F

        # letzter Status
        self._lock = threading.Lock()
        self._last_status_time = 0.0
        self.status: Dict[str, Any] = {
            # gefüllte Felder nach ersten Statusframes
            # 'p20_kpa', 'p21_kpa', 'pwm1', 'pwm2', 'mode', 'ts'
        }

        # Byte-Puffer für eingehende Daten
        self._rx_buf = bytearray()
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

        # initial Status anfordern
        self.request_status()
        self._wait_for_recent_status(0.5)

    # ---------- Word Packing/Unpacking ----------
    @staticmethod
    def _make_word(addr6: int, payload26: int) -> int:
        return ((addr6 & 0x3F) << 26) | (payload26 & 0x03FF_FFFF)

    @staticmethod
    def _u32_to_be_bytes(word: int) -> bytes:
        return bytes([(word >> 24) & 0xFF, (word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF])

    @staticmethod
    def _be_bytes_to_u32(b: bytes) -> int:
        return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]

    # ---------- Senden ----------
    def _send_word(self, addr: int, payload: int) -> None:
        word = self._make_word(addr, payload)
        self.dev.write_bytes(self._u32_to_be_bytes(word))

    def request_status(self) -> None:
        # BASE+6, Bit0=1
        self._send_word(self.base + 6, 0x1)

    def set_target_pressure(self, kpa: Union[int, float]) -> None:
        # BASE+1, raw = kPa*10 @25..10
        raw = max(0, min(0xFFFF, int(round(float(kpa) * 10.0))))
        payload = (raw & 0xFFFF) << 10
        self._send_word(self.base + 1, payload)

    def set_manual_vp1(self, value: int) -> None:
        # BASE+0, MODE ignoriert; VP1/VP2 setzen -> STATE=MANUAL
        v1 = max(0, min(255, int(value)))
        v2 = int(self.status.get("pwm2", 0)) & 0xFF
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_manual_vp2(self, value: int) -> None:
        v2 = max(0, min(255, int(value)))
        v1 = int(self.status.get("pwm1", 0)) & 0xFF
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_state_manual(self) -> None:
        """State wird durch PWM-Frame ausgelöst: wir senden die aktuellen PWM-Werte (oder 0/0)."""
        v1 = int(self.status.get("pwm1", 0)) & 0xFF
        v2 = int(self.status.get("pwm2", 0)) & 0xFF
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_ramp(self, speed_kpa_s: float, end_kpa: float) -> None:
        """Startdruck übernimmt die Firmware aus aktuellem Druck."""
        raw_speed = max(0, min(0xFFFF, int(round(speed_kpa_s * 100.0))))  # 0.01 kPa/s
        raw_end   = max(0, min(0x03FF,  int(round(end_kpa))))              # 10 Bit kPa
        payload   = (raw_speed << 10) | raw_end
        self._send_word(self.base + 5, payload)

    # ---------- Empfangen ----------
    def _reader_loop(self) -> None:
        while self._running:
            chunk = self.dev.read_bytes(64)  # bis zu 64 Bytes auf einmal
            if chunk:
                self._rx_buf.extend(chunk)
                self._drain_words()
            else:
                time.sleep(0.002)

    def _drain_words(self) -> None:
        # in 4-Byte-Schritten parsen; Überhang bleibt im Buffer
        while len(self._rx_buf) >= 4:
            word_bytes = bytes(self._rx_buf[:4])
            del self._rx_buf[:4]
            try:
                word = self._be_bytes_to_u32(word_bytes)
                self._handle_status_word(word)
            except Exception:
                # bei Fehler nicht resyncen (Protokoll ist framing-fest), ggf. weiter lesen
                continue

    def _handle_status_word(self, word: int) -> None:
        addr = (word >> 26) & 0x3F
        payload = word & 0x03FF_FFFF
        with self._lock:
            now = time.time()
            self._last_status_time = now
            self.status["ts"] = now

            if addr == (self.base + 2):
                p20_raw = (payload >> 10) & 0xFFFF
                self.status["p20_kpa"] = float(p20_raw) * 0.1
            elif addr == (self.base + 3):
                p21_raw = (payload >> 10) & 0xFFFF
                self.status["p21_kpa"] = float(p21_raw) * 0.1
            elif addr == (self.base + 4):
                mode2 = (payload >> 24) & 0x03
                vp1   = (payload >> 16) & 0xFF
                vp2   = (payload >>  8) & 0xFF
                self.status["mode"] = {0: "MANUAL", 1: "RAMP", 2: "AUTOMATIC"}.get(mode2, "UNKNOWN")
                self.status["pwm1"] = vp1
                self.status["pwm2"] = vp2
            # andere Adressen ignorieren (SET echos werden nicht gesendet)

    # ---------- Utilities ----------
    def is_recent(self, max_age_s: float = 0.5) -> bool:
        with self._lock:
            return (time.time() - self._last_status_time) <= max_age_s

    def _wait_for_recent_status(self, timeout: float = 0.5) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.is_recent(0.3):
                return True
            time.sleep(0.01)
        return False

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            # zusätzliche Komfort-Felder
            out = dict(self.status)
            # „beste“ Druck-Wahl analog MINI1:
            p21 = out.get("p21_kpa"); p20 = out.get("p20_kpa")
            if p21 is not None and p20 is not None:
                best = p20 if (p21 < 150 and abs(p21 - p20) < 5) else p21
                out["pressure_kpa"] = best
            elif p21 is not None:
                out["pressure_kpa"] = p21
            elif p20 is not None:
                out["pressure_kpa"] = p20
            return out

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()


# ====================== MAXI ======================
class MAXI:
    """
    MAXI-Treiber für das neue binäre Protokoll.

    Status (Arduino -> Python):
      Frame: 0xAA 0x55, LEN=25, PAYLOAD[LEN], CS=sum(PAYLOAD)&0xFF
      PAYLOAD:
        [0]     : FLAGS Bit0=GasLeak, Bit1=V1, Bit2=V2, Bit3=V3, Bit4=FanIn, Bit5=FanOut
        [1..2]  : P11_ADC  (LE, uint16)
        [3..4]  : P31_ADC  (LE, uint16)
        [5..6]  : FlowCO2  (LE, uint16, x100)
        [7..10] : TotCO2   (LE, uint32, x100)
        [11..12]: FlowN2   (LE, uint16, x100)
        [13..16]: TotN2    (LE, uint32, x100)
        [17..18]: FlowBut  (LE, uint16, x100)
        [19..22]: TotBut   (LE, uint32, x100)
        [23..24]: Padding

    SET (Python -> Arduino):
      Frame: 0xA5 0x5A 0x01 0x04  PAYLOAD[4]  CS
      PAYLOAD ist little-endian 32-bit WORD: WORD = (DATA26 << 6) | ADDR6
      CS = sum(PAYLOAD) & 0xFF

    Adressen (ADDR6):
      0x00=RequestStatus, 0x01=V1, 0x02=V2, 0x03=V3, 0x04=FanIn, 0x05=FanOut,
      0x10=FlowCO2, 0x11=TotCO2, 0x12=FlowN2, 0x13=TotN2, 0x14=FlowBut, 0x15=TotBut
    """

    # --- Konstanten ---
    SYNC0 = 0xAA
    SYNC1 = 0x55
    STATUS_LEN = 25  # falls Firmware auf 23 umgestellt wird, hier anpassen

    H0 = 0xA5
    H1 = 0x5A
    CMD_SET = 0x01
    SET_LEN = 0x04

    # ADDR6
    A_RequestStatus = 0x00
    A_V1, A_V2, A_V3 = 0x01, 0x02, 0x03
    A_FanIn, A_FanOut = 0x04, 0x05
    A_FlowCO2, A_TotCO2 = 0x10, 0x11
    A_FlowN2,  A_TotN2  = 0x12, 0x13
    A_FlowBut, A_TotBut = 0x14, 0x15

    def __init__(self, port: str):
        self.dev = SerialDevice(port, 115200, name="MAXI")
        self._lock = threading.Lock()
        self._running = True
        self._rx = bytearray()
        self._last_status_ts = 0.0

        # strukturierte Statusdaten (SI-Einheiten)
        self.status: Dict[str, Any] = {
            "io": {"V1": False, "V2": False, "V3": False, "FanIn": False, "FanOut": False, "GasLeak": False},
            "adc": {"P11": None, "P31": None},
            "mfc": {
                "co2":  {"flow": None, "total": None},
                "n2":   {"flow": None, "total": None},
                "butan":{"flow": None, "total": None},
            },
        }

        threading.Thread(target=self._reader_loop, daemon=True).start()
        # initial Status anfordern
        self.request_status()

    # -------------------- Helpers: encode/decode --------------------
    @staticmethod
    def _u26_from_float100(x: Union[int, float]) -> int:
        v = int(round(float(x) * 100.0))
        if v < 0: v = 0
        if v > 0x03FF_FFFF: v = 0x03FF_FFFF
        return v

    @staticmethod
    def _le32(word: int) -> bytes:
        return bytes([word & 0xFF, (word >> 8) & 0xFF, (word >> 16) & 0xFF, (word >> 24) & 0xFF])

    @staticmethod
    def _checksum_sum(data: bytes) -> int:
        return sum(data) & 0xFF

    # -------------------- Senden --------------------
    def _send_set(self, addr6: int, data26: int) -> None:
        word = ((data26 & 0x03FF_FFFF) << 6) | (addr6 & 0x3F)
        payload = self._le32(word)
        frame = bytes([self.H0, self.H1, self.CMD_SET, self.SET_LEN]) + payload + bytes([self._checksum_sum(payload)])
        self.dev.write_bytes(frame)

    def request_status(self) -> None:
        self._send_set(self.A_RequestStatus, 1)

    # --- IO ---
    def set_v1(self, open_: bool) -> None:    self._send_set(self.A_V1,    1 if open_ else 0)
    def set_v2(self, open_: bool) -> None:    self._send_set(self.A_V2,    1 if open_ else 0)
    def set_v3(self, open_: bool) -> None:    self._send_set(self.A_V3,    1 if open_ else 0)
    def set_fan_in(self, on: bool) -> None:   self._send_set(self.A_FanIn, 1 if on else 0)
    def set_fan_out(self, on: bool) -> None:  self._send_set(self.A_FanOut,1 if on else 0)

    # --- MFC Setpoints / Totals (float, SI) ---
    def set_flow_co2(self, flow: float) -> None:   self._send_set(self.A_FlowCO2, self._u26_from_float100(flow))
    def set_flow_n2(self, flow: float) -> None:    self._send_set(self.A_FlowN2,  self._u26_from_float100(flow))
    def set_flow_butan(self, flow: float) -> None: self._send_set(self.A_FlowBut, self._u26_from_float100(flow))

    def set_total_co2(self, value: float = 0.0) -> None:   self._send_set(self.A_TotCO2, self._u26_from_float100(value))
    def set_total_n2(self, value: float = 0.0) -> None:    self._send_set(self.A_TotN2,  self._u26_from_float100(value))
    def set_total_butan(self, value: float = 0.0) -> None: self._send_set(self.A_TotBut, self._u26_from_float100(value))

    # -------------------- Empfangen --------------------
    def _reader_loop(self) -> None:
        while self._running:
            chunk = self.dev.read_bytes(64)
            if chunk:
                self._rx.extend(chunk)
                self._drain_status_frames()
            else:
                time.sleep(0.002)

    def _find_sync(self) -> int:
        # suche nach 0xAA 0x55
        for i in range(max(0, len(self._rx) - 1)):
            if self._rx[i] == self.SYNC0 and self._rx[i + 1] == self.SYNC1:
                return i
        return -1

    def _drain_status_frames(self) -> None:
        while True:
            idx = self._find_sync()
            if idx < 0:
                # zu viele Bytes? begrenzen
                if len(self._rx) > 2048:
                    del self._rx[:-2]
                return
            # entferne Bytes vor Sync
            if idx > 0:
                del self._rx[:idx]
            if len(self._rx) < 3:
                return
            # [0]=AA, [1]=55, [2]=LEN
            length = self._rx[2]
            total = 3 + length + 1  # +checksum
            if len(self._rx) < total:
                return
            # vollständigen Frame herauslösen
            payload = bytes(self._rx[3:3 + length])
            cs = self._rx[3 + length]
            del self._rx[:total]

            if length not in (self.STATUS_LEN, 23):
                continue
            if (self._checksum_sum(payload) & 0xFF) != cs:
                continue
            try:
                self._apply_status_payload(payload)
            except Exception:
                continue

    @staticmethod
    def _le16(b: bytes, off: int) -> int:
        return b[off] | (b[off + 1] << 8)

    @staticmethod
    def _le32_val(b: bytes, off: int) -> int:
        return b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24)

    def _apply_status_payload(self, p: bytes) -> None:
        # siehe Layout oben
        flags = p[0]
        P11 = self._le16(p, 1)
        P31 = self._le16(p, 3)
        flowCO2 = self._le16(p, 5) / 100.0
        totCO2  = self._le32_val(p, 7) / 100.0
        flowN2  = self._le16(p, 11) / 100.0
        totN2   = self._le32_val(p, 13) / 100.0
        flowBut = self._le16(p, 17) / 100.0
        totBut  = self._le32_val(p, 19) / 100.0

        with self._lock:
            self.status["io"]["GasLeak"] = bool(flags & (1 << 0))
            self.status["io"]["V1"]      = bool(flags & (1 << 1))
            self.status["io"]["V2"]      = bool(flags & (1 << 2))
            self.status["io"]["V3"]      = bool(flags & (1 << 3))
            self.status["io"]["FanIn"]   = bool(flags & (1 << 4))
            self.status["io"]["FanOut"]  = bool(flags & (1 << 5))

            self.status["adc"]["P11"] = P11
            self.status["adc"]["P31"] = P31

            self.status["mfc"]["co2"]["flow"]    = flowCO2
            self.status["mfc"]["co2"]["total"]   = totCO2
            self.status["mfc"]["n2"]["flow"]     = flowN2
            self.status["mfc"]["n2"]["total"]    = totN2
            self.status["mfc"]["butan"]["flow"]  = flowBut
            self.status["mfc"]["butan"]["total"] = totBut

            self._last_status_ts = time.time()

    # -------------------- Public API --------------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "ts": self._last_status_ts,
                **self.status,
            }

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()


class VaderDeviceDriver:
    """
    High-Level-Kapselung für MINI1, MINI2 und MAXI (neues Protokoll).
    V4 ist entfernt; stattdessen nutzt vac_all V1..V3 und FanIn/FanOut.
    """
    def __init__(self, mini1_port: str, mini2_port: str, maxi_port: str, mini1_max_samples: int = 300_000):
        self.mini1 = MINI1(mini1_port, max_samples=mini1_max_samples)
        self.mini2 = MINI2(mini2_port)
        self.maxi  = MAXI(maxi_port)

    def set_manual_PWM_in(self, value: int) -> None:
        """Setzt VP1 (PWM_in) am MINI2 im MANUAL-Mode."""
        self.mini2.set_manual_vp1(value)

    def set_manual_PWM_out(self, value: int) -> None:
        """Setzt VP2 (PWM_out) am MINI2 im MANUAL-Mode."""
        self.mini2.set_manual_vp2(value)

    def set_v3(self, open_: bool) -> None:
        """Steuert Ventil V3 am MAXI."""
        self.maxi.set_v3(open_)

    def storage_volume(self, open_: bool) -> None:
        self.maxi.set_v1(open_)
        self.maxi.set_v2(open_)

    def vac_all(self, vacuum_pressure_kpa: float, timeout_s: int = 120) -> None:
        # Alle MFCs zu
        self.maxi.set_flow_butan(0)
        self.maxi.set_flow_co2(0)
        self.maxi.set_flow_n2(0)

        # V1..V3 öffnen, zur Evakuierung
        self.maxi.set_v1(True)
        self.maxi.set_v2(True)
        self.maxi.set_v3(True)

        start = time.time()
        while True:
            p = self.mini1.pressure
            if p is not None and p <= vacuum_pressure_kpa:
                break
            if time.time() - start > timeout_s:
                break
            time.sleep(0.5)

        # zurücksetzen
        self.maxi.set_v1(False)
        self.maxi.set_v2(False)
        self.maxi.set_v3(False)

        # MINI2 druckseitig auf 0
        self.mini2.set_target_pressure(0)
        self.mini2.set_manual_vp1(0)
        self.mini2.set_manual_vp2(0)

    def use_gas(self, n2: float, co2: float, butan: float) -> None:
        self.maxi.set_flow_n2(n2)
        self.maxi.set_flow_co2(co2)
        self.maxi.set_flow_butan(butan)

    def setpoint_pressure(self, kpa: Union[int, float]) -> None:
        self.mini2.set_target_pressure(kpa)

    def get_all_status(self, mini1_last_n: Optional[int] = 100) -> Dict[str, Any]:
        self.mini2.request_status()
        self.maxi.request_status()
        self.mini2._wait_for_recent_status(0.5)

        return {
            "mini1": {
                "latest_pressure": self.mini1.pressure,
                "recent_series": self.mini1.get_last_n(mini1_last_n),
            },
            "mini2": self.mini2.get_status(),
            "maxi":  self.maxi.get_status(),
        }

    def reset_all_totalisators(self) -> None:
        self.maxi.set_total_butan(0.0)
        self.maxi.set_total_co2(0.0)
        self.maxi.set_total_n2(0.0)

    def close(self) -> None:
        self.mini1.stop()
        self.mini2.stop()
        self.maxi.stop()

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()