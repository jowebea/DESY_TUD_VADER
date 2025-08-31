import logging
import threading
import time
import re
from collections import deque
from datetime import datetime, timezone
from typing import Optional, Tuple, Dict, Any, List, Union

import serial

# === MAXI Addresses ===
ADDRESS_V1, ADDRESS_V2, ADDRESS_V3, ADDRESS_V4 = 1, 2, 3, 4
ADDRESS_FAN_IN, ADDRESS_FAN_OUT, ADDRESS_TEMP, ADDRESS_HEATER, ADDRESS_VS3, ADDRESS_ERROR = 8, 9, 10, 11, 12, 13
ADDRESS_MFC_BUTAN_FLOW, ADDRESS_MFC_BUTAN_TOTAL = 20, 21
ADDRESS_MFC_CO2_FLOW,   ADDRESS_MFC_CO2_TOTAL   = 30, 31
ADDRESS_MFC_N2_FLOW,    ADDRESS_MFC_N2_TOTAL    = 40, 41

# === MINI2 Addresses ===
ADR_TARGET_P, ADR_MAXSTEP, ADR_ERR, ADR_STATE = 3, 6, 13, 14
ADR_MANUAL_VP1, ADR_MANUAL_VP2 = 7, 8

# ====================== Serial wrapper ======================
class SerialDevice:
    def __init__(self, port: str, baudrate: int = 115200, name: str = ""):
        self.name = name
        self._lock = threading.Lock()
        self.ser = serial.Serial(port, baudrate, timeout=0.1)

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
    def __init__(self, port: str, max_samples: int = 300_000):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.dev = SerialDevice(port, 115200, name="MINI1")
        self._series: deque[Tuple[datetime, float]] = deque(maxlen=max_samples)
        self._lock = threading.Lock()
        self.p20: Optional[float] = None
        self.p21: Optional[float] = None
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

    @staticmethod
    def _compute_pressure(p20: float, p21: float) -> float:
        return p20 if (p21 < 150 and abs(p21 - p20) < 5) else p21

    def _reader_loop(self) -> None:
        while self._running:
            line = self.dev.readline()
            if not line:
                time.sleep(0.01)
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                p20_val, p21_val = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            pressure = self._compute_pressure(p20_val, p21_val)
            with self._lock:
                self.p20, self.p21 = p20_val, p21_val
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

# ====================== MINI2 ======================
class MINI2:
    """
    Protokoll (vereinfacht):
      - Status:  "1; 0; 0"
      - Setzen:  "2; <address>; <value>"
    """
    def __init__(self, port: str):
        self.dev = SerialDevice(port, 115200, name="MINI2")
        self.status: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._last_status_time = 0.0
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()
        self.request_status()
        self._wait_for_recent_status(0.3)

    # --- I/O ---
    def _send_raw(self, s: str) -> None:
        self.dev.write_line(s)

    def _send_cmd(self, command: int, address: int, value: Union[int, float]) -> None:
        self._send_raw(f"{command}; {address}; {value}")

    def request_status(self) -> None:
        self._send_cmd(1, 0, 0)

    # --- Control helpers used by high-level driver ---
    def set_target_pressure(self, kpa: Union[int, float]) -> None:
        self._send_cmd(2, ADR_TARGET_P, kpa)

    def set_state_manual(self) -> None:
        self._send_cmd(2, ADR_STATE, 1)

    def set_state_auto(self) -> None:
        self._send_cmd(2, ADR_STATE, 0)

    def set_manual_vp1(self, value: int) -> None:
        self._send_cmd(2, ADR_MANUAL_VP1, int(value))

    def set_manual_vp2(self, value: int) -> None:
        self._send_cmd(2, ADR_MANUAL_VP2, int(value))

    # --- Reader/Parser ---
    def _reader_loop(self) -> None:
        while self._running:
            line = self.dev.readline()
            if not line:
                time.sleep(0.005)
                continue
            self._parse_status_line(line)

    def _parse_status_line(self, line: str) -> None:
        if not line or line.strip() == "---":
            return
        low = line.lower()
        with self._lock:
            self._last_status_time = time.time()

            def f_after() -> Optional[float]:
                try:
                    return float(line.split(":", 1)[1].strip())
                except Exception:
                    return None

            def i_after() -> Optional[int]:
                try:
                    return int(line.split(":", 1)[1].strip())
                except Exception:
                    return None

            if low.startswith("leak_sensor:"):
                self.status["leak_sensor"] = i_after(); return
            if low.startswith("p (kpa):"):
                self.status["pressure_kpa"] = f_after(); return
            if low.startswith("setpoint p (kpa)"):
                self.status["setpoint_kpa"] = f_after()
                m = re.search(r"\[(\d+)\]", line); 
                if m: self.status["setpoint_index"] = int(m.group(1))
                return
            if low.startswith("p21 (kpa):") or low.startswith("p20 (kpa):"):
                tag = "p21" if "p21" in low else "p20"
                val = line.split(":", 1)[1].strip()
                m = re.match(r"\s*([+-]?\d+(?:\.\d+)?)\s*\(([-+]?\d+)\)", val)
                if m:
                    self.status[f"{tag}_kpa"] = float(m.group(1))
                    self.status[f"{tag}_raw"] = int(m.group(2))
                else:
                    try: self.status[f"{tag}_kpa"] = float(val)
                    except ValueError: pass
                return
            if low.startswith("kp:"):
                self.status["kp"] = f_after(); return
            if low.startswith("pwm1:"):
                self.status["pwm1"] = i_after(); return
            if low.startswith("pwm2:"):
                self.status["pwm2"] = i_after(); return
            # unbekannte Zeilen: ignorieren

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
            return dict(self.status)

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()

# ====================== MAXI ======================
class MAXI:
    """
    - Status:  "1; 0; 0"
    - Setzen:  "2; <address>; <value>"
    """
    _IO_MAP = {
        1: ("v1_power", "switch"),
        2: ("v2_power", "switch"),
        3: ("v3_power", "switch"),
        4: ("v4_power", "switch"),
        8: ("fan_in_power", "switch"),
        9: ("fan_out_power", "switch"),
        11: ("heater_pwm", "numeric"),
        12: ("vs3_power", "switch"),
        20: ("mfc_butan.setpoint", "float_100"),
        21: ("mfc_butan.totalisator", "float_100"),
        30: ("mfc_co2.setpoint", "float_100"),
        31: ("mfc_co2.totalisator", "float_100"),
        40: ("mfc_n2.setpoint", "float_100"),
        41: ("mfc_n2.totalisator", "float_100"),
    }

    def __init__(self, port: str):
        self.dev = SerialDevice(port, 115200, name="MAXI")
        self.status: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._last_status_ts = 0.0
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

    # --- Senden ---
    def _send(self, cmd: int, address: int, value: Union[int, float]) -> None:
        self.dev.write_line(f"{cmd}; {address}; {value}")

    def request_status(self) -> None:
        self._send(1, 0, 0)

    def set_component(self, address: int, value: int) -> None:
        self._send(2, address, int(value))

    def set_valve(self, valve_id: int, on: bool) -> None:
        self.set_component(valve_id, 1 if on else 0)

    def activate_v4(self, enable: bool) -> None:
        self.set_valve(4, enable)

    # --- MFC ---
    @staticmethod
    def _to_cLpm(flow_lpm: float) -> int:
        return int(round(flow_lpm * 100.0))

    def set_flow_butan(self, flow_lpm: float) -> None: self._send(2, 20, self._to_cLpm(flow_lpm))
    def set_flow_co2(self,   flow_lpm: float) -> None: self._send(2, 30, self._to_cLpm(flow_lpm))
    def set_flow_n2(self,    flow_lpm: float) -> None: self._send(2, 40, self._to_cLpm(flow_lpm))

    def set_totalisator_butan(self, value_lpm: float = 0.0) -> None: self._send(2, 21, self._to_cLpm(value_lpm))
    def set_totalisator_co2(self,   value_lpm: float = 0.0) -> None: self._send(2, 31, self._to_cLpm(value_lpm))
    def set_totalisator_n2(self,    value_lpm: float = 0.0) -> None: self._send(2, 41, self._to_cLpm(value_lpm))

    # Reset-Aliase (wurden im High-level aufgerufen)
    def reset_totalisator_butan(self) -> None: self.set_totalisator_butan(0.0)
    def reset_totalisator_co2(self)   -> None: self.set_totalisator_co2(0.0)
    def reset_totalisator_n2(self)    -> None: self.set_totalisator_n2(0.0)

    # --- Reader / Parser ---
    def _reader_loop(self) -> None:
        while self._running:
            line = self.dev.readline()
            if not line:
                time.sleep(0.002)
                continue
            self._parse_status_line(line)

    def _touch(self) -> None:
        self._last_status_ts = time.time()

    def _set_nested(self, key_path: str, value: Any) -> None:
        parts = key_path.split(".")
        with self._lock:
            d = self.status
            for p in parts[:-1]:
                d = d.setdefault(p, {})
            d[parts[-1]] = value
            self._touch()

    def _ensure_mfc(self, name: str) -> None:
        with self._lock:
            self.status.setdefault(name, {})
            self._touch()

    def _parse_status_line(self, line: str) -> None:
        low = line.lower().strip()

        if "butan/methan mfc" in low: self._ensure_mfc("mfc_butan"); return
        if "co2 mfc"         in low: self._ensure_mfc("mfc_co2");   return
        if "n2/argon mfc"    in low: self._ensure_mfc("mfc_n2");    return

        m_id = re.search(r"\[([0-9]{1,3})\]\s*:\s*([+-]?\d+(?:\.\d+)?)", line)
        if m_id:
            id_num, val_str = int(m_id.group(1)), m_id.group(2)
            try: val_num = float(val_str)
            except ValueError: val_num = val_str
            if id_num in self._IO_MAP:
                key, kind = self._IO_MAP[id_num]
                if kind == "switch":
                    on = float(val_num) != 0.0
                    self._set_nested(key, "ON" if on else "OFF")
                    self._set_nested(f"{key}_raw", int(float(val_num)))
                elif kind == "numeric":
                    self._set_nested(key, int(float(val_num)))
                elif kind == "float_100":
                    self._set_nested(key, float(val_num))
                if ".setpoint" in key:     self._set_nested(key.split(".")[0] + ".setpoint", float(val_num))
                if ".totalisator" in key: self._set_nested(key.split(".")[0] + ".totalisator", float(val_num))
            return

        m_sens = re.search(r"^([a-z0-9_]+)\s*:\s*([+-]?\d+(?:\.\d+)?)\s*(kpa|°c|c)?\s*\((\d+)\)\s*$", low)
        if m_sens:
            name, value = m_sens.group(1), float(m_sens.group(2))
            self._set_nested(name, value)
            self._set_nested(f"{name}_code", int(m_sens.group(4)))
            return

        m_unit = re.search(r"^\s*unit:\s*(.+)$", low)
        if m_unit:
            unit = m_unit.group(1).strip()
            target = self._last_mfc_key()
            if target: self._set_nested(f"{target}.unit", unit)
            return

        m_flow_named = re.search(r"^\s*flow\s+(butan|co2|n2)\s*:\s*([+-]?\d+(?:\.\d+)?)", low)
        if m_flow_named:
            gas, val = m_flow_named.group(1), float(m_flow_named.group(2))
            mfc = {"butan": "mfc_butan", "co2": "mfc_co2", "n2": "mfc_n2"}[gas]
            self._set_nested(f"{mfc}.flow", val)
            return

        m_temp = re.search(r"^\s*temperature\s*:\s*([+-]?\d+(?:\.\d+)?)", low)
        if m_temp:
            target = self._last_mfc_key()
            if target: self._set_nested(f"{target}.temperature", float(m_temp.group(1)))
            return

    def _last_mfc_key(self) -> Optional[str]:
        with self._lock:
            for k in ("mfc_n2", "mfc_co2", "mfc_butan"):
                if k in self.status: return k
        return None

    def wait_for_fresh_status(self, timeout: float = 1.0) -> bool:
        before = self._last_status_ts
        self.request_status()
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._last_status_ts > before: return True
            time.sleep(0.01)
        return False

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.status)

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()

# ====================== High-level Driver ======================
class VaderDeviceDriver:
    def __init__(self, mini1_port: str, mini2_port: str, maxi_port: str, mini1_max_samples: int = 300_000):
        self.mini1 = MINI1(mini1_port, max_samples=mini1_max_samples)
        self.mini2 = MINI2(mini2_port)
        self.maxi  = MAXI(maxi_port)

    def storage_volume(self, open_: bool) -> None:
        self.maxi.set_valve(ADDRESS_V1, open_)
        self.maxi.set_valve(ADDRESS_V2, open_)

    def vac_all(self, vacuum_pressure_kpa: float, timeout_s: int = 120) -> None:
        # Alle MFCs zu
        self.maxi.set_flow_butan(0); self.maxi.set_flow_co2(0); self.maxi.set_flow_n2(0)
        # V1..V3 öffnen
        self.maxi.set_valve(ADDRESS_V1, True)
        self.maxi.set_valve(ADDRESS_V2, True)
        self.maxi.set_valve(ADDRESS_V3, True)
        # MINI2 manuell voll auf
        self.mini2.set_state_manual()
        self.mini2.set_manual_vp1(255); self.mini2.set_manual_vp2(255)
        time.sleep(2); self.maxi.activate_v4(True)

        start = time.time()
        while True:
            p = self.mini1.pressure
            if p is not None and p <= vacuum_pressure_kpa: break
            if time.time() - start > timeout_s: break
            time.sleep(0.5)

        # zurücksetzen
        self.maxi.set_valve(ADDRESS_V1, False)
        self.maxi.set_valve(ADDRESS_V2, False)
        self.maxi.set_valve(ADDRESS_V3, False)
        self.mini2.set_manual_vp1(0); self.mini2.set_manual_vp2(0); self.mini2.set_state_auto()

    def use_gas(self, n2: float, co2: float, butan: float) -> None:
        # Eingaben 0..100 -> l/min = value/10
        self.maxi.set_flow_n2(n2/10.0); self.maxi.set_flow_co2(co2/10.0); self.maxi.set_flow_butan(butan/10.0)

    def setpoint_pressure(self, kpa: Union[int, float]) -> None:
        self.mini2.set_target_pressure(kpa)

    def activate_vac(self, enable: bool) -> None:
        self.maxi.activate_v4(enable)

    def get_all_status(self, mini1_last_n: Optional[int] = 100) -> Dict[str, Any]:
        self.mini2.request_status(); self.maxi.request_status()
        time.sleep(0.05)
        return {
            "mini1": {
                "p20": self.mini1.p20, "p21": self.mini1.p21,
                "latest_combined_pressure": self.mini1.pressure,
                "recent_series": self.mini1.get_last_n(mini1_last_n),
            },
            "mini2": self.mini2.get_status(),
            "maxi":  self.maxi.get_status(),
        }

    def reset_all_totalisators(self) -> None:
        self.maxi.reset_totalisator_butan()
        self.maxi.reset_totalisator_co2()
        self.maxi.reset_totalisator_n2()

    def close(self) -> None:
        self.mini1.stop(); self.mini2.stop(); self.maxi.stop()

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()

# === Beispiel ===
if __name__ == "__main__":
    # macOS: ersetze durch deine Ports (siehe ls /dev/cu.*)
    MINI1_PORT = "/dev/cu.usbmodem1133101"
    MINI2_PORT = "/dev/cu.usbmodem1133301"
    MAXI_PORT  = "/dev/cu.usbmodem1133201"

    logging.basicConfig(level=logging.INFO)
    driver = VaderDeviceDriver(MINI1_PORT, MINI2_PORT, MAXI_PORT)
    try:
        driver.storage_volume(open_=True)
        driver.use_gas(n2=30, co2=20, butan=10)
        driver.setpoint_pressure(100)
        driver.activate_vac(True)
        driver.vac_all(vacuum_pressure_kpa=2.0)
        print("Status:", driver.get_all_status(mini1_last_n=10))
        driver.reset_all_totalisators()
    finally:
        driver.close()