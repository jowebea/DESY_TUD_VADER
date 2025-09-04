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
                time.sleep(0.0001)
                continue
            pressure = float(line)
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

import logging

# ====================== MAXI ======================
class MAXI:
    """
    Protokoll:
      - Status anfordern: "1; 0; 0"
      - Setzen:           "2; <address>; <value>"

    Garantie:
      - Vor jedem Senden wird der serielle Eingangspuffer geleert.
      - Alle sendenden/empfangenden Methoden warten auf eine frische Statusantwort
        und liefern True/False (Erfolg).
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

    def __init__(self, port: str, status_timeout: float = 1.0):
        self.dev = SerialDevice(port, 115200, name="MAXI")
        self.status: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._last_status_ts = 0.0
        self._running = True
        self._current_mfc: Optional[str] = None
        self._status_timeout_default = float(status_timeout)

        threading.Thread(target=self._reader_loop, daemon=True).start()

        logging.info("MAXI initialisiert auf Port %s", port)
        time.sleep(0.5)
        # Direkt initialen Status anfordern
        if self.request_status(timeout=self._status_timeout_default):
            logging.info("Initialer Status erfolgreich empfangen.")
        else:
            logging.warning("Kein initialer Status empfangen!")

    # ---------- Low-Level Hilfen ----------

    def _touch(self) -> None:
        self._last_status_ts = time.time()

    def _flush_input(self, drain_time: float = 0.05) -> None:
        logging.debug("Flushing input buffer ...")
        try:
            with self.dev._lock:
                try:
                    self.dev.ser.reset_input_buffer()
                    logging.debug("Input buffer mit reset_input_buffer() geleert.")
                    return
                except Exception:
                    pass
        except Exception:
            pass

        # Fallback
        t_end = time.time() + drain_time
        flushed = 0
        while time.time() < t_end:
            line = self.dev.readline()
            if line is None:
                break
            flushed += 1
        if flushed > 0:
            logging.debug("Fallback flush: %d alte Zeilen verworfen.", flushed)

    def _wait_for_status_after(self, before_ts: float, timeout: float) -> bool:
        logging.debug("Warte auf frischen Status (timeout=%.2fs)...", timeout)
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._last_status_ts > before_ts:
                    logging.debug("Frischer Status empfangen (ts=%.3f).", self._last_status_ts)
                    return True
            time.sleep(0.01)
        logging.warning("Timeout: Kein frischer Status innerhalb %.2fs.", timeout)
        return False

    def _send_only(self, cmd: int, address: int, value: Union[int, float]) -> None:
        msg = f"{cmd}; {address}; {value}"
        logging.debug("Sende: %s", msg)
        self.dev.write_line(msg)

    def _send_and_wait_status(self, cmd: int, address: int, value: Union[int, float], timeout: Optional[float] = None) -> bool:
        if timeout is None:
            timeout = self._status_timeout_default

        self._flush_input()

        before = self._last_status_ts
        self._send_only(cmd, address, value)

        if cmd != 1:
            time.sleep(0.01)
            self._send_only(1, 0, 0)  # Status anfordern

        ok = self._wait_for_status_after(before, timeout)
        if ok:
            logging.info("Befehl erfolgreich bestätigt: %s; %d; %s", cmd, address, value)
        else:
            logging.warning("Befehl NICHT bestätigt: %s; %d; %s", cmd, address, value)
        return ok

    def _set_nested(self, dotted_key: str, value: Any) -> None:
        """Schreibt einen Wert in self.status anhand eines dotted keys, z.B. 'mfc_co2.setpoint'."""
        with self._lock:
            node = self.status
            parts = dotted_key.split(".")
            for p in parts[:-1]:
                node = node.setdefault(p, {})
            node[parts[-1]] = value
            self._touch()  # markiere, dass frischer Status vorliegt

    def _ensure_mfc(self, name: str) -> None:
        """Stellt sicher, dass ein MFC-Unterknoten existiert."""
        with self._lock:
            self.status.setdefault(name, {})
            # typische Felder anlegen (optional)
            self.status[name].setdefault("setpoint", None)
            self.status[name].setdefault("totalisator", None)

    def _set_current_mfc(self, name: Optional[str]) -> None:
        with self._lock:
            self._current_mfc = name

    def _get_current_mfc(self) -> Optional[str]:
        with self._lock:
            return self._current_mfc
        
    # ---------- Öffentliche Sende-/Empfangs-APIs ----------

    def request_status(self, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(1, 0, 0, timeout=timeout)

    def set_component(self, address: int, value: int, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(2, address, int(value), timeout=timeout)

    def set_valve(self, valve_id: int, on: bool, timeout: Optional[float] = None) -> bool:
        return self.set_component(valve_id, 1 if on else 0, timeout=timeout)

    def activate_v4(self, enable: bool, timeout: Optional[float] = None) -> bool:
        return self.set_valve(4, enable, timeout=timeout)

    # --- MFC ---
    @staticmethod
    def set_flow_butan(self, flow: float, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(2, 20, flow, timeout=timeout)

    def set_flow_co2(self, flow: float, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(2, 30, flow, timeout=timeout)

    def set_flow_n2(self, flow: float, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(2, 40, flow, timeout=timeout)

    def set_totalisator_butan(self, value: float = 0.0, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(2, 21, value, timeout=timeout)

    def set_totalisator_co2(self, value: float = 0.0, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(2, 31, value, timeout=timeout)

    def set_totalisator_n2(self, value: float = 0.0, timeout: Optional[float] = None) -> bool:
        return self._send_and_wait_status(2, 41, value, timeout=timeout)

    def reset_totalisator_butan(self, timeout: Optional[float] = None) -> bool:
        return self.set_totalisator_butan(0.0, timeout=timeout)

    def reset_totalisator_co2(self, timeout: Optional[float] = None) -> bool:
        return self.set_totalisator_co2(0.0, timeout=timeout)

    def reset_totalisator_n2(self, timeout: Optional[float] = None) -> bool:
        return self.set_totalisator_n2(0.0, timeout=timeout)

    # ---------- Reader / Parser ----------

    def _reader_loop(self) -> None:
        logging.debug("Reader-Thread gestartet.")
        while self._running:
            line = self.dev.readline()
            if not line:
                time.sleep(0.002)
                continue
            logging.debug("Empfangen: %s", line)
            self._parse_status_line(line)

    # ... (Parser bleibt wie vorher, nur ggf. Logging ergänzen) ...

    # ---------- Getter / Shutdown ----------

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.status)

    def stop(self) -> None:
        logging.info("MAXI stop() aufgerufen.")
        self._running = False
        time.sleep(0.05)
        self.dev.close()
    
    def _parse_status_line(self, line: str) -> None:
        low = line.lower().strip()

        # Abschnittsüberschriften der MFCs
        if "butan/methan mfc" in low:
            self._ensure_mfc("mfc_butan")
            self._set_current_mfc("mfc_butan")
            return
        if "co2 mfc" in low:
            self._ensure_mfc("mfc_co2")
            self._set_current_mfc("mfc_co2")
            return
        if "n2/argon mfc" in low:
            self._ensure_mfc("mfc_n2")
            self._set_current_mfc("mfc_n2")
            return

        # Abschnittsende optional zurücksetzen
        if low.startswith("------- mfcs -------") or low.startswith("---"):
            self._set_current_mfc(None)

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
            target = self._get_current_mfc()
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
            target = self._get_current_mfc()
            if target: self._set_nested(f"{target}.temperature", float(m_temp.group(1)))
            return



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
        self.mini2.set_target_pressure(0)
        self.maxi.activate_v4(True)

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
        self.mini2.set_target_pressure(0)
        

    def use_gas(self, n2: int, co2: int, butan: int) -> None:
        # Eingaben 0..10 -> l/min 
        self.maxi.set_flow_n2(flow=n2); self.maxi.set_flow_co2(flow=co2); self.maxi.set_flow_butan(flow=butan)

    def setpoint_pressure(self, kpa: Union[int, float]) -> None:
        self.mini2.set_target_pressure(kpa)

    def activate_vac(self, enable: bool) -> None:
        self.maxi.activate_v4(enable)

    def get_all_status(self, mini1_last_n: Optional[int] = 100) -> Dict[str, Any]:
        # aktiv Status triggern + auf frische Daten warten
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
        self.maxi.reset_totalisator_butan()
        self.maxi.reset_totalisator_co2()
        self.maxi.reset_totalisator_n2()

    def close(self) -> None:
        self.mini1.stop(); self.mini2.stop(); self.maxi.stop()

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()
