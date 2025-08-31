import threading
import time
import re
import serial
from typing import Optional, Tuple, Dict, Any
from collections import deque
from datetime import datetime, timezone
from typing import Optional, Tuple, List


# === Commands ===
NO_COMMAND = 0
SEND_STATUS = 1
SET = 2

# === MAXI Addresses ===

ADDRESS_V1 = 1
ADDRESS_V2 = 2
ADDRESS_V3 = 3
ADDRESS_V4 = 4
ADDRESS_FAN_IN = 8
ADDRESS_FAN_OUT = 9
ADDRESS_TEMP = 10
ADDRESS_HEATER = 11
ADDRESS_VS3 = 12
ADDRESS_ERROR = 13

ADDRESS_MFC_BUTAN_FLOW = 20
ADDRESS_MFC_BUTAN_TOTAL = 21
ADDRESS_MFC_CO2_FLOW = 30
ADDRESS_MFC_CO2_TOTAL = 31
ADDRESS_MFC_N2_FLOW = 40
ADDRESS_MFC_N2_TOTAL = 41

# === MINI2 Addresses ===
ADR_TARGET_P = 3
ADR_MAXSTEP = 6
ADR_ERR = 13
ADR_STATE = 14
ADR_MANUAL_VP1 = 7
ADR_MANUAL_VP2 = 8


# === Helper ===
def make_cmd(command: int, address: int, value: Any) -> str:
    r =  f"{command};{address};{value}\n"
    return r

# === Serial wrapper ===
class SerialDevice:
    def __init__(self, port: str, baudrate: int = 115200, name: str = ""):
        self.name = name
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self._lock = threading.Lock()

    def write_cmd(self, cmd: str):
        with self._lock:
            self.ser.write(cmd.encode())

    def readline(self) -> Optional[str]:
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                return line
        except Exception:
            return None
        return None

    def close(self):
        self.ser.close()

# === MINI1 ===

class MINI1:
    def __init__(self, port: str, max_samples: int = 300_000):
        """
        :param port: serieller Port
        :param max_samples: maximale Anzahl gespeicherter Messwerte (default 300.000)
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.dev = SerialDevice(port, 115200, name="MINI1")
        self._max_samples = max_samples
        self._lock = threading.Lock()

        # Aktuelle Rohwerte
        self.p20: Optional[float] = None
        self.p21: Optional[float] = None

        # Zeitreihe: deque von (timestamp: datetime, pressure: float), begrenzt auf max_samples
        self._series: deque[Tuple[datetime, float]] = deque(maxlen=self._max_samples)

        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _compute_pressure(self, p20: float, p21: float) -> float:
        """Berechnet den verwendeten Druck nach Regel."""
        if p21 < 150 and abs(p21 - p20) < 5:
            return p20
        else:
            return p21

    def _reader_loop(self):
        while self._running.is_set():
            try:
                raw = self.dev.readline()
                if not raw:
                    time.sleep(0.01)
                    continue

                # Decodieren, falls bytes
                if isinstance(raw, bytes):
                    try:
                        line = raw.decode(errors="ignore").strip()
                    except Exception:
                        line = raw.decode("utf-8", errors="ignore").strip()
                else:
                    line = str(raw).strip()

                parts = line.split(',')
                if len(parts) < 2:
                    self.logger.debug("Ungültige Zeile (nicht genug Teile): %r", line)
                    continue

                try:
                    p20_val = float(parts[0])
                    p21_val = float(parts[1])
                except ValueError:
                    self.logger.debug("Konvertierungsfehler in Zeile: %r", line)
                    continue

                timestamp = datetime.now(timezone.utc)
                pressure = self._compute_pressure(p20_val, p21_val)

                with self._lock:
                    self.p20 = p20_val
                    self.p21 = p21_val
                    self._series.append((timestamp, pressure))  # automatic drop oldest if over capacity

            except Exception as e:
                self.logger.exception("Fehler im Reader-Loop: %s", e)
                time.sleep(0.1)

    def get_last_n(self, n: Optional[int] = None) -> List[Tuple[datetime, float]]:
        """
        Liefert die letzten n Samples. Wenn n nicht angegeben ist, alle aktuell gespeicherten (bis max_samples).
        """
        with self._lock:
            if n is None or n >= len(self._series):
                return list(self._series)
            else:
                # letzte n Einträge
                return list(self._series)[-n:]

    @property
    def pressure(self) -> Optional[float]:
        """Zuletzt berechneter Druckwert."""
        with self._lock:
            if self._series:
                return self._series[-1][1]
            return None

    def set_max_samples(self, max_samples: int):
        """Ändert die Kapazität des Puffers. Bestehende Werte werden in neuen deque übernommen (älteste fallen ggf. weg)."""
        with self._lock:
            new_series = deque(self._series, maxlen=max_samples)
            self._series = new_series
            self._max_samples = max_samples

    def stop(self):
        """Beendet den Hintergrundthread sauber."""
        self._running.clear()
        self._thread.join(timeout=1)
        self.dev.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

# === MINI2  ===
class MINI2:
    def __init__(self, port: str):
        """
        Treiber für das Druckregelungsgerät über serielle Schnittstelle.
        Startet Reader-Thread und holt initialen Status.
        """
        self.dev = SerialDevice(port, 115200, name="MINI2")
        self.status: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._last_status_time = 0.0  # Zeitstempel letzte gültige Statuszeile
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._running = True
        self._thread.start()
        # Initialen Status anfordern und kurz warten
        self.request_status()
        self._wait_for_recent_status(timeout=0.2)

    def _reader_loop(self):
        while self._running:
            try:
                line = self.dev.readline()
                if line:
                    self._parse_status_line(line.strip())
                else:
                    # kein Input, leicht schlafen
                    time.sleep(0.005)
            except Exception:
                # robust gegen I/O-Fehler
                time.sleep(0.01)

    def _parse_status_line(self, line: str):
        """
        Parst eine einzelne Zeile der Statusausgabe.
        Erwartet Format wie:
          "Pressure P21 (cal) [1002]: 118.3 [raw 345]"
          "State [1000]: automatic"
        """
        try:
            lower = line.lower()

            with self._lock:
                # Aktualisiere Zeitstempel, sobald eine valide Zeile kommt (vereinfachte Annahme)
                self._last_status_time = time.time()

                # State
                if "state" in lower and "automatic" in lower:
                    self.status["state"] = "automatic"
                elif "state" in lower and "manual" in lower:
                    self.status["state"] = "manual"
                elif "target pressure" in lower:
                    # z.B. "Target Pressure (kPa) [1006]: 50"
                    val = float(self._extract_value_after_colon(line))
                    self.status["target_pressure_kpa"] = val
                elif "max step" in lower:
                    val = int(self._extract_value_after_colon(line))
                    self.status["max_step"] = val
                elif "error flag" in lower:
                    self.status["error"] = 'true' in line.lower()
                elif "pressure p21 (cal)" in lower:
                    cal, raw = self._parse_cal_raw(line)
                    self.status.setdefault("pressure_p21", {})["calibrated"] = cal
                    if raw is not None:
                        self.status.setdefault("pressure_p21", {})["raw"] = raw
                elif "pressure p20 (cal)" in lower:
                    cal, raw = self._parse_cal_raw(line)
                    self.status.setdefault("pressure_p20", {})["calibrated"] = cal
                    if raw is not None:
                        self.status.setdefault("pressure_p20", {})["raw"] = raw
                elif "pwm1 (vp1)" in lower:
                    val = int(self._extract_value_after_colon(line))
                    self.status["pwm_vp1"] = val
                elif "pwm2 (vp2)" in lower:
                    val = int(self._extract_value_after_colon(line))
                    self.status["pwm_vp2"] = val
                elif "vs2 state" in lower:
                    # OPEN / CLOSE
                    if "open" in lower:
                        self.status["vs2_state"] = "OPEN"
                    elif "close" in lower:
                        self.status["vs2_state"] = "CLOSE"
                elif "manual vp1 target" in lower:
                    val = int(self._extract_value_after_colon(line))
                    self.status["manual_vp1_target"] = val
                elif "manual vp2 target" in lower:
                    val = int(self._extract_value_after_colon(line))
                    self.status["manual_vp2_target"] = val
                # sonst: ignorieren
        except Exception:
            # resilient bleiben; Logs könnten hier ergänzt werden
            pass

    def _extract_value_after_colon(self, line: str) -> str:
        """Hilfsfunktion: extrahiert Text nach ']:', trimmt."""
        if ']: ' in line:
            return line.split(']:', 1)[-1].strip()
        elif ':' in line:
            return line.split(':', 1)[-1].strip()
        return ""

    def _parse_cal_raw(self, line: str) -> (Optional[float], Optional[int]):
        """
        Extrahiert kalibrierte und rohe Werte aus z.B.
        "Pressure P21 (cal) [1002]: 123.45 [raw 678]"
        """
        # Kalibrierter Wert direkt nach ']:'
        val_part = self._extract_value_after_colon(line)
        cal = None
        raw = None
        # kalibrierter Wert ist das erste Token (float)
        tokens = val_part.split()
        if tokens:
            try:
                cal = float(tokens[0])
            except ValueError:
                cal = None
        # roher Wert durch regex suchen: [raw 345]
        m = re.search(r'\[raw\s+(\d+)\]', line, re.IGNORECASE)
        if m:
            try:
                raw = int(m.group(1))
            except ValueError:
                raw = None
        return cal, raw

    def set_target_pressure(self, kpa: int):
        self.dev.write_cmd(make_cmd(SET, ADR_TARGET_P, kpa))

    def set_max_step(self, step: int):
        self.dev.write_cmd(make_cmd(SET, ADR_MAXSTEP, step))

    def set_state_manual(self):
        self.dev.write_cmd(make_cmd(SET, ADR_STATE, 1))

    def set_state_auto(self):
        self.dev.write_cmd(make_cmd(SET, ADR_STATE, 0))

    def set_manual_vp1(self, val: int):
        self.dev.write_cmd(make_cmd(SET, ADR_MANUAL_VP1, val))

    def set_manual_vp2(self, val: int):
        self.dev.write_cmd(make_cmd(SET, ADR_MANUAL_VP2, val))

    def request_status(self):
        """Schickt Statusanforderung; der Reader-Thread füllt self.status nach."""
        self.dev.write_cmd(make_cmd(SEND_STATUS, 0, 0))

    def _wait_for_recent_status(self, timeout: float = 0.5) -> bool:
        """Wartet bis eine Statuszeile innerhalb `timeout` Sekunden empfangen wurde."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if time.time() - self._last_status_time < 0.2:
                    return True
            time.sleep(0.01)
        return False

    def wait_for_state(self, desired: str, timeout: float = 1.0) -> bool:
        """
        Wartet darauf, dass das Gerät den gewünschten State meldet ("automatic" oder "manual").
        Gibt True zurück, wenn erreicht, sonst False.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self.status.get("state") == desired:
                    return True
            # erneute Statusanforderung als Recovery
            self.request_status()
            time.sleep(0.05)
        return False

    def is_error(self) -> bool:
        with self._lock:
            return bool(self.status.get("error", False))

    def get_state(self) -> Optional[str]:
        with self._lock:
            return self.status.get("state")

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.status)  # Kopie

    def stop(self):
        self._running = False
        self._thread.join()
        self.dev.close()

# === MAXI ===

class MAXI:
    MFC_SLAVE_BUTAN = 1
    MFC_SLAVE_CO2 = 2
    MFC_SLAVE_N2 = 3

    # ID-to-key mapping für Parser
    _ID_MAP = {
        "3001": ("V1", "switch"),
        "3002": ("V2", "switch"),
        "3003": ("V3", "switch"),
        "3004": ("V4", "switch"),
        "3008": ("fan_in", "switch"),
        "3009": ("fan_out", "switch"),
        "3011": ("heater_pwm", "numeric"),
        "3012": ("VS3", "switch"),
        "3020": ("leak_sensor", "numeric"),
        "3021": ("pt100_storage_raw", "numeric"),
        "3022": ("pt100_vacuum_raw", "numeric"),
        "3023": ("pt100_housing_raw", "numeric"),
        "3024": ("p11_raw", "numeric"),
        "3025": ("p30_raw", "numeric"),
        "3026": ("p31_raw", "numeric"),
        "3030": ("mfc_butan.unit", "string"),
        "3031": ("mfc_butan.flow", "float"),
        "3032": ("mfc_butan.setpoint", "float"),
        "3033": ("mfc_butan.temperature", "float"),
        "3034": ("mfc_butan.totalisator", "float"),
        "3040": ("mfc_co2.unit", "string"),
        "3041": ("mfc_co2.flow", "float"),
        "3042": ("mfc_co2.setpoint", "float"),
        "3043": ("mfc_co2.temperature", "float"),
        "3044": ("mfc_co2.totalisator", "float"),
        "3050": ("mfc_n2.unit", "string"),
        "3051": ("mfc_n2.flow", "float"),
        "3052": ("mfc_n2.setpoint", "float"),
        "3053": ("mfc_n2.temperature", "float"),
        "3054": ("mfc_n2.totalisator", "float"),
    }

    def __init__(self, port: str):
        self.dev = SerialDevice(port, 115200, name="MAXI")  # erwartet vorhandene SerialDevice-Klasse
        self.status: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._last_status_ts = 0.0
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._running = True
        self._thread.start()

    def _reader_loop(self):
        # Naiver Parser, liest Zeilen und aktualisiert Status
        while self._running:
            try:
                line = self.dev.readline()
                if not line:
                    time.sleep(0.002)
                    continue
                self._parse_status_line(line.strip())
            except Exception:
                # still alive; swallow parse errors
                pass
            time.sleep(0.001)

    def _set_nested(self, key_path: str, value: Any):
        parts = key_path.split(".")
        with self._lock:
            d = self.status
            for p in parts[:-1]:
                if p not in d or not isinstance(d[p], dict):
                    d[p] = {}
                d = d[p]
            d[parts[-1]] = value
            self._last_status_ts = time.time()

    def _parse_status_line(self, line: str):
        low = line.lower()

        # Allgemeine Schalter (ON/OFF bzw 0/1)
        for id_str, (key, kind) in self._ID_MAP.items():
            if f"[{id_str}]" in line:
                if kind == "switch":
                    # suche ON / OFF oder 1 / 0
                    if re.search(r"\b(on|1)\b", low):
                        self._set_nested(key, "ON")
                    elif re.search(r"\b(off|0)\b", low):
                        self._set_nested(key, "OFF")
                    else:
                        # fallback: alles nach ':' nehmen und interpretieren
                        raw = line.split("]:")[-1].strip()
                        self._set_nested(key, raw)
                elif kind == "numeric":
                    # rohe Zahl
                    m = re.search(rf"\[{id_str}\]:\s*([0-9.+-eE]+)", line)
                    if m:
                        val = m.group(1)
                        try:
                            if "." in val or "e" in val.lower():
                                self._set_nested(key, float(val))
                            else:
                                self._set_nested(key, int(val))
                        except:
                            self._set_nested(key, val)
                    else:
                        # alternative: letzte Zahl
                        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
                        if nums:
                            try:
                                self._set_nested(key, float(nums[-1]))
                            except:
                                self._set_nested(key, nums[-1])
                elif kind == "float":
                    # Flow, Temperature, Totalisator etc.
                    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
                    if nums:
                        try:
                            self._set_nested(key, float(nums[0]))
                        except:
                            self._set_nested(key, nums[0])
                elif kind == "string":
                    # Einheit, ASCII-String: alles nach ']:'
                    val = line.split("]:")[-1].strip()
                    self._set_nested(key, val)
        # Zusätzliche heuristiken: wenn MFC-Header erkannt, initialisieren
        if "butan/methan mfc" in low:
            with self._lock:
                self.status.setdefault("mfc_butan", {})
        if "co2 mfc" in low:
            with self._lock:
                self.status.setdefault("mfc_co2", {})
        if "n2/argon mfc" in low:
            with self._lock:
                self.status.setdefault("mfc_n2", {})

    def set_valve(self, address: int, value: int):
        self.dev.write_cmd(make_cmd(SET, address, value))

    def set_flow_butan(self, flow: float):
        self.dev.write_cmd(make_cmd(SET, ADDRESS_MFC_BUTAN_FLOW, int(flow)))

    def reset_totalisator_butan(self):
        self.dev.write_cmd(make_cmd(SET, ADDRESS_MFC_BUTAN_TOTAL, 0))

    def set_flow_co2(self, flow: float):
        self.dev.write_cmd(make_cmd(SET, ADDRESS_MFC_CO2_FLOW, int(flow)))

    def reset_totalisator_co2(self):
        self.dev.write_cmd(make_cmd(SET, ADDRESS_MFC_CO2_TOTAL, 0))

    def set_flow_n2(self, flow: float):
        self.dev.write_cmd(make_cmd(SET, ADDRESS_MFC_N2_FLOW, int(flow)))

    def reset_totalisator_n2(self):
        self.dev.write_cmd(make_cmd(SET, ADDRESS_MFC_N2_TOTAL, 0))

    def activate_v4(self, enable: bool):
        self.set_valve(ADDRESS_V4, 1 if enable else 0)

    def request_status(self):
        self.dev.write_cmd(make_cmd(SEND_STATUS, 0, 0))

    def wait_for_fresh_status(self, timeout: float = 1.0) -> bool:
        """Fordere Status an und warte bis er aktualisiert wurde."""
        before = self._last_status_ts
        self.request_status()
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._last_status_ts > before:
                    return True
            time.sleep(0.01)
        return False

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.status)

    def pretty_print(self):
        st = self.get_status()
        lines = []
        # Ventile
        for v in ["V1", "V2", "V3", "V4", "VS3"]:
            if v in st:
                lines.append(f"{v}: {st[v]}")
        for f in ["fan_in", "fan_out"]:
            if f in st:
                lines.append(f"{f}: {st[f]}")
        if "heater_pwm" in st:
            lines.append(f"Heater PWM: {st['heater_pwm']}")
        # Sensoren
        for s in ["leak_sensor", "pt100_storage_raw", "pt100_vacuum_raw", "pt100_housing_raw", "p11_raw", "p30_raw", "p31_raw"]:
            if s in st:
                lines.append(f"{s}: {st[s]}")
        # MFCs
        for mfc in ["mfc_butan", "mfc_co2", "mfc_n2"]:
            if mfc in st:
                sub = st[mfc]
                unit = sub.get("unit", "").strip()
                prefix = mfc.replace("mfc_", "").upper()
                lines.append(f"{prefix} MFC:")
                if unit:
                    lines.append(f"  Einheit: {unit}")
                for k in ["flow", "setpoint", "temperature", "totalisator"]:
                    if k in sub:
                        val = sub[k]
                        lines.append(f"  {k}: {val} {unit if unit else ''}".strip())
        print("\n".join(lines))

    def stop(self):
        self._running = False
        self._thread.join()
        try:
            self.dev.close()
        except:
            pass

# === High-level Gerätetreiber ===
class VaderDeviceDriver:
    def __init__(self, mini1_port: str, mini2_port: str, maxi_port: str, mini1_max_samples: int = 300_000):
        self.mini1 = MINI1(mini1_port, max_samples=mini1_max_samples)
        self.mini2 = MINI2(mini2_port)
        self.maxi = MAXI(maxi_port)

    def storage_volume(self, open: bool):
        val = 1 if open else 0
        self.maxi.set_valve(ADDRESS_V1, val)
        self.maxi.set_valve(ADDRESS_V2, val)

    def vac_all(self, vacuum_pressure_kpa: float, timeout_s: int = 120):
        """
        Evakuiert bis der kombinierte Druck (nach Regel aus P20/P21) <= vacuum_pressure_kpa ist
        oder das Timeout erreicht ist.
        """
        # Alle MFC-Flows auf 0
        self.maxi.set_flow_butan(0)
        self.maxi.set_flow_co2(0)
        self.maxi.set_flow_n2(0)
        # V1,V2,V3 öffnen
        self.maxi.set_valve(ADDRESS_V1, 1)
        self.maxi.set_valve(ADDRESS_V2, 1)
        self.maxi.set_valve(ADDRESS_V3, 1)
        # MINI2 Proportionalventile voll im manual mode
        self.mini2.set_state_manual()
        self.mini2.set_manual_vp1(255)
        self.mini2.set_manual_vp2(255)

        time.sleep(2)
        self.maxi.activate_v4(True)

        start = time.time()
        while True:
            current_pressure = self.mini1.pressure  # bereits nach der Regel berechnet
            if current_pressure is not None and current_pressure <= vacuum_pressure_kpa:
                break
            if time.time() - start > timeout_s:
                break
            time.sleep(0.5)

        # Schließen / zurücksetzen
        self.maxi.set_valve(ADDRESS_V1, 0)
        self.maxi.set_valve(ADDRESS_V2, 0)
        self.maxi.set_valve(ADDRESS_V3, 0)
        self.mini2.set_manual_vp1(0)
        self.mini2.set_manual_vp2(0)
        self.mini2.set_state_auto()

    def use_gas(self, n2: float, co2: float, butan: float):
        # Eingaben 0..100, Flows = value/10
        self.maxi.set_flow_n2(n2 / 10.0)
        self.maxi.set_flow_co2(co2 / 10.0)
        self.maxi.set_flow_butan(butan / 10.0)

    def setpoint_pressure(self, kpa: int):
        self.mini2.set_target_pressure(kpa)

    def activate_vac(self, enable: bool):
        self.maxi.activate_v4(enable)

    def get_all_status(self, mini1_last_n: Optional[int] = 100) -> Dict[str, Any]:
        """
        Liefert Status aller Geräte. Für MINI1 wird zusätzlich die kombinierte Druckzeitreihe
        der letzten `mini1_last_n` Samples mit ausgegeben.
        """
        self.mini2.request_status()
        self.maxi.request_status()
        time.sleep(0.05)  # kurz warten, damit Statusantworten verfügbar sind

        mini1_series = self.mini1.get_last_n(mini1_last_n)
        return {
            "mini1": {
                "p20": self.mini1.p20,
                "p21": self.mini1.p21,
                "latest_combined_pressure": self.mini1.pressure,
                "recent_series": mini1_series,  # Liste von (timestamp, pressure)
            },
            "mini2": self.mini2.get_status(),
            "maxi": self.maxi.get_status()
        }

    def reset_all_totalisators(self):
        self.maxi.reset_totalisator_butan()
        self.maxi.reset_totalisator_co2()
        self.maxi.reset_totalisator_n2()

    def close(self):
        self.mini1.stop()
        self.mini2.stop()
        self.maxi.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# === Beispiel ===
if __name__ == "__main__":
    driver = VaderDeviceDriver("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2")
    try:
        driver.storage_volume(open=True)
        driver.use_gas(n2=30, co2=20, butan=10)
        driver.setpoint_pressure(100)
        driver.activate_vac(True)
        driver.vac_all(vacuum_pressure_kpa=2.0)
        status = driver.get_all_status()
        print("Status nach Ablauf:", status)
        driver.reset_all_totalisators()
    finally:
        driver.close()