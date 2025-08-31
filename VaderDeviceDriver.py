import threading, time, re, serial
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
import re
import time
import threading
from typing import Dict, Any, Optional, Tuple, Union

class MINI2:
    """
    Treiber für das MINI2-Druckregelgerät.

    Protokoll (vereinfacht):
      - Status anfordern:  "1; 0; 0"
      - Zieldruck setzen:  "2; 3; <kPa>"

    Erwartete Statusausgabe (Beispiel):
      ---
      leak_sensor: 0
      P (kPa): 0.00
      Setpoint P (kPa) [3]: -10.00
      P21 (kPa): 0 (0)
      P20 (kPa): 0.00 (0)
      Kp: 20.00
      PWM1: 100
      PWM2: 0
    """

    def __init__(self, port: str):
        self.dev = SerialDevice(port, 115200, name="MINI2")
        self.status: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._last_status_time = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        # Initialstatus holen
        self.request_status()
        self._wait_for_recent_status(timeout=0.3)

    # ------------------------- I/O & Threading -------------------------

    def _reader_loop(self):
        while self._running:
            try:
                line = self.dev.readline()
                if not line:
                    time.sleep(0.005)
                    continue
                self._parse_status_line(line.strip())
            except Exception:
                # Robust gegen sporadische I/O-Fehler
                time.sleep(0.01)

    def _send(self, cmd: str):
        """
        Sendet einen Rohbefehl wie '1; 0; 0' oder '2; 3; -10'.
        """
        # SerialDevice.write_cmd erwartet i.d.R. einen String-Befehl.
        # Keine zusätzliche Formatierung/Checksummen notwendig (vereinfachtes Protokoll).
        self.dev.write_cmd(cmd)

    # ------------------------- Parsing -------------------------

    def _parse_status_line(self, line: str):
        """
        Parst einzelne Statuszeilen des vereinfachten Outputs.
        Ignoriert Trenner ('---') und leere Zeilen.
        """
        if not line or line == '---':
            return

        try:
            lower = line.lower()
            with self._lock:
                # Zeitstempel aktualisieren, sobald eine lesbare Zeile kommt
                self._last_status_time = time.time()

                # leak_sensor: <int>
                if lower.startswith("leak_sensor:"):
                    self.status["leak_sensor"] = self._parse_int_after_colon(line)
                    return

                # P (kPa): <float>
                if lower.startswith("p (kpa):"):
                    self.status["pressure_kpa"] = self._parse_float_after_colon(line)
                    return

                # Setpoint P (kPa) [3]: <float>
                if lower.startswith("setpoint p (kpa)"):
                    self.status["setpoint_kpa"] = self._parse_float_after_colon(line)
                    # optional: Index in [] extrahieren (hier immer 3)
                    idx = re.search(r'\[(\d+)\]', line)
                    if idx:
                        self.status["setpoint_index"] = int(idx.group(1))
                    return

                # P21 (kPa): <float_or_int> (<raw_int>)
                if lower.startswith("p21 (kpa):"):
                    cal, raw = self._parse_cal_raw_parentheses(line)
                    self.status["p21_kpa"] = cal
                    if raw is not None:
                        self.status["p21_raw"] = raw
                    return

                # P20 (kPa): <float> (<raw_int>)
                if lower.startswith("p20 (kpa):"):
                    cal, raw = self._parse_cal_raw_parentheses(line)
                    self.status["p20_kpa"] = cal
                    if raw is not None:
                        self.status["p20_raw"] = raw
                    return

                # Kp: <float>
                if lower.startswith("kp:"):
                    self.status["kp"] = self._parse_float_after_colon(line)
                    return

                # PWM1: <int>
                if lower.startswith("pwm1:"):
                    self.status["pwm1"] = self._parse_int_after_colon(line)
                    return

                # PWM2: <int>
                if lower.startswith("pwm2:"):
                    self.status["pwm2"] = self._parse_int_after_colon(line)
                    return

                # Unbekannte Zeilen still ignorieren
        except Exception:
            # Parser bleibt resilient
            pass

    def _parse_float_after_colon(self, line: str) -> Optional[float]:
        if ':' in line:
            try:
                return float(line.split(':', 1)[1].strip())
            except ValueError:
                return None
        return None

    def _parse_int_after_colon(self, line: str) -> Optional[int]:
        if ':' in line:
            try:
                return int(line.split(':', 1)[1].strip())
            except ValueError:
                return None
        return None

    def _parse_cal_raw_parentheses(self, line: str) -> Tuple[Optional[float], Optional[int]]:
        """
        Erwartet Format wie:
          "P21 (kPa): 0 (0)"
          "P20 (kPa): 0.00 (0)"
        Liefert (kalibriert_float, raw_int)
        """
        val = self._parse_text_after_colon(line)
        if not val:
            return None, None
        # Split bei Leerzeichen vor '('
        # Beispiel: "0.00 (0)"  -> cal_str="0.00", raw_str="0"
        m = re.match(r'\s*([+-]?\d+(?:\.\d+)?)\s*\(([-+]?\d+)\)\s*', val)
        if m:
            try:
                cal = float(m.group(1))
            except ValueError:
                cal = None
            try:
                raw = int(m.group(2))
            except ValueError:
                raw = None
            return cal, raw
        # Fallback: nur ein Zahlentoken ohne Klammern
        try:
            return float(val.strip()), None
        except ValueError:
            return None, None

    def _parse_text_after_colon(self, line: str) -> str:
        return line.split(':', 1)[1].strip() if ':' in line else ""

    # ------------------------- Öffentliche API -------------------------

    def set_target_pressure(self, kpa: Union[int, float]):
        """
        Setzt den Zieldruck in kPa gemäß vereinfachtem Protokoll:
          send("2; 3; {kPa}")
        """
        self._send(f"2; 3; {kpa}")

    def request_status(self):
        """
        Fordert eine Statusausgabe an (mehrere Zeilen, beginnend mit '---').
        """
        self._send("1; 0; 0")

    def get_status(self) -> Dict[str, Any]:
        """Gibt eine Kopie des zuletzt geparsten Status zurück."""
        with self._lock:
            return dict(self.status)

    def is_recent(self, max_age_s: float = 0.5) -> bool:
        """True, wenn kürzlich (<= max_age_s) eine Statuszeile empfangen wurde."""
        with self._lock:
            return (time.time() - self._last_status_time) <= max_age_s

    def _wait_for_recent_status(self, timeout: float = 0.5) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_recent(max_age_s=0.3):
                return True
            time.sleep(0.01)
        return False

    def stop(self):
        self._running = False
        self._thread.join(timeout=0.5)
        try:
            self.dev.close()
        except Exception:
            pass

# === MAXI ===

# Erwartet eine vorhandene SerialDevice-Klasse mit:
#   - write_line(str)  -> schreibt eine vollständige Zeile (inkl. '\n' wenn nötig)
#   - readline()       -> liest eine Zeile (ohne '\n')
class MAXI:
    """
    Vereinfachter Treiber für das MAXI-System mit Textprotokoll:

    - Status anfordern:  "1; 0; 0"
    - Werte setzen:      "2; <address>; <value>"
        * MFC-Fluss:     Address 20 (Butan/Button), 30 (CO2), 40 (N2); value in 1/100 l/min (int)
        * Totalisator:   Address 21 (Butan/Button), 31 (CO2), 41 (N2); value in 1/100 l/min (int)
        * Schalter/PWM:  Address 1,2,3,4,8,9,11,12 etc. (0/1 oder numerisch)
    """

    # ID -> Schlüssel (entspricht den [id]-Angaben im Dump)
    _IO_MAP = {
        1:  ("v1_power", "switch"),
        2:  ("v2_power", "switch"),
        3:  ("v3_power", "switch"),
        4:  ("v4_power", "switch"),
        8:  ("fan_in_power", "switch"),
        9:  ("fan_out_power", "switch"),
        11: ("heater_pwm", "numeric"),
        12: ("vs3_power", "switch"),
        # MFC-Setpoints/Totalisatoren (werden zusätzlich in mfc_*.setpoint / .totalisator gespiegelt)
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
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    # ---------- Senden ----------
    def _send(self, cmd: int, address: int, value: int):
        self.dev.write_line(f"{cmd}; {address}; {value}")

    def request_status(self):
        """Fordert einen Status-Dump an (siehe Beispiel: send('1; 0; 0'))."""
        self._send(1, 0, 0)

    # Allgemeine Setter (Schalter, PWM etc.)
    def set_component(self, address: int, value: int):
        """Setzt beliebige Adresse (z.B. 1..4,8,9,11,12)."""
        self._send(2, address, int(value))

    # Ventil-/Komfortfunktionen
    def set_valve(self, valve_id: int, on: bool):
        self.set_component(valve_id, 1 if on else 0)

    def activate_v4(self, enable: bool):
        self.set_valve(4, enable)

    # ---------- MFC: Fluss/Totalisator ----------
    @staticmethod
    def _to_centiliters_per_min(flow_l_per_min: float) -> int:
        # Wert in 1/100 l/min, gerundet
        return int(round(flow_l_per_min * 100.0))

    def set_flow_butan(self, flow_l_per_min: float):
        self._send(2, 20, self._to_centiliters_per_min(flow_l_per_min))

    def set_flow_co2(self, flow_l_per_min: float):
        self._send(2, 30, self._to_centiliters_per_min(flow_l_per_min))

    def set_flow_n2(self, flow_l_per_min: float):
        self._send(2, 40, self._to_centiliters_per_min(flow_l_per_min))

    def set_totalisator_butan(self, value_l_per_min: float = 0.0):
        self._send(2, 21, self._to_centiliters_per_min(value_l_per_min))

    def set_totalisator_co2(self, value_l_per_min: float = 0.0):
        self._send(2, 31, self._to_centiliters_per_min(value_l_per_min))

    def set_totalisator_n2(self, value_l_per_min: float = 0.0):
        self._send(2, 41, self._to_centiliters_per_min(value_l_per_min))

    # ---------- Reader / Parser ----------
    def _reader_loop(self):
        while self._running:
            try:
                line = self.dev.readline()
                if not line:
                    time.sleep(0.002)
                    continue
                self._parse_status_line(line.rstrip("\r\n"))
            except Exception:
                # keep running on parse/IO errors
                pass
            time.sleep(0.001)

    def _touch(self):
        self._last_status_ts = time.time()

    def _set_nested(self, key_path: str, value: Any):
        parts = key_path.split(".")
        with self._lock:
            d = self.status
            for p in parts[:-1]:
                if p not in d or not isinstance(d[p], dict):
                    d[p] = {}
                d = d[p]
            d[parts[-1]] = value
            self._touch()

    def _ensure_mfc(self, name: str):
        with self._lock:
            self.status.setdefault(name, {})
            self._touch()

    def _parse_status_line(self, line: str):
        low = line.lower().strip()

        # Abschnittsmarker -> MFC-Blöcke initialisieren
        if "butan/methan mfc" in low:
            self._ensure_mfc("mfc_butan")
            return
        if "co2 mfc" in low:
            self._ensure_mfc("mfc_co2")
            return
        if "n2/argon mfc" in low:
            self._ensure_mfc("mfc_n2")
            return

        # Einfaches [id]: value-Muster für Aktoren/Setpoints/Totalisatoren
        m_id = re.search(r"\[([0-9]{1,3})\]\s*:\s*([+-]?\d+(?:\.\d+)?)", line)
        if m_id:
            id_num = int(m_id.group(1))
            val_str = m_id.group(2)
            try:
                val_num = float(val_str)
            except ValueError:
                val_num = val_str

            if id_num in self._IO_MAP:
                key, kind = self._IO_MAP[id_num]
                if kind == "switch":
                    # 0/1 als "OFF"/"ON" speichern, zusätzlich numeric unter <key>_raw
                    on = float(val_num) != 0.0
                    self._set_nested(key, "ON" if on else "OFF")
                    self._set_nested(f"{key}_raw", int(float(val_num)))
                elif kind == "numeric":
                    self._set_nested(key, int(float(val_num)))
                elif kind == "float_100":
                    # Gerät liefert in l/min (Anzeige). Falls das Gerät die Anzeige in l/min zeigt,
                    # nehmen wir den gelesenen Wert als float; Sendewege nutzen *_to_centiliters_per_min.
                    self._set_nested(key, float(val_num))
                # MFC-Spiegelung für Setpoints/Totalisatoren
                if ".setpoint" in key:
                    self._set_nested(key.split(".")[0] + ".setpoint", float(val_num))
                if ".totalisator" in key:
                    self._set_nested(key.split(".")[0] + ".totalisator", float(val_num))
            return

        # Sensor-Zeilen wie: "pt100_storage: 24.0 (167)" oder "p11: 167 kPa (36)"
        m_sens = re.search(r"^([a-z0-9_]+)\s*:\s*([+-]?\d+(?:\.\d+)?)\s*(kpa|°c|c)?\s*\((\d+)\)\s*$", low)
        if m_sens:
            name = m_sens.group(1)
            value = float(m_sens.group(2))
            # Einheit nicht zwingend genutzt; Klammerwert (ID/roh) mit ablegen
            self._set_nested(name, value)
            self._set_nested(f"{name}_code", int(m_sens.group(4)))
            return

        # MFC-Detailzeilen innerhalb eines Blocks
        # unit: ln/min
        m_unit = re.search(r"^\s*unit:\s*(.+)$", low)
        if m_unit:
            unit = m_unit.group(1).strip()
            # letzte initialisierte MFC bestimmen (pragmatisch über vorhandene keys)
            target = self._last_mfc_key()
            if target:
                self._set_nested(f"{target}.unit", unit)
            return

        # flow Butan / flow CO2 / flow N2:
        m_flow_named = re.search(r"^\s*flow\s+(butan|co2|n2)\s*:\s*([+-]?\d+(?:\.\d+)?)", low)
        if m_flow_named:
            gas = m_flow_named.group(1)
            val = float(m_flow_named.group(2))
            mfc = {"butan": "mfc_butan", "co2": "mfc_co2", "n2": "mfc_n2"}[gas]
            self._set_nested(f"{mfc}.flow", val)
            return

        # flow (set point) [xx]: 0.00  -> bereits durch [id] oben erfasst

        # temperature: 24.30
        m_temp = re.search(r"^\s*temperature\s*:\s*([+-]?\d+(?:\.\d+)?)", low)
        if m_temp:
            target = self._last_mfc_key()
            if target:
                self._set_nested(f"{target}.temperature", float(m_temp.group(1)))
            return

        # totalisator [xx]:  -> bereits durch [id] oben erfasst

    def _last_mfc_key(self) -> Optional[str]:
        # heuristisch: nimm den zuletzt vorhandenen MFC in Status (Reihenfolge Butan -> CO2 -> N2)
        with self._lock:
            for k in ("mfc_n2", "mfc_co2", "mfc_butan"):
                if k in self.status:
                    return k
        return None

    # ---------- Utilities ----------
    def wait_for_fresh_status(self, timeout: float = 1.0) -> bool:
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
            # flache Kopie
            return dict(self.status)

    def pretty_print(self):
        st = self.get_status()
        def g(path, default=""):
            parts = path.split(".")
            d = st
            for p in parts:
                if not isinstance(d, dict) or p not in d:
                    return default
                d = d[p]
            return d

        lines = []
        # Aktoren
        for key in ("v1_power","v2_power","v3_power","v4_power","fan_in_power","fan_out_power","heater_pwm","vs3_power"):
            if key in st: lines.append(f"{key}: {st[key]}")
        # Sensoren
        for s in ("leak_sensor","pt100_storage","pt100_vacuum","pt100_housing","p11","p31"):
            if s in st: lines.append(f"{s}: {st[s]}")
        # MFCs
        for mfc in ("mfc_butan","mfc_co2","mfc_n2"):
            if mfc in st:
                lines.append(mfc.upper() + ":")
                for k in ("unit","flow","setpoint","temperature","totalisator"):
                    val = g(f"{mfc}.{k}", None)
                    if val is not None and val != "":
                        lines.append(f"  {k}: {val}")
        print("\n".join(lines))

    def stop(self):
        self._running = False
        try:
            self._thread.join(timeout=1.0)
        except:
            pass
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