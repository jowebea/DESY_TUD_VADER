#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
import logging
from collections import deque
from typing import Optional, Dict, Any, List

# === pyTango ===
from tango import DevState, AttrWriteType, DevLong, AttrDataFormat
from tango.server import Device, attribute, command, run, device_property

# === Treiber ===
from VaderDeviceDriver import VaderDeviceDriver

# ===== Helpers for robust MAXI parsing =====
def _coerce_float(v, default=0.0):
    try:
        if v is None:
            return float(default)
        if isinstance(v, (int, float)):
            return float(v)
        # strings like "0.7", "0,7"
        s = str(v).strip().replace(",", ".")
        return float(s)
    except Exception:
        return float(default)

_GAS_ALIASES = {
    "butan": ("butan", "but", "butane", "c4", "c4h10"),
    "co2":   ("co2", "CO2", "carbon_dioxide"),
    "n2":    ("n2", "N2", "nitrogen"),
}

_MFC_KEYS = {
    "flow": ("flow", "q", "q_nl_min", "flow_nl_min", "flow_sccm"),
    "total": ("total", "sum", "integral", "accum"),
    "setpoint": ("setpoint", "sp", "target", "set"),
}

def _extract_mfc_gas_block(mfc_dict: dict, gas: str) -> dict:
    """Return the most plausible sub-dict for a gas, considering aliases."""
    if not isinstance(mfc_dict, dict):
        return {}
    # Exact
    if gas in mfc_dict and isinstance(mfc_dict[gas], dict):
        return mfc_dict[gas]
    # Aliases
    for alias in _GAS_ALIASES.get(gas, (gas,)):
        block = mfc_dict.get(alias)
        if isinstance(block, dict):
            return block
    # Fallback empty
    return {}

def _get_first_key(d: dict, keys: tuple):
    for k in keys:
        if k in d:
            return k
    return None


# ---------------- JSON-Programm lesen ----------------
def read_program_from_json(filename: str) -> List[Dict[str, Any]]:
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    program = []
    for i, step in enumerate(data):
        t = step.get("type")
        if t == "Normal":
            program.append({"type": "Normal", "target_pressure": int(step["target_pressure"])})
        elif t == "Ramp":
            program.append({
                "type": "Ramp",
                "start_pressure": int(step["start_pressure"]),
                "end_pressure":   int(step["end_pressure"]),
                "ramp_speed":     float(step["ramp_speed"]),
            })
        elif t == "delay":
            program.append({"type": "delay", "delay_time": int(step["delay_time"])})
        else:
            raise ValueError(f"Unbekannter Step-Typ in JSON an Position {i}: {t}")
    return program


class Vader(Device):
    # ============ Geräteeigenschaften ============
    mini1_port = device_property(dtype=str, default_value="/dev/ttyACM0")
    mini2_port = device_property(dtype=str, default_value="/dev/ttyACM2")
    maxi_port  = device_property(dtype=str, default_value="/dev/ttyACM1")

    # Logging/Diagnose-Einstellungen für MAXI
    maxi_log_every_s = device_property(dtype=float, default_value=2.0)
    maxi_log_to_file = device_property(dtype=bool, default_value=False)
    maxi_log_file    = device_property(dtype=str, default_value="/tmp/vader_maxi_status.jsonl")

    # ============ Hilfsroutinen für MAXI-Kommunikation ============
    def _update_maxi_cache_from_status(self, status: Dict[str, Any]) -> None:
        """Nur Ist-Werte und IO-Flags in den Cache übernehmen, Setpoints nicht überschreiben.
        Robuste Extraktion mit Gas-/Feld-Aliases.
        """
        if not isinstance(status, dict):
            return
        # IO (inkl. V3 falls vorhanden)
        io = status.get("io") or {}
        for k in ("V1", "V2", "V3", "FanIn", "FanOut", "GasLeak"):
            if k in io:
                self.cache_maxi["io"][k] = bool(io[k])

        # MFC-Werte extrahieren
        mfc = status.get("mfc") or {}
        for gas in ("butan", "co2", "n2"):
            block = _extract_mfc_gas_block(mfc, gas)
            if not block:
                continue
            # Flow
            flow_key = _get_first_key(block, _MFC_KEYS["flow"])
            if flow_key:
                self.cache_maxi["mfc"][gas]["flow"] = _coerce_float(block.get(flow_key), 0.0)
            # Total
            total_key = _get_first_key(block, _MFC_KEYS["total"])
            if total_key:
                self.cache_maxi["mfc"][gas]["total"] = _coerce_float(block.get(total_key), 0.0)
            # Optional: wenn Firmware Setpoint mitsendet, nur internen Diagnose-Cache updaten, NICHT Attribute-Setpoints
            # (Attribut-Setpoints bleiben vom Client gesteuert)

        adc = status.get("adc") or {}
        self.cache_maxi["adc"]["P11"] = adc.get("P11")
        self.cache_maxi["adc"]["P31"] = adc.get("P31")
        self.cache_maxi["ts"] = time.time()

    def _async(self, fn, *args, **kwargs) -> None:
        """Feuer-und-vergiss: führt MAXI-Kommandos im Hintergrund aus und aktualisiert danach den Cache."""
        def worker():
            try:
                fn(*args, **kwargs)  # z.B. driver.set_flow_co2(v)
                status = self.driver.maxi_get_status()  # frischen Status holen (Roundtrip)
                self._update_maxi_cache_from_status(status)
                self._log_maxi_status(status)
            except Exception:
                # Absichtlich still – Tango-Schreibaufruf soll nicht blockieren/fehlschlagen
                pass
        threading.Thread(target=worker, daemon=True).start()

    # ============ Hilfsroutinen: MAXI-Snapshot & Logging ============
    def _build_maxi_snapshot(self, status: Dict[str, Any]) -> Dict[str, Any]:
        snap: Dict[str, Any] = {"ts": time.time(), "io": {}, "mfc": {}, "adc": {}}
        try:
            io = (status or {}).get("io") or {}
            for k in ("V1", "V2", "V3", "FanIn", "FanOut", "GasLeak"):
                if k in io:
                    snap["io"][k] = bool(io[k])
            mfc = (status or {}).get("mfc") or {}
            for gas in ("butan", "co2", "n2"):
                block = _extract_mfc_gas_block(mfc, gas)
                g: Dict[str, Any] = {}
                fk = _get_first_key(block, _MFC_KEYS["flow"]) if block else None
                if fk is not None:
                    g["flow"] = _coerce_float(block.get(fk), 0.0)
                tk = _get_first_key(block, _MFC_KEYS["total"]) if block else None
                if tk is not None:
                    g["total"] = _coerce_float(block.get(tk), 0.0)
                sk = _get_first_key(block, _MFC_KEYS["setpoint"]) if block else None
                if sk is not None:
                    try:
                        g["setpoint"] = float(block.get(sk)) if block.get(sk) is not None else None
                    except Exception:
                        g["setpoint"] = None
                snap["mfc"][gas] = g
            adc = (status or {}).get("adc") or {}
            for k in ("P11", "P31"):
                if k in adc:
                    snap["adc"][k] = adc[k]
        except Exception:
            pass
        return snap

    def _log_maxi_status(self, status: Dict[str, Any]) -> None:
        # Ringpuffer anlegen, falls noch nicht vorhanden
        if not hasattr(self, "_maxi_log"):
            self._maxi_log = deque(maxlen=2000)
            self._last_maxi_log_ts = 0.0
            self._last_maxi_snapshot = None
        snap = self._build_maxi_snapshot(status)
        self._maxi_log.append(snap)
        now = snap.get("ts", time.time())
        interval = float(getattr(self, "maxi_log_every_s", 2.0))
        # Änderungserkennung auf MFC/IO
        changed = False
        try:
            prev = getattr(self, "_last_maxi_snapshot", None)
            if prev is None:
                changed = True
            else:
                changed = (prev.get("mfc") != snap.get("mfc")) or (prev.get("io") != snap.get("io"))
        except Exception:
            changed = True
        if changed or (now - getattr(self, "_last_maxi_log_ts", 0.0)) >= interval:
            short = {
                "ts": snap.get("ts"),
                "io": snap.get("io", {}),
                "butan": snap.get("mfc", {}).get("butan", {}),
                "co2":   snap.get("mfc", {}).get("co2", {}),
                "n2":    snap.get("mfc", {}).get("n2", {}),
            }
            try:
                self.logger.info("MAXI status %s", json.dumps(short, ensure_ascii=False))
            except Exception:
                pass
            self._last_maxi_log_ts = now
            self._last_maxi_snapshot = snap

    # ============ Attribute ============
    # large_high_pressure_volume = (V1 && V2)
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def large_high_pressure_volume(self) -> bool:
        return bool(self.cache_maxi["io"].get("V1")) and bool(self.cache_maxi["io"].get("V2"))

    def write_large_high_pressure_volume(self, value: bool):
        v = bool(value)
        # V1 und V2 werden „versteckt“ gesteuert
        self.driver.set_v1(v)
        self.driver.set_v2(v)
        self.cache_maxi["io"]["V1"] = v
        self.cache_maxi["io"]["V2"] = v

    # Einzelventile V1/V2: **entfernt als Attribute** (intern weiterhin genutzt)
    # V3 bleibt als einzelnes Attribut
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def V3(self) -> bool:
        return bool(self.cache_maxi["io"].get("V3", False))

    def write_V3(self, value: bool):
        v = bool(value)
        self.driver.set_v3(v)
        self.cache_maxi["io"]["V3"] = v

    # Fans (beide gemeinsam)
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def Fans(self) -> bool:
        return bool(self.cache_maxi["io"].get("FanIn")) and bool(self.cache_maxi["io"].get("FanOut"))

    def write_Fans(self, value: bool):
        v = bool(value)
        self.driver.set_fans(v)
        self.cache_maxi["io"]["FanIn"]  = v
        self.cache_maxi["io"]["FanOut"] = v

    # Gas Leak (nur Anzeige)
    @attribute(dtype=bool)
    def GasLeak(self) -> bool:
        return bool(self.cache_maxi["io"].get("GasLeak", False))

    # --------- Setpoint-Flows (READ/WRITE, nur positiv) ---------
    # WICHTIG: liest/schreibt *separate* Setpoint-Felder (kein Ist-Flow mehr) und schreibt asynchron zum Gerät.
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0.0)
    def setpoint_flow_Butan_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["butan"].get("setpoint") or 0.0)

    def write_setpoint_flow_Butan_nl_per_min(self, value: float):
        v = max(0.0, float(value))
        self.cache_maxi["mfc"]["butan"]["setpoint"] = v  # sofort sichtbar
        self._async(self.driver.set_flow_butan, v)           # non-blocking Geräteaufruf

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0.0)
    def setpoint_flow_CO2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["co2"].get("setpoint") or 0.0)

    def write_setpoint_flow_CO2_nl_per_min(self, value: float):
        v = max(0.0, float(value))
        self.cache_maxi["mfc"]["co2"]["setpoint"] = v
        self._async(self.driver.set_flow_co2, v)

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0.0)
    def setpoint_flow_N2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["n2"].get("setpoint") or 0.0)

    def write_setpoint_flow_N2_nl_per_min(self, value: float):
        v = max(0.0, float(value))
        self.cache_maxi["mfc"]["n2"]["setpoint"] = v
        self._async(self.driver.set_flow_n2, v)

    # --------- Ist-Flows (READ ONLY) ---------
    @attribute(dtype=float, dformat=AttrDataFormat.SCALAR)
    def flow_Butan_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["butan"].get("flow") or 0.0)

    @attribute(dtype=float, dformat=AttrDataFormat.SCALAR)
    def flow_CO2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["co2"].get("flow") or 0.0)

    @attribute(dtype=float, dformat=AttrDataFormat.SCALAR)
    def flow_N2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["n2"].get("flow") or 0.0)

    # Totals (READ/WRITE, nur positiv)
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0.0)
    def total_flow_CO2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["co2"].get("total") or 0.0)

    def write_total_flow_CO2_nl_per_min(self, value: float):
        v = max(0.0, float(value))
        self.driver.set_total_co2(v)

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0.0)
    def total_flow_N2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["n2"].get("total") or 0.0)

    def write_total_flow_N2_nl_per_min(self, value: float):
        v = max(0.0, float(value))
        self.driver.set_total_n2(v)

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0.0)
    def total_flow_Butan_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["butan"].get("total") or 0.0)

    def write_total_flow_Butan_nl_per_min(self, value: float):
        v = max(0.0, float(value))
        self.driver.set_total_butan(v)

    # MINI2 / Druck / PWM
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0.0)
    def setpoint_pressure_kPa(self) -> float:
        return float(self.cache_mini2.get("setpoint_kpa") or 0.0)

    def write_setpoint_pressure_kPa(self, value: float):
        v = max(0.0, float(value))
        self.driver.setpoint_pressure(v)
        self.cache_mini2["setpoint_kpa"] = v

    @attribute(dtype=DevLong, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0, max_value=255)
    def vp1(self) -> int:
        return int(self.cache_mini2.get("pwm1") or 0)

    def write_vp1(self, value: int):
        v = max(0, min(255, int(value)))
        self.driver.set_manual_PWM_in(v)
        self.cache_mini2["pwm1"] = v

    @attribute(dtype=DevLong, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR, min_value=0, max_value=255)
    def vp2(self) -> int:
        return int(self.cache_mini2.get("pwm2") or 0)

    def write_vp2(self, value: int):
        v = max(0, min(255, int(value)))
        self.driver.set_manual_PWM_out(v)
        self.cache_mini2["pwm2"] = v

    # Aktueller Druck aus MINI1 (READ ONLY)
    @attribute(dtype=float, dformat=AttrDataFormat.SCALAR)
    def pressure_kPa(self) -> float:
        p = self.cache_mini1.get("pressure")
        return float("nan") if p is None else float(p)

    # ============ Diagnose-Attribute & -Kommandos ============
    @attribute(dtype=str)
    def maxi_status_log(self) -> str:
        """Gibt die letzten ~200 MAXI-Snapshots als JSON-Array zurück."""
        try:
            entries = list(self._maxi_log)[-200:]
            return json.dumps(entries, ensure_ascii=False)
        except Exception:
            return "[]"

    @command(dtype_in=str, dtype_out=str)
    def ExportMaxiLog(self, path: str) -> str:
        """Schreibt den gesamten Ringpuffer als JSONL-Datei (eine Zeile pro Snapshot)."""
        try:
            n = 0
            with open(path, "w", encoding="utf-8") as f:
                for item in self._maxi_log:
                    f.write(json.dumps(item, ensure_ascii=False) + "\n")
                    n += 1
            return f"OK: wrote {n} lines to {path}"
        except Exception as e:
            return f"ERROR: {e}"

    # ============ Programmsteuerung ============
    @command(dtype_in=str)
    def RunProgram(self, json_path: str):
        self.StopProgram()
        self._program_stop_evt.clear()
        self._program_thread = threading.Thread(
            target=self._program_worker, args=(json_path,), daemon=True
        )
        self._program_thread.start()

    @command()
    def StopProgram(self):
        self._program_stop_evt.set()
        if self._program_thread and self._program_thread.is_alive():
            self._program_thread.join(timeout=0.1)
        self._program_thread = None

    # ============ Lifecycle ============
    def init_device(self):
        Device.init_device(self)
        self.set_state(DevState.INIT)
        self.set_status("Initialisiere Treiber...")

        # --- Logging für MAXI-Status vorbereiten ---
        self.logger = logging.getLogger("Vader.Tango")
        if not self.logger.handlers:
            self.logger.setLevel(logging.INFO)
        self._maxi_log = deque(maxlen=2000)
        self._last_maxi_log_ts = 0.0
        self._last_maxi_snapshot = None
        if getattr(self, "maxi_log_to_file", False):
            try:
                fh = logging.FileHandler(self.maxi_log_file)
                fh.setLevel(logging.INFO)
                fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s", "%H:%M:%S"))
                self.logger.addHandler(fh)
                self._maxi_log_file_handler = fh
            except Exception:
                pass

        self.driver = VaderDeviceDriver(
            mini1_port=self.mini1_port,
            mini2_port=self.mini2_port,
            maxi_port=self.maxi_port
        )

        # Caches: jetzt mit *eigenen* Setpoint-Feldern
        self.cache_maxi: Dict[str, Any] = {
            "io": {"V1": False, "V2": False, "V3": False, "FanIn": False, "FanOut": False, "GasLeak": False},
            "mfc": {
                "butan": {"setpoint": 0.0, "flow": 0.0, "total": 0.0},
                "co2":   {"setpoint": 0.0, "flow": 0.0, "total": 0.0},
                "n2":    {"setpoint": 0.0, "flow": 0.0, "total": 0.0},
            },
            "adc": {"P11": None, "P31": None},
            "ts": 0.0,
        }
        self.cache_mini2: Dict[str, Any] = {
            "setpoint_kpa": 0.0, "pwm1": 0, "pwm2": 0,
            "mode": "UNKNOWN", "p20_kpa": None, "p21_kpa": None, "ts": 0.0
        }
        self.cache_mini1: Dict[str, Any] = {"pressure": None, "ts": 0.0}

        # Hardware-Startzustand: V1/V2/V3 False, Setpoints = 0
        try:
            self.driver.set_v1(False)
            self.driver.set_v2(False)
            self.driver.set_v3(False)
            self.driver.set_fans(False)

            # Alle Gas-Setpoints auf 0
            self.driver.set_flow_butan(0.0)
            self.driver.set_flow_co2(0.0)
            self.driver.set_flow_n2(0.0)

            # Druck-Setpoint auf 0
            self.driver.setpoint_pressure(0.0)
        except Exception:
            # Hardware ggf. nicht erreichbar – Cache bleibt auf Default
            pass

        self._program_thread: Optional[threading.Thread] = None
        self._program_stop_evt = threading.Event()

        self._stop_evt = threading.Event()
        self._t_mini1 = threading.Thread(target=self._poll_mini1, daemon=True)
        self._t_mini2 = threading.Thread(target=self._poll_mini2, daemon=True)
        self._t_maxi  = threading.Thread(target=self._poll_maxi,  daemon=True)
        self._t_mini1.start(); self._t_mini2.start(); self._t_maxi.start()

        self.set_state(DevState.ON)
        self.set_status("Bereit.")

    def delete_device(self):
        self.StopProgram()
        self._stop_evt.set()
        for t in (self._t_mini1, self._t_mini2, self._t_maxi):
            if t and t.is_alive():
                t.join(timeout=0.2)
        try:
            self.driver.close()
        except Exception:
            pass
        # File-Handler sauber abklemmen
        try:
            if hasattr(self, "_maxi_log_file_handler"):
                self.logger.removeHandler(self._maxi_log_file_handler)
        except Exception:
            pass
        self.set_state(DevState.OFF)
        self.set_status("Gestoppt.")

    # ============ Poller ============
    def _poll_mini1(self):
        period = 1.0 / 20.0
        while not self._stop_evt.is_set():
            try:
                p = self.driver.mini1.pressure
                self.cache_mini1["pressure"] = p
                self.cache_mini1["ts"] = time.time()
            except Exception:
                pass
            time.sleep(period)

    def _poll_mini2(self):
        period = 2.0
        while not self._stop_evt.is_set():
            try:
                self.driver.mini2_request_status()
                status = self.driver.mini2_get_status()
                self.cache_mini2["pwm1"] = int(status.get("pwm1") or 0)
                self.cache_mini2["pwm2"] = int(status.get("pwm2") or 0)
                self.cache_mini2["mode"] = status.get("mode") or "UNKNOWN"
                self.cache_mini2["ts"] = time.time()
            except Exception:
                pass
            time.sleep(period)

    def _poll_maxi(self):
        period = 0.5  # schnellere Aktualisierung; watchdog-Style Timing
        next_due = 0.0
        while not self._stop_evt.is_set():
            now = time.time()
            if now < next_due:
                time.sleep(min(0.05, next_due - now))
                continue
            try:
                # Einige Firmwares senden zwischendurch Heartbeats ohne MFC-Block.
                # Wir lesen im Zweifel zweimal und nehmen den ersten mit MFC-Daten.
                status = self.driver.maxi_get_status()
                if not isinstance(status, dict) or not status.get("mfc"):
                    status2 = self.driver.maxi_get_status()
                    if isinstance(status2, dict):
                        status = status2
                self._update_maxi_cache_from_status(status)
                self._log_maxi_status(status)
            except Exception:
                pass
            next_due = time.time() + period

    # ============ Programm-Worker ============
    def _program_worker(self, json_path: str):
        try:
            prog = read_program_from_json(json_path)
        except Exception as e:
            self.set_status(f"Programm-Fehler: {e}")
            return

        n = len(prog)
        self.set_status(f"Programm gestartet: {json_path} (Schritte: {n})")

        for idx, step in enumerate(prog, start=1):
            if self._program_stop_evt.is_set():
                self.set_status("Programm abgebrochen.")
                return
            try:
                t = step["type"]
                if t == "Normal":
                    target = max(0.0, float(step["target_pressure"]))
                    self.set_status(f"[{idx}/{n}] Normal: setpoint → {target:.3f} kPa")
                    self.driver.setpoint_pressure(target)
                    self.cache_mini2["setpoint_kpa"] = target

                elif t == "Ramp":
                    start_p = max(0.0, float(step["start_pressure"]))
                    end_p   = max(0.0, float(step["end_pressure"]))
                    speed   = max(0.0, float(step["ramp_speed"]))  # kPa/s
                    self.set_status(f"[{idx}/{n}] Ramp: {start_p:.3f} → {end_p:.3f} kPa @ {speed:.3f} kPa/s")
                    self.driver.setpoint_pressure(start_p)
                    time.sleep(0.2)
                    self.driver.set_ramp(speed_kpa_s=speed, end_kpa=end_p)
                    approx = (abs(end_p - start_p) / max(1e-6, speed)) if speed > 0 else 0.0
                    t_end = time.time() + min(approx, 3600)
                    while time.time() < t_end:
                        if self._program_stop_evt.is_set():
                            break
                        time.sleep(0.1)

                elif t == "delay":
                    ms = int(step["delay_time"])
                    self.set_status(f"[{idx}/{n}] Delay: {ms} ms")
                    t_end = time.time() + ms / 1000.0
                    while time.time() < t_end:
                        if self._program_stop_evt.is_set():
                            break
                        time.sleep(0.05)

                else:
                    self.set_status(f"[{idx}/{n}] Unbekannter Typ: {t} – übersprungen")

            except Exception as e:
                self.set_status(f"[{idx}/{n}] Schrittfehler: {e} – weiter")

        self.set_status("Programm abgeschlossen.")


# ---- main ----
if __name__ == "__main__":
    run((Vader,))
