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
try:
    from tango.server import pipe
    from tango import PipeWriteType
    _HAS_PIPES = True
except Exception:
    _HAS_PIPES = False

# === Treiber ===
from VaderDeviceDriver import VaderDeviceDriver


# ---------------- Helpers ----------------
def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def _get_first(d: dict, *keys, default=None):
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return default


# ---------------- JSON-Programm lesen ----------------
def read_program_from_json(filename: str) -> List[Dict[str, Any]]:
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    program: List[Dict[str, Any]] = []

    for i, raw in enumerate(data):
        if not isinstance(raw, dict):
            raise ValueError(f"Step {i}: kein Objekt")

        t = raw.get("type")
        if t == "Normal":
            target = _get_first(raw, "target_pressure_kPa", "target_pressure", default=None)
            if target is None:
                raise ValueError(f"Step {i} Normal: target missing")
            program.append({"type": "Normal", "target_pressure_kPa": float(target)})

        elif t == "delay":
            ms = _get_first(raw, "delay_time_ms", "delay_time", default=None)
            if ms is None:
                raise ValueError(f"Step {i} delay: delay_time_ms missing")
            program.append({"type": "delay", "delay_time_ms": int(ms)})

        elif t == "Ramp":
            start = _get_first(raw, "start_pressure_kPa", "start_pressure", default=None)
            end   = _get_first(raw, "end_pressure_kPa", "end_pressure", default=None)
            speed = _get_first(raw, "ramp_speed_kPa_sec", "ramp_speed_kP_sec", "ramp_speed", default=None)
            if end is None or speed is None:
                raise ValueError(f"Step {i} Ramp: end or speed missing")
            step = {"type": "Ramp",
                    "start_pressure_kPa": None if start is None else float(start),
                    "end_pressure_kPa": float(end),
                    "ramp_speed_kPa_sec": float(speed)}
            program.append(step)

        elif t in ("set_butan", "set_co2", "set_n2"):
            flow = raw.get("flow_nl_min")
            if flow is None:
                raise ValueError(f"Step {i} {t}: flow_nl_min missing")
            program.append({"type": t, "flow_nl_min": float(flow)})

        elif t in ("open_vp1", "open_vp2"):
            pwm = raw.get("PWM")
            if pwm is None:
                raise ValueError(f"Step {i} {t}: PWM missing")
            program.append({"type": t, "PWM": int(pwm)})
        
        elif t == "eject_high_pressure_volume":
            val = _get_first(raw, "value", "state", "open", default=None)
            if isinstance(val, str):
                val = val.strip().lower() in ("1", "true", "yes", "on")
            if val is None:
                raise ValueError(f"Step {i} eject_high_pressure_volume: value missing")
            program.append({"type": "eject_high_pressure_volume", "value": bool(val)})

        else:
            raise ValueError(f"Unbekannter Step-Typ an Position {i}: {t}")
    return program


class Vader(Device):
    # ============ Geräteeigenschaften ============
    mini1_port = device_property(dtype=str, default_value="/dev/ttyACM0")
    mini2_port = device_property(dtype=str, default_value="/dev/ttyACM2")
    maxi_port  = device_property(dtype=str, default_value="/dev/ttyACM1")
    mini1_pipe_default_points = device_property(dtype=int, default_value=10000)

    # Logging/Diagnose-Einstellungen für MAXI
    maxi_log_every_s = device_property(dtype=float, default_value=2.0)
    maxi_log_to_file = device_property(dtype=bool, default_value=False)
    maxi_log_file    = device_property(dtype=str, default_value="/tmp/vader_maxi_status.jsonl")

    # ============ Kleine Helfer ============
    def _log_maxi_status(self, status: Dict[str, Any]) -> None:
        if not hasattr(self, "_maxi_log"):
            self._maxi_log = deque(maxlen=2000)
            self._last_maxi_log_ts = 0.0
        snap = {
            "ts": time.time(),
            "io": {
                "V1": bool((status.get("io") or {}).get("V1", False)),
                "V2": bool((status.get("io") or {}).get("V2", False)),
                "V3": bool((status.get("io") or {}).get("V3", False)),
                "FanIn":  bool((status.get("io") or {}).get("FanIn", False)),
                "FanOut": bool((status.get("io") or {}).get("FanOut", False)),
                "GasLeak": bool((status.get("io") or {}).get("GasLeak", False)),
            },
            "adc": {
                "P11": (status.get("adc") or {}).get("P11"),
                "P31": (status.get("adc") or {}).get("P31"),
            },
        }
        self._maxi_log.append(snap)
        now = snap["ts"]
        if (now - getattr(self, "_last_maxi_log_ts", 0.0)) >= float(getattr(self, "maxi_log_every_s", 2.0)):
            try:
                self.logger.info("MAXI status %s", json.dumps({"ts": snap["ts"], "io": snap["io"]}, ensure_ascii=False))
            except Exception:
                pass
            self._last_maxi_log_ts = now

    def _apply_io_cache(self, status: Dict[str, Any]) -> None:
        io = status.get("io") or {}
        for k in ("V1", "V2", "V3", "FanIn", "FanOut", "GasLeak"):
            if k in io:
                self.cache_maxi["io"][k] = bool(io[k])
        adc = status.get("adc") or {}
        self.cache_maxi["adc"]["P11"] = adc.get("P11")
        self.cache_maxi["adc"]["P31"] = adc.get("P31")
        self.cache_maxi["ts"] = time.time()

    # Einheitlicher Pfad für MFC-Setpoints mit Echo-Bestätigung:
    # Treibermethoden geben den *übernommenen* Wert zurück (Echo OK, float).
    def _set_mfc_setpoint(self, gas: str, value: float, setter_fn) -> None:
        v = _clamp(value, 0.0, 10.0)  # Limit 0..10
        try:
            confirmed = setter_fn(v)            # ← Treiber liefert bestätigten Wert
            confirmed = _clamp(float(confirmed), 0.0, 10.0)
            self.cache_maxi["mfc"][gas]["setpoint"] = confirmed
            # Status refresh (optional) für IO/ADC
            try:
                status = self.driver.maxi_get_status()
                if isinstance(status, dict):
                    self._apply_io_cache(status)
                    self._log_maxi_status(status)
            except Exception:
                pass
        except Exception:
            # Keine Cache-Änderung bei Fehler
            pass

    # ============ Attribute ============
    # large_high_pressure_volume = (V1 && V2)
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def large_high_pressure_volume(self) -> bool:
        return bool(self.cache_maxi["io"].get("V1")) and bool(self.cache_maxi["io"].get("V2"))

    def write_large_high_pressure_volume(self, value: bool):
        v = bool(value)
        self.driver.set_v1(v)
        self.driver.set_v2(v)
        self.cache_maxi["io"]["V1"] = v
        self.cache_maxi["io"]["V2"] = v

    # Ehemals V3 → eject_high_pressure
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def eject_high_pressure(self) -> bool:
        return bool(self.cache_maxi["io"].get("V3", False))

    def write_eject_high_pressure(self, value: bool):
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

    # --------- Setpoint-Flows (READ/WRITE, 0..10.00, Wert = bestätigter Echo-Wert) ---------
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR,
               min_value=0.0, max_value=10.0)
    def setpoint_flow_Butan_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["butan"].get("setpoint") or 0.0)

    def write_setpoint_flow_Butan_nl_per_min(self, value: float):
        self._set_mfc_setpoint("butan", value, self.driver.set_flow_butan)

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR,
               min_value=0.0, max_value=10.0)
    def setpoint_flow_CO2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["co2"].get("setpoint") or 0.0)

    def write_setpoint_flow_CO2_nl_per_min(self, value: float):
        self._set_mfc_setpoint("co2", value, self.driver.set_flow_co2)

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR,
               min_value=0.0, max_value=10.0)
    def setpoint_flow_N2_nl_per_min(self) -> float:
        return float(self.cache_maxi["mfc"]["n2"].get("setpoint") or 0.0)

    def write_setpoint_flow_N2_nl_per_min(self, value: float):
        self._set_mfc_setpoint("n2", value, self.driver.set_flow_n2)

    # MINI2 / Druck / PWM
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR,
               min_value=0.0, max_value=1000.0)
    def setpoint_pressure_kPa(self) -> float:
        return float(self.cache_mini2.get("setpoint_kpa") or 0.0)

    def write_setpoint_pressure_kPa(self, value: float):
        v = _clamp(value, 0.0, 1000.0)
        self.driver.setpoint_pressure(v)
        self.cache_mini2["setpoint_kpa"] = v

    # vp1 → valve_increase_pressure
    @attribute(dtype=DevLong, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR,
               min_value=0, max_value=255)
    def valve_increase_pressure(self) -> int:
        return int(self.cache_mini2.get("pwm1") or 0)

    def write_valve_increase_pressure(self, value: int):
        v = int(_clamp(int(value), 0, 255))
        self.driver.set_manual_PWM_in(v)
        self.cache_mini2["pwm1"] = v

    # vp2 → valve_reduce_pressure
    @attribute(dtype=DevLong, access=AttrWriteType.READ_WRITE, dformat=AttrDataFormat.SCALAR,
               min_value=0, max_value=255)
    def valve_reduce_pressure(self) -> int:
        return int(self.cache_mini2.get("pwm2") or 0)

    def write_valve_reduce_pressure(self, value: int):
        v = int(_clamp(int(value), 0, 255))
        self.driver.set_manual_PWM_out(v)
        self.cache_mini2["pwm2"] = v

    # Aktueller Druck aus MINI1 (READ ONLY)
    @attribute(dtype=float, dformat=AttrDataFormat.SCALAR)
    def pressure_kPa(self) -> float:
        p = self.cache_mini1.get("pressure")
        return float("nan") if p is None else float(p)
    if _HAS_PIPES:
        @pipe(access=PipeWriteType.READ)
        def mini1_pressure_series(self):
            """
            Liefert die jüngste MINI1-Druckserie als Pipe-Payload.
            Keys:
            - timestamps_iso: Liste ISO8601 Strings
            - pressure_kpa:   Liste floats
            - count:          Anzahl Punkte
            Hinweis: Anzahl Punkte per Geräteeigenschaft mini1_pipe_default_points (Default 1000).
            """
            # Hole die letzten N Stichproben aus dem Treiber (deque[Tuple[datetime, float]])
            n = int(getattr(self, "mini1_pipe_default_points", 1000))
            try:
                series = self.driver.mini1_get_last_n(n) or []
            except Exception:
                series = []

            ts = []
            pv = []
            for dt, p in series:
                try:
                    ts.append(dt.isoformat())
                    pv.append(float(p))
                except Exception:
                    # robust gegenüber fehlerhaften Einträgen
                    continue

            return {
                "timestamps_iso": ts,
                "pressure_kpa": pv,
                "count": len(pv),
            }
    else:
        @command(dtype_in=int, dtype_out=str)
        def GetMini1SeriesJSON(self, n: int) -> str:
            """
            Fallback, falls Pipes im Client/Server nicht unterstützt werden.
            Gibt die letzten n Punkte als JSON-String zurück.
            """
            try:
                series = self.driver.mini1_get_last_n(int(max(0, n))) or []
            except Exception:
                series = []
            out = {
                "timestamps_iso": [],
                "pressure_kpa": [],
                "count": 0,
            }
            for dt, p in series:
                try:
                    out["timestamps_iso"].append(dt.isoformat())
                    out["pressure_kpa"].append(float(p))
                except Exception:
                    pass
            out["count"] = len(out["pressure_kpa"])
            try:
                return json.dumps(out, ensure_ascii=False)
            except Exception:
                # als Minimal-Fallback eine sehr einfache Darstellung
                return json.dumps({"count": out["count"]})
            
    # ============ Diagnose-Kommandos ============
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

        # --- Logging vorbereiten ---
        self.logger = logging.getLogger("Vader.Tango")
        if not self.logger.handlers:
            self.logger.setLevel(logging.INFO)
        self._maxi_log = deque(maxlen=2000)
        self._last_maxi_log_ts = 0.0
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

        # Caches (nur Setpoints):
        self.cache_maxi: Dict[str, Any] = {
            "io": {"V1": False, "V2": False, "V3": False, "FanIn": False, "FanOut": False, "GasLeak": False},
            "mfc": {
                "butan": {"setpoint": 0.0},
                "co2":   {"setpoint": 0.0},
                "n2":    {"setpoint": 0.0},
            },
            "adc": {"P11": None, "P31": None},
            "ts": 0.0,
        }
        self.cache_mini2: Dict[str, Any] = {
            "setpoint_kpa": 0.0, "pwm1": 0, "pwm2": 0,
            "mode": "UNKNOWN", "p20_kpa": None, "p21_kpa": None, "ts": 0.0
        }
        self.cache_mini1: Dict[str, Any] = {"pressure": None, "ts": 0.0}

        # Hardware-Startzustand
        try:
            self.driver.set_v1(False)
            self.driver.set_v2(False)
            self.driver.set_v3(False)
            self.driver.set_fans(False)
            self.driver.set_flow_butan(0.0)
            self.driver.set_flow_co2(0.0)
            self.driver.set_flow_n2(0.0)
            self.driver.setpoint_pressure(0.0)
        except Exception:
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
        period = 0.5
        next_due = 0.0
        while not self._stop_evt.is_set():
            now = time.time()
            if now < next_due:
                time.sleep(min(0.05, next_due - now))
                continue
            try:
                status = self.driver.maxi_get_status()
                if isinstance(status, dict):
                    self._apply_io_cache(status)
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
                    target = _clamp(step["target_pressure_kPa"], 0.0, 1000.0)
                    self.set_status(f"[{idx}/{n}] Normal: setpoint → {target:.3f} kPa")
                    self.driver.setpoint_pressure(target)
                    self.cache_mini2["setpoint_kpa"] = target

                elif t == "Ramp":
                    # start optional → wenn None, vom aktuellen Setpoint ausgehen
                    start_p = step.get("start_pressure_kPa")
                    if start_p is None:
                        start_p = float(self.cache_mini2.get("setpoint_kpa") or 0.0)
                    start_p = _clamp(start_p, 0.0, 1000.0)
                    end_p   = _clamp(step["end_pressure_kPa"], 0.0, 1000.0)
                    speed   = max(0.0, float(step["ramp_speed_kPa_sec"]))
                    self.set_status(f"[{idx}/{n}] Ramp: {start_p:.3f} → {end_p:.3f} kPa @ {speed:.3f} kPa/s")
                    self.driver.setpoint_pressure(start_p)
                    time.sleep(0.2)
                    self.driver.set_ramp(speed_kpa_s=speed, end_kpa=end_p)
                    approx = (abs(end_p - start_p) / max(1e-6, speed)) if speed > 0 else 0.0
                    t_end = time.time() + min(approx, 3600)
                    while time.time() < t_end and not self._program_stop_evt.is_set():
                        time.sleep(0.1)
                    self.cache_mini2["setpoint_kpa"] = end_p  # Ziel gesetzt

                elif t == "delay":
                    ms = int(step["delay_time_ms"])
                    self.set_status(f"[{idx}/{n}] Delay: {ms} ms")
                    t_end = time.time() + ms / 1000.0
                    while time.time() < t_end and not self._program_stop_evt.is_set():
                        time.sleep(0.05)

                elif t == "set_butan":
                    q = _clamp(step["flow_nl_min"], 0.0, 10.0)
                    self.set_status(f"[{idx}/{n}] set_butan: {q:.3f} NL/min")
                    self._set_mfc_setpoint("butan", q, self.driver.set_flow_butan)

                elif t == "set_co2":
                    q = _clamp(step["flow_nl_min"], 0.0, 10.0)
                    self.set_status(f"[{idx}/{n}] set_co2: {q:.3f} NL/min")
                    self._set_mfc_setpoint("co2", q, self.driver.set_flow_co2)

                elif t == "set_n2":
                    q = _clamp(step["flow_nl_min"], 0.0, 10.0)
                    self.set_status(f"[{idx}/{n}] set_n2: {q:.3f} NL/min")
                    self._set_mfc_setpoint("n2", q, self.driver.set_flow_n2)

                elif t == "open_vp1":
                    pwm = int(_clamp(int(step["PWM"]), 0, 255))
                    self.set_status(f"[{idx}/{n}] open_vp1: PWM={pwm}")
                    self.driver.set_manual_PWM_in(pwm)
                    self.cache_mini2["pwm1"] = pwm

                elif t == "open_vp2":
                    pwm = int(_clamp(int(step["PWM"]), 0, 255))
                    self.set_status(f"[{idx}/{n}] open_vp2: PWM={pwm}")
                    self.driver.set_manual_PWM_out(pwm)
                    self.cache_mini2["pwm2"] = pwm

                elif t == "eject_high_pressure_volume":
                    v = bool(step["value"])
                    self.set_status(f"[{idx}/{n}] eject_high_pressure_volume: {'OPEN' if v else 'CLOSE'}")
                    self.driver.set_v3(v)
                    self.cache_maxi["io"]["V3"] = v
                else:
                    self.set_status(f"[{idx}/{n}] Unbekannter Typ: {t} – übersprungen")

            except Exception as e:
                self.set_status(f"[{idx}/{n}] Schrittfehler: {e} – weiter")

        self.set_status("Programm abgeschlossen.")


# ---- main ----
if __name__ == "__main__":
    run((Vader,))