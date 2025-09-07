#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import csv
import time
import json
import threading
import random
from typing import Dict, Any

import PyTango as tango


def _now_ts():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


class Vader(tango.Device_4Impl):
    """
    Dummy-Server, der sich wie der echte 'Vader'-Server verhält.
    - gleiche Attribute (Typ/Mode) und Kommandos wie dein funktionierendes Beispiel
    - keine Hardware, nur Simulation (für UI-/Client-Tests)
    """

    def __init__(self, cl, name):
        tango.Device_4Impl.__init__(self, cl, name)
        # Locks/Flags
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._prog_stop = threading.Event()
        self._prog_thread = None
        self._last_integrate = time.time()

        # Zustände/Caches (Dummy)
        self._mini1: Dict[str, Any] = {"pressure_kpa": 101.3}
        self._mini2: Dict[str, Any] = {
            "p20_kpa": None, "p21_kpa": None,
            "setpoint_kpa": 0.0,
            "pwm1": 0, "pwm2": 0,
            "mode": "IDLE",
        }
        self._maxi: Dict[str, Any] = {
            "p11": None, "p31": None,
            "v1_power": False, "v2_power": False, "v3_power": False, "v4_power": False,
            "fan_in_power": False, "fan_out_power": False,
            "vs3_power": False,
            "heater_pwm": 0,
            "mfc_butan": {"setpoint": 0.0, "flow": 0.0, "total": 0.0},
            "mfc_n2":    {"setpoint": 0.0, "flow": 0.0, "total": 0.0},
            "mfc_co2":   {"setpoint": 0.0, "flow": 0.0, "total": 0.0},
            "leak_sensor": 0,
        }

        Vader.init_device(self)

    # ---------- Lifecycle ----------
    def init_device(self):
        self.get_device_properties(self.get_device_class())
        self.set_state(tango.DevState.INIT)
        try:
            # Threads starten
            self._stop_evt.clear()
            self._t_mini1 = threading.Thread(target=self._loop_mini1_20hz, daemon=True)
            self._t_mini2 = threading.Thread(target=self._loop_mini2_05hz, daemon=True)
            self._t_maxi  = threading.Thread(target=self._loop_maxi_05hz,  daemon=True)
            self._t_mini1.start()
            self._t_mini2.start()
            self._t_maxi.start()

            self.set_state(tango.DevState.ON)
            self.set_status("Dummy bereit.")
        except Exception as e:
            self.set_state(tango.DevState.FAULT)
            self.set_status(f"Init failed: {e}")

    def delete_device(self):
        try:
            self.StopProgram()
        except Exception:
            pass
        self._stop_evt.set()
        for t in (getattr(self, "_t_mini1", None), getattr(self, "_t_mini2", None), getattr(self, "_t_maxi", None)):
            if t and t.is_alive():
                t.join(timeout=0.5)
        self.set_state(tango.DevState.OFF)
        self.set_status("Dummy gestoppt.")

    # ---------- Simulation Loops ----------
    def _loop_mini1_20hz(self):
        """20 Hz: Drucksimulation + Totals-Integration + Leak-Heuristik."""
        period = 1.0 / 20.0
        tau_base = 2.5
        while not self._stop_evt.is_set():
            t0 = time.time()
            with self._lock:
                now = t0
                dt = max(1e-4, now - self._last_integrate)
                self._last_integrate = now

                # Druck nähert sich Setpoint an (1. Ordnung), PWM beschleunigt
                sp = float(self._mini2["setpoint_kpa"])
                p  = float(self._mini1["pressure_kpa"])
                pwm_gain = 1.0 + 0.01 * (self._mini2["pwm1"] + self._mini2["pwm2"])
                tau = max(0.3, tau_base / pwm_gain)
                p += (sp - p) * (dt / tau) + random.gauss(0.0, 0.01)
                self._mini1["pressure_kpa"] = max(0.0, p)

                # Flows integrieren -> Totals (l/min -> l)
                for key in ("mfc_butan", "mfc_n2", "mfc_co2"):
                    flow = max(0.0, float(self._maxi[key]["setpoint"]))
                    # „Flow“: wir setzen den Ist-Flow auf den Soll (vereinfacht)
                    self._maxi[key]["flow"] = flow
                    self._maxi[key]["total"] += flow * (dt / 60.0)

                # GasLeak-„Blips“: höhere Wahrscheinlichkeit bei viel Flow & ausgeschalteten Fans
                tflow = self._maxi["mfc_butan"]["flow"] + self._maxi["mfc_n2"]["flow"] + self._maxi["mfc_co2"]["flow"]
                fans_on = bool(self._maxi["fan_in_power"] or self._maxi["fan_out_power"])
                leak_prob = 0.02 if (tflow > 5.0 and not fans_on) else 0.003
                self._maxi["leak_sensor"] = 1 if (random.random() < leak_prob) else 0

            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    def _loop_mini2_05hz(self):
        """0.5 Hz: MINI2-„Mode“ ableiten (nur Anzeigezweck)."""
        period = 2.0
        while not self._stop_evt.is_set():
            with self._lock:
                if abs(self._mini2["setpoint_kpa"] - self._mini1["pressure_kpa"]) > 2.0:
                    mode = "AUTOMATIC"
                elif (self._mini2["pwm1"] + self._mini2["pwm2"]) > 0:
                    mode = "MANUAL"
                else:
                    mode = "IDLE"
                self._mini2["mode"] = mode
            time.sleep(period)

    def _loop_maxi_05hz(self):
        """0.5 Hz: hier keine zusätzliche Physik nötig; Platzhalter für später."""
        period = 2.0
        while not self._stop_evt.is_set():
            time.sleep(period)

    # ---------- CSV-Programm (wie beim echten) ----------
    def _run_program(self, path: str):
        """
        CSV mit Spalten:
        - Haltezeit_ms
        - Druck_kPa
        - Butan_nlmin
        - N2_nlmin
        - CO2_nlmin
        """
        self._prog_stop.clear()
        with open(path, newline="", encoding="utf-8") as f:
            rdr = csv.DictReader(f)
            cols = {c.lower(): c for c in (rdr.fieldnames or [])}
            required = ["haltezeit_ms", "druck_kpa", "butan_nlmin", "n2_nlmin", "co2_nlmin"]
            for r in required:
                if r not in cols:
                    raise tango.DevFailed(f"CSV fehlt Spalte: {r}")
            for row in rdr:
                if self._prog_stop.is_set():
                    break
                hold_ms = float(row[cols["haltezeit_ms"]])
                p_kpa   = float(row[cols["druck_kpa"]])
                butan   = float(row[cols["butan_nlmin"]])
                n2      = float(row[cols["n2_nlmin"]])
                co2     = float(row[cols["co2_nlmin"]])
                with self._lock:
                    self._mini2["setpoint_kpa"] = p_kpa
                    self._maxi["mfc_butan"]["setpoint"] = max(0.0, butan)
                    self._maxi["mfc_n2"]["setpoint"]    = max(0.0, n2)
                    self._maxi["mfc_co2"]["setpoint"]   = max(0.0, co2)
                t_end = time.perf_counter() + max(0.0, hold_ms) / 1000.0
                while not self._prog_stop.is_set() and time.perf_counter() < t_end:
                    time.sleep(0.01)

    # =================== Attribute (READ/WRITE) ===================
    # --- Drücke Probenvolumen ---
    def read_Mini1Pressure(self, attr):
        with self._lock:
            # im echten Code stand "latest_combined_pressure";
            # hier geben wir den simulierten Druck zurück
            attr.set_value(float(self._mini1["pressure_kpa"]))

    def read_P21(self, attr):
        with self._lock:
            attr.set_value(float(self._mini2["p21_kpa"] or float("nan")))

    def read_P20(self, attr):
        with self._lock:
            attr.set_value(float(self._mini2["p20_kpa"] or float("nan")))

    # --- High/Low Pressure Volumes ---
    def read_P11(self, attr):
        with self._lock:
            attr.set_value(float(self._maxi["p11"] or float("nan")))

    def read_P31(self, attr):
        with self._lock:
            attr.set_value(float(self._maxi["p31"] or float("nan")))

    # --- Setpoint ---
    def read_SetpointKpa(self, attr):
        with self._lock:
            attr.set_value(float(self._mini2["setpoint_kpa"]))

    def write_SetpointKpa(self, attr):
        v = float(attr.get_write_value())
        with self._lock:
            self._mini2["setpoint_kpa"] = v

    # --- Gas Leak ---
    def read_GasLeak(self, attr):
        with self._lock:
            attr.set_value(int(self._maxi["leak_sensor"]))

    # --- Hilfsfunktionen für Schalter ---
    def _get_sw(self, key: str) -> bool:
        v = self._maxi.get(key)
        if isinstance(v, str):
            return v.upper() == "ON"
        if isinstance(v, (int, float)):
            return int(v) != 0
        return bool(v)

    def _set_sw(self, key: str, on: bool):
        self._maxi[key] = bool(on)

    # --- Ventilzustände (MAXI) ---
    def read_V1(self, attr):  attr.set_value(self._get_sw("v1_power"))
    def write_V1(self, attr): self._set_sw("v1_power", attr.get_write_value())

    def read_V2(self, attr):  attr.set_value(self._get_sw("v2_power"))
    def write_V2(self, attr): self._set_sw("v2_power", attr.get_write_value())

    def read_V3(self, attr):  attr.set_value(self._get_sw("v3_power"))
    def write_V3(self, attr): self._set_sw("v3_power", attr.get_write_value())

    def read_V4(self, attr):  attr.set_value(self._get_sw("v4_power"))
    def write_V4(self, attr): self._set_sw("v4_power", attr.get_write_value())

    def read_FanIn(self, attr):  attr.set_value(self._get_sw("fan_in_power"))
    def write_FanIn(self, attr):
        v = bool(attr.get_write_value())
        self._set_sw("fan_in_power", v)

    def read_FanOut(self, attr): attr.set_value(self._get_sw("fan_out_power"))
    def write_FanOut(self, attr):
        v = bool(attr.get_write_value())
        self._set_sw("fan_out_power", v)

    def read_VS3(self, attr):  attr.set_value(self._get_sw("vs3_power"))
    def write_VS3(self, attr): self._set_sw("vs3_power", attr.get_write_value())

    # --- vp1/vp2 & Mode (MINI2) ---
    def read_vp1PWM(self, attr):
        with self._lock:
            attr.set_value(int(self._mini2["pwm1"]))

    def write_vp1PWM(self, attr):
        v = int(attr.get_write_value())
        with self._lock:
            self._mini2["pwm1"] = max(0, min(255, v))

    def read_vp2PWM(self, attr):
        with self._lock:
            attr.set_value(int(self._mini2["pwm2"]))

    def write_vp2PWM(self, attr):
        v = int(attr.get_write_value())
        with self._lock:
            self._mini2["pwm2"] = max(0, min(255, v))

    def read_Mini2Mode(self, attr):
        with self._lock:
            # String wie im echten: "AUTO"/"MANUAL"/"IDLE"
            attr.set_value(str(self._mini2["mode"]))

    # --- Flüsse (Soll) ---
    def read_FlowButan(self, attr):
        with self._lock:
            attr.set_value(float(self._maxi["mfc_butan"]["setpoint"]))

    def write_FlowButan(self, attr):
        v = max(0.0, float(attr.get_write_value()))
        with self._lock:
            self._maxi["mfc_butan"]["setpoint"] = v

    def read_FlowN2(self, attr):
        with self._lock:
            attr.set_value(float(self._maxi["mfc_n2"]["setpoint"]))

    def write_FlowN2(self, attr):
        v = max(0.0, float(attr.get_write_value()))
        with self._lock:
            self._maxi["mfc_n2"]["setpoint"] = v

    def read_FlowCO2(self, attr):
        with self._lock:
            attr.set_value(float(self._maxi["mfc_co2"]["setpoint"]))

    def write_FlowCO2(self, attr):
        v = max(0.0, float(attr.get_write_value()))
        with self._lock:
            self._maxi["mfc_co2"]["setpoint"] = v

    # =================== Commands ===================
    def GetStatus(self):
        """kompatibel zu deinem echten Server: JSON-Status zurückgeben."""
        with self._lock:
            st = {
                "ts": _now_ts(),
                "mini1": {"latest_combined_pressure": self._mini1["pressure_kpa"]},
                "mini2": {
                    "p20_kpa": self._mini2["p20_kpa"],
                    "p21_kpa": self._mini2["p21_kpa"],
                    "setpoint_kpa": self._mini2["setpoint_kpa"],
                    "pressure_kpa": self._mini1["pressure_kpa"],
                    "pwm1": self._mini2["pwm1"],
                    "pwm2": self._mini2["pwm2"],
                    "mode": self._mini2["mode"],
                },
                "maxi": {
                    "p11": self._maxi["p11"], "p31": self._maxi["p31"],
                    "v1_power": self._maxi["v1_power"], "v2_power": self._maxi["v2_power"],
                    "v3_power": self._maxi["v3_power"], "v4_power": self._maxi["v4_power"],
                    "fan_in_power": self._maxi["fan_in_power"], "fan_out_power": self._maxi["fan_out_power"],
                    "vs3_power": self._maxi["vs3_power"], "heater_pwm": self._maxi["heater_pwm"],
                    "mfc_butan": dict(self._maxi["mfc_butan"]),
                    "mfc_n2":    dict(self._maxi["mfc_n2"]),
                    "mfc_co2":   dict(self._maxi["mfc_co2"]),
                    "leak_sensor": self._maxi["leak_sensor"],
                }
            }
        return json.dumps(st, default=str)

    def StorageVolume(self, open_):
        with self._lock:
            self._maxi["v1_power"] = bool(open_)
            self._maxi["v2_power"] = bool(open_)

    def ActivateVac(self, enable):
        # Im Dummy nur V4 toggeln, echte Pumpe gibt's nicht
        with self._lock:
            self._maxi["v4_power"] = bool(enable)

    def PurgeHighPressure(self):
        # Dummy-Sequenz: kurz V3 öffnen, dann zu
        with self._lock:
            self._maxi["mfc_butan"]["setpoint"] = 0.0
            self._maxi["mfc_n2"]["setpoint"] = 0.0
            self._maxi["mfc_co2"]["setpoint"] = 0.0
            self._maxi["v1_power"] = False
            self._maxi["v2_power"] = False
            self._maxi["v3_power"] = True
        time.sleep(3.0)
        with self._lock:
            self._maxi["v3_power"] = False

    def RunProgramFile(self, path):
        if self._prog_thread and self._prog_thread.is_alive():
            raise tango.DevFailed("Programm läuft bereits")
        self._prog_stop.clear()

        def th():
            try:
                self._run_program(path)
                self.set_status("Programm fertig (Dummy)")
            except Exception as e:
                self.set_state(tango.DevState.FAULT)
                self.set_status(f"Programmfehler (Dummy): {e}")

        self._prog_thread = threading.Thread(target=th, daemon=True)
        self._prog_thread.start()
        self.set_status(f"Programm gestartet (Dummy): {path}")

    def StopProgram(self):
        self._prog_stop.set()
        if self._prog_thread and self._prog_thread.is_alive():
            self._prog_thread.join(timeout=0.5)
        self._prog_thread = None

    def SetLogging(self, enabled):
        # Dummy: keine Datei – nur Statusmeldung
        self.set_status(f"Logging {'ON' if enabled else 'OFF'} (Dummy)")

    def SetLoggingPeriodMs(self, period_ms):
        # Dummy: wird nicht verwendet
        self.set_status(f"LoggingPeriodMs={int(period_ms)} (Dummy)")


class VaderClass(tango.DeviceClass):
    # Properties wie im echten Server (Defaults ok, Dummy nutzt sie nicht)
    device_property_list = {
        "Mini1Port":      [tango.DevString, "Serial port MINI1", []],
        "Mini2Port":      [tango.DevString, "Serial port MINI2", []],
        "MaxiPort":       [tango.DevString, "Serial port MAXI",  []],
        "Mini1MaxSamples":[tango.DevLong,   "Ringpuffer MINI1",  [300000]],
        "LoggingEnabled": [tango.DevBoolean,"Logger Autostart",  [True]],
        "LoggingPeriodMs":[tango.DevLong,   "Logger Periode ms", [5]],
        "LogDirectory":   [tango.DevString, "Log-Verzeichnis",   ["/tmp"]],
    }

    cmd_list = {
        "GetStatus":           [[tango.DevVoid],[tango.DevString]],
        "StorageVolume":       [[tango.DevBoolean],[tango.DevVoid]],
        "ActivateVac":         [[tango.DevBoolean],[tango.DevVoid]],
        "PurgeHighPressure":   [[tango.DevVoid],[tango.DevVoid]],
        "RunProgramFile":      [[tango.DevString],[tango.DevVoid]],
        "StopProgram":         [[tango.DevVoid],[tango.DevVoid]],
        "SetLogging":          [[tango.DevBoolean],[tango.DevVoid]],
        "SetLoggingPeriodMs":  [[tango.DevLong],[tango.DevVoid]],
    }

    attr_list = {
        # Drücke Probenvolumen
        "Mini1Pressure": [[tango.DevDouble, tango.SCALAR, tango.READ]],
        "P21":           [[tango.DevDouble, tango.SCALAR, tango.READ]],
        "P20":           [[tango.DevDouble, tango.SCALAR, tango.READ]],
        # High-/Low-Pressure Volumes
        "P11":           [[tango.DevDouble, tango.SCALAR, tango.READ]],
        "P31":           [[tango.DevDouble, tango.SCALAR, tango.READ]],
        # Setpoint/Leak
        "SetpointKpa":   [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
        "GasLeak":       [[tango.DevLong,   tango.SCALAR, tango.READ]],
        # Ventile MAXI
        "V1":            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
        "V2":            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
        "V3":            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
        "V4":            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
        "FanIn":         [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
        "FanOut":        [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
        "VS3":           [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
        # MINI2 Ventile (PWM) + Modus
        "vp1PWM":        [[tango.DevLong,    tango.SCALAR, tango.READ_WRITE]],
        "vp2PWM":        [[tango.DevLong,    tango.SCALAR, tango.READ_WRITE]],
        "Mini2Mode":     [[tango.DevString,  tango.SCALAR, tango.READ]],
        # MFC-Flüsse (Soll)
        "FlowButan":     [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
        "FlowN2":        [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
        "FlowCO2":       [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
    }


def main():
    try:
        util = tango.Util(sys.argv)
        util.add_class(VaderClass, Vader, "Vader")  # ⇐ exakt wie dein Beispiel
        util.server_init()
        util.server_run()
    except tango.DevFailed as e:
        print("DevFailed:", e)
    except Exception as e:
        print("Exception:", e)


if __name__ == "__main__":
    main()