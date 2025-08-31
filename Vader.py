#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os, csv, json, time, threading, traceback
import PyTango as tango

# === Importiere deinen Driver ===
from vader_driver import VaderDeviceDriver  # ggf. Pfad anpassen

# ====== Hilfsfunktionen ======
def now_ts():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

# ====== Tango Device ======
class Vader(tango.Device_4Impl):
    """
    Tango-Device für das VADER-System.
    Bildet Drücke, Ventilzustände, Gasflüsse, Leck und Setpoint ab.
    Führt CSV-Programme aus und loggt kontinuierlich (default: 5 ms).
    """

    def __init__(self, cl, name):
        tango.Device_4Impl.__init__(self, cl, name)
        self._drv = None
        self._status = {}
        self._status_ts = 0.0
        self._status_lock = threading.Lock()
        self._log_thread = None
        self._log_stop = threading.Event()
        self._prog_thread = None
        self._prog_stop = threading.Event()
        Vader.init_device(self)

    # ---------- Lifecycle ----------
    def init_device(self):
        self.get_device_properties(self.get_device_class())
        self.set_state(tango.DevState.INIT)
        try:
            self._drv = VaderDeviceDriver(
                self.Mini1Port, self.Mini2Port, self.MaxiPort, self.Mini1MaxSamples
            )
            self._ensure_log_dir()
            # Logger ggf. auto-start
            if self.LoggingEnabled:
                self._start_logger()
            self.set_state(tango.DevState.ON)
            self.set_status("Connected.")
        except Exception as e:
            self.set_state(tango.DevState.FAULT)
            self.set_status(f"Init failed: {e}\n{traceback.format_exc()}")

    def delete_device(self):
        try:
            self._stop_logger()
        except Exception:
            pass
        try:
            if self._drv: self._drv.close()
        except Exception:
            pass

    # ---------- Status/Cache ----------
    def _refresh_status(self, max_age=0.05):
        with self._status_lock:
            now = time.time()
            if (now - self._status_ts) < max_age and self._status:
                return self._status
            try:
                # Triggert die Geräte-Statusantworten (Treiber liest asynchron)
                self._drv.mini2.request_status()
                self._drv.maxi.request_status()
                time.sleep(0.01)
                self._status = self._drv.get_all_status(mini1_last_n=1)
                self._status_ts = now
                self.set_state(tango.DevState.ON)
                self.set_status("OK")
            except Exception as e:
                self.set_state(tango.DevState.ALARM)
                self.set_status(f"Read error: {e}")
            return self._status

    def _g(self, default=None, *path):
        d = self._status
        for p in path:
            if isinstance(d, dict) and p in d: d = d[p]
            else: return default
        return d

    # ---------- Logger ----------
    def _ensure_log_dir(self):
        if not self.LogDirectory:
            self.LogDirectory = "/tmp"
        os.makedirs(self.LogDirectory, exist_ok=True)

    def _start_logger(self):
        if self._log_thread and self._log_thread.is_alive():
            return
        self._log_stop.clear()
        fname = os.path.join(
            self.LogDirectory, time.strftime("vader_%Y%m%d_%H%M%S.csv")
        )
        period_s = max(1, int(self.LoggingPeriodMs)) / 1000.0

        def run():
            # Header
            fields = [
                "timestamp",
                "mini1_pressure_kpa","mini1_p21_kpa","mini1_p20_kpa",
                "mini2_setpoint_kpa","mini2_pressure_kpa","mini2_mode",
                "p11_kpa","p31_kpa",
                "v1","v2","v3","v4","fan_in","fan_out","vs3","heater_pwm",
                "vp1_pwm","vp2_pwm",
                "mfc_butan_set","mfc_n2_set","mfc_co2_set",
                "mfc_butan_flow","mfc_n2_flow","mfc_co2_flow",
                "leak_sensor"
            ]
            with open(fname, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                next_t = time.perf_counter()
                while not self._log_stop.is_set():
                    try:
                        st = self._refresh_status(max_age=0.0)
                        mx = st.get("maxi", {})
                        m2 = st.get("mini2", {})
                        row = {
                            "timestamp": now_ts(),
                            "mini1_pressure_kpa": st.get("mini1", {}).get("latest_combined_pressure"),
                            "mini1_p21_kpa": m2.get("p21_kpa"),
                            "mini1_p20_kpa": m2.get("p20_kpa"),
                            "mini2_setpoint_kpa": m2.get("setpoint_kpa"),
                            "mini2_pressure_kpa": m2.get("pressure_kpa"),
                            "mini2_mode": "MANUAL" if m2.get("pwm1") is not None and m2.get("pwm2") is not None and m2.get("kp") is not None and m2.get("leak_sensor") is not None and (m2.get("setpoint_index") is not None) and (m2.get("setpoint_kpa") is not None) and (m2.get("pressure_kpa") is not None) and False else ("AUTO" if True else "UNKNOWN"),
                            "p11_kpa": mx.get("p11"),
                            "p31_kpa": mx.get("p31"),
                            "v1": mx.get("v1_power"),
                            "v2": mx.get("v2_power"),
                            "v3": mx.get("v3_power"),
                            "v4": mx.get("v4_power"),
                            "fan_in": mx.get("fan_in_power"),
                            "fan_out": mx.get("fan_out_power"),
                            "vs3": mx.get("vs3_power"),
                            "heater_pwm": mx.get("heater_pwm"),
                            "vp1_pwm": m2.get("pwm1"),
                            "vp2_pwm": m2.get("pwm2"),
                            "mfc_butan_set":  (mx.get("mfc_butan",{}) or {}).get("setpoint"),
                            "mfc_n2_set":     (mx.get("mfc_n2",{}) or {}).get("setpoint"),
                            "mfc_co2_set":    (mx.get("mfc_co2",{}) or {}).get("setpoint"),
                            "mfc_butan_flow": (mx.get("mfc_butan",{}) or {}).get("flow"),
                            "mfc_n2_flow":    (mx.get("mfc_n2",{}) or {}).get("flow"),
                            "mfc_co2_flow":   (mx.get("mfc_co2",{}) or {}).get("flow"),
                            "leak_sensor": mx.get("leak_sensor"),
                        }
                        w.writerow(row)
                    except Exception:
                        pass
                    next_t += period_s
                    sleep = next_t - time.perf_counter()
                    if sleep > 0: time.sleep(sleep)

        self._log_thread = threading.Thread(target=run, daemon=True)
        self._log_thread.start()
        self.set_status(f"Logging to {fname} every {self.LoggingPeriodMs} ms")

    def _stop_logger(self):
        self._log_stop.set()
        if self._log_thread:
            self._log_thread.join(timeout=1.0)
        self._log_thread = None

    # ---------- CSV-Programm ----------
    def _run_program(self, path: str):
        self._prog_stop.clear()
        with open(path, newline="") as f:
            rdr = csv.DictReader(f)
            # erlaubte Headernamen
            cols = {c.lower(): c for c in rdr.fieldnames or []}
            required = ["haltezeit_ms","druck_kpa","butan_nlmin","n2_nlmin","co2_nlmin"]
            for r in required:
                if r not in cols:
                    raise tango.DevFailed(f"CSV fehlt Spalte: {r}")
            for row in rdr:
                if self._prog_stop.is_set(): break
                try:
                    hold_ms = float(row[cols["haltezeit_ms"]])
                    p_kpa   = float(row[cols["druck_kpa"]])
                    butan   = float(row[cols["butan_nlmin"]])
                    n2      = float(row[cols["n2_nlmin"]])
                    co2     = float(row[cols["co2_nlmin"]])
                except Exception as e:
                    raise tango.DevFailed(f"CSV Parsefehler: {e}")

                # Zielwerte setzen
                self._drv.setpoint_pressure(p_kpa)
                self._drv.maxi.set_flow_butan(butan)
                self._drv.maxi.set_flow_n2(n2)
                self._drv.maxi.set_flow_co2(co2)

                # Haltezeit abwarten (in kleinen Schritten, damit Stop greift)
                t_end = time.perf_counter() + max(0.0, hold_ms)/1000.0
                while not self._prog_stop.is_set() and time.perf_counter() < t_end:
                    time.sleep(0.01)

    # =================== Attribute ===================
    # Probenvolumen
    def read_Mini1Pressure(self, attr):
        st = self._refresh_status()
        attr.set_value(float(st.get("mini1",{}).get("latest_combined_pressure") or float("nan")))

    def read_P21(self, attr):
        st = self._refresh_status()
        attr.set_value(float(st.get("mini2",{}).get("p21_kpa") or float("nan")))

    def read_P20(self, attr):
        st = self._refresh_status()
        attr.set_value(float(st.get("mini2",{}).get("p20_kpa") or float("nan")))

    # High/Low Pressure Volumes
    def read_P11(self, attr):
        st = self._refresh_status()
        attr.set_value(float(st.get("maxi",{}).get("p11") or float("nan")))

    def read_P31(self, attr):
        st = self._refresh_status()
        attr.set_value(float(st.get("maxi",{}).get("p31") or float("nan")))

    # Setpoint
    def read_SetpointKpa(self, attr):
        st = self._refresh_status()
        attr.set_value(float(st.get("mini2",{}).get("setpoint_kpa") or float("nan")))

    def write_SetpointKpa(self, attr):
        self._drv.setpoint_pressure(float(attr.get_write_value()))
        self._refresh_status(max_age=0.0)

    # Gas Leak
    def read_GasLeak(self, attr):
        st = self._refresh_status()
        val = st.get("maxi",{}).get("leak_sensor")
        attr.set_value(int(val if isinstance(val,int) else 0))

    # Ventilzustände (MAXI)
    def _read_sw(self, key):
        st = self._refresh_status()
        v = st.get("maxi",{}).get(key)
        if isinstance(v,str): return v.upper()=="ON"
        if isinstance(v,(int,float)): return int(v)!=0
        return False

    def _write_valve(self, addr_name, on):
        address = {
            "v1_power":1,"v2_power":2,"v3_power":3,"v4_power":4,
            "fan_in_power":8,"fan_out_power":9,"vs3_power":12
        }[addr_name]
        if addr_name.startswith("v"):
            self._drv.maxi.set_valve(address, bool(on))
        else:
            self._drv.maxi.set_component(address, 1 if on else 0)
        self._refresh_status(max_age=0.0)

    def read_V1(self, attr): attr.set_value(self._read_sw("v1_power"))
    def write_V1(self, attr): self._write_valve("v1_power", attr.get_write_value())
    def read_V2(self, attr): attr.set_value(self._read_sw("v2_power"))
    def write_V2(self, attr): self._write_valve("v2_power", attr.get_write_value())
    def read_V3(self, attr): attr.set_value(self._read_sw("v3_power"))
    def write_V3(self, attr): self._write_valve("v3_power", attr.get_write_value())
    def read_V4(self, attr): attr.set_value(self._read_sw("v4_power"))
    def write_V4(self, attr): self._write_valve("v4_power", attr.get_write_value())
    def read_FanIn(self, attr): attr.set_value(self._read_sw("fan_in_power"))
    def write_FanIn(self, attr): self._write_valve("fan_in_power", attr.get_write_value())
    def read_FanOut(self, attr): attr.set_value(self._read_sw("fan_out_power"))
    def write_FanOut(self, attr): self._write_valve("fan_out_power", attr.get_write_value())
    def read_VS3(self, attr): attr.set_value(self._read_sw("vs3_power"))
    def write_VS3(self, attr): self._write_valve("vs3_power", attr.get_write_value())

    # vp1/vp2 & Mode (MINI2)
    def read_vp1PWM(self, attr):
        st = self._refresh_status(); attr.set_value(int(st.get("mini2",{}).get("pwm1") or 0))
    def write_vp1PWM(self, attr):
        self._drv.mini2.set_manual_vp1(int(attr.get_write_value())); self._refresh_status(max_age=0.0)
    def read_vp2PWM(self, attr):
        st = self._refresh_status(); attr.set_value(int(st.get("mini2",{}).get("pwm2") or 0))
    def write_vp2PWM(self, attr):
        self._drv.mini2.set_manual_vp2(int(attr.get_write_value())); self._refresh_status(max_age=0.0)
    def read_Mini2Mode(self, attr):
        # 0=Auto, 1=Manual (wir lesen das nicht direkt -> heuristisch via PWM write erlaubt)
        # Für Klarheit expose ich einen String: "AUTO"/"MANUAL"
        attr.set_value("AUTO")  # wenn du den Zustand aus ADR_STATE bekommst, hier umsetzen

    # Flüsse
    def read_FlowButan(self, attr):
        st=self._refresh_status(); attr.set_value(float((st.get("maxi",{}).get("mfc_butan",{}) or {}).get("setpoint") or 0.0))
    def write_FlowButan(self, attr):
        self._drv.maxi.set_flow_butan(float(attr.get_write_value())); self._refresh_status(max_age=0.0)
    def read_FlowN2(self, attr):
        st=self._refresh_status(); attr.set_value(float((st.get("maxi",{}).get("mfc_n2",{}) or {}).get("setpoint") or 0.0))
    def write_FlowN2(self, attr):
        self._drv.maxi.set_flow_n2(float(attr.get_write_value())); self._refresh_status(max_age=0.0)
    def read_FlowCO2(self, attr):
        st=self._refresh_status(); attr.set_value(float((st.get("maxi",{}).get("mfc_co2",{}) or {}).get("setpoint") or 0.0))
    def write_FlowCO2(self, attr):
        self._drv.maxi.set_flow_co2(float(attr.get_write_value())); self._refresh_status(max_age=0.0)

    # =================== Commands ===================
    def GetStatus(self):
        self._refresh_status(max_age=0.0)
        return json.dumps(self._status, default=str)

    def StorageVolume(self, open_):
        """V1+V2 gemeinsam schalten (Probenvolumen <-> High Pressure Volume)."""
        self._drv.storage_volume(bool(open_)); self._refresh_status(max_age=0.0)

    def ActivateVac(self, enable):
        """Vakuumpumpe (V4) ein/aus."""
        self._drv.activate_vac(bool(enable)); self._refresh_status(max_age=0.0)

    def PurgeHighPressure(self):
        """
        Gaswechsel im High-Pressure Volume:
        1) MFCs schließen, V1/V2 schließen.
        2) V3 öffnen (Verbindung zu Vakuum 1)
        3) 3 s warten, damit sich der Gasstrom einstellt.
        """
        # MFCs zu
        self._drv.maxi.set_flow_butan(0.0)
        self._drv.maxi.set_flow_n2(0.0)
        self._drv.maxi.set_flow_co2(0.0)
        # V1/V2 zu
        self._drv.maxi.set_valve(1, False); self._drv.maxi.set_valve(2, False)
        # V3 auf, 3 s warten
        self._drv.maxi.set_valve(3, True)
        time.sleep(3.0)
        # V3 wieder zu
        self._drv.maxi.set_valve(3, False)
        self._refresh_status(max_age=0.0)

    def RunProgramFile(self, path):
        """CSV mit Spalten: Haltezeit_ms, Druck_kPa, Butan_nlmin, N2_nlmin, CO2_nlmin"""
        if self._prog_thread and self._prog_thread.is_alive():
            raise tango.DevFailed("Programm läuft bereits")
        self._prog_stop.clear()
        def th():
            try:
                self._run_program(path)
                self.set_status("Programm fertig")
            except Exception as e:
                self.set_state(tango.DevState.FAULT)
                self.set_status(f"Programmfehler: {e}")
        self._prog_thread = threading.Thread(target=th, daemon=True)
        self._prog_thread.start()
        self.set_status(f"Programm gestartet: {path}")

    def StopProgram(self):
        self._prog_stop.set()

    def SetLogging(self, enabled):
        if enabled:
            self._start_logger()
        else:
            self._stop_logger()

    def SetLoggingPeriodMs(self, period_ms):
        self.LoggingPeriodMs = int(period_ms)
        if self._log_thread and self._log_thread.is_alive():
            # Neustarten, damit neue Periode greift
            self._stop_logger(); self._start_logger()

# ====== DeviceClass ======
class VaderClass(tango.DeviceClass):
    # Device Properties
    device_property_list = {
        "Mini1Port":      [tango.DevString, "Serial port MINI1", []],
        "Mini2Port":      [tango.DevString, "Serial port MINI2", []],
        "MaxiPort":       [tango.DevString, "Serial port MAXI",  []],
        "Mini1MaxSamples":[tango.DevLong,   "Ringpuffer MINI1",  [300000]],
        "LoggingEnabled": [tango.DevBoolean,"Logger Autostart",  [True]],
        "LoggingPeriodMs":[tango.DevLong,   "Logger Periode ms", [5]],
        "LogDirectory":   [tango.DevString, "Log-Verzeichnis",   ["/tmp"]],
    }

    # Commands
    cmd_list = {
        "GetStatus":       [[tango.DevVoid],[tango.DevString]],
        "StorageVolume":   [[tango.DevBoolean],[tango.DevVoid]],
        "ActivateVac":     [[tango.DevBoolean],[tango.DevVoid]],
        "PurgeHighPressure":[[tango.DevVoid],[tango.DevVoid]],
        "RunProgramFile":  [[tango.DevString],[tango.DevVoid]],
        "StopProgram":     [[tango.DevVoid],[tango.DevVoid]],
        "SetLogging":      [[tango.DevBoolean],[tango.DevVoid]],
        "SetLoggingPeriodMs": [[tango.DevLong],[tango.DevVoid]],
    }

    # Attributes
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

# ====== main ======
def main():
    try:
        util = tango.Util(sys.argv)
        util.add_class(VaderClass, Vader, "Vader")
        util.server_init()
        util.server_run()
    except tango.DevFailed as e:
        print("DevFailed:", e)
    except Exception as e:
        print("Exception:", e)

if __name__ == "__main__":
    main()