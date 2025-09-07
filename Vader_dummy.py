#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, time, json, threading, random
import PyTango as tango


# -------- Hilfsfunktion für JSON-Programm --------
def read_program_from_json(filename):
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data


# ====================== Device ======================
class Vader(tango.Device_4Impl):
    """
    Dummy-Server: bedient sich wie der echte VADER,
    aber simuliert nur (kein Hardware-Zugriff).
    """

    def __init__(self, cl, name):
        tango.Device_4Impl.__init__(self, cl, name)
        self._lock = threading.Lock()
        # Statusvariablen
        self._io = {"V1": False, "V2": False, "V3": False, "Fans": False, "GasLeak": False}
        self._mfc = {"butan": {"flow": 0.0, "total": 0.0},
                     "co2":   {"flow": 0.0, "total": 0.0},
                     "n2":    {"flow": 0.0, "total": 0.0}}
        self._mini1_pressure = 101.3
        self._setpoint = 0.0
        self._vp1 = 0
        self._vp2 = 0
        self._mode = "IDLE"
        self._prog_thread = None
        self._prog_stop = threading.Event()
        self._stop_evt = threading.Event()
        self._last_integrate = time.time()
        Vader.init_device(self)

    # ---------- Lifecycle ----------
    def init_device(self):
        self.set_state(tango.DevState.INIT)
        try:
            self._stop_evt.clear()
            # Simulations-Threads
            self._t_mini1 = threading.Thread(target=self._loop_mini1, daemon=True)
            self._t_status = threading.Thread(target=self._loop_status, daemon=True)
            self._t_mini1.start()
            self._t_status.start()
            self.set_state(tango.DevState.ON)
            self.set_status("Dummy läuft.")
        except Exception as e:
            self.set_state(tango.DevState.FAULT)
            self.set_status(f"Init-Fehler: {e}")

    def delete_device(self):
        self._stop_evt.set()
        for t in (getattr(self, "_t_mini1", None), getattr(self, "_t_status", None)):
            if t and t.is_alive():
                t.join(timeout=0.5)
        self.set_state(tango.DevState.OFF)
        self.set_status("Dummy gestoppt.")

    # ---------- Simulations-Threads ----------
    def _loop_mini1(self):
        """20 Hz Drucksimulation + Flows integrieren"""
        period = 1/20
        while not self._stop_evt.is_set():
            t0 = time.time()
            with self._lock:
                dt = t0 - self._last_integrate
                self._last_integrate = t0
                # Druck gleicht sich langsam an Setpoint an
                self._mini1_pressure += (self._setpoint - self._mini1_pressure) * dt/2.0
                self._mini1_pressure += random.gauss(0, 0.01)
                # Totalisatoren integrieren
                for gas in self._mfc.values():
                    gas["total"] += gas["flow"] * (dt/60.0)
                # GasLeak: zufällig
                self._io["GasLeak"] = (random.random() < 0.01)
            time.sleep(period)

    def _loop_status(self):
        """0.5 Hz: Mini2-Mode ableiten"""
        while not self._stop_evt.is_set():
            with self._lock:
                if abs(self._setpoint - self._mini1_pressure) > 2.0:
                    self._mode = "AUTOMATIC"
                elif (self._vp1 + self._vp2) > 0:
                    self._mode = "MANUAL"
                else:
                    self._mode = "IDLE"
            time.sleep(2.0)

    # ---------- Attribute ----------
    def read_Storage(self, attr):
        with self._lock:
            attr.set_value(self._io["V1"] and self._io["V2"])

    def read_V1(self, attr):  attr.set_value(self._io["V1"])
    def read_V2(self, attr):  attr.set_value(self._io["V2"])
    def read_V3(self, attr):  attr.set_value(self._io["V3"])
    def read_Fans(self, attr): attr.set_value(self._io["Fans"])
    def read_GasLeak(self, attr): attr.set_value(self._io["GasLeak"])
    def read_FlowButan(self, attr): attr.set_value(self._mfc["butan"]["flow"])
    def read_FlowCO2(self, attr): attr.set_value(self._mfc["co2"]["flow"])
    def read_FlowN2(self, attr): attr.set_value(self._mfc["n2"]["flow"])
    def read_TotalButan(self, attr): attr.set_value(self._mfc["butan"]["total"])
    def read_TotalCO2(self, attr): attr.set_value(self._mfc["co2"]["total"])
    def read_TotalN2(self, attr): attr.set_value(self._mfc["n2"]["total"])
    def read_setPoint_pressure(self, attr): attr.set_value(self._setpoint)
    def read_vp1(self, attr): attr.set_value(self._vp1)
    def read_vp2(self, attr): attr.set_value(self._vp2)

    # ---------- Commands ----------
    def SetStorage(self, open_):  self._io["V1"] = self._io["V2"] = bool(open_)
    def SetV3(self, open_):       self._io["V3"] = bool(open_)
    def SetFans(self, on):        self._io["Fans"] = bool(on)
    def SetFlowButan(self, flow): self._mfc["butan"]["flow"] = max(0.0, flow)
    def SetFlowCO2(self, flow):   self._mfc["co2"]["flow"]   = max(0.0, flow)
    def SetFlowN2(self, flow):    self._mfc["n2"]["flow"]    = max(0.0, flow)
    def SetTotalButan(self, val): self._mfc["butan"]["total"] = max(0.0, val)
    def SetTotalCO2(self, val):   self._mfc["co2"]["total"]   = max(0.0, val)
    def SetTotalN2(self, val):    self._mfc["n2"]["total"]    = max(0.0, val)
    def SetSetPointPressure(self, kpa): self._setpoint = float(kpa)
    def SetVP1(self, val):        self._vp1 = max(0, min(255, int(val)))
    def SetVP2(self, val):        self._vp2 = max(0, min(255, int(val)))

    def RunProgram(self, path):
        if self._prog_thread and self._prog_thread.is_alive():
            raise tango.DevFailed("Programm läuft bereits")
        self._prog_stop.clear()
        def th():
            prog = read_program_from_json(path)
            for step in prog:
                if self._prog_stop.is_set(): break
                t = step.get("type")
                if t == "Normal":
                    self.SetSetPointPressure(step["target_pressure"])
                elif t == "Ramp":
                    self.SetSetPointPressure(step["end_pressure"])
                elif t == "delay":
                    time.sleep(step["delay_time"]/1000.0)
            self.set_status("Programm abgeschlossen")
        self._prog_thread = threading.Thread(target=th, daemon=True)
        self._prog_thread.start()

    def StopProgram(self):
        self._prog_stop.set()
        if self._prog_thread and self._prog_thread.is_alive():
            self._prog_thread.join(timeout=0.3)
        self._prog_thread = None


# ====================== DeviceClass ======================
class VaderClass(tango.DeviceClass):
    cmd_list = {
        "SetStorage":          [[tango.DevBoolean],[tango.DevVoid]],
        "SetV3":               [[tango.DevBoolean],[tango.DevVoid]],
        "SetFans":             [[tango.DevBoolean],[tango.DevVoid]],
        "SetFlowButan":        [[tango.DevDouble], [tango.DevVoid]],
        "SetFlowCO2":          [[tango.DevDouble], [tango.DevVoid]],
        "SetFlowN2":           [[tango.DevDouble], [tango.DevVoid]],
        "SetTotalButan":       [[tango.DevDouble], [tango.DevVoid]],
        "SetTotalCO2":         [[tango.DevDouble], [tango.DevVoid]],
        "SetTotalN2":          [[tango.DevDouble], [tango.DevVoid]],
        "SetSetPointPressure": [[tango.DevDouble], [tango.DevVoid]],
        "SetVP1":              [[tango.DevLong],   [tango.DevVoid]],
        "SetVP2":              [[tango.DevLong],   [tango.DevVoid]],
        "RunProgram":          [[tango.DevString], [tango.DevVoid]],
        "StopProgram":         [[tango.DevVoid],   [tango.DevVoid]],
    }

    attr_list = {
        "Storage":          [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        "V1":               [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        "V2":               [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        "V3":               [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        "Fans":             [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        "GasLeak":          [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        "FlowButan":        [[tango.DevDouble,  tango.SCALAR, tango.READ]],
        "FlowCO2":          [[tango.DevDouble,  tango.SCALAR, tango.READ]],
        "FlowN2":           [[tango.DevDouble,  tango.SCALAR, tango.READ]],
        "TotalButan":       [[tango.DevDouble,  tango.SCALAR, tango.READ]],
        "TotalCO2":         [[tango.DevDouble,  tango.SCALAR, tango.READ]],
        "TotalN2":          [[tango.DevDouble,  tango.SCALAR, tango.READ]],
        "setPoint_pressure":[[tango.DevDouble,  tango.SCALAR, tango.READ]],
        "vp1":              [[tango.DevLong,    tango.SCALAR, tango.READ]],
        "vp2":              [[tango.DevLong,    tango.SCALAR, tango.READ]],
    }


# ====================== main ======================
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