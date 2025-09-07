#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
from datetime import datetime
from typing import Optional, Dict, Any, List, Union

# === pyTango ===
from tango import DevState
from tango.server import Device, attribute, command, run, device_property

# === Treiber (ANPASSEN!) ===
# Beispiel: from drivers.vader_driver import VaderDeviceDriver
from your_driver_module import VaderDeviceDriver  # <-- ANPASSEN!


# ---------------- JSON-Programm lesen ----------------
def read_program_from_json(filename: str) -> List[Dict[str, Any]]:
    """
    Erwartet z.B.:
      [{"type":"Normal","target_pressure":120},
       {"type":"Ramp","start_pressure":0,"end_pressure":120,"ramp_speed":50},
       {"type":"delay","delay_time":5000}]
    """
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


class VaderDS(Device):
    #VaderDeviceDriver("mock://mini1", "mock://mini2", "mock://maxi")
    # ============ Geräteeigenschaften (im Tango-DB konfigurierbar) ============
    mini1_port = device_property(dtype=str, default_value="mock://mini1")
    # mini1_port = device_property(dtype=str, default_value="/dev/ttyACM0")
    mini2_port = device_property(dtype=str, default_value="mock://mini2")
    # mini2_port = device_property(dtype=str, default_value="/dev/ttyACM2")
    maxi_port  = device_property(dtype=str, default_value="mock://maxi")
    # maxi_port  = device_property(dtype=str, default_value="/dev/ttyACM1")

    # ============ Attribute (lesen/schreiben) ============
    # Schalt-/Status-Attribute
    @attribute(dtype=bool)    # Storage = (V1 && V2) als Anzeige
    def Storage(self) -> bool:
        return bool(self.cache_maxi["io"].get("V1")) and bool(self.cache_maxi["io"].get("V2"))

    @attribute(dtype=bool)    # Einzelne schaltbare/anzeigbare States
    def V1(self) -> bool: return bool(self.cache_maxi["io"].get("V1"))
    @attribute(dtype=bool)
    def V2(self) -> bool: return bool(self.cache_maxi["io"].get("V2"))
    @attribute(dtype=bool)
    def V3(self) -> bool: return bool(self.cache_maxi["io"].get("V3", False))  # V3-Flag aus MAXI-Status (wir führen mit)
    @attribute(dtype=bool)
    def Fans(self) -> bool:
        return bool(self.cache_maxi["io"].get("FanIn")) and bool(self.cache_maxi["io"].get("FanOut"))

    @attribute(dtype=bool)
    def GasLeak(self) -> bool:
        return bool(self.cache_maxi["io"].get("GasLeak", False))

    # Flows (l/min o.ä., SI – Treiber skaliert)
    @attribute(dtype=float)
    def FlowButan(self) -> float: return float(self.cache_maxi["mfc"]["butan"].get("flow") or 0.0)

    @attribute(dtype=float)
    def FlowCO2(self) -> float:   return float(self.cache_maxi["mfc"]["co2"].get("flow") or 0.0)

    @attribute(dtype=float)
    def FlowN2(self) -> float:    return float(self.cache_maxi["mfc"]["n2"].get("flow") or 0.0)

    # Totals (z.B. in gleichen Einheiten × Zeit; Treiber de-skaliert)
    @attribute(dtype=float)
    def TotalCO2(self) -> float:  return float(self.cache_maxi["mfc"]["co2"].get("total") or 0.0)

    @attribute(dtype=float)
    def TotalN2(self) -> float:   return float(self.cache_maxi["mfc"]["n2"].get("total") or 0.0)

    @attribute(dtype=float)
    def TotalButan(self) -> float: return float(self.cache_maxi["mfc"]["butan"].get("total") or 0.0)

    # MINI2 / Druck / PWM
    @attribute(dtype=float)  # zuletzt gesetzter Soll-Druck (wir cachen)
    def setPoint_pressure(self) -> float: return float(self.cache_mini2.get("setpoint_kpa") or 0.0)

    @attribute(dtype=int)    # vp1 (PWM_in)
    def vp1(self) -> int: return int(self.cache_mini2.get("pwm1") or 0)

    @attribute(dtype=int)    # vp2 (PWM_out)
    def vp2(self) -> int: return int(self.cache_mini2.get("pwm2") or 0)

    # ============ Schreib-Attribute (optional via Commands; hier read-only) ============
    # (Du wolltest "manuelles Setzen" als Funktionen/Kommandos – daher sind die Attribute selbst read-only.)

    # ============ Kommandos (manuelles Setzen) ============
    @command(dtype_in=bool)
    def SetStorage(self, open_: bool):
        """V1 und V2 gemeinsam setzen."""
        self.driver.set_v1(open_)
        self.driver.set_v2(open_)
        # lokalen Flags gleich aktualisieren
        self.cache_maxi["io"]["V1"] = bool(open_)
        self.cache_maxi["io"]["V2"] = bool(open_)

    @command(dtype_in=bool)
    def SetV3(self, open_: bool):
        self.driver.set_v3(open_)
        self.cache_maxi["io"]["V3"] = bool(open_)

    @command(dtype_in=bool)
    def SetFans(self, on: bool):
        """Beide Lüfter (FanIn & FanOut) gemeinsam."""
        self.driver.set_fans(on)
        self.cache_maxi["io"]["FanIn"]  = bool(on)
        self.cache_maxi["io"]["FanOut"] = bool(on)

    @command(dtype_in=float)
    def SetFlowButan(self, flow: float):
        self.driver.set_flow_butan(flow)

    @command(dtype_in=float)
    def SetFlowCO2(self, flow: float):
        self.driver.set_flow_co2(flow)

    @command(dtype_in=float)
    def SetFlowN2(self, flow: float):
        self.driver.set_flow_n2(flow)

    @command(dtype_in=float)
    def SetTotalCO2(self, value: float):
        self.driver.set_total_co2(value)

    @command(dtype_in=float)
    def SetTotalN2(self, value: float):
        self.driver.set_total_n2(value)

    @command(dtype_in=float)
    def SetTotalButan(self, value: float):
        self.driver.set_total_butan(value)

    @command(dtype_in=float)
    def SetSetPointPressure(self, kpa: float):
        self.driver.setpoint_pressure(kpa)
        self.cache_mini2["setpoint_kpa"] = float(kpa)

    @command(dtype_in=int)
    def SetVP1(self, value: int):
        self.driver.set_manual_PWM_in(int(value))
        self.cache_mini2["pwm1"] = int(value)

    @command(dtype_in=int)
    def SetVP2(self, value: int):
        self.driver.set_manual_PWM_out(int(value))
        self.cache_mini2["pwm2"] = int(value)

    # ============ Programmsteuerung ============
    @command(dtype_in=str)
    def RunProgram(self, json_path: str):
        """JSON-Programm asynchron starten. Bereits laufendes Programm wird abgebrochen."""
        self.StopProgram()
        self._program_stop_evt.clear()
        self._program_thread = threading.Thread(
            target=self._program_worker, args=(json_path,), daemon=True
        )
        self._program_thread.start()

    @command()
    def StopProgram(self):
        """Laufendes Programm stoppen (best effort)."""
        self._program_stop_evt.set()
        if self._program_thread and self._program_thread.is_alive():
            # kurz warten, dann Referenz lösen
            self._program_thread.join(timeout=0.1)
        self._program_thread = None

    # ============ Lifecycle ============
    def init_device(self):
        Device.init_device(self)

        self.set_state(DevState.INIT)
        self.set_status("Initialisiere Treiber...")

        # Treiber
        self.driver = VaderDeviceDriver(
            mini1_port=self.mini1_port,
            mini2_port=self.mini2_port,
            maxi_port=self.maxi_port
        )

        # Caches (werden durch Poller befüllt)
        self.cache_maxi: Dict[str, Any] = {
            "io": {"V1": False, "V2": False, "V3": False, "FanIn": False, "FanOut": False, "GasLeak": False},
            "mfc": {"butan": {"flow": 0.0, "total": 0.0},
                    "co2":   {"flow": 0.0, "total": 0.0},
                    "n2":    {"flow": 0.0, "total": 0.0}},
            "adc": {"P11": None, "P31": None},
            "ts": 0.0,
        }
        self.cache_mini2: Dict[str, Any] = {
            "setpoint_kpa": 0.0, "pwm1": 0, "pwm2": 0,
            "mode": "UNKNOWN", "p20_kpa": None, "p21_kpa": None, "ts": 0.0
        }
        self.cache_mini1: Dict[str, Any] = {"pressure": None, "ts": 0.0}

        # Programm-Thread
        self._program_thread: Optional[threading.Thread] = None
        self._program_stop_evt = threading.Event()

        # Poller-Threads (MINI1: 20 Hz; MINI2: 0.5 Hz; MAXI: 0.5 Hz)
        self._stop_evt = threading.Event()
        self._t_mini1 = threading.Thread(target=self._poll_mini1, daemon=True)
        self._t_mini2 = threading.Thread(target=self._poll_mini2, daemon=True)
        self._t_maxi  = threading.Thread(target=self._poll_maxi,  daemon=True)
        self._t_mini1.start()
        self._t_mini2.start()
        self._t_maxi.start()

        self.set_state(DevState.ON)
        self.set_status("Bereit.")

    def delete_device(self):
        self.StopProgram()
        self._stop_evt.set()
        # Threads beenden
        for t in (self._t_mini1, self._t_mini2, self._t_maxi):
            if t and t.is_alive():
                t.join(timeout=0.2)
        # Treiber schließen
        try:
            self.driver.close()
        except Exception:
            pass
        self.set_state(DevState.OFF)
        self.set_status("Gestoppt.")

    # ============ Poller ============
    def _poll_mini1(self):
        """20 Hz: MINI1 (Druck)"""
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
        """0.5 Hz: MINI2 Status (PWM/Mode/Setpoint)"""
        period = 2.0
        while not self._stop_evt.is_set():
            try:
                self.driver.mini2_request_status()
                status = self.driver.mini2_get_status()  # dict
                # bekannte Felder übernehmen
                self.cache_mini2["pwm1"] = int(status.get("pwm1") or 0)
                self.cache_mini2["pwm2"] = int(status.get("pwm2") or 0)
                self.cache_mini2["mode"] = status.get("mode") or "UNKNOWN"
                # setpoint ist nicht im Status – wir halten den letzten gesetzten Wert im Cache
                # optional: aus Druckfeldern (p20/p21) könnte man "pressure_kpa" übernehmen
                self.cache_mini2["ts"] = time.time()
            except Exception:
                pass
            time.sleep(period)

    def _poll_maxi(self):
        """0.5 Hz: MAXI Status (Ventile/Lüfter/Flows/Totals/GasLeak)"""
        period = 2.0
        while not self._stop_evt.is_set():
            try:
                self.driver.maxi_request_status()
                status = self.driver.maxi_get_status()  # dict {"io":...,"mfc":...,"adc":...}
                # io
                io = status.get("io") or {}
                for k in ("V1", "V2", "FanIn", "FanOut", "GasLeak"):
                    if k in io:
                        self.cache_maxi["io"][k] = bool(io[k])
                # (V3 ist in MAXI-Status nicht explizit – wir führen es lokal nach SetV3)
                # mfc
                mfc = status.get("mfc") or {}
                for gas in ("butan", "co2", "n2"):
                    mg = mfc.get(gas) or {}
                    self.cache_maxi["mfc"][gas]["flow"]  = float(mg.get("flow") or 0.0)
                    self.cache_maxi["mfc"][gas]["total"] = float(mg.get("total") or 0.0)
                # adc (optional)
                adc = status.get("adc") or {}
                self.cache_maxi["adc"]["P11"] = adc.get("P11")
                self.cache_maxi["adc"]["P31"] = adc.get("P31")
                self.cache_maxi["ts"] = time.time()
            except Exception:
                pass
            time.sleep(period)

    # ============ Programm-Worker ============
    def _program_worker(self, json_path: str):
        """Führt das JSON-Programm aus; bricht ab, wenn Stop gesetzt wird."""
        try:
            prog = read_program_from_json(json_path)
        except Exception as e:
            self.set_status(f"Programm-Fehler: {e}")
            return
        self.set_status(f"Programm gestartet: {json_path} ({len(prog)} Schritte)")
        for step in prog:
            if self._program_stop_evt.is_set():
                self.set_status("Programm abgebrochen.")
                return
            t = step["type"]
            try:
                if t == "Normal":
                    target = float(step["target_pressure"])
                    self.driver.setpoint_pressure(target)
                    self.cache_mini2["setpoint_kpa"] = target

                elif t == "Ramp":
                    start_p = float(step["start_pressure"])
                    end_p   = float(step["end_pressure"])
                    speed   = float(step["ramp_speed"])  # kPa/s
                    # Startdruck kurz anfahren
                    self.driver.setpoint_pressure(start_p)
                    time.sleep(0.2)
                    # Rampe setzen
                    self.driver.set_ramp(speed_kpa_s=speed, end_kpa=end_p)
                    # Optionale grobe Dauer warten (mit Abbruchprüfung)
                    approx = abs(end_p - start_p) / max(1e-6, speed)
                    t_end = time.time() + min(approx, 3600)
                    while time.time() < t_end:
                        if self._program_stop_evt.is_set():
                            break
                        time.sleep(0.1)

                elif t == "delay":
                    ms = int(step["delay_time"])
                    t_end = time.time() + ms / 1000.0
                    while time.time() < t_end:
                        if self._program_stop_evt.is_set():
                            break
                        time.sleep(0.05)

            except Exception as e:
                self.set_status(f"Programm-Schrittfehler: {e}")
                # weiter zum nächsten Schritt
                continue

        self.set_status("Programm abgeschlossen.")

# ---- main ----
if __name__ == "__main__":
    run((VaderDS,))