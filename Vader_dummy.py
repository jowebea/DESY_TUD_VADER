#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import random
import threading
import time
from typing import Dict, Any, List

from tango import DevState
from tango.server import Device, attribute, command, run, device_property


# ---------------- JSON-Programm lesen ----------------
def read_program_from_json(filename: str) -> List[Dict[str, Any]]:
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    prog = []
    for i, step in enumerate(data):
        t = step.get("type")
        if t == "Normal":
            prog.append({"type": "Normal", "target_pressure": float(step["target_pressure"])})
        elif t == "Ramp":
            prog.append({
                "type": "Ramp",
                "start_pressure": float(step["start_pressure"]),
                "end_pressure":   float(step["end_pressure"]),
                "ramp_speed":     float(step["ramp_speed"]),
            })
        elif t == "delay":
            prog.append({"type": "delay", "delay_time": int(step["delay_time"])})
        else:
            raise ValueError(f"Unbekannter Step-Typ an Pos {i}: {t}")
    return prog


class VaderDummyDS(Device):
    """
    Dummy-Tango-Server zum Testen des Interfaces.
    Simuliert MINI1/2/MAXI:
      - Drucksim (20 Hz)
      - Status/Flows/Totals (0.5 Hz sichtbar)
      - JSON-Programm (Normal/Ramp/delay)
    """

    # „Geräteeigenschaften“ – hier nur Dummies/Startwerte
    mini1_port = device_property(dtype=str, default_value="DUMMY")
    mini2_port = device_property(dtype=str, default_value="DUMMY")
    maxi_port  = device_property(dtype=str, default_value="DUMMY")

    # ------------- Attribute (anzeigen) -------------
    @attribute(dtype=bool)  # Storage als V1 && V2
    def Storage(self) -> bool:
        return bool(self._io["V1"] and self._io["V2"])

    @attribute(dtype=bool)
    def V1(self) -> bool:
        return bool(self._io["V1"])

    @attribute(dtype=bool)
    def V2(self) -> bool:
        return bool(self._io["V2"])

    @attribute(dtype=bool)
    def V3(self) -> bool:
        return bool(self._io["V3"])

    @attribute(dtype=bool)
    def Fans(self) -> bool:
        return bool(self._io["FanIn"] and self._io["FanOut"])

    @attribute(dtype=bool)
    def GasLeak(self) -> bool:
        return bool(self._io["GasLeak"])

    @attribute(dtype=float)
    def FlowButan(self) -> float:
        return float(self._mfc["butan"]["flow"])

    @attribute(dtype=float)
    def FlowCO2(self) -> float:
        return float(self._mfc["co2"]["flow"])

    @attribute(dtype=float)
    def FlowN2(self) -> float:
        return float(self._mfc["n2"]["flow"])

    @attribute(dtype=float)
    def TotalButan(self) -> float:
        return float(self._mfc["butan"]["total"])

    @attribute(dtype=float)
    def TotalCO2(self) -> float:
        return float(self._mfc["co2"]["total"])

    @attribute(dtype=float)
    def TotalN2(self) -> float:
        return float(self._mfc["n2"]["total"])

    @attribute(dtype=float)
    def setPoint_pressure(self) -> float:
        return float(self._mini2["setpoint_kpa"])

    @attribute(dtype=int)
    def vp1(self) -> int:
        return int(self._mini2["pwm1"])

    @attribute(dtype=int)
    def vp2(self) -> int:
        return int(self._mini2["pwm2"])

    # ------------- Kommandos (manuelles Setzen) -------------
    @command(dtype_in=bool)  # V1+V2 gemeinsam
    def SetStorage(self, open_: bool):
        self._io["V1"] = bool(open_)
        self._io["V2"] = bool(open_)

    @command(dtype_in=bool)
    def SetV3(self, open_: bool):
        self._io["V3"] = bool(open_)

    @command(dtype_in=bool)
    def SetFans(self, on: bool):
        self._io["FanIn"] = bool(on)
        self._io["FanOut"] = bool(on)

    @command(dtype_in=float)
    def SetFlowButan(self, flow: float):
        self._mfc["butan"]["flow"] = max(0.0, float(flow))

    @command(dtype_in=float)
    def SetFlowCO2(self, flow: float):
        self._mfc["co2"]["flow"] = max(0.0, float(flow))

    @command(dtype_in=float)
    def SetFlowN2(self, flow: float):
        self._mfc["n2"]["flow"] = max(0.0, float(flow))

    @command(dtype_in=float)
    def SetTotalButan(self, value: float):
        self._mfc["butan"]["total"] = max(0.0, float(value))

    @command(dtype_in=float)
    def SetTotalCO2(self, value: float):
        self._mfc["co2"]["total"] = max(0.0, float(value))

    @command(dtype_in=float)
    def SetTotalN2(self, value: float):
        self._mfc["n2"]["total"] = max(0.0, float(value))

    @command(dtype_in=float)
    def SetSetPointPressure(self, kpa: float):
        self._mini2["setpoint_kpa"] = float(kpa)

    @command(dtype_in=int)
    def SetVP1(self, value: int):
        self._mini2["pwm1"] = max(0, min(255, int(value)))

    @command(dtype_in=int)
    def SetVP2(self, value: int):
        self._mini2["pwm2"] = max(0, min(255, int(value)))

    # ------------- Programme -------------
    @command(dtype_in=str)
    def RunProgram(self, json_path: str):
        self.StopProgram()
        self._prog_stop.clear()
        self._prog_thread = threading.Thread(target=self._program_worker, args=(json_path,), daemon=True)
        self._prog_thread.start()

    @command()
    def StopProgram(self):
        self._prog_stop.set()
        if self._prog_thread and self._prog_thread.is_alive():
            self._prog_thread.join(timeout=0.2)
        self._prog_thread = None

    # ------------- Lifecycle -------------
    def init_device(self):
        Device.init_device(self)
        self.set_state(DevState.INIT)
        self.set_status("Initialisiere Dummy…")

        # Zustände/Caches
        self._io: Dict[str, Any] = {
            "V1": False, "V2": False, "V3": False,
            "FanIn": False, "FanOut": False, "GasLeak": False,
        }
        self._mfc: Dict[str, Dict[str, float]] = {
            "butan": {"flow": 0.0, "total": 0.0},
            "co2":   {"flow": 0.0, "total": 0.0},
            "n2":    {"flow": 0.0, "total": 0.0},
        }
        self._mini1: Dict[str, Any] = {"pressure_kpa": 101.3}  # Start bei Atmosphärendruck
        self._mini2: Dict[str, Any] = {"setpoint_kpa": 0.0, "pwm1": 0, "pwm2": 0, "mode": "MANUAL"}

        # Steuerflags/Threads
        self._stop_evt = threading.Event()
        self._prog_stop = threading.Event()
        self._prog_thread = None

        # Poll-/Simulations-Threads
        self._t_mini1 = threading.Thread(target=self._loop_mini1_20hz, daemon=True)  # 20 Hz
        self._t_mini2 = threading.Thread(target=self._loop_mini2_05hz, daemon=True)  # 0.5 Hz
        self._t_maxi  = threading.Thread(target=self._loop_maxi_05hz,  daemon=True)  # 0.5 Hz

        self._last_integrate = time.time()
        self._t_mini1.start()
        self._t_mini2.start()
        self._t_maxi.start()

        self.set_state(DevState.ON)
        self.set_status("Dummy bereit.")

    def delete_device(self):
        self.StopProgram()
        self._stop_evt.set()
        for t in (self._t_mini1, self._t_mini2, self._t_maxi):
            if t and t.is_alive():
                t.join(timeout=0.3)
        self.set_state(DevState.OFF)
        self.set_status("Dummy gestoppt.")

    # ------------- Simulation Loops -------------
    def _loop_mini1_20hz(self):
        """
        20 Hz „Physik“:
          - Druck nähert sich setPoint an (1. Ordnung) mit Dämpfung/Trägheit.
          - PWM1/PWM2 wirken als „Durchsatz“ & beeinflussen Annäherungsgeschwindigkeit.
          - Totals = Integral der Flüsse (Butan/CO2/N2).
          - Leichtes Rauschen; GasLeak gelegentlich abhängig von Flows/Fans.
        """
        period = 1.0 / 20.0
        tau_base = 2.5   # s: Grundträgheit
        while not self._stop_evt.is_set():
            t0 = time.time()

            # --- Integrationsschritt ---
            now = t0
            dt = max(1e-4, now - self._last_integrate)
            self._last_integrate = now

            # Druckmodell
            sp = float(self._mini2["setpoint_kpa"])
            p  = float(self._mini1["pressure_kpa"])
            pwm_gain = 1.0 + 0.01 * (self._mini2["pwm1"] + self._mini2["pwm2"])  # 1 .. ~6.1
            tau = tau_base / pwm_gain
            dp = (sp - p) * (dt / max(0.3, tau))
            dp += random.gauss(0.0, 0.01)  # Rauschen
            p = max(0.0, p + dp)
            self._mini1["pressure_kpa"] = p

            # Flows integrieren -> Totals
            for gas in ("butan", "co2", "n2"):
                flow = float(self._mfc[gas]["flow"])  # Einheit z. B. l/min
                self._mfc[gas]["total"] += max(0.0, flow) * (dt / 60.0)

            # GasLeak-Heuristik
            if (self._mfc["butan"]["flow"] + self._mfc["co2"]["flow"] + self._mfc["n2"]["flow"]) > 5.0 and not self._io["FanIn"] and not self._io["FanOut"]:
                leak_prob = 0.05
            else:
                leak_prob = 0.005
            self._io["GasLeak"] = (random.random() < leak_prob)

            # Loop timing
            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    def _loop_mini2_05hz(self):
        """0.5 Hz: Statuspflege MINI2 (hier nur Mode-Ableitung)."""
        period = 2.0
        while not self._stop_evt.is_set():
            try:
                if abs(self._mini2["setpoint_kpa"] - self._mini1["pressure_kpa"]) > 2.0:
                    mode = "AUTOMATIC"
                elif (self._mini2["pwm1"] + self._mini2["pwm2"]) > 0:
                    mode = "MANUAL"
                else:
                    mode = "IDLE"
                self._mini2["mode"] = mode
            except Exception:
                pass
            time.sleep(period)

    def _loop_maxi_05hz(self):
        """0.5 Hz: Sichtbarer Refresh für MAXI (hier keine zusätzliche Physik nötig)."""
        period = 2.0
        while not self._stop_evt.is_set():
            time.sleep(period)

    # ------------- Programm-Worker -------------
    def _program_worker(self, json_path: str):
        try:
            program = read_program_from_json(json_path)
        except Exception as e:
            self.set_status(f"Programm-Fehler: {e}")
            return

        self.set_status(f"Programm gestartet: {json_path} ({len(program)} Schritte)")
        for step in program:
            if self._prog_stop.is_set():
                self.set_status("Programm abgebrochen.")
                return

            t = step["type"]
            try:
                if t == "Normal":
                    target = float(step["target_pressure"])
                    self._mini2["setpoint_kpa"] = target

                elif t == "Ramp":
                    start_p = float(step["start_pressure"])
                    end_p   = float(step["end_pressure"])
                    speed   = float(step["ramp_speed"])  # kPa/s

                    # Start setzen
                    self._mini2["setpoint_kpa"] = start_p
                    time.sleep(0.2)

                    # Rampe: setpoint linear bewegen
                    direction = 1.0 if end_p >= start_p else -1.0
                    sp = start_p
                    t_last = time.time()
                    while (direction > 0 and sp < end_p) or (direction < 0 and sp > end_p):
                        if self._prog_stop.is_set():
                            break
                        now = time.time()
                        dt = now - t_last
                        t_last = now
                        sp += direction * speed * dt
                        if direction > 0:
                            sp = min(sp, end_p)
                        else:
                            sp = max(sp, end_p)
                        self._mini2["setpoint_kpa"] = sp
                        time.sleep(0.05)

                elif t == "delay":
                    ms = int(step["delay_time"])
                    t_end = time.time() + ms / 1000.0
                    while time.time() < t_end:
                        if self._prog_stop.is_set():
                            break
                        time.sleep(0.05)

            except Exception as e:
                self.set_status(f"Programm-Schrittfehler: {e}")
                continue

        self.set_status("Programm abgeschlossen.")


# ---- main ----
if __name__ == "__main__":
    run((VaderDummyDS,))