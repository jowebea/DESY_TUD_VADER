#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
from typing import Optional, Dict, Any, List

# === pyTango ===
from tango import DevState, AttrWriteType
from tango.server import Device, attribute, command, run, device_property

# === Treiber (ANPASSEN!) ===
from VaderDeviceDriver import VaderDeviceDriver


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


class Vader(Device):
    # ============ Geräteeigenschaften (im Tango-DB konfigurierbar) ============
    mini1_port = device_property(dtype=str, default_value="mock://mini1")
    mini2_port = device_property(dtype=str, default_value="mock://mini2")
    maxi_port  = device_property(dtype=str, default_value="mock://maxi")

    # ============ Attribute (READ/WRITE) ============
    # Hinweis: Schreib-Attribute ersetzen die früheren Set*-Kommandos.

    # Storage = (V1 && V2)
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def Storage(self) -> bool:
        return bool(self.cache_maxi["io"].get("V1")) and bool(self.cache_maxi["io"].get("V2"))

    def write_Storage(self, value: bool):
        self.driver.set_v1(bool(value))
        self.driver.set_v2(bool(value))
        self.cache_maxi["io"]["V1"] = bool(value)
        self.cache_maxi["io"]["V2"] = bool(value)

    # Einzelventile
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def V1(self) -> bool:
        return bool(self.cache_maxi["io"].get("V1"))

    def write_V1(self, value: bool):
        self.driver.set_v1(bool(value))
        self.cache_maxi["io"]["V1"] = bool(value)

    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def V2(self) -> bool:
        return bool(self.cache_maxi["io"].get("V2"))

    def write_V2(self, value: bool):
        self.driver.set_v2(bool(value))
        self.cache_maxi["io"]["V2"] = bool(value)

    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def V3(self) -> bool:
        return bool(self.cache_maxi["io"].get("V3", False))

    def write_V3(self, value: bool):
        self.driver.set_v3(bool(value))
        self.cache_maxi["io"]["V3"] = bool(value)

    # Fans (beide gemeinsam)
    @attribute(dtype=bool, access=AttrWriteType.READ_WRITE)
    def Fans(self) -> bool:
        return bool(self.cache_maxi["io"].get("FanIn")) and bool(self.cache_maxi["io"].get("FanOut"))

    def write_Fans(self, value: bool):
        self.driver.set_fans(bool(value))
        self.cache_maxi["io"]["FanIn"]  = bool(value)
        self.cache_maxi["io"]["FanOut"] = bool(value)

    # Gas Leak (nur Anzeige)
    @attribute(dtype=bool)
    def GasLeak(self) -> bool:
        return bool(self.cache_maxi["io"].get("GasLeak", False))

    # Flows (Sollwerte)
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE)
    def FlowButan(self) -> float:
        return float(self.cache_maxi["mfc"]["butan"].get("flow") or 0.0)

    def write_FlowButan(self, value: float):
        self.driver.set_flow_butan(float(value))

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE)
    def FlowCO2(self) -> float:
        return float(self.cache_maxi["mfc"]["co2"].get("flow") or 0.0)

    def write_FlowCO2(self, value: float):
        self.driver.set_flow_co2(float(value))

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE)
    def FlowN2(self) -> float:
        return float(self.cache_maxi["mfc"]["n2"].get("flow") or 0.0)

    def write_FlowN2(self, value: float):
        self.driver.set_flow_n2(float(value))

    # Totals (frei setzbar, z. B. Reset=0.0)
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE)
    def TotalCO2(self) -> float:
        return float(self.cache_maxi["mfc"]["co2"].get("total") or 0.0)

    def write_TotalCO2(self, value: float):
        self.driver.set_total_co2(float(value))

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE)
    def TotalN2(self) -> float:
        return float(self.cache_maxi["mfc"]["n2"].get("total") or 0.0)

    def write_TotalN2(self, value: float):
        self.driver.set_total_n2(float(value))

    @attribute(dtype=float, access=AttrWriteType.READ_WRITE)
    def TotalButan(self) -> float:
        return float(self.cache_maxi["mfc"]["butan"].get("total") or 0.0)

    def write_TotalButan(self, value: float):
        self.driver.set_total_butan(float(value))

    # MINI2 / Druck / PWM
    @attribute(dtype=float, access=AttrWriteType.READ_WRITE)
    def setPoint_pressure(self) -> float:
        return float(self.cache_mini2.get("setpoint_kpa") or 0.0)

    def write_setPoint_pressure(self, value: float):
        self.driver.setpoint_pressure(float(value))
        self.cache_mini2["setpoint_kpa"] = float(value)

    @attribute(dtype=int, access=AttrWriteType.READ_WRITE)   # vp1 (PWM_in)
    def vp1(self) -> int:
        return int(self.cache_mini2.get("pwm1") or 0)

    def write_vp1(self, value: int):
        self.driver.set_manual_PWM_in(int(value))
        self.cache_mini2["pwm1"] = int(value)

    @attribute(dtype=int, access=AttrWriteType.READ_WRITE)   # vp2 (PWM_out)
    def vp2(self) -> int:
        return int(self.cache_mini2.get("pwm2") or 0)

    def write_vp2(self, value: int):
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
        for t in (self._t_mini1, self._t_mini2, self._t_maxi):
            if t and t.is_alive():
                t.join(timeout=0.2)
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
                status = self.driver.mini2_get_status()
                self.cache_mini2["pwm1"] = int(status.get("pwm1") or 0)
                self.cache_mini2["pwm2"] = int(status.get("pwm2") or 0)
                self.cache_mini2["mode"] = status.get("mode") or "UNKNOWN"
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
                status = self.driver.maxi_get_status()
                io = status.get("io") or {}
                for k in ("V1", "V2", "FanIn", "FanOut", "GasLeak"):
                    if k in io:
                        self.cache_maxi["io"][k] = bool(io[k])
                mfc = status.get("mfc") or {}
                for gas in ("butan", "co2", "n2"):
                    mg = mfc.get(gas) or {}
                    self.cache_maxi["mfc"][gas]["flow"]  = float(mg.get("flow") or 0.0)
                    self.cache_maxi["mfc"][gas]["total"] = float(mg.get("total") or 0.0)
                adc = status.get("adc") or {}
                self.cache_maxi["adc"]["P11"] = adc.get("P11")
                self.cache_maxi["adc"]["P31"] = adc.get("P31")
                self.cache_maxi["ts"] = time.time()
            except Exception:
                pass
            time.sleep(period)

    # ============ Programm-Worker ============
    def _program_worker(self, json_path: str):
        """Führt das JSON-Programm aus; bricht ab, wenn Stop gesetzt wird. Meldet Schritte via set_status()."""
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

            t = step["type"]
            try:
                if t == "Normal":
                    target = float(step["target_pressure"])
                    self.set_status(f"[{idx}/{n}] Normal: setpoint → {target:.3f} kPa")
                    self.driver.setpoint_pressure(target)
                    self.cache_mini2["setpoint_kpa"] = target

                elif t == "Ramp":
                    start_p = float(step["start_pressure"])
                    end_p   = float(step["end_pressure"])
                    speed   = float(step["ramp_speed"])  # kPa/s
                    self.set_status(f"[{idx}/{n}] Ramp: {start_p:.3f} → {end_p:.3f} kPa @ {speed:.3f} kPa/s")
                    # Startdruck kurz anfahren
                    self.driver.setpoint_pressure(start_p)
                    time.sleep(0.2)
                    # Rampe setzen (Firmware übernimmt ab aktuellem Druck)
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