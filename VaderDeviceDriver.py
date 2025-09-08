# -*- coding: utf-8 -*-
"""
VaderDeviceDriver (High-Level)
- Kapselt MINI1, MINI2, MAXI.
- Public API beibehalten; zusätzliche Komfort- und Diagnose-Funktionen.
- Setpoint-Setzen liefert intern ein Echo (wird geloggt und abgelegt).

NEU (MAXI-Integration fix):
- Sämtliche MAXI-Aufrufe gehen über ein serialisierendes _mx_roundtrip().
- Mapping der Setpoint-Echos an die internen last_setpoints robuster gegenüber None.
- request_status() bleibt nicht-blockierend; get_status() macht den vollständigen Roundtrip.
- __main__: falscher Aufruf drv.maxi.maxi_get_status() -> korrigiert zu drv.maxi_get_status().
"""

import logging
import time
import threading
from typing import Optional, Tuple, Dict, Any, List, Union

from mini1 import MINI1
from mini2 import MINI2
from maxi import MAXI
from utilities import MODULE_LOGGER_NAME


class VaderDeviceDriver:
    """
    High-Level-Kapselung für MINI1, MINI2 und MAXI (funktioniert mit mock:// Ports).
    Public API unverändert (alle bisherigen Methoden existieren weiterhin und behalten Rückgabewerte).
    Ergänzungen:
      - set_flows(n2=?, co2=?, butan=?, verify=True, timeout_s=1.0, tol=0.05)
      - get_last_setpoints(), get_last_status()
      - assert_setpoints(...)
    """

    # -----------------------------
    # Konstruktion / Ressourcen
    # -----------------------------
    def __init__(self, mini1_port: str, mini2_port: str, maxi_port: str, mini1_max_samples: int = 300_000):
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.VaderDeviceDriver")

        # Geräte instanziieren
        self.mini1 = MINI1(mini1_port, max_samples=mini1_max_samples)
        self.mini2 = MINI2(mini2_port)
        self.maxi  = MAXI(maxi_port)

        # Diagnosepuffer
        self.last_status: Dict[str, Any] = {}
        self.last_setpoints: Dict[str, Optional[float]] = {"n2": None, "co2": None, "butan": None}

        # Lock für alle synchronen MAXI-Roundtrips (ein Befehl -> genau ein Status-Frame)
        self._mx_lock = threading.Lock()

        self.logger.info("Driver ready (mini1=%s mini2=%s maxi=%s)", mini1_port, mini2_port, maxi_port)

    def close(self) -> None:
        self.logger.info("API close()")
        # Reihenfolge: Sensoren/Regler beenden
        try:
            self.mini1.stop()
        finally:
            try:
                self.mini2.stop()
            finally:
                self.maxi.stop()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # =============================
    # MINI2 (Druckcontroller)
    # =============================
    def set_manual_PWM_in(self, value: int) -> None:
        self.logger.info("API set_manual_PWM_in(%d)", value)
        self.mini2.set_manual_vp1(value)

    def set_manual_PWM_out(self, value: int) -> None:
        self.logger.info("API set_manual_PWM_out(%d)", value)
        self.mini2.set_manual_vp2(value)

    def set_state_manual(self) -> None:
        self.logger.info("API set_state_manual()")
        self.mini2.set_state_manual()

    def set_ramp(self, speed_kpa_s: float, end_kpa: float) -> None:
        self.logger.info("API set_ramp(%.2f, %.1f)", speed_kpa_s, end_kpa)
        self.mini2.set_ramp(speed_kpa_s, end_kpa)

    def setpoint_pressure(self, kpa: Union[int, float]) -> None:
        self.logger.info("API setpoint_pressure(%.2f)", float(kpa))
        self.mini2.set_target_pressure(kpa)

    def mini2_request_status(self) -> None:
        self.logger.debug("API mini2_request_status()")
        self.mini2.request_status()

    def mini2_get_status(self) -> Dict[str, Any]:
        s = self.mini2.get_status()
        self.logger.debug("API mini2_get_status()->pressure=%.2f", s.get("pressure_kpa", float("nan")))
        return s

    # =============================
    # MAXI (MFCs, Ventile, Lüfter)
    # =============================
    def _mx_roundtrip(self, fn, *args, **kwargs) -> Dict[str, Any]:
        """
        Serialisiert MAXI-Aufrufe & legt Echo/Status ab.
        Erwartet: fn(...) gibt direkt das vom MAXI geparste Status-Dict zurück.
        """
        with self._mx_lock:
            status = fn(*args, **kwargs)

        # Status ablegen
        if isinstance(status, dict):
            self.last_status = status

            # Setpoint-Echos (falls von Firmware mitgesendet) ablegen
            try:
                mfc = status.get("mfc") or {}
                for gas in ("n2", "co2", "butan"):
                    sp = (mfc.get(gas) or {}).get("setpoint", None)
                    # Nur übernehmen, wenn Setpoint-Feld tatsächlich vorhanden ist
                    if sp is not None:
                        self.last_setpoints[gas] = float(sp)
            except Exception:
                # defensive: niemals eine MAXI-Aktion wegen Diagnose blockieren
                pass

        return status

    # --- Ventile & Lüfter (API unverändert) ---
    def set_v1(self, open_: bool) -> None:
        self.logger.info("API set_v1(%s)", open_)
        self._mx_roundtrip(self.maxi.set_v1, open_)

    def set_v2(self, open_: bool) -> None:
        self.logger.info("API set_v2(%s)", open_)
        self._mx_roundtrip(self.maxi.set_v2, open_)

    def set_v3(self, open_: bool) -> None:
        self.logger.info("API set_v3(%s)", open_)
        self._mx_roundtrip(self.maxi.set_v3, open_)

    def set_fans(self, on: bool) -> None:
        self.logger.info("API set_fans(%s)", on)
        # zwei getrennte Roundtrips, bewusst sequenziell (jede Aktion -> Status)
        self._mx_roundtrip(self.maxi.set_fan_in, on)
        self._mx_roundtrip(self.maxi.set_fan_out, on)

    # --- Flows (API unverändert: None) ---
    def set_flow_co2(self, flow: float) -> None:
        self.logger.info("API set_flow_co2(%.2f)", flow)
        s = self._mx_roundtrip(self.maxi.set_flow_co2, float(flow))
        sp = (s.get("mfc", {}).get("co2", {}) if isinstance(s, dict) else {}).get("setpoint")
        self.logger.debug("ECHO setpoint CO2 = %s", f"{float(sp):.3f}" if sp is not None else "None")

    def set_flow_n2(self, flow: float) -> None:
        self.logger.info("API set_flow_n2(%.2f)", flow)
        s = self._mx_roundtrip(self.maxi.set_flow_n2, float(flow))
        sp = (s.get("mfc", {}).get("n2", {}) if isinstance(s, dict) else {}).get("setpoint")
        self.logger.debug("ECHO setpoint N2  = %s", f"{float(sp):.3f}" if sp is not None else "None")

    def set_flow_butan(self, flow: float) -> None:
        self.logger.info("API set_flow_butan(%.2f)", flow)
        s = self._mx_roundtrip(self.maxi.set_flow_butan, float(flow))
        sp = (s.get("mfc", {}).get("butan", {}) if isinstance(s, dict) else {}).get("setpoint")
        self.logger.debug("ECHO setpoint BUT = %s", f"{float(sp):.3f}" if sp is not None else "None")

    # --- Totalisatoren (API unverändert) ---
    def set_total_co2(self, value: float = 0.0) -> None:
        self.logger.info("API set_total_co2(%.2f)", value)
        self._mx_roundtrip(self.maxi.set_total_co2, float(value))

    def set_total_n2(self, value: float = 0.0) -> None:
        self.logger.info("API set_total_n2(%.2f)", value)
        self._mx_roundtrip(self.maxi.set_total_n2, float(value))

    def set_total_butan(self, value: float = 0.0) -> None:
        self.logger.info("API set_total_butan(%.2f)", value)
        self._mx_roundtrip(self.maxi.set_total_butan, float(value))

    def reset_totalisator_co2(self) -> None:
        self.logger.info("API reset_totalisator_co2()")
        self.set_total_co2(0.0)

    def reset_totalisator_n2(self) -> None:
        self.logger.info("API reset_totalisator_n2()")
        self.set_total_n2(0.0)

    def reset_totalisator_butan(self) -> None:
        self.logger.info("API reset_totalisator_butan()")
        self.set_total_butan(0.0)

    def maxi_request_status(self) -> None:
        """
        Nicht-blockierende Status-Anfrage – sendet nur das Request-Frame.
        (MAXI.request_status() kümmert sich bereits um TX-Fehlerbehandlung.)
        """
        self.logger.debug("API maxi_request_status()")
        with self._mx_lock:
            self.maxi.request_status()

    def maxi_get_status(self) -> Dict[str, Any]:
        """
        Blockierender vollständiger Roundtrip:
        sendet Request + liest genau EIN Status-Frame (Heartbeat-resistent).
        """
        self.logger.debug("API maxi_get_status()")
        return self._mx_roundtrip(self.maxi.get_status)

    # =============================
    # MINI1 (Druckserie)
    # =============================
    def mini1_get_last_n(self, n: int):
        self.logger.debug("API mini1_get_last_n(%d)", n)
        return self.mini1.get_last_n(n)

    # =============================
    # Komfort / High-Level
    # =============================
    def get_last_setpoints(self) -> Dict[str, Optional[float]]:
        """Zuletzt vom Gerät (MAXI) gesehenes Setpoint-Echo pro Gas."""
        return dict(self.last_setpoints)

    def get_last_status(self) -> Dict[str, Any]:
        """Zuletzt gesehenes MAXI-Status-Dict (von irgendeinem Roundtrip)."""
        return dict(self.last_status) if self.last_status else {}

    def assert_setpoints(self, *, n2: Optional[float] = None, co2: Optional[float] = None,
                         butan: Optional[float] = None, tol: float = 0.05) -> bool:
        """
        Prüft rein das Setpoint-Echo (nicht den Flow!).
        True, wenn alle angegebenen Gase innerhalb Toleranz liegen.
        """
        sp = self.get_last_setpoints()
        ok = True
        if n2 is not None:
            v = sp.get("n2")
            if v is None or abs(v - float(n2)) > tol:
                self.logger.error("N2 Setpoint-Echo mismatch: saw=%s, want=%.3f", v, float(n2))
                ok = False
        if co2 is not None:
            v = sp.get("co2")
            if v is None or abs(v - float(co2)) > tol:
                self.logger.error("CO2 Setpoint-Echo mismatch: saw=%s, want=%.3f", v, float(co2))
                ok = False
        if butan is not None:
            v = sp.get("butan")
            if v is None or abs(v - float(butan)) > tol:
                self.logger.error("BUTAN Setpoint-Echo mismatch: saw=%s, want=%.3f", v, float(butan))
                ok = False
        return ok

    def set_flows(self,
                  n2: Optional[float] = None,
                  co2: Optional[float] = None,
                  butan: Optional[float] = None,
                  *,
                  verify: bool = True,
                  timeout_s: float = 1.0,
                  tol: float = 0.05) -> None:
        """
        Bequeme Sammel-API: setzt beliebige Kombinationen von MFC-Setpoints.
        verify=True: wartet kurz und verifiziert per Setpoint-Echo (nicht Flow).

        Rückgabewert: None (API-Kompatibilität beibehalten).
        Erfolg/Fehler wird geloggt; letzter Status ist via get_last_status() verfügbar.
        """
        if n2 is not None:
            self.set_flow_n2(float(n2))
        if co2 is not None:
            self.set_flow_co2(float(co2))
        if butan is not None:
            self.set_flow_butan(float(butan))

        if not verify:
            return

        # robuster Verifikations-Loop (nur Echo, nicht Mess-Flow)
        t_end = time.time() + max(0.05, float(timeout_s))
        want = {
            "n2": float(n2) if n2 is not None else None,
            "co2": float(co2) if co2 is not None else None,
            "butan": float(butan) if butan is not None else None,
        }
        while time.time() < t_end:
            self.maxi_get_status()  # frischen Status holen (Roundtrip)
            if self._echo_matches(want, tol):
                self.logger.info("Setpoint-Echo OK (within ±%.3f): %s", tol, {k: v for k, v in want.items() if v is not None})
                return
            time.sleep(0.05)

        # letzter Versuch
        self.maxi_get_status()
        if self._echo_matches(want, tol):
            self.logger.info("Setpoint-Echo OK (within ±%.3f) after final read.", tol)
            return

        # nicht erfolgreich → Fehler loggen
        current = self.get_last_setpoints()
        mismatches: Dict[str, Dict[str, Optional[float]]] = {}
        for k, w in want.items():
            if w is None:
                continue
            v = current.get(k)
            if v is None or abs(v - w) > tol:
                mismatches[k] = {"want": w, "have": v}
        self.logger.error("Setpoint-Echo NICHT erreicht: %s", mismatches)

    def _echo_matches(self, want: Dict[str, Optional[float]], tol: float) -> bool:
        sp = self.get_last_setpoints()
        for gas, w in want.items():
            if w is None:
                continue
            v = sp.get(gas)
            if v is None or abs(v - w) > tol:
                return False
        return True

    # =============================
    # Bestehende High-Level-Sequenzen
    # =============================
    def storage_volume(self, open_: bool) -> None:
        self.logger.info("API storage_volume(%s)", open_)
        self._mx_roundtrip(self.maxi.set_v1, open_)
        self._mx_roundtrip(self.maxi.set_v2, open_)

    def vac_all(self, vacuum_pressure_kpa: float, timeout_s: int = 120) -> None:
        """
        Evakuiert die Anlage bis 'vacuum_pressure_kpa' oder Timeout.
        Setzt währenddessen alle MFC-Setpoints auf 0.
        """
        self.logger.info("API vac_all(target=%.2f kPa, timeout=%ds)", vacuum_pressure_kpa, timeout_s)
        # MFCs auf 0 (mit Echo/Status)
        self.set_flows(n2=0.0, co2=0.0, butan=0.0, verify=False)

        # Ventile / Lüfter
        self.set_v1(True)
        self.set_v2(True)
        self.set_v3(True)
        self.set_fans(True)

        start = time.time()
        while True:
            p = self.mini1.pressure
            if p is not None and p <= vacuum_pressure_kpa:
                self.logger.info("Vacuum reached: %.2f kPa <= %.2f kPa", p, vacuum_pressure_kpa)
                break
            if time.time() - start > timeout_s:
                self.logger.warning("Vacuum timeout after %ds (last=%.2f kPa)",
                                    timeout_s, p if p is not None else float("nan"))
                break
            time.sleep(0.5)

        # zurücksetzen
        self.set_v1(False)
        self.set_v2(False)
        self.set_v3(False)
        self.set_fans(False)
        # MINI2 entlasten
        self.mini2.set_target_pressure(0)
        self.mini2.set_manual_vp1(0)
        self.mini2.set_manual_vp2(0)

    def use_gas(self, n2: float, co2: float, butan: float) -> None:
        """
        Beibehaltener Kurzweg: setzt alle drei Setpoints (ohne Verifikation).
        """
        self.logger.info("API use_gas(n2=%.2f, co2=%.2f, but=%.2f)", n2, co2, butan)
        self.set_flow_n2(n2)
        self.set_flow_co2(co2)
        self.set_flow_butan(butan)

    def get_all_status(self, mini1_last_n: Optional[int] = 100) -> Dict[str, Any]:
        """
        Frischt MINI2 & MAXI Status an, wartet kurz auf MINI2, liefert kombinierten Dict.
        """
        self.logger.debug("API get_all_status(mini1_last_n=%s)", mini1_last_n)
        # Statusanforderungen stoßen Updates an
        self.mini2.request_status()
        self.maxi_request_status()
        # kurze Wartezeit, damit MINI2 Status aktualisiert
        try:
            # falls vorhanden; sonst einfach kleine Pause
            self.mini2._wait_for_recent_status(0.5)  # type: ignore[attr-defined]
        except Exception:
            time.sleep(0.1)

        res = {
            "mini1": {
                "latest_pressure": self.mini1.pressure,
                "recent_series": self.mini1.get_last_n(mini1_last_n) if mini1_last_n else [],
            },
            "mini2": self.mini2.get_status(),
            "maxi":  self.maxi_get_status(),
        }
        self.logger.debug("All status collected")
        return res

    def reset_all_totalisators(self) -> None:
        self.logger.info("API reset_all_totalisators()")
        self.reset_totalisator_butan()
        self.reset_totalisator_co2()
        self.reset_totalisator_n2()


# ============================================================
# Optionaler, knapper Demo-Einstieg (keine Tests / kein CI)
# ============================================================
if __name__ == "__main__":
    import os

    logging.basicConfig(
        level=os.environ.get("VADER_LOGLEVEL", "INFO").upper(),
        format="%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s",
        datefmt=":%H:%M:%S",
    )
    log = logging.getLogger("VaderDeviceDriver.main")

    # Beispiel: reale Ports via ENV, sonst mock
    mini1_port = os.environ.get("VADER_MINI1_PORT", "/dev/cu.usbmodem1133101")
    mini2_port = os.environ.get("VADER_MINI2_PORT", "/dev/cu.usbmodem1133301")
    maxi_port  = os.environ.get("VADER_MAXI_PORT",  "/dev/cu.usbmodem1133201")

    with VaderDeviceDriver(mini1_port, mini2_port, maxi_port) as drv:
        # kleine Statusrunde
        s_all = drv.get_all_status(mini1_last_n=5)
        log.info("INIT status: %s", {k: (list(v.keys()) if isinstance(v, dict) else type(v).__name__)
                                     for k, v in s_all.items()})

        # Beispiel: Setpoints setzen & verifizieren (nur Echo)
        drv.set_flows(n2=5.0, co2=10.0, butan=0.0, verify=True, timeout_s=1.0, tol=0.05)
        log.info("Last setpoints: %s", drv.get_last_setpoints())

        # KORRIGIERT: vollständigen MAXI-Status holen (kein .maxi_get_status() in MAXI)
        s_maxi = drv.maxi_get_status()
        log.info("MAXI status: %s", s_maxi)