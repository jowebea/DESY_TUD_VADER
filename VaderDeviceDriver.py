# -*- coding: utf-8 -*-
import logging
import time
from typing import Optional, Tuple, Dict, Any, List, Union
from datetime import datetime

from utilities import MODULE_LOGGER_NAME
from mini1 import MINI1
from mini2 import MINI2
from maxi import MAXI

class VaderDeviceDriver:
    """
    High-Level-Kapselung für MINI1, MINI2 und MAXI (funktioniert mit mock:// Ports).
    Public API unverändert.
    """
    def __init__(self, mini1_port: str, mini2_port: str, maxi_port: str, mini1_max_samples: int = 300_000):
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.VaderDeviceDriver")
        self.mini1 = MINI1(mini1_port, max_samples=mini1_max_samples)
        self.mini2 = MINI2(mini2_port)
        self.maxi  = MAXI(maxi_port)
        self.logger.info("Driver ready (mini1=%s mini2=%s maxi=%s)", mini1_port, mini2_port, maxi_port)

    # -----------------------------
    # MINI2
    # -----------------------------
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

    def mini2_request_status(self) -> None:
        self.logger.debug("API mini2_request_status()")
        self.mini2.request_status()

    def mini2_get_status(self) -> Dict[str, Any]:
        s = self.mini2.get_status()
        self.logger.debug("API mini2_get_status()->pressure=%.2f", s.get("pressure_kpa", float("nan")))
        return s

    # -----------------------------
    # MAXI
    # -----------------------------
    def set_v1(self, open_: bool) -> None:
        self.logger.info("API set_v1(%s)", open_)
        self.maxi.set_v1(open_)

    def set_v2(self, open_: bool) -> None:
        self.logger.info("API set_v2(%s)", open_)
        self.maxi.set_v2(open_)

    def set_v3(self, open_: bool) -> None:
        self.logger.info("API set_v3(%s)", open_)
        self.maxi.set_v3(open_)

    def set_fans(self, on: bool) -> None:
        self.logger.info("API set_fans(%s)", on)
        self.maxi.set_fan_in(on)
        self.maxi.set_fan_out(on)

    def set_flow_co2(self, flow: float) -> None:
        self.logger.info("API set_flow_co2(%.2f)", flow)
        self.maxi.set_flow_co2(flow)

    def set_flow_n2(self, flow: float) -> None:
        self.logger.info("API set_flow_n2(%.2f)", flow)
        self.maxi.set_flow_n2(flow)

    def set_flow_butan(self, flow: float) -> None:
        self.logger.info("API set_flow_butan(%.2f)", flow)
        self.maxi.set_flow_butan(flow)

    def set_total_co2(self, value: float = 0.0) -> None:
        self.logger.info("API set_total_co2(%.2f)", value)
        self.maxi.set_total_co2(value)

    def set_total_n2(self, value: float = 0.0) -> None:
        self.logger.info("API set_total_n2(%.2f)", value)
        self.maxi.set_total_n2(value)

    def set_total_butan(self, value: float = 0.0) -> None:
        self.logger.info("API set_total_butan(%.2f)", value)
        self.maxi.set_total_butan(value)

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
        self.logger.debug("API maxi_request_status()")
        self.maxi.request_status()

    def maxi_get_status(self) -> Dict[str, Any]:
        s = self.maxi.get_status()
        self.logger.debug("API maxi_get_status()")
        return s

    # -----------------------------
    # MINI1
    # -----------------------------
    def mini1_get_last_n(self, n: int):
        self.logger.debug("API mini1_get_last_n(%d)", n)
        return self.mini1.get_last_n(n)

    # -----------------------------
    # bestehende Methoden
    # -----------------------------
    def storage_volume(self, open_: bool) -> None:
        self.logger.info("API storage_volume(%s)", open_)
        self.maxi.set_v1(open_)
        self.maxi.set_v2(open_)

    def vac_all(self, vacuum_pressure_kpa: float, timeout_s: int = 120) -> None:
        self.logger.info("API vac_all(target=%.2f kPa, timeout=%ds)", vacuum_pressure_kpa, timeout_s)
        self.maxi.set_flow_butan(0)
        self.maxi.set_flow_co2(0)
        self.maxi.set_flow_n2(0)
        self.maxi.set_v1(True)
        self.maxi.set_v2(True)
        self.maxi.set_v3(True)
        self.set_fans(True)

        start = time.time()
        while True:
            p = self.mini1.pressure
            if p is not None and p <= vacuum_pressure_kpa:
                self.logger.info("Vacuum reached: %.2f kPa <= %.2f kPa", p, vacuum_pressure_kpa)
                break
            if time.time() - start > timeout_s:
                self.logger.warning("Vacuum timeout after %ds (last=%.2f kPa)", timeout_s, p if p is not None else float("nan"))
                break
            time.sleep(0.5)

        self.maxi.set_v1(False)
        self.maxi.set_v2(False)
        self.maxi.set_v3(False)
        self.set_fans(False)
        self.mini2.set_target_pressure(0)
        self.mini2.set_manual_vp1(0)
        self.mini2.set_manual_vp2(0)

    def use_gas(self, n2: float, co2: float, butan: float) -> None:
        self.logger.info("API use_gas(n2=%.2f, co2=%.2f, but=%.2f)", n2, co2, butan)
        self.maxi.set_flow_n2(n2)
        self.maxi.set_flow_co2(co2)
        self.maxi.set_flow_butan(butan)

    def setpoint_pressure(self, kpa: Union[int, float]) -> None:
        self.logger.info("API setpoint_pressure(%.2f)", float(kpa))
        self.mini2.set_target_pressure(kpa)

    def get_all_status(self, mini1_last_n: Optional[int] = 100) -> Dict[str, Any]:
        self.logger.debug("API get_all_status(mini1_last_n=%s)", mini1_last_n)
        self.mini2.request_status()
        self.maxi.request_status()
        self.mini2._wait_for_recent_status(0.5)
        res = {
            "mini1": {
                "latest_pressure": self.mini1.pressure,
                "recent_series": self.mini1.get_last_n(mini1_last_n)
            },
            "mini2": self.mini2.get_status(),
            "maxi":  self.maxi.get_status(),
        }
        self.logger.debug("All status collected")
        return res

    def reset_all_totalisators(self) -> None:
        self.logger.info("API reset_all_totalisators()")
        self.reset_totalisator_butan()
        self.reset_totalisator_co2()
        self.reset_totalisator_n2()

    def close(self) -> None:
        self.logger.info("API close()")
        self.mini1.stop()
        self.mini2.stop()
        self.maxi.stop()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ============================================================
# DEMO
# ============================================================
# ============================================================
# DEMO + TESTS
# ============================================================
if __name__ == "__main__":
    import os
    level = os.environ.get("VADER_LOG_LEVEL", "INFO").upper()
    logging.basicConfig(
        level=getattr(logging, level, logging.DEBUG),
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S"
    )
    logging.getLogger("vader.mock.SerialDevice.MAXI").setLevel(logging.INFO)
    logging.getLogger("vader.mock.MAXI").setLevel(logging.INFO)

    _log = logging.getLogger(MODULE_LOGGER_NAME)

    # -----------------------------
    # Hilfsfunktionen für die Tests
    # -----------------------------
    # --- NEU: tiefe, robuste Suche nach numerischen Werten ---
    from math import isfinite
    from typing import Any, List, Tuple, Optional

    def _iter_deep_items(obj: Any, path: str = "") -> List[Tuple[str, Any]]:
        """Erzeugt (pfad, wert) für alle Blätter (auch in Listen)."""
        out = []
        if isinstance(obj, dict):
            for k, v in obj.items():
                p = f"{path}.{k}" if path else str(k)
                out.extend(_iter_deep_items(v, p))
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                p = f"{path}[{i}]"
                out.extend(_iter_deep_items(v, p))
        else:
            out.append((path, obj))
        return out

    def _score_flow_candidate(path_low: str, gas: str) -> int:
        """
        Scoring: höhere Punkte = wahrscheinlicher echter Flow.
        Bevorzugt Pfade mit 'flow'/'sccm'/'slm' und exaktem Gasnamen,
        meidet 'setpoint'/'target'/'total'.
        """
        score = 0
        if "flow" in path_low: score += 4
        if "sccm" in path_low or "slm" in path_low: score += 2
        if gas in path_low: score += 3
        # negative Hinweise
        if "set" in path_low or "target" in path_low or "sp" in path_low: score -= 5
        if "total" in path_low or "integral" in path_low: score -= 3
        if "valve" in path_low or "v" in path_low and "flow" not in path_low: score -= 2
        return score

    def _find_flow_candidates(status: dict, gas: str) -> List[Tuple[str, float, int]]:
        """
        Liefert sortierte Kandidaten (pfad, wert, score) für einen Gas-Flow.
        """
        gas = gas.lower()
        items = _iter_deep_items(status)
        cands: List[Tuple[str, float, int]] = []
        for path, val in items:
            # dict-Wrapper wie {"value": x} behandeln
            if isinstance(val, dict) and "value" in val:
                try:
                    val = float(val["value"])
                except Exception:
                    continue
            if isinstance(val, (int, float)) and isfinite(val):
                path_low = path.lower()
                # nur Pfade berücksichtigen, die nach Flow aussehen
                if any(t in path_low for t in ["flow", "sccm", "slm"]):
                    score = _score_flow_candidate(path_low, gas)
                    cands.append((path, float(val), score))
        # falls nichts mit 'flow', erlauben wir fallback auf gas + numeric
        if not cands:
            for path, val in items:
                if isinstance(val, (int, float)) and isfinite(val):
                    path_low = path.lower()
                    if gas in path_low:
                        score = 1  # schwacher Treffer
                        cands.append((path, float(val), score))
        # beste oben
        cands.sort(key=lambda x: x[2], reverse=True)
        return cands

    def _extract_flow_from_status(status: dict, gas: str, logger: logging.Logger) -> Optional[float]:
        cands = _find_flow_candidates(status, gas)
        if not cands:
            logger.debug("FLOW-SUCHE: Keine Kandidaten für %s gefunden.", gas.upper())
            return None
        # Top-Kandidat loggen (plus ein paar weitere zur Diagnose)
        logger.debug("FLOW-SUCHE [%s]: Top=%s (%.4f, score=%d); weitere=%s",
                    gas.upper(), cands[0][0], cands[0][1], cands[0][2],
                    [p for p,_,_ in cands[1:3]])
        return cands[0][1]

    # --- NEU: optionaler Fallback über Totalisator-Ableitung ---
    def _extract_total_from_status(status: dict, gas: str, logger: logging.Logger) -> Optional[float]:
        gas = gas.lower()
        items = _iter_deep_items(status)
        best = None
        for path, val in items:
            if isinstance(val, dict) and "value" in val:
                try:
                    val = float(val["value"])
                except Exception:
                    continue
            if isinstance(val, (int, float)) and isfinite(val):
                pl = path.lower()
                if gas in pl and ("total" in pl or "integral" in pl or "sum" in pl or "totalizer" in pl):
                    best = float(val)
                    # keine weitere Heuristik nötig: erster Treffer reicht
                    break
        if best is None:
            logger.debug("TOTAL-SUCHE: Kein Totalisator für %s gefunden.", gas.upper())
        return best

    def _wait_and_measure_flow(drv: "VaderDeviceDriver", gas: str, setpoint: float,
                            timeout_s: float = 8.0, poll_s: float = 0.25,
                            tol_abs_zero: float = 0.15, tol_rel: float = 0.05,
                            use_total_fallback: bool = True) -> Optional[float]:
        """
        Aktualisiert Status aktiv über maxi_request_status(), sucht Flow tief rekursiv.
        Optionaler Fallback: Flow ≈ d(Total)/dt, falls kein direkter Flow zu finden ist.
        """
        gas = gas.lower()
        setter = {
            "n2":   drv.set_flow_n2,
            "co2":  drv.set_flow_co2,
            "butan":drv.set_flow_butan,
            "butane": drv.set_flow_butan,  # kleine Kulanz
        }[gas]

        _log.info("TEST: Setze %s-Setpoint auf %.3f …", gas.upper(), setpoint)
        setter(setpoint)

        t0 = time.time()
        last_flow: Optional[float] = None

        # für Total-Fallback Werte puffern
        total_start = total_last = None
        t_total_start = t_total_last = None

        while time.time() - t0 < timeout_s:
            # WICHTIG: erst Status anfordern, dann lesen
            drv.maxi_request_status()
            time.sleep(0.05)
            s = drv.maxi_get_status()

            # Primär: direkter Flow
            val = _extract_flow_from_status(s, gas, _log)
            if val is not None:
                last_flow = val
                if setpoint == 0.0:
                    if abs(val) <= tol_abs_zero:
                        break
                else:
                    tol = max(tol_abs_zero, tol_rel * max(1e-9, setpoint))
                    if abs(val - setpoint) <= tol:
                        break

            # Sekundär: Totalisator-Ableitung als Schätzer
            if use_total_fallback:
                tot = _extract_total_from_status(s, gas, _log)
                now = time.time()
                if tot is not None:
                    if total_start is None:
                        total_start = tot
                        t_total_start = now
                    total_last = tot
                    t_total_last = now

            time.sleep(poll_s)

        # Falls kein direkter Flow, versuche Ableitung ΔTotal/Δt
        if last_flow is None and use_total_fallback and total_last is not None and t_total_start is not None:
            dt = max(1e-6, (t_total_last - t_total_start))
            deriv = (total_last - total_start) / dt
            _log.warning("TEST: Kein direkter %s-Flow gefunden; nutze d(Total)/dt ≈ %.4f", gas.upper(), deriv)
            last_flow = float(deriv)

        if last_flow is None:
            _log.error("TEST: Konnte für %s weiterhin keinen Mess-Flow bestimmen.", gas.upper())
        else:
            _log.info("TEST: Gemessener %s-Flow nach Setpoint=%.3f: %.3f", gas.upper(), setpoint, last_flow)
        return last_flow