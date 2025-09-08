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
    _log.info("Starte Demo mit realen Ports (unten auskommentiert auch mock)")
    # with VaderDeviceDriver("mock://mini1", "mock://mini2", "mock://maxi") as drv:
    with VaderDeviceDriver(
        "/dev/tty.usbmodem1133101",
        "mock://mini2",  # "/dev/tty.usbmodem1133301",
        "/dev/tty.usbmodem1133201"
    ) as drv:
        _log.info("Starte Mock…")
        time.sleep(3)
        _log.info("MINI2 Status: %s", drv.mini2_get_status())
        drv.setpoint_pressure(50.0)
        time.sleep(1.0)
        time.sleep(0.2)
        _log.info("MAXI Status: %s", drv.maxi_get_status())
        _log.info("MINI1 letzter Druck: %s", drv.mini1.pressure)
        drv.set_ramp(5.0, 80.0)
        time.sleep(1.5)
        _log.info("ALL: %s", list(drv.get_all_status(mini1_last_n=5).keys()))