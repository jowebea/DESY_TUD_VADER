# -*- coding: utf-8 -*-
"""
maxi.py
- MAXI Treiber (neues Protokoll mit Sync + Payload + CS)
- MAXI Emulator (_MaxiEmu)
"""

import logging
import threading
import time
import math
import struct
from typing import Dict, Any, Union

from utilities import (
    MODULE_LOGGER_NAME, _hexdump, SerialDevice,
    _MockFirmwareMailbox, _BaseEmu, _MockEndpoint
)

# ====================== MAXI Emulator ======================

class _MaxiEmu(_BaseEmu):
    """
    Emuliert MAXI-Protokoll.
    Status:
      0xAA 0x55, LEN=25, PAYLOAD[LEN], CS=sum(PAYLOAD)&0xFF
    SET:
      0xA5 0x5A 0x01 0x04  PAYLOAD(le32)  CS
      PAYLOAD le32: WORD = (DATA26<<6) | ADDR6
    """
    SYNC0, SYNC1 = 0xAA, 0x55
    H0, H1, CMD_SET, SET_LEN = 0xA5, 0x5A, 0x01, 0x04

    # ADDR6
    A_RequestStatus = 0x00
    A_V1, A_V2, A_V3 = 0x01, 0x02, 0x03
    A_FanIn, A_FanOut = 0x04, 0x05
    A_FlowCO2, A_TotCO2 = 0x10, 0x11
    A_FlowN2,  A_TotN2  = 0x12, 0x13
    A_FlowBut, A_TotBut = 0x14, 0x15

    STATUS_LEN = 25

    def __init__(self, ep: _MockEndpoint):
        super().__init__(ep, "MAXI-EMU")
        self._rx = bytearray()
        self.io = {"V1": False, "V2": False, "V3": False, "FanIn": False, "FanOut": False, "GasLeak": False}
        self.adc_P11 = 1234
        self.adc_P31 = 2345
        self.flow = {"co2": 0.0, "n2": 0.0, "but": 0.0}
        self.total = {"co2": 0.0, "n2": 0.0, "but": 0.0}
        self._last = time.time()

    # Helpers
    @staticmethod
    def _sumcs(b: bytes) -> int:
        return sum(b) & 0xFF

    @staticmethod
    def _le16(v: int) -> bytes:
        return struct.pack("<H", v & 0xFFFF)

    @staticmethod
    def _le32(v: int) -> bytes:
        return struct.pack("<I", v & 0xFFFFFFFF)

    def _apply_set(self, addr6: int, data26: int) -> None:
        self.logger.debug("SET addr=0x%02X data=%d", addr6, data26)
        if addr6 == self.A_RequestStatus:
            self._send_status()
            return
        if addr6 == self.A_V1:     self.io["V1"] = (data26 != 0)
        elif addr6 == self.A_V2:   self.io["V2"] = (data26 != 0)
        elif addr6 == self.A_V3:   self.io["V3"] = (data26 != 0)
        elif addr6 == self.A_FanIn:  self.io["FanIn"] = (data26 != 0)
        elif addr6 == self.A_FanOut: self.io["FanOut"] = (data26 != 0)
        elif addr6 == self.A_FlowCO2: self.flow["co2"] = min(9999.0, data26/100.0)
        elif addr6 == self.A_FlowN2:  self.flow["n2"]  = min(9999.0, data26/100.0)
        elif addr6 == self.A_FlowBut: self.flow["but"] = min(9999.0, data26/100.0)
        elif addr6 == self.A_TotCO2:  self.total["co2"] = data26/100.0
        elif addr6 == self.A_TotN2:   self.total["n2"]  = data26/100.0
        elif addr6 == self.A_TotBut:  self.total["but"] = data26/100.0

    def _host_write_bytes(self, data: bytes) -> None:
        self._rx.extend(data)
        self.logger.debug("RX += %d B (hex: %s)", len(data), _hexdump(data))
        while True:
            if len(self._rx) < 6:
                return
            h0, h1, cmd, ln = self._rx[0], self._rx[1], self._rx[2], self._rx[3]
            if h0 != self.H0 or h1 != self.H1 or cmd != self.CMD_SET or ln != self.SET_LEN:
                self.logger.warning("Desync byte (0x%02X) - resync", self._rx[0])
                self._rx.pop(0)
                continue
            total = 4 + ln + 1
            if len(self._rx) < total:
                return
            payload = bytes(self._rx[4:4+ln])
            cs = self._rx[4+ln]
            del self._rx[:total]
            if self._sumcs(payload) != cs:
                self.logger.warning("Checksum mismatch: exp=%02X got=%02X payload=%s", self._sumcs(payload), cs, _hexdump(payload))
                continue
            import struct as _st
            word, = _st.unpack("<I", payload)
            addr6 = word & 0x3F
            data26 = (word >> 6) & 0x03FF_FFFF
            self._apply_set(addr6, data26)

    def _dynamics(self, dt: float) -> None:
        self.total["co2"] += self.flow["co2"] * dt / 60.0
        self.total["n2"]  += self.flow["n2"]  * dt / 60.0
        self.total["but"] += self.flow["but"] * dt / 60.0
        self.adc_P11 = int(1500 + 300*math.sin(time.time()/7))
        self.adc_P31 = int(2200 + 400*math.sin(time.time()/9 + 1.0))
        self.io["GasLeak"] = (int(time.time()) % 127 == 0)

    def _pack_status_payload(self) -> bytes:
        flags = (
            (1 if self.io["GasLeak"] else 0) |
            (1 if self.io["V1"] else 0) << 1 |
            (1 if self.io["V2"] else 0) << 2 |
            (1 if self.io["V3"] else 0) << 3 |
            (1 if self.io["FanIn"] else 0) << 4 |
            (1 if self.io["FanOut"] else 0) << 5
        )

        p = bytearray()
        p.append(flags & 0xFF)
        p += self._le16(self.adc_P11)
        p += self._le16(self.adc_P31)
        p += self._le16(int(round(self.flow["co2"]*100)))
        p += self._le32(int(round(self.total["co2"]*100)))
        p += self._le16(int(round(self.flow["n2"]*100)))
        p += self._le32(int(round(self.total["n2"]*100)))
        p += self._le16(int(round(self.flow["but"]*100)))
        p += self._le32(int(round(self.total["but"]*100)))
        while len(p) < self.STATUS_LEN:
            p.append(0)
        return bytes(p[:self.STATUS_LEN])

    def _send_status(self) -> None:
        payload = self._pack_status_payload()
        frame = bytes([self.SYNC0, self.SYNC1, len(payload)]) + payload + bytes([self._sumcs(payload)])
        self.ep.feed(frame)
        self.logger.debug("Status sent (%d B payload, cs=%02X): %s", len(payload), self._sumcs(payload), _hexdump(frame))

    def _loop(self) -> None:
        acc = 0.0
        while self._running:
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                self._host_write_bytes(data)
            now = time.time()
            if not hasattr(self, "_t_prev"):
                self._t_prev = now
            dt = now - self._t_prev
            self._t_prev = now
            self._dynamics(dt)
            acc += dt
            if acc >= 0.04:
                acc = 0.0
                self._send_status()
            time.sleep(0.005)
        self.logger.debug("Loop exited")

# ====================== MAXI Treiber ======================

class MAXI:
    """
    MAXI-Treiber (API unverändert), spricht mit MAXI-Emu oben.
    """
    SYNC0 = 0xAA
    SYNC1 = 0x55
    STATUS_LEN = 25

    H0 = 0xA5
    H1 = 0x5A
    CMD_SET = 0x01
    SET_LEN = 0x04

    A_RequestStatus = 0x00
    A_V1, A_V2, A_V3 = 0x01, 0x02, 0x03
    A_FanIn, A_FanOut = 0x04, 0x05
    A_FlowCO2, A_TotCO2 = 0x10, 0x11
    A_FlowN2,  A_TotN2  = 0x12, 0x13
    A_FlowBut, A_TotBut = 0x14, 0x15

    def __init__(self, port: str):
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.MAXI")
        self.dev = SerialDevice(port, 115200, name="MAXI")
        self._lock = threading.Lock()
        self._running = True
        self._rx = bytearray()
        self._last_status_ts = 0.0

        self.status: Dict[str, Any] = {
            "io": {"V1": False, "V2": False, "V3": False, "FanIn": False, "FanOut": False, "GasLeak": False},
            "adc": {"P11": None, "P31": None},
            "mfc": {
                "co2":  {"flow": None, "total": None},
                "n2":   {"flow": None, "total": None},
                "butan":{"flow": None, "total": None},
            },
        }

        threading.Thread(target=self._reader_loop, daemon=True, name="MAXI-reader").start()
        self.request_status()
        self._wait_for_first_status(5.0)
        self.logger.info("MAXI started")

    @staticmethod
    def _u26_from_float100(x: Union[int, float]) -> int:
        v = int(round(float(x) * 100.0))
        if v < 0: v = 0
        if v > 0x03FF_FFFF: v = 0x03FF_FFFF
        return v

    @staticmethod
    def _le32(word: int) -> bytes:
        return bytes([word & 0xFF, (word >> 8) & 0xFF, (word >> 16) & 0xFF, (word >> 24) & 0xFF])

    @staticmethod
    def _checksum_sum(data: bytes) -> int:
        return sum(data) & 0xFF
    
    def _wait_for_first_status(self, timeout: float = 5.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self._last_status_ts > 0:
                return True
            time.sleep(0.02)
        return False

    def _send_set(self, addr6: int, data26: int) -> None:
        word = ((data26 & 0x03FF_FFFF) << 6) | (addr6 & 0x3F)
        payload = self._le32(word)
        frame = bytes([self.H0, self.H1, self.CMD_SET, self.SET_LEN]) + payload + bytes([self._checksum_sum(payload)])
        self.logger.debug("TX SET addr=0x%02X data=%d -> %s", addr6, data26, _hexdump(frame))
        self.dev.write_bytes(frame)

    def request_status(self) -> None:
        self.logger.info("Request status")
        self._send_set(self.A_RequestStatus, 1)

    def set_v1(self, open_: bool) -> None:    self.logger.info("set_v1(%s)", open_); self._send_set(self.A_V1,    1 if open_ else 0)
    def set_v2(self, open_: bool) -> None:    self.logger.info("set_v2(%s)", open_); self._send_set(self.A_V2,    1 if open_ else 0)
    def set_v3(self, open_: bool) -> None:    self.logger.info("set_v3(%s)", open_); self._send_set(self.A_V3,    1 if open_ else 0)
    def set_fan_in(self, on: bool) -> None:   self.logger.info("set_fan_in(%s)", on); self._send_set(self.A_FanIn, 1 if on else 0)
    def set_fan_out(self, on: bool) -> None:  self.logger.info("set_fan_out(%s)", on); self._send_set(self.A_FanOut,1 if on else 0)

    def set_flow_co2(self, flow: float) -> None:   self.logger.info("set_flow_co2(%.2f)", flow); self._send_set(self.A_FlowCO2, self._u26_from_float100(flow))
    def set_flow_n2(self, flow: float) -> None:    self.logger.info("set_flow_n2(%.2f)", flow);  self._send_set(self.A_FlowN2,  self._u26_from_float100(flow))
    def set_flow_butan(self, flow: float) -> None: self.logger.info("set_flow_butan(%.2f)", flow); self._send_set(self.A_FlowBut, self._u26_from_float100(flow))

    def set_total_co2(self, value: float = 0.0) -> None:   self.logger.info("reset_total_co2(%.2f)", value); self._send_set(self.A_TotCO2, self._u26_from_float100(value))
    def set_total_n2(self, value: float = 0.0) -> None:    self.logger.info("reset_total_n2(%.2f)", value); self._send_set(self.A_TotN2,  self._u26_from_float100(value))
    def set_total_butan(self, value: float = 0.0) -> None: self.logger.info("reset_total_butan(%.2f)", value); self._send_set(self.A_TotBut, self._u26_from_float100(value))

    def _reader_loop(self) -> None:
        while self._running:
            chunk = self.dev.read_bytes(64)
            if chunk:
                self._rx.extend(chunk)
                self.logger.debug("RX += %d B (hex: %s)", len(chunk), _hexdump(chunk))
                self._drain_status_frames()
            else:
                time.sleep(0.002)
        self.logger.debug("Reader loop exited")

    def _find_sync(self) -> int:
        for i in range(max(0, len(self._rx) - 1)):
            if self._rx[i] == self.SYNC0 and self._rx[i + 1] == self.SYNC1:
                return i
        return -1

    @staticmethod
    def _le16(b: bytes, off: int) -> int:
        return b[off] | (b[off + 1] << 8)

    @staticmethod
    def _le32_val(b: bytes, off: int) -> int:
        return b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24)

    def _drain_status_frames(self) -> None:
        while True:
            idx = self._find_sync()
            if idx < 0:
                if len(self._rx) > 2048:
                    del self._rx[:-2]
                return
            if idx > 0:
                self.logger.warning("Discarding %d B before sync", idx)
                del self._rx[:idx]
            if len(self._rx) < 3:
                return
            length = self._rx[2]
            total = 3 + length + 1
            if len(self._rx) < total:
                return
            payload = bytes(self._rx[3:3 + length])
            cs = self._rx[3 + length]
            del self._rx[:total]

            if length not in (self.STATUS_LEN, 23):
                self.logger.warning("Unexpected length=%d (payload=%s)", length, _hexdump(payload))
                continue
            if (self._checksum_sum(payload) & 0xFF) != cs:
                self.logger.warning("Checksum mismatch: exp=%02X got=%02X payload=%s", self._checksum_sum(payload), cs, _hexdump(payload))
                continue
            try:
                self._apply_status_payload(payload)
            except Exception as e:
                self.logger.exception("apply_status_payload failed: %s", e)
                continue

    def _apply_status_payload(self, p: bytes) -> None:
        flags = p[0]
        P11 = self._le16(p, 1)
        P31 = self._le16(p, 3)
        flowCO2 = self._le16(p, 5) / 100.0
        totCO2  = self._le32_val(p, 7) / 100.0
        flowN2  = self._le16(p, 11) / 100.0
        totN2   = self._le32_val(p, 13) / 100.0
        flowBut = self._le16(p, 17) / 100.0
        totBut  = self._le32_val(p, 19) / 100.0

        with self._lock:
            self.status["io"]["GasLeak"] = bool(flags & (1 << 0))
            self.status["io"]["V1"]      = bool(flags & (1 << 1))
            self.status["io"]["V2"]      = bool(flags & (1 << 2))
            self.status["io"]["V3"]      = bool(flags & (1 << 3))
            self.status["io"]["FanIn"]   = bool(flags & (1 << 4))
            self.status["io"]["FanOut"]  = bool(flags & (1 << 5))

            self.status["adc"]["P11"] = P11
            self.status["adc"]["P31"] = P31

            self.status["mfc"]["co2"]["flow"]    = flowCO2
            self.status["mfc"]["co2"]["total"]   = totCO2
            self.status["mfc"]["n2"]["flow"]     = flowN2
            self.status["mfc"]["n2"]["total"]    = totN2
            self.status["mfc"]["butan"]["flow"]  = flowBut
            self.status["mfc"]["butan"]["total"] = totBut

            self._last_status_ts = time.time()
        self.logger.debug("Status: IO=%s ADC=%s CO2(%.2f/%.2f) N2(%.2f/%.2f) BUT(%.2f/%.2f)",
                          self.status["io"], self.status["adc"],
                          flowCO2, totCO2, flowN2, totN2, flowBut, totBut)

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            out = {"ts": self._last_status_ts, **self.status}
        self.logger.debug("get_status -> ts=%.3f", self._last_status_ts)
        return out

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()
        self.logger.info("Stopped")