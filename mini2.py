# -*- coding: utf-8 -*-
"""
mini2.py
- MINI2 Treiber (Binärprotokoll, 32-bit Big-Endian Words)
- MINI2 Emulator (_Mini2Emu)
"""

import logging
import threading
import time
import math
from typing import Optional, Tuple, Dict, Any, List, Union

from utilities import (
    MODULE_LOGGER_NAME, _hexdump, SerialDevice,
    _MockFirmwareMailbox, _BaseEmu, _MockEndpoint
)

# ====================== MINI2 Emulator ======================

class _Mini2Emu(_BaseEmu):
    """
    Emuliert das MINI2 Binärprotokoll.
    - Setpoint, MANUAL (PWM), Ramp
    - Liefert STATUS-Frames für BASE+2/3/4
    """
    def __init__(self, ep: _MockEndpoint, base_addr: int = 0x05):
        super().__init__(ep, "MINI2-EMU")
        self.base = base_addr & 0x3F
        self._rx = bytearray()
        self.mode = "MANUAL"  # MANUAL | AUTOMATIC | RAMP
        self.vp1 = 0
        self.vp2 = 0
        self.setpoint_kpa = 0.0
        self.p20 = 101.3
        self.p21 = 101.2
        self.ramp_active = False
        self.ramp_speed = 0.0  # kPa/s
        self.ramp_end = 0.0    # kPa
        self._last = time.time()

    # Hilfen
    @staticmethod
    def _be_to_u32(b: bytes) -> int:
        return (b[0]<<24)|(b[1]<<16)|(b[2]<<8)|b[3]

    @staticmethod
    def _u32_to_be(word: int) -> bytes:
        return bytes([(word>>24)&0xFF,(word>>16)&0xFF,(word>>8)&0xFF,word&0xFF])

    @staticmethod
    def _mk_word(addr6: int, payload26: int) -> int:
        return ((addr6 & 0x3F) << 26) | (payload26 & 0x03FF_FFFF)

    def _host_write_bytes(self, data: bytes) -> None:
        self._rx.extend(data)
        self.logger.debug("RX += %d B (hex: %s)", len(data), _hexdump(data))
        while len(self._rx) >= 4:
            w = self._be_to_u32(self._rx[:4])
            del self._rx[:4]
            self.logger.debug("Handle set word: 0x%08X", w)
            self._handle_set_word(w)

    def _handle_set_word(self, w: int) -> None:
        addr = (w >> 26) & 0x3F
        payload = w & 0x03FF_FFFF

        if addr == (self.base + 6):  # COMMANDS
            if payload & 0x1:
                self.logger.debug("Request: send status triplet")
                self._send_status_triplet()
        elif addr == (self.base + 1):  # SETPOINT
            raw = (payload >> 10) & 0xFFFF
            old = self.setpoint_kpa
            self.setpoint_kpa = raw / 10.0
            self.logger.info("Setpoint: %.1f -> %.1f kPa", old, self.setpoint_kpa)
            self.mode = "AUTOMATIC"
            self.ramp_active = False
        elif addr == (self.base + 0):  # VP1/VP2 setzen -> MANUAL
            self.vp1 = (payload >> 16) & 0xFF
            self.vp2 = (payload >> 8) & 0xFF
            self.logger.info("Manual PWM: vp1=%d vp2=%d", self.vp1, self.vp2)
            self.mode = "MANUAL"
            self.ramp_active = False
        elif addr == (self.base + 5):  # RAMP
            raw_speed = (payload >> 10) & 0xFFFF
            raw_end = payload & 0x3FF
            self.ramp_speed = raw_speed / 100.0
            self.ramp_end = float(raw_end)
            self.logger.info("Ramp: speed=%.2f kPa/s -> end=%.1f kPa", self.ramp_speed, self.ramp_end)
            self.mode = "RAMP"
            self.ramp_active = True

    def _pressure_dynamics(self, dt: float) -> None:
        if self.mode == "MANUAL":
            target = 101.3 + 0.3*(self.vp1-128)/128.0*100 - 0.3*(self.vp2-128)/128.0*100
            tau = 3.0
            self.p21 += (target - self.p21) * (1 - math.exp(-dt/tau))
        elif self.mode == "AUTOMATIC":
            tau = 2.0
            self.p21 += (self.setpoint_kpa - self.p21) * (1 - math.exp(-dt/tau))
        elif self.mode == "RAMP":
            if self.ramp_active:
                direction = 1.0 if self.ramp_end > self.p21 else -1.0
                step = direction * self.ramp_speed * dt
                if (direction > 0 and self.p21 + step >= self.ramp_end) or (direction < 0 and self.p21 + step <= self.ramp_end):
                    self.p21 = self.ramp_end
                    self.ramp_active = False
                    self.mode = "AUTOMATIC"
                    self.setpoint_kpa = self.ramp_end
                    self.logger.debug("Ramp reached end -> switch to AUTOMATIC")
                else:
                    self.p21 += step
            else:
                tau = 2.0
                self.p21 += (self.ramp_end - self.p21) * (1 - math.exp(-dt/tau))

        self.p20 += (self.p21 - self.p20) * 0.3
        self.p20 += 0.02 * math.sin(time.time())

    def _send_status_triplet(self) -> None:
        mode2 = {"MANUAL":0, "RAMP":1, "AUTOMATIC":2}.get(self.mode, 3)
        p20_raw = int(round(max(0.0, min(0xFFFF/10.0, self.p20)) * 10))
        p21_raw = int(round(max(0.0, min(0xFFFF/10.0, self.p21)) * 10))

        w2 = self._mk_word(self.base + 2, (p20_raw & 0xFFFF) << 10)
        w3 = self._mk_word(self.base + 3, (p21_raw & 0xFFFF) << 10)
        w4 = self._mk_word(self.base + 4, ((mode2 & 0x3) << 24) | ((self.vp1 & 0xFF) << 16) | ((self.vp2 & 0xFF) << 8))

        for w in (w2, w3, w4):
            b = self._u32_to_be(w)
            self.ep.feed(b)
        self.logger.debug("Status triplet sent: p20=%.1f p21=%.1f mode=%s", p20_raw/10.0, p21_raw/10.0, self.mode)

    def _loop(self) -> None:
        tick = 0.0
        while self._running:
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                self._host_write_bytes(data)

            now = time.time()
            dt = now - self._last
            self._last = now
            self._pressure_dynamics(dt)

            tick += dt
            if tick >= 0.04:
                tick = 0.0
                self._send_status_triplet()
            time.sleep(0.005)
        self.logger.debug("Loop exited")

# ====================== MINI2 Treiber ======================

class MINI2:
    """
    MINI2 – Binärprotokoll (32-Bit Words, Big-Endian) – mit robustem Bring-up und Shutdown.
    """
    def __init__(self, port: str, base_addr: int = 0x05, boot_delay: float = 1.8):
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.MINI2")
        self.dev = SerialDevice(port, 115200, name="MINI2")
        self.base = int(base_addr) & 0x3F

        self._lock = threading.Lock()
        self._last_status_time = 0.0
        self.status: Dict[str, Any] = {}

        self._rx_buf = bytearray()
        self._running = True
        self.logger.info("MINI2 started (base=0x%02X)", self.base)

        # --- Robust wie MAXI: Boot-Reset abwarten & Eingangsbuffer leeren ---
        time.sleep(boot_delay)
        t0 = time.time()
        while time.time() - t0 < 0.25:
            junk = self.dev.read_bytes(512)
            if not junk:
                time.sleep(0.01)

        self._reader_thr = threading.Thread(target=self._reader_loop, daemon=True, name="MINI2-reader")
        self._reader_thr.start()

        self.request_status()
        self._wait_for_recent_status(0.8)

    # ---------- Word helpers ----------
    @staticmethod
    def _make_word(addr6: int, payload26: int) -> int:
        return ((addr6 & 0x3F) << 26) | (payload26 & 0x03FF_FFFF)

    @staticmethod
    def _u32_to_be_bytes(word: int) -> bytes:
        return bytes([(word >> 24) & 0xFF, (word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF])

    @staticmethod
    def _be_bytes_to_u32(b: bytes) -> int:
        return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]

    # ---------- TX ----------
    def _send_word(self, addr: int, payload: int) -> None:
        word = self._make_word(addr, payload)
        b = self._u32_to_be_bytes(word)
        self.logger.debug("TX word addr=0x%02X payload=0x%06X -> %s", addr & 0x3F, payload & 0x03FF_FFFF, _hexdump(b))
        self.dev.write_bytes(b)
        # winziger Delay, damit USB/CDC garantiert sendet
        time.sleep(0.003)

    # ---------- API ----------
    def request_status(self) -> None:
        self.logger.info("Request status")
        self._send_word(self.base + 6, 0x1)

    def set_target_pressure(self, kpa: Union[int, float]) -> None:
        raw = max(0, min(0xFFFF, int(round(float(kpa) * 10.0))))
        payload = (raw & 0xFFFF) << 10
        self.logger.info("Set target pressure: %.1f kPa (raw=%d)", float(kpa), raw)
        self._send_word(self.base + 1, payload)

    def set_manual_vp1(self, value: int) -> None:
        v1 = max(0, min(255, int(value)))
        v2 = int(self.status.get("pwm2", 0)) & 0xFF
        self.logger.info("Set manual vp1=%d (keep vp2=%d)", v1, v2)
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_manual_vp2(self, value: int) -> None:
        v2 = max(0, min(255, int(value)))
        v1 = int(self.status.get("pwm1", 0)) & 0xFF
        self.logger.info("Set manual vp2=%d (keep vp1=%d)", v2, v1)
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_state_manual(self) -> None:
        v1 = int(self.status.get("pwm1", 0)) & 0xFF
        v2 = int(self.status.get("pwm2", 0)) & 0xFF
        self.logger.info("Force MANUAL with vp1=%d vp2=%d", v1, v2)
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_ramp(self, speed_kpa_s: float, end_kpa: float) -> None:
        raw_speed = max(0, min(0xFFFF, int(round(speed_kpa_s * 100.0))))
        raw_end   = max(0, min(0x03FF,  int(round(end_kpa))))
        payload   = (raw_speed << 10) | raw_end
        self.logger.info("Set ramp: speed=%.2f kPa/s end=%.1f kPa (raw_speed=%d raw_end=%d)", speed_kpa_s, end_kpa, raw_speed, raw_end)
        self._send_word(self.base + 5, payload)

    # ---------- RX ----------
    def _reader_loop(self) -> None:
        while self._running:
            try:
                chunk = self.dev.read_bytes(64)
            except Exception as e:
                # Typisch beim gleichzeitigen close(): 'Bad file descriptor'
                self.logger.debug("reader_loop terminating due to read exception: %s", e)
                break

            if chunk:
                self._rx_buf.extend(chunk)
                self.logger.debug("RX += %d B (hex: %s)", len(chunk), _hexdump(chunk))
                self._drain_words()
            else:
                time.sleep(0.002)
        self.logger.debug("Reader loop exited")

    def _drain_words(self) -> None:
        while len(self._rx_buf) >= 4:
            word_bytes = bytes(self._rx_buf[:4])
            del self._rx_buf[:4]
            try:
                word = self._be_bytes_to_u32(word_bytes)
                self.logger.debug("Status word: 0x%08X", word)
                self._handle_status_word(word)
            except Exception as e:
                self.logger.exception("drain_words failed: %s", e)
                continue

    def _handle_status_word(self, word: int) -> None:
        addr = (word >> 26) & 0x3F
        payload = word & 0x03FF_FFFF
        with self._lock:
            now = time.time()
            self._last_status_time = now
            self.status["ts"] = now

            if addr == (self.base + 2):
                p20_raw = (payload >> 10) & 0xFFFF
                self.status["p20_kpa"] = float(p20_raw) * 0.1
            elif addr == (self.base + 3):
                p21_raw = (payload >> 10) & 0xFFFF
                self.status["p21_kpa"] = float(p21_raw) * 0.1
            elif addr == (self.base + 4):
                mode2 = (payload >> 24) & 0x03
                vp1   = (payload >> 16) & 0xFF
                vp2   = (payload >>  8) & 0xFF
                self.status["mode"] = {0: "MANUAL", 1: "RAMP", 2: "AUTOMATIC"}.get(mode2, "UNKNOWN")
                self.status["pwm1"] = vp1
                self.status["pwm2"] = vp2
        self.logger.debug("Status updated: %s", {k:self.status.get(k) for k in ("p20_kpa","p21_kpa","mode","pwm1","pwm2")})

    # ---------- Utils ----------
    def is_recent(self, max_age_s: float = 0.6) -> bool:
        with self._lock:
            ok = (time.time() - self._last_status_time) <= max_age_s
        self.logger.debug("is_recent(%.2f) -> %s", max_age_s, ok)
        return ok

    def _wait_for_recent_status(self, timeout: float = 0.8) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.is_recent(0.5):
                self.logger.debug("Recent status available")
                return True
            time.sleep(0.01)
        self.logger.warning("Timeout waiting for recent status (%.3fs)", timeout)
        return False

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            out = dict(self.status)
            p21 = out.get("p21_kpa"); p20 = out.get("p20_kpa")
            if p21 is not None and p20 is not None:
                best = p20 if (p21 < 150 and abs(p21 - p20) < 5) else p21
                out["pressure_kpa"] = best
            elif p21 is not None:
                out["pressure_kpa"] = p21
            elif p20 is not None:
                out["pressure_kpa"] = p20
        self.logger.debug("get_status -> %s", {k:out.get(k) for k in ("pressure_kpa","mode","pwm1","pwm2")})
        return out

    # ---------- Shutdown ----------
    def stop(self) -> None:
        self._running = False
        try:
            if hasattr(self, "_reader_thr") and self._reader_thr.is_alive():
                self._reader_thr.join(timeout=0.5)
        except Exception:
            pass
        try:
            self.dev.close()
        finally:
            self.logger.info("Stopped")