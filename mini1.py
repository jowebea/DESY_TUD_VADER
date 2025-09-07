# -*- coding: utf-8 -*-
"""
mini1.py
- MINI1 Treiber (ASCII-Zeilen mit Druck)
- MINI1 Emulator (_Mini1Emu)
"""

import logging
import threading
import time
import math
from collections import deque
from datetime import datetime, timezone
from typing import Optional, Tuple, Dict, Any, List, Union

from utilities import (
    MODULE_LOGGER_NAME, _hexdump, SerialDevice,
    _MockFirmwareMailbox, _BaseEmu, _MockEndpoint
)

# ====================== MINI1 Emulator ======================

class _Mini1Emu(_BaseEmu):
    """Erzeugt fortlaufend ASCII-Druckwerte (kPa) mit sanfter Dynamik."""
    def __init__(self, ep: _MockEndpoint):
        super().__init__(ep, "MINI1-EMU")
        self.p = 101.3  # Startdruck (kPa)
        self._t0 = time.time()
        self._target = self.p

    def _apply_host_cmds(self, data: bytes) -> None:
        try:
            text = data.decode(errors="ignore")
            for line in text.splitlines():
                if line.startswith("setpoint:"):
                    v = float(line.split(":", 1)[1])
                    old = self._target
                    self._target = max(0.0, min(200.0, v))
                    self.logger.info("Host setpoint changed: %.2f -> %.2f", old, self._target)
        except Exception as e:
            self.logger.exception("apply_host_cmds failed: %s", e)

    def _dynamics(self, dt: float) -> None:
        tau = 2.5
        self.p += (self._target - self.p) * (1 - math.exp(-dt / tau))
        wiggle = 0.05 * math.sin(2 * math.pi * (time.time() - self._t0) / 10.0)
        noise = 0.02 * (2 * (hash((int(time.time()*10), id(self))) & 1) - 1)
        self.p = max(0.0, self.p + wiggle + noise)

    def _loop(self) -> None:
        last = time.time()
        acc = 0.0
        while self._running:
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                self._apply_host_cmds(data)

            now = time.time()
            dt = now - last
            last = now
            acc += dt
            self._dynamics(dt)

            if acc >= 0.02:
                acc = 0.0
                line = f"{self.p:.2f}\n".encode()
                self.ep.feed(line)
            time.sleep(0.005)
        self.logger.debug("Loop exited")

# ====================== MINI1 Treiber ======================

class MINI1:
    """
    MINI1 liefert weiterhin ASCII-Zahlenzeilen (Druck). Wir puffern eine Zeitreihe.
    """
    def __init__(self, port: str, max_samples: int = 300_000):
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.MINI1")
        self.dev = SerialDevice(port, 115200, name="MINI1")
        self._series: deque[Tuple[datetime, float]] = deque(maxlen=max_samples)
        self._lock = threading.Lock()
        self._running = True
        self.logger.info("MINI1 started (max_samples=%d)", max_samples)
        threading.Thread(target=self._reader_loop, daemon=True, name="MINI1-reader").start()

    @staticmethod
    def _compute_pressure(p20: float, p21: float) -> float:
        return p20 if (p21 < 150 and abs(p21 - p20) < 5) else p21

    def _reader_loop(self) -> None:
        while self._running:
            line = self.dev.readline()
            if not line:
                time.sleep(0.0005)
                continue
            try:
                pressure = float(line)
            except ValueError:
                self.logger.warning("Invalid MINI1 line: %r", line)
                continue
            with self._lock:
                self._series.append((datetime.now(timezone.utc), pressure))
        self.logger.debug("Reader loop exited")

    def get_last_n(self, n: Optional[int] = None) -> List[Tuple[datetime, float]]:
        with self._lock:
            data = list(self._series)
        self.logger.debug("get_last_n(%s) -> %d samples", n, len(data) if n is None else min(n, len(data)))
        return data if n is None or n >= len(data) else data[-n:]

    @property
    def pressure(self) -> Optional[float]:
        with self._lock:
            val = self._series[-1][1] if self._series else None
        self.logger.debug("pressure -> %s", "None" if val is None else f"{val:.2f} kPa")
        return val

    def set_max_samples(self, max_samples: int) -> None:
        from collections import deque as _dq
        with self._lock:
            self._series = _dq(self._series, maxlen=max_samples)
        self.logger.info("Set max_samples -> %d", max_samples)

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()
        self.logger.info("Stopped")

    def __enter__(self): return self
    def __exit__(self, *exc): self.stop()