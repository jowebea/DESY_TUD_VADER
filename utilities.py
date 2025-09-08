# -*- coding: utf-8 -*-
"""
utilities.py
Gemeinsame Utilities:
- Logging-Helper & Defaults
- In-Memory-Serial-Bus (Mock-Endpunkte, Registry, Firmware-Mailbox)
- Basis-Emulator-Klasse + Echo-Emulator
- SerialDevice-Wrapper (nutzt mock:// Ports oder echte serielle Ports)

Hinweis: Öffentliche APIs der Treiber bleiben unverändert.
"""

import logging
import threading
import time
from collections import deque
from typing import Optional, Tuple, Dict, Any
import os
import struct

MODULE_LOGGER_NAME = "vader.mock"
_log = logging.getLogger(MODULE_LOGGER_NAME)

# Maximale Länge für Hexdumps in Logs
MAX_HEXDUMP = int(os.environ.get("VADER_MAX_HEXDUMP", "64"))

def _hexdump(data: bytes, max_len: int = MAX_HEXDUMP) -> str:
    if not data:
        return "<empty>"
    view = data[:max_len]
    hexs = " ".join(f"{b:02X}" for b in view)
    if len(data) > max_len:
        hexs += f" … (+{len(data)-max_len} B)"
    return hexs

def _safebytes(b: Optional[bytes]) -> int:
    return 0 if b is None else len(b)

# Optional: pyserial kapseln (nicht notwendig für mock://)
try:
    import serial  # type: ignore
except Exception:
    serial = None  # noqa: F401

# ===================== In-Memory Serial Bus =====================

class _MockEndpoint:
    """Endpunkt mit RX-Puffer und Condvar für blockierendes Lesen."""
    def __init__(self, name: str):
        self.name = name
        self._rx = deque()  # type: deque[bytes]
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._open = True
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}._MockEndpoint.{self.name}")
        self.logger.debug("Endpoint created")

    def feed(self, data: bytes) -> None:
        with self._cv:
            if not self._open:
                self.logger.debug("feed() called while closed; dropping %d B", len(data))
                return
            if data:
                self._rx.append(data)
                self._cv.notify_all()
                self.logger.debug("feed -> %d B to RX (hex: %s)", len(data), _hexdump(data))

    def read(self, n: int, timeout: float) -> bytes:
        deadline = time.time() + timeout
        out = bytearray()
        with self._cv:
            while len(out) < n and self._open:
                while self._rx and len(out) < n:
                    chunk = self._rx.popleft()
                    need = n - len(out)
                    if len(chunk) <= need:
                        out.extend(chunk)
                    else:
                        out.extend(chunk[:need])
                        rest = chunk[need:]
                        self._rx.appendleft(rest)
                if len(out) >= n or not self._open:
                    break
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._cv.wait(timeout=remaining)
        data = bytes(out)
        if data:
            self.logger.debug("read(%d, %.3fs) -> %d B (hex: %s)", n, timeout, len(data), _hexdump(data))
        else:
            self.logger.debug("read(%d, %.3fs) -> timeout/empty", n, timeout)
        return data

    def readline(self, timeout: float) -> Optional[bytes]:
        deadline = time.time() + timeout
        buf = bytearray()
        with self._cv:
            while self._open:
                while self._rx:
                    chunk = self._rx.popleft()
                    buf.extend(chunk)
                    idx = buf.find(b"\n")
                    if idx >= 0:
                        line = bytes(buf[:idx+1])
                        rest = bytes(buf[idx+1:])
                        buf.clear()
                        if rest:
                            self._rx.appendleft(rest)
                        self.logger.debug("readline(%.3fs) -> %d B (hex: %s)", timeout, len(line), _hexdump(line))
                        return line
                remaining = deadline - time.time()
                if remaining <= 0:
                    self.logger.debug("readline(%.3fs) -> timeout", timeout)
                    return None
                self._cv.wait(timeout=remaining)
        self.logger.debug("readline -> closed")
        return None

    def close(self) -> None:
        with self._cv:
            self._open = False
            self._cv.notify_all()
            self.logger.debug("Endpoint closed")

class _MockFirmwareMailbox:
    """Zwischenpuffer für vom Host gesendete Daten (Write)."""
    _lock = threading.Lock()
    _queues: Dict[int, deque] = {}
    _logger = logging.getLogger(f"{MODULE_LOGGER_NAME}._MockFirmwareMailbox")

    @classmethod
    def post(cls, endpoint: _MockEndpoint, data: bytes) -> None:
        key = id(endpoint)
        with cls._lock:
            q = cls._queues.setdefault(key, deque())
            q.append(data)
        cls._logger.debug("post -> ep=%s %d B (hex: %s)", endpoint.name, len(data), _hexdump(data))

    @classmethod
    def get(cls, endpoint: _MockEndpoint, max_bytes: int = 4096) -> bytes:
        key = id(endpoint)
        out = bytearray()
        with cls._lock:
            q = cls._queues.setdefault(key, deque())
            while q and len(out) < max_bytes:
                chunk = q.popleft()
                if len(out) + len(chunk) <= max_bytes:
                    out.extend(chunk)
                else:
                    need = max_bytes - len(out)
                    out.extend(chunk[:need])
                    rest = chunk[need:]
                    q.appendleft(rest)
                    break
        data = bytes(out)
        if data:
            cls._logger.debug("get  <- ep=%s %d B (hex: %s)", endpoint.name, len(data), _hexdump(data))
        return data

class _MockSerialRegistry:
    """Registry für 'mock://...' Ports und Firmware-Emulatoren."""
    _inst = None

    def __init__(self):
        self._lock = threading.Lock()
        self._endpoints: Dict[str, _MockEndpoint] = {}
        self._emulators: Dict[str, "_BaseEmu"] = {}
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}._MockSerialRegistry")

    @classmethod
    def get(cls) -> "_MockSerialRegistry":
        if cls._inst is None:
            cls._inst = _MockSerialRegistry()
        return cls._inst

    def attach(self, uri: str) -> _MockEndpoint:
        with self._lock:
            if uri not in self._endpoints:
                self.logger.info("Attach new endpoint: %s", uri)
                ep = _MockEndpoint(uri)
                self._endpoints[uri] = ep
                self._ensure_emu(uri, ep)
            else:
                self.logger.debug("Attach existing endpoint: %s", uri)
            return self._endpoints[uri]

    def _ensure_emu(self, uri: str, ep: _MockEndpoint) -> None:
        if uri in self._emulators:
            return
        # Emulator-Zuweisung erfolgt in den Teilmodulen (mini1/mini2/maxi)
        # Um Zyklen zu vermeiden: Lazy-Import hier.
        if uri == "mock://mini1":
            from mini1 import _Mini1Emu
            emu = _Mini1Emu(ep)
        elif uri == "mock://mini2":
            from mini2 import _Mini2Emu
            emu = _Mini2Emu(ep)
        elif uri == "mock://maxi":
            from maxi import _MaxiEmu
            emu = _MaxiEmu(ep)
        else:
            emu = _EchoEmu(ep)  # Fallback
        self._emulators[uri] = emu
        self.logger.info("Starting emulator for %s: %s", uri, emu.name)
        emu.start()

    def close(self, uri: str) -> None:
        with self._lock:
            self.logger.info("Close requested for %s", uri)
            ep = self._endpoints.get(uri)
            if ep:
                ep.close()
            emu = self._emulators.get(uri)
            if emu:
                emu.stop()

# ====================== SerialDevice Wrapper ======================

class SerialDevice:
    """
    Dünne Serial-Wrapper-Klasse mit Lock.
    Unterstützt Bytes IO + Line IO.
    """
    def __init__(self, port: str, baudrate: int = 115200, name: str = ""):
        self.name = name or "SERIAL"
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.SerialDevice.{self.name}")
        self._lock = threading.Lock()
        self._mock = None  # type: Optional[_MockEndpoint]
        self._timeout = 0.1
        self._serial = None  # wird nur gesetzt, wenn kein mock://

        # WICHTIG: mock:// zuerst behandeln – kein pyserial.open() davor!
        if port.startswith("mock://"):
            self.logger.info("Using mock endpoint for %s @ %s", self.name, port)
            self._mock = _MockSerialRegistry.get().attach(port)
            return

        # Reale serielle Schnittstelle
        if serial is None:
            raise RuntimeError("pyserial nicht verfügbar und kein 'mock://'-Port angegeben.")
        self.logger.info("Opening real serial for %s: port=%s baud=%d", self.name, port, baudrate)
        self._serial = serial.Serial(port, baudrate, timeout=self._timeout)
        try:
            self._serial.setDTR(True)
            self._serial.setRTS(True)
        except Exception:
            pass

    # --- Byte-IO ---
    def write_bytes(self, data: bytes) -> None:
        with self._lock:
            self.logger.debug("write_bytes(%d B): %s", len(data), _hexdump(data))
            if self._mock:
                _MockFirmwareMailbox.post(self._mock, data)
            else:
                self._serial.write(data)  # type: ignore

    def read_bytes(self, n: int) -> bytes:
        try:
            if self._mock:
                data = self._mock.read(n, timeout=self._timeout)
            else:
                data = self._serial.read(n)  # type: ignore
            if data:
                self.logger.debug("read_bytes(%d) -> %d B: %s", n, len(data), _hexdump(data))
            else:
                self.logger.debug("read_bytes(%d) -> empty/timeout", n)
            return data
        except Exception as e:
            self.logger.exception("read_bytes(%d) failed: %s", n, e)
            return b""

    # --- Line-IO ---
    def write_line(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        data = line.encode()
        with self._lock:
            self.logger.debug("write_line(%d B): %r", len(data), line.strip())
            if self._mock:
                _MockFirmwareMailbox.post(self._mock, data)
            else:
                self._serial.write(data)  # type: ignore

    def readline(self) -> Optional[str]:
        try:
            if self._mock:
                b = self._mock.readline(timeout=self._timeout)
                if not b:
                    self.logger.debug("readline -> None")
                    return None
                s = b.decode(errors="ignore").strip()
                self.logger.debug("readline -> %r", s)
                return s
            else:
                line = self._serial.readline()  # type: ignore
                if not line:
                    self.logger.debug("readline(real) -> None")
                    return None
                s = line.decode(errors="ignore").strip()
                self.logger.debug("readline(real) -> %r", s)
                return s
        except Exception as e:
            self.logger.exception("readline failed: %s", e)
            return None

    def close(self) -> None:
        try:
            if self._mock:
                self.logger.info("close() mock endpoint (no-op for shared)")
            else:
                self.logger.info("close() real serial")
                self._serial.close()  # type: ignore
        except Exception as e:
            self.logger.warning("close() ignored exception: %s", e)

# ====================== Emulator-Basis + Echo ======================

class _BaseEmu:
    def __init__(self, ep: _MockEndpoint, name: str):
        self.ep = ep
        self.name = name
        self._running = False
        self._t: Optional[threading.Thread] = None
        self.logger = logging.getLogger(f"{MODULE_LOGGER_NAME}.{self.name}")

    def start(self) -> None:
        self._running = True
        self._t = threading.Thread(target=self._loop, daemon=True, name=self.name)
        self._t.start()
        self.logger.debug("Thread started")

    def stop(self) -> None:
        self._running = False
        self.logger.debug("Stop requested")

    def _loop(self) -> None:
        raise NotImplementedError

class _EchoEmu(_BaseEmu):
    def __init__(self, ep: _MockEndpoint):
        super().__init__(ep, "ECHO-EMU")

    def _loop(self) -> None:
        while self._running:
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                self.logger.debug("echo %d B", len(data))
                self.ep.feed(data)
            time.sleep(0.01)
        self.logger.debug("Loop exited")