# -*- coding: utf-8 -*-
"""
Vader Mock Hardware Emulator
----------------------------
Drop-in-Mock für die vorhandenen Treiber-Klassen MINI1, MINI2, MAXI.
Statt echter serieller Ports werden In-Memory Verbindungen genutzt.
Portnamen:
  - "mock://mini1"
  - "mock://mini2"
  - "mock://maxi"

Hinweis:
- Public APIs von MINI1/MINI2/MAXI/VaderDeviceDriver bleiben unverändert.
- Die Simulatoren erzeugen plausible, deterministische Werte.
"""

import logging
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Optional, Tuple, Dict, Any, List, Union
import math
import struct

# ============================================================
# Optional: pyserial ist NICHT notwendig. Wir kapseln den Import.
try:
    import serial  # type: ignore
except Exception:  # kein echtes serial verfügbar – egal, wir mocken ohnehin
    serial = None  # noqa: F401

# ============================================================
# In-Memory Serial Bus & Devices
# ============================================================

class _MockEndpoint:
    """Endpunkt mit RX-Puffer und Condvar für blockierendes Lesen."""
    def __init__(self, name: str):
        self.name = name
        self._rx = deque()  # type: deque[bytes]
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._open = True

    def feed(self, data: bytes) -> None:
        with self._cv:
            if not self._open:
                return
            if data:
                self._rx.append(data)
                self._cv.notify_all()

    def read(self, n: int, timeout: float) -> bytes:
        deadline = time.time() + timeout
        out = bytearray()
        with self._cv:
            while len(out) < n and self._open:
                # vorhandene bytes entnehmen
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
                # warten bis Daten kommen oder Timeout
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._cv.wait(timeout=remaining)
        return bytes(out)

    def readline(self, timeout: float) -> Optional[bytes]:
        deadline = time.time() + timeout
        buf = bytearray()
        with self._cv:
            while self._open:
                # vorhandene Bytes konsumieren bis \n
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
                        return line
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._cv.wait(timeout=remaining)
        return None

    def close(self) -> None:
        with self._cv:
            self._open = False
            self._cv.notify_all()


class _MockSerialRegistry:
    """Registry für 'mock://...' Ports und Firmware-Emulatoren."""
    _inst = None

    def __init__(self):
        self._lock = threading.Lock()
        self._endpoints: Dict[str, _MockEndpoint] = {}
        self._emulators: Dict[str, "_BaseEmu"] = {}
        # Lazy initialisiert

    @classmethod
    def get(cls) -> "_MockSerialRegistry":
        if cls._inst is None:
            cls._inst = _MockSerialRegistry()
        return cls._inst

    def attach(self, uri: str) -> _MockEndpoint:
        with self._lock:
            if uri not in self._endpoints:
                ep = _MockEndpoint(uri)
                self._endpoints[uri] = ep
                # Zugehörigen Emulator starten (einmal pro URI)
                self._ensure_emu(uri, ep)
            return self._endpoints[uri]

    def _ensure_emu(self, uri: str, ep: _MockEndpoint) -> None:
        if uri in self._emulators:
            return
        # Map URI -> Emulator
        if uri == "mock://mini1":
            emu = _Mini1Emu(ep)
        elif uri == "mock://mini2":
            emu = _Mini2Emu(ep)
        elif uri == "mock://maxi":
            emu = _MaxiEmu(ep)
        else:
            emu = _EchoEmu(ep)  # Fallback
        self._emulators[uri] = emu
        emu.start()

    def close(self, uri: str) -> None:
        with self._lock:
            ep = self._endpoints.get(uri)
            if ep:
                ep.close()
            emu = self._emulators.get(uri)
            if emu:
                emu.stop()


# ============================================================
# SerialDevice Wrapper (nutzt Mock, wenn "mock://..." Port)
# ============================================================

class SerialDevice:
    """
    Dünne Serial-Wrapper-Klasse mit Lock.
    Unterstützt Bytes IO + Line IO.
    """
    def __init__(self, port: str, baudrate: int = 115200, name: str = ""):
        self.name = name
        self._lock = threading.Lock()
        self._mock = None  # type: Optional[_MockEndpoint]
        self._timeout = 0.1

        if port.startswith("mock://"):
            self._mock = _MockSerialRegistry.get().attach(port)
            self._serial = None
        else:
            if serial is None:
                raise RuntimeError("pyserial nicht verfügbar und kein 'mock://'-Port angegeben.")
            self._serial = serial.Serial(port, baudrate, timeout=self._timeout)

    # --- Byte-IO ---
    def write_bytes(self, data: bytes) -> None:
        with self._lock:
            if self._mock:
                # zum Emulator senden
                _MockFirmwareMailbox.post(self._mock, data)
            else:
                self._serial.write(data)  # type: ignore

    def read_bytes(self, n: int) -> bytes:
        try:
            if self._mock:
                return self._mock.read(n, timeout=self._timeout)
            else:
                return self._serial.read(n)  # type: ignore
        except Exception:
            return b""

    # --- Line-IO ---
    def write_line(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        data = line.encode()
        with self._lock:
            if self._mock:
                _MockFirmwareMailbox.post(self._mock, data)
            else:
                self._serial.write(data)  # type: ignore

    def readline(self) -> Optional[str]:
        try:
            if self._mock:
                b = self._mock.readline(timeout=self._timeout)
                if not b:
                    return None
                return b.decode(errors="ignore").strip()
            else:
                line = self._serial.readline()  # type: ignore
                if not line:
                    return None
                return line.decode(errors="ignore").strip()
        except Exception:
            return None

    def close(self) -> None:
        try:
            if self._mock:
                # Endpoint bleibt für andere Nutzer bestehen; Emulator aber stoppen:
                # (nur wenn niemand anderes nutzt – hier einfach idempotent)
                pass
            else:
                self._serial.close()  # type: ignore
        except Exception:
            pass


# ============================================================
# Firmware-Mailbox: Host->Emu (Write) & Emu->Host (Feed)
# ============================================================

class _MockFirmwareMailbox:
    """Zwischenpuffer für vom Host gesendete Daten (Write)."""
    _lock = threading.Lock()
    _queues: Dict[int, deque] = {}

    @classmethod
    def post(cls, endpoint: _MockEndpoint, data: bytes) -> None:
        key = id(endpoint)
        with cls._lock:
            q = cls._queues.setdefault(key, deque())
            q.append(data)

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
        return bytes(out)


# ============================================================
# Emulator-Basis
# ============================================================

class _BaseEmu:
    def __init__(self, ep: _MockEndpoint, name: str):
        self.ep = ep
        self.name = name
        self._running = False
        self._t = None  # type: Optional[threading.Thread]

    def start(self) -> None:
        self._running = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def stop(self) -> None:
        self._running = False

    def _loop(self) -> None:
        raise NotImplementedError


# ============================================================
# MINI1 Emulator (ASCII Druck-Zahlenzeilen)
# ============================================================

class _Mini1Emu(_BaseEmu):
    """
    Erzeugt fortlaufend ASCII-Druckwerte (kPa) mit sanfter Dynamik.
    """
    def __init__(self, ep: _MockEndpoint):
        super().__init__(ep, "MINI1-EMU")
        self.p = 101.3  # Startdruck (kPa)
        self._t0 = time.time()
        self._target = self.p

    def _apply_host_cmds(self, data: bytes) -> None:
        # Für MINI1 wird i.d.R. nichts gesendet – ignorieren,
        # Aber falls doch: erlauben setpoint:<kpa>\n zum Testen.
        try:
            text = data.decode(errors="ignore")
            for line in text.splitlines():
                if line.startswith("setpoint:"):
                    v = float(line.split(":", 1)[1])
                    self._target = max(0.0, min(200.0, v))
        except Exception:
            pass

    def _dynamics(self, dt: float) -> None:
        # einfacher 1. Ordnung Annäherer
        tau = 2.5
        self.p += (self._target - self.p) * (1 - math.exp(-dt / tau))
        # kleine Welligkeit/Noise
        wiggle = 0.05 * math.sin(2 * math.pi * (time.time() - self._t0) / 10.0)
        noise = 0.02 * (2 * (hash((int(time.time()*10), id(self))) & 1) - 1)
        self.p = max(0.0, self.p + wiggle + noise)

    def _loop(self) -> None:
        last = time.time()
        acc = 0.0
        while self._running:
            # Host-Kommandos abholen
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                self._apply_host_cmds(data)

            now = time.time()
            dt = now - last
            last = now
            acc += dt
            self._dynamics(dt)

            # ca. 50 Zeilen/Sekunde (20 ms)
            if acc >= 0.02:
                acc = 0.0
                line = f"{self.p:.2f}\n".encode()
                self.ep.feed(line)

            time.sleep(0.005)


# ============================================================
# MINI2 Emulator (32-bit Big-Endian Words)
# ============================================================

class _Mini2Emu(_BaseEmu):
    """
    Emuliert das MINI2 Binärprotokoll.
    - Setpoint, MANUAL (PWM), Ramp
    - Liefert STATUS-Frames für BASE+2/3/4

    Protokoll (wie in deinem Treiber):
      Word = [ADDR6 | PAYLOAD26] (Big Endian ausgesendet)
    """
    def __init__(self, ep: _MockEndpoint, base_addr: int = 0x05):
        super().__init__(ep, "MINI2-EMU")
        self.base = base_addr & 0x3F
        self._rx = bytearray()
        # Zustand
        self.mode = "MANUAL"  # MANUAL | AUTOMATIC | RAMP
        self.vp1 = 0
        self.vp2 = 0
        self.setpoint_kpa = 0.0
        self.p20 = 101.3
        self.p21 = 101.2
        # Rampenparameter
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
        # ganze 4-Byte-Wörter konsumieren
        while len(self._rx) >= 4:
            w = self._be_to_u32(self._rx[:4])
            del self._rx[:4]
            self._handle_set_word(w)

    def _handle_set_word(self, w: int) -> None:
        addr = (w >> 26) & 0x3F
        payload = w & 0x03FF_FFFF

        if addr == (self.base + 6):  # COMMANDS
            if payload & 0x1:
                self._send_status_triplet()
        elif addr == (self.base + 1):  # SETPOINT
            raw = (payload >> 10) & 0xFFFF
            self.setpoint_kpa = raw / 10.0
            self.mode = "AUTOMATIC"
            self.ramp_active = False
        elif addr == (self.base + 0):  # VP1/VP2 setzen -> MANUAL
            self.vp1 = (payload >> 16) & 0xFF
            self.vp2 = (payload >> 8) & 0xFF
            self.mode = "MANUAL"
            self.ramp_active = False
        elif addr == (self.base + 5):  # RAMP
            raw_speed = (payload >> 10) & 0xFFFF
            raw_end = payload & 0x3FF
            self.ramp_speed = raw_speed / 100.0
            self.ramp_end = float(raw_end)
            self.mode = "RAMP"
            self.ramp_active = True

    def _pressure_dynamics(self, dt: float) -> None:
        # Modell:
        # - MANUAL: Druck nähert sich abhängig von VP1/VP2 einem "manuellen" Target
        # - AUTOMATIC: exponentielle Annäherung an setpoint_kpa
        # - RAMP: linear mit ramp_speed bis ramp_end, danach AUTOMATIC auf Enddruck

        if self.mode == "MANUAL":
            # VP1 (In), VP2 (Out): grobe Heuristik für Richtung
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
                else:
                    self.p21 += step
            else:
                tau = 2.0
                self.p21 += (self.ramp_end - self.p21) * (1 - math.exp(-dt/tau))

        # p20 folgt p21 mit kleiner Differenz/Noise
        self.p20 += (self.p21 - self.p20) * 0.3
        self.p20 += 0.02 * math.sin(time.time())

    def _send_status_triplet(self) -> None:
        # BASE+2: P20, BASE+3: P21, BASE+4: MODE/PWM
        mode2 = {"MANUAL":0, "RAMP":1, "AUTOMATIC":2}.get(self.mode, 3)
        p20_raw = int(round(max(0.0, min(0xFFFF/10.0, self.p20)) * 10))
        # p21_raw = int(round(max(0.0, min(0.0xFFFF/10.0, self.p21)) * 10))
        # Achtung Tippfehler: 0.0xFFFF -> 0xFFFF (fix)
        p21_raw = int(round(max(0.0, min(0xFFFF/10.0, self.p21)) * 10))

        w2 = self._mk_word(self.base + 2, (p20_raw & 0xFFFF) << 10)
        w3 = self._mk_word(self.base + 3, (p21_raw & 0xFFFF) << 10)
        w4 = self._mk_word(self.base + 4, ((mode2 & 0x3) << 24) | ((self.vp1 & 0xFF) << 16) | ((self.vp2 & 0xFF) << 8))

        for w in (w2, w3, w4):
            self.ep.feed(self._u32_to_be(w))

    def _loop(self) -> None:
        tick = 0.0
        while self._running:
            # Host Writes
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                self._host_write_bytes(data)

            # Dynamik
            now = time.time()
            dt = now - self._last
            self._last = now
            self._pressure_dynamics(dt)

            # regelmäßige Statusframes ~25 Hz
            tick += dt
            if tick >= 0.04:
                tick = 0.0
                self._send_status_triplet()
            time.sleep(0.005)


# ============================================================
# MAXI Emulator (neues Protokoll)
# ============================================================

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
        # Zustand
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
        if addr6 == self.A_RequestStatus:
            # sofort Status frame schicken
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
        # nichts zurücksenden – Status kommt zyklisch

    def _host_write_bytes(self, data: bytes) -> None:
        self._rx.extend(data)
        # minimale Parser-State-Maschine
        while True:
            if len(self._rx) < 6:
                return
            # Header suchen
            h0, h1, cmd, ln = self._rx[0], self._rx[1], self._rx[2], self._rx[3]
            if h0 != self.H0 or h1 != self.H1 or cmd != self.CMD_SET or ln != self.SET_LEN:
                # Byte verwerfen um zu resyncen
                self._rx.pop(0)
                continue
            total = 4 + ln + 1
            if len(self._rx) < total:
                return
            payload = bytes(self._rx[4:4+ln])
            cs = self._rx[4+ln]
            del self._rx[:total]
            if self._sumcs(payload) != cs:
                continue
            word, = struct.unpack("<I", payload)
            addr6 = word & 0x3F
            data26 = (word >> 6) & 0x03FF_FFFF
            self._apply_set(addr6, data26)

    def _dynamics(self, dt: float) -> None:
        # einfache Massenbilanz für Totalisatoren
        self.total["co2"] += self.flow["co2"] * dt / 60.0  # [unit: e.g. slpm * min]
        self.total["n2"]  += self.flow["n2"]  * dt / 60.0
        self.total["but"] += self.flow["but"] * dt / 60.0
        # ADC leicht variieren
        self.adc_P11 = int(1500 + 300*math.sin(time.time()/7))
        self.adc_P31 = int(2200 + 400*math.sin(time.time()/9 + 1.0))
        # GasLeak zufällig/selten
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
        # Padding
        while len(p) < self.STATUS_LEN:
            p.append(0)
        return bytes(p[:self.STATUS_LEN])

    def _send_status(self) -> None:
        payload = self._pack_status_payload()
        frame = bytes([self.SYNC0, self.SYNC1, len(payload)]) + payload + bytes([self._sumcs(payload)])
        self.ep.feed(frame)

    def _loop(self) -> None:
        acc = 0.0
        while self._running:
            # Host-Kommandos abholen
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                self._host_write_bytes(data)
            # Dynamik
            now = time.time()
            if not hasattr(self, "_t_prev"):
                self._t_prev = now
            dt = now - self._t_prev
            self._t_prev = now
            self._dynamics(dt)

            acc += dt
            # ~25 Hz Status
            if acc >= 0.04:
                acc = 0.0
                self._send_status()

            time.sleep(0.005)


# Fallback: echo
class _EchoEmu(_BaseEmu):
    def _loop(self) -> None:
        while self._running:
            data = _MockFirmwareMailbox.get(self.ep)
            if data:
                # Spiegeln
                self.ep.feed(data)
            time.sleep(0.01)


# ============================================================
# Deine Treiber-Klassen – UNVERÄNDERT bis auf SerialDevice (oben)
# ============================================================

# ====================== MINI1 ======================
class MINI1:
    """
    MINI1 liefert weiterhin ASCII-Zahlenzeilen (Druck). Wir puffern eine Zeitreihe.
    """
    def __init__(self, port: str, max_samples: int = 300_000):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.dev = SerialDevice(port, 115200, name="MINI1")
        self._series: deque[Tuple[datetime, float]] = deque(maxlen=max_samples)
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

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
                continue
            with self._lock:
                self._series.append((datetime.now(timezone.utc), pressure))

    def get_last_n(self, n: Optional[int] = None) -> List[Tuple[datetime, float]]:
        with self._lock:
            data = list(self._series)
        return data if n is None or n >= len(data) else data[-n:]

    @property
    def pressure(self) -> Optional[float]:
        with self._lock:
            return self._series[-1][1] if self._series else None

    def set_max_samples(self, max_samples: int) -> None:
        with self._lock:
            self._series = deque(self._series, maxlen=max_samples)

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()

    def __enter__(self): return self
    def __exit__(self, *exc): self.stop()


# ====================== MINI2 ======================
class MINI2:
    """
    MINI2 – Binärprotokoll (32-Bit Words, Big-Endian) – unverändert.
    """
    def __init__(self, port: str, base_addr: int = 0x05):
        self.dev = SerialDevice(port, 115200, name="MINI2")
        self.base = int(base_addr) & 0x3F

        self._lock = threading.Lock()
        self._last_status_time = 0.0
        self.status: Dict[str, Any] = {}

        self._rx_buf = bytearray()
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

        self.request_status()
        self._wait_for_recent_status(0.5)

    @staticmethod
    def _make_word(addr6: int, payload26: int) -> int:
        return ((addr6 & 0x3F) << 26) | (payload26 & 0x03FF_FFFF)

    @staticmethod
    def _u32_to_be_bytes(word: int) -> bytes:
        return bytes([(word >> 24) & 0xFF, (word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF])

    @staticmethod
    def _be_bytes_to_u32(b: bytes) -> int:
        return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]

    def _send_word(self, addr: int, payload: int) -> None:
        word = self._make_word(addr, payload)
        self.dev.write_bytes(self._u32_to_be_bytes(word))

    def request_status(self) -> None:
        self._send_word(self.base + 6, 0x1)

    def set_target_pressure(self, kpa: Union[int, float]) -> None:
        raw = max(0, min(0xFFFF, int(round(float(kpa) * 10.0))))
        payload = (raw & 0xFFFF) << 10
        self._send_word(self.base + 1, payload)

    def set_manual_vp1(self, value: int) -> None:
        v1 = max(0, min(255, int(value)))
        v2 = int(self.status.get("pwm2", 0)) & 0xFF
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_manual_vp2(self, value: int) -> None:
        v2 = max(0, min(255, int(value)))
        v1 = int(self.status.get("pwm1", 0)) & 0xFF
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_state_manual(self) -> None:
        v1 = int(self.status.get("pwm1", 0)) & 0xFF
        v2 = int(self.status.get("pwm2", 0)) & 0xFF
        payload = (0 << 24) | (v1 << 16) | (v2 << 8)
        self._send_word(self.base + 0, payload)

    def set_ramp(self, speed_kpa_s: float, end_kpa: float) -> None:
        raw_speed = max(0, min(0xFFFF, int(round(speed_kpa_s * 100.0))))
        raw_end   = max(0, min(0x03FF,  int(round(end_kpa))))
        payload   = (raw_speed << 10) | raw_end
        self._send_word(self.base + 5, payload)

    def _reader_loop(self) -> None:
        while self._running:
            chunk = self.dev.read_bytes(64)
            if chunk:
                self._rx_buf.extend(chunk)
                self._drain_words()
            else:
                time.sleep(0.002)

    def _drain_words(self) -> None:
        while len(self._rx_buf) >= 4:
            word_bytes = bytes(self._rx_buf[:4])
            del self._rx_buf[:4]
            try:
                word = self._be_bytes_to_u32(word_bytes)
                self._handle_status_word(word)
            except Exception:
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

    def is_recent(self, max_age_s: float = 0.5) -> bool:
        with self._lock:
            return (time.time() - self._last_status_time) <= max_age_s

    def _wait_for_recent_status(self, timeout: float = 0.5) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.is_recent(0.3):
                return True
            time.sleep(0.01)
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
            return out

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()


# ====================== MAXI ======================
class MAXI:
    """
    MAXI-Treiber (unverändert), spricht mit MAXI-Emu oben.
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

        threading.Thread(target=self._reader_loop, daemon=True).start()
        self.request_status()

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

    def _send_set(self, addr6: int, data26: int) -> None:
        word = ((data26 & 0x03FF_FFFF) << 6) | (addr6 & 0x3F)
        payload = self._le32(word)
        frame = bytes([self.H0, self.H1, self.CMD_SET, self.SET_LEN]) + payload + bytes([self._checksum_sum(payload)])
        self.dev.write_bytes(frame)

    def request_status(self) -> None:
        self._send_set(self.A_RequestStatus, 1)

    def set_v1(self, open_: bool) -> None:    self._send_set(self.A_V1,    1 if open_ else 0)
    def set_v2(self, open_: bool) -> None:    self._send_set(self.A_V2,    1 if open_ else 0)
    def set_v3(self, open_: bool) -> None:    self._send_set(self.A_V3,    1 if open_ else 0)
    def set_fan_in(self, on: bool) -> None:   self._send_set(self.A_FanIn, 1 if on else 0)
    def set_fan_out(self, on: bool) -> None:  self._send_set(self.A_FanOut,1 if on else 0)

    def set_flow_co2(self, flow: float) -> None:   self._send_set(self.A_FlowCO2, self._u26_from_float100(flow))
    def set_flow_n2(self, flow: float) -> None:    self._send_set(self.A_FlowN2,  self._u26_from_float100(flow))
    def set_flow_butan(self, flow: float) -> None: self._send_set(self.A_FlowBut, self._u26_from_float100(flow))

    def set_total_co2(self, value: float = 0.0) -> None:   self._send_set(self.A_TotCO2, self._u26_from_float100(value))
    def set_total_n2(self, value: float = 0.0) -> None:    self._send_set(self.A_TotN2,  self._u26_from_float100(value))
    def set_total_butan(self, value: float = 0.0) -> None: self._send_set(self.A_TotBut, self._u26_from_float100(value))

    def _reader_loop(self) -> None:
        while self._running:
            chunk = self.dev.read_bytes(64)
            if chunk:
                self._rx.extend(chunk)
                self._drain_status_frames()
            else:
                time.sleep(0.002)

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
                continue
            if (self._checksum_sum(payload) & 0xFF) != cs:
                continue
            try:
                self._apply_status_payload(payload)
            except Exception:
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

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "ts": self._last_status_ts,
                **self.status,
            }

    def stop(self) -> None:
        self._running = False
        time.sleep(0.05)
        self.dev.close()


# ======================= VaderDeviceDriver =====================
class VaderDeviceDriver:
    """
    High-Level-Kapselung für MINI1, MINI2 und MAXI (funktioniert mit mock:// Ports)
    """
    def __init__(self, mini1_port: str, mini2_port: str, maxi_port: str, mini1_max_samples: int = 300_000):
        self.mini1 = MINI1(mini1_port, max_samples=mini1_max_samples)
        self.mini2 = MINI2(mini2_port)
        self.maxi  = MAXI(maxi_port)

    # MINI2
    def set_manual_PWM_in(self, value: int) -> None:  self.mini2.set_manual_vp1(value)
    def set_manual_PWM_out(self, value: int) -> None: self.mini2.set_manual_vp2(value)
    def set_state_manual(self) -> None:               self.mini2.set_state_manual()
    def set_ramp(self, speed_kpa_s: float, end_kpa: float) -> None: self.mini2.set_ramp(speed_kpa_s, end_kpa)
    def mini2_request_status(self) -> None:           self.mini2.request_status()
    def mini2_get_status(self) -> Dict[str, Any]:     return self.mini2.get_status()

    # MAXI
    def set_v1(self, open_: bool) -> None: self.maxi.set_v1(open_)
    def set_v2(self, open_: bool) -> None: self.maxi.set_v2(open_)
    def set_v3(self, open_: bool) -> None: self.maxi.set_v3(open_)
    def set_fans(self, on: bool) -> None:
        self.maxi.set_fan_in(on); self.maxi.set_fan_out(on)
    def set_flow_co2(self, flow: float) -> None: self.maxi.set_flow_co2(flow)
    def set_flow_n2(self, flow: float) -> None:  self.maxi.set_flow_n2(flow)
    def set_flow_butan(self, flow: float) -> None: self.maxi.set_flow_butan(flow)
    def set_total_co2(self, value: float = 0.0) -> None: self.maxi.set_total_co2(value)
    def set_total_n2(self, value: float = 0.0) -> None:  self.maxi.set_total_n2(value)
    def set_total_butan(self, value: float = 0.0) -> None: self.maxi.set_total_butan(value)
    def reset_totalisator_co2(self) -> None: self.set_total_co2(0.0)
    def reset_totalisator_n2(self) -> None:  self.set_total_n2(0.0)
    def reset_totalisator_butan(self) -> None: self.set_total_butan(0.0)
    def maxi_request_status(self) -> None: self.maxi.request_status()
    def maxi_get_status(self) -> Dict[str, Any]: return self.maxi.get_status()

    # MINI1
    def mini1_get_last_n(self, n: int) -> List[Tuple[datetime, float]]:
        return self.mini1.get_last_n(n)

    # bestehende Methoden
    def storage_volume(self, open_: bool) -> None:
        self.maxi.set_v1(open_); self.maxi.set_v2(open_)

    def vac_all(self, vacuum_pressure_kpa: float, timeout_s: int = 120) -> None:
        self.maxi.set_flow_butan(0); self.maxi.set_flow_co2(0); self.maxi.set_flow_n2(0)
        self.maxi.set_v1(True); self.maxi.set_v2(True); self.maxi.set_v3(True); self.set_fans(True)

        start = time.time()
        while True:
            p = self.mini1.pressure
            if p is not None and p <= vacuum_pressure_kpa:
                break
            if time.time() - start > timeout_s:
                break
            time.sleep(0.5)

        self.maxi.set_v1(False); self.maxi.set_v2(False); self.maxi.set_v3(False); self.set_fans(False)
        self.mini2.set_target_pressure(0); self.mini2.set_manual_vp1(0); self.mini2.set_manual_vp2(0)

    def use_gas(self, n2: float, co2: float, butan: float) -> None:
        self.maxi.set_flow_n2(n2); self.maxi.set_flow_co2(co2); self.maxi.set_flow_butan(butan)

    def setpoint_pressure(self, kpa: Union[int, float]) -> None:
        self.mini2.set_target_pressure(kpa)

    def get_all_status(self, mini1_last_n: Optional[int] = 100) -> Dict[str, Any]:
        self.mini2.request_status()
        self.maxi.request_status()
        self.mini2._wait_for_recent_status(0.5)
        return {
            "mini1": {"latest_pressure": self.mini1.pressure, "recent_series": self.mini1.get_last_n(mini1_last_n)},
            "mini2": self.mini2.get_status(),
            "maxi":  self.maxi.get_status(),
        }

    def reset_all_totalisators(self) -> None:
        self.reset_totalisator_butan()
        self.reset_totalisator_co2()
        self.reset_totalisator_n2()

    def close(self) -> None:
        self.mini1.stop(); self.mini2.stop(); self.maxi.stop()

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()


# ============================================================
# DEMO
# ============================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    with VaderDeviceDriver("mock://mini1", "mock://mini2", "mock://maxi") as drv:
        print("Starte Mock…")
        time.sleep(0.3)
        print("MINI2 Status:", drv.mini2_get_status())
        drv.setpoint_pressure(50.0)
        time.sleep(1.0)
        print("MINI2 nach Setpoint:", drv.mini2_get_status())
        drv.set_flow_co2(2.5)
        drv.set_flow_n2(1.0)
        drv.set_v1(True)
        time.sleep(0.2)
        print("MAXI Status:", drv.maxi_get_status())
        print("MINI1 letzter Druck:", drv.mini1.pressure)
        drv.set_ramp(5.0, 80.0)  # 5 kPa/s bis 80 kPa
        time.sleep(1.5)
        print("MINI2 Ramp Status:", drv.mini2_get_status())
        print("ALL:", list(drv.get_all_status(mini1_last_n=5).keys()))