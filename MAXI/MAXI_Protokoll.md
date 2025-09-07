
## get_Status: Python
from typing import Dict, Any

SYNC0, SYNC1 = 0xAA, 0x55
PAYLOAD_LEN = 25

def checksum(payload: bytes) -> int:
    return sum(payload) & 0xFF

def parse_status_frame(frame: bytes) -> Dict[str, Any]:
    if len(frame) < 2 + 1 + PAYLOAD_LEN + 1:
        raise ValueError("Frame zu kurz")

    if frame[0] != SYNC0 or frame[1] != SYNC1:
        raise ValueError("Ungültiger Header")

    if frame[2] != PAYLOAD_LEN:
        raise ValueError(f"Falsche Länge: {frame[2]}")

    payload = frame[3:3+PAYLOAD_LEN]
    cs = frame[3+PAYLOAD_LEN]
    if checksum(payload) != cs:
        raise ValueError("Checksumme falsch")

    i = 0
    flags = payload[i]; i += 1

    def get16():
        nonlocal i
        v = payload[i] | (payload[i+1] << 8)
        i += 2
        return v

    def get32():
        nonlocal i
        v = payload[i] | (payload[i+1] << 8) | (payload[i+2] << 16) | (payload[i+3] << 24)
        i += 4
        return v

    out = {}
    out["GasLeak"] = bool(flags & 0x01)
    out["V1"]      = bool(flags & 0x02)
    out["V2"]      = bool(flags & 0x04)
    out["V3"]      = bool(flags & 0x08)
    out["FanIn"]   = bool(flags & 0x10)
    out["FanOut"]  = bool(flags & 0x20)

    out["P11"] = get16()
    out["P31"] = get16()
    out["Flow_CO2"]        = get16() / 100.0
    out["Totalisator_CO2"] = get32() / 100.0
    out["Flow_N2"]         = get16() / 100.0
    out["Totalisator_N2"]  = get32() / 100.0
    out["Flow_Butan"]      = get16() / 100.0
    out["Totalisator_Butan"] = get32() / 100.0

    return out

## SET_params Python

# Python – SET-Protokoll mit zusätzlicher Adresse "RequestStatus"
# Baut ein 4-Byte-Payload-Word: WORD = (DATA26 << 6) | ADDR6
# Frame: [0xA5, 0x5A, 0x01, 0x04, payload(4 LE), checksum]

HEADER = bytes([0xA5, 0x5A])
CMD_SET = 0x01
LEN = 0x04

ADDR = {
    "RequestStatus": 0x00,  # NEU: Status anfordern (DATA ignoriert)
    "V1": 0x01, "V2": 0x02, "V3": 0x03, "FanIn": 0x04, "FanOut": 0x05,
    "Flow_CO2": 0x10, "Totalisator_CO2": 0x11,
    "Flow_N2": 0x12, "Totalisator_N2": 0x13,
    "Flow_Butan": 0x14, "Totalisator_Butan": 0x15,
}

def _u26(x: int) -> int:
    if x < 0: x = 0
    if x > 0x3FFFFFF: x = 0x3FFFFFF
    return x

def _scale100(value: float) -> int:
    return _u26(int(round(value * 100.0)))

def build_set_frame(name: str, value: float | int = 0) -> bytes:
    if name not in ADDR:
        raise ValueError(f"Unbekannte Variable: {name}")
    a = ADDR[name]

    if name == "RequestStatus":
        data26 = 0  # DATA wird ignoriert
    elif name in ("V1","V2","V3","FanIn","FanOut"):
        data26 = _u26(1 if int(value) != 0 else 0)
    else:
        data26 = _scale100(float(value))

    word = (data26 << 6) | (a & 0x3F)
    payload = word.to_bytes(4, "little")
    cs = bytes([sum(payload) & 0xFF])
    return HEADER + bytes([CMD_SET, LEN]) + payload + cs



## send_status: Arduino
#include <Arduino.h>

static const uint8_t SYNC0 = 0xAA;
static const uint8_t SYNC1 = 0x55;
static const uint8_t PAYLOAD_LEN = 25;

static inline uint8_t checksum(const uint8_t *buf, uint8_t len) {
uint16_t s = 0;
for (uint8_t i = 0; i < len; i++) s += buf[i];
return (uint8_t)(s & 0xFF);
}

static inline uint16_t floatToU16(double v) {
if (v < 0) v = 0;
uint32_t raw = (uint32_t) llround(v * 100.0); // zwei Nachkommastellen
if (raw > 0xFFFFu) raw = 0xFFFFu;
return (uint16_t) raw;
}

static inline uint32_t floatToU32(double v) {
if (v < 0) v = 0;
uint64_t raw = (uint64_t) llround(v * 100.0);
if (raw > 0xFFFFFFFFull) raw = 0xFFFFFFFFull;
return (uint32_t) raw;
}

void sendStatusFrame(
bool GasLeak, bool V1, bool V2, bool V3, bool FanIn, bool FanOut,
int16_t P11, int16_t P31,
double FlowCO2, double TotCO2,
double FlowN2,  double TotN2,
double FlowBut, double TotBut
) {
uint8_t payload[PAYLOAD_LEN];
uint8_t i = 0;

// Flags in 1 Byte
uint8_t flags = 0;
flags |= (GasLeak ? 1 : 0) << 0;
flags |= (V1      ? 1 : 0) << 1;
flags |= (V2      ? 1 : 0) << 2;
flags |= (V3      ? 1 : 0) << 3;
flags |= (FanIn   ? 1 : 0) << 4;
flags |= (FanOut  ? 1 : 0) << 5;
payload[i++] = flags;

// Helfer zum Schreiben (Little Endian)
auto put16 = [&](uint16_t v) {
    payload[i++] = v & 0xFF;
    payload[i++] = (v >> 8) & 0xFF;
};
auto put32 = [&](uint32_t v) {
    payload[i++] = v & 0xFF;
    payload[i++] = (v >> 8) & 0xFF;
    payload[i++] = (v >> 16) & 0xFF;
    payload[i++] = (v >> 24) & 0xFF;
};

put16(P11);
put16(P31);
put16(floatToU16(FlowCO2));
put32(floatToU32(TotCO2));
put16(floatToU16(FlowN2));
put32(floatToU32(TotN2));
put16(floatToU16(FlowBut));
put32(floatToU32(TotBut));

// Frame senden
Serial.write(SYNC0);
Serial.write(SYNC1);
Serial.write(PAYLOAD_LEN);
Serial.write(payload, PAYLOAD_LEN);
uint8_t cs = checksum(payload, PAYLOAD_LEN);
Serial.write(cs);
}

## Arduino SET_PARAMS

// Arduino – SET-Protokoll mit zusätzlicher Adresse "RequestStatus"
// Erwartet Frame: [0xA5, 0x5A, 0x01, 0x04, payload(4 LE), checksum]
// payload => WORD = (DATA26 << 6) | ADDR6

#include <Arduino.h>

static const uint8_t H0 = 0xA5;
static const uint8_t H1 = 0x5A;
static const uint8_t CMD_SET = 0x01;
static const uint8_t LEN = 0x04;

enum Addr : uint8_t {
  A_RequestStatus = 0x00, // NEU
  A_V1=0x01, A_V2=0x02, A_V3=0x03, A_FanIn=0x04, A_FanOut=0x05,
  A_FlowCO2=0x10, A_TotCO2=0x11, A_FlowN2=0x12, A_TotN2=0x13,
  A_FlowBut=0x14, A_TotBut=0x15
};

static inline bool readFrame(Stream& s, uint8_t& cmd, uint8_t& len, uint8_t payload[4], uint8_t& cs) {
  while (s.available()) {
    if (s.peek() == H0) {
      s.read();
      if (s.read() == H1) {
        cmd = s.read();
        len = s.read();
        if (len != LEN) return false;
        for (uint8_t i=0;i<LEN;i++) payload[i] = s.read();
        cs = s.read();
        uint16_t sum = payload[0]+payload[1]+payload[2]+payload[3];
        return ((sum & 0xFF) == cs);
      }
    } else {
      s.read();
    }
  }
  return false;
}

static inline uint32_t le32(const uint8_t p[4]) {
  return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}

static inline float from_u26_to_float100(uint32_t u26) {
  if (u26 > 0x3FFFFFFu) u26 = 0x3FFFFFFu;
  return (float)u26 / 100.0f;
}

// Externe Funktion: sendStatusFrame(...); // bereits implementiert

// Beispielhafte Zielvariablen:
bool V1=false, V2=false, V3=false, FanIn=false, FanOut=false;
float Flow_CO2=0, Tot_CO2=0, Flow_N2=0, Tot_N2=0, Flow_Butan=0, Tot_Butan=0;

void applySet(uint8_t addr6, uint32_t data26) {
  switch (addr6) {
    case A_RequestStatus:
      sendStatusFrame(
        /*GasLeak*/false, V1, V2, V3, FanIn, FanOut,
        /*P11*/0, /*P31*/0,
        Flow_CO2, Tot_CO2, Flow_N2, Tot_N2, Flow_Butan, Tot_Butan
      );
      break;

    case A_V1: V1 = (data26 & 1u); break;
    case A_V2: V2 = (data26 & 1u); break;
    case A_V3: V3 = (data26 & 1u); break;
    case A_FanIn:  FanIn  = (data26 & 1u); break;
    case A_FanOut: FanOut = (data26 & 1u); break;

    case A_FlowCO2: Flow_CO2 = from_u26_to_float100(data26); break;
    case A_TotCO2:  Tot_CO2  = from_u26_to_float100(data26); break;
    case A_FlowN2:  Flow_N2  = from_u26_to_float100(data26); break;
    case A_TotN2:   Tot_N2   = from_u26_to_float100(data26); break;
    case A_FlowBut: Flow_Butan = from_u26_to_float100(data26); break;
    case A_TotBut:  Tot_Butan  = from_u26_to_float100(data26); break;
    default: break;
  }
}

void pollSetCommands() {
  uint8_t cmd,len,payload[4],cs;
  while (readFrame(Serial, cmd, len, payload, cs)) {
    if (cmd != CMD_SET) continue;
    uint32_t word = le32(payload);
    uint8_t addr6 = word & 0x3F;
    uint32_t data26 = word >> 6;
    applySet(addr6, data26);
  }
}

void setup() { Serial.begin(115200); }
void loop()  { pollSetCommands(); }