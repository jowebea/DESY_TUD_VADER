#include <Controllino.h>
#include "ModbusRtu.h"
#include <RunningMedian.h>
#include <Arduino.h>

// =======================================================
//  Neue serielle Protokolle (siehe Vorgabe)
// =======================================================
// ---- send_status (Binary Frame) ----
static const uint8_t SYNC0 = 0xAA;
static const uint8_t SYNC1 = 0x55;
static const uint8_t STATUS_PAYLOAD_LEN = 25;  // Vorgabe

static inline uint8_t sum_checksum(const uint8_t *buf, uint8_t len) {
  uint16_t s = 0; for (uint8_t i = 0; i < len; i++) s += buf[i];
  return (uint8_t)(s & 0xFF);
}
static inline uint16_t floatToU16_100(double v) {
  if (v < 0) v = 0; uint32_t raw = (uint32_t) llround(v * 100.0);
  if (raw > 0xFFFFu) raw = 0xFFFFu; return (uint16_t) raw;
}
static inline uint32_t floatToU32_100(double v) {
  if (v < 0) v = 0; uint64_t raw = (uint64_t) llround(v * 100.0);
  if (raw > 0xFFFFFFFFull) raw = 0xFFFFFFFFull; return (uint32_t) raw;
}

// ---- SET (framed) ----
static const uint8_t H0 = 0xA5;
static const uint8_t H1 = 0x5A;
static const uint8_t CMD_SET = 0x01;
static const uint8_t SET_LEN = 0x04; // 4-Byte-Payload

enum Addr : uint8_t {
  A_RequestStatus = 0x00,
  A_V1=0x01, A_V2=0x02, A_V3=0x03, A_FanIn=0x04, A_FanOut=0x05,
  A_FlowCO2=0x10, A_TotCO2=0x11, A_FlowN2=0x12, A_TotN2=0x13,
  A_FlowBut=0x14, A_TotBut=0x15
};

static inline bool readSetFrame(Stream& s, uint8_t& cmd, uint8_t& len, uint8_t payload[4], uint8_t& cs) {
  while (s.available()) {
    if (s.peek() == H0) {
      s.read();
      if (s.read() != H1) continue;
      if (!s.available()) return false;
      cmd = s.read();
      if (!s.available()) return false;
      len = s.read();
      if (len != SET_LEN) return false;
      for (uint8_t i=0;i<SET_LEN;i++) {
        while(!s.available());
        payload[i] = s.read();
      }
      while(!s.available());
      cs = s.read();
      uint16_t sum = payload[0]+payload[1]+payload[2]+payload[3];
      return ((sum & 0xFF) == cs);
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
  if (u26 > 0x3FFFFFFu) u26 = 0x3FFFFFFu; // 26 Bit Max
  return (float)u26 / 100.0f;
}

// =======================================================
//  Modbus / IO bestehend (leicht bereinigt)
// =======================================================
#define MasterModbusAdd  0
#define RS485Serial      3
Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 0);

#define MFC_BUTAN  1
#define MFC_CO2    2
#define MFC_N2     3

#define MODBUS_TIMEOUT 500

// Analoge Eingänge
#define LEAK_SENSOR   CONTROLLINO_A3
#define PT100_STORAGE CONTROLLINO_A7
#define PT100_VACUUM  CONTROLLINO_A6
#define PT100_HOUSING CONTROLLINO_A8
#define P11           CONTROLLINO_A9
#define P30           CONTROLLINO_A5
#define P31           CONTROLLINO_A4

// Digitale Ausgänge
#define FAN_IN   CONTROLLINO_RELAY_00
bool fan_in_power;
#define FAN_OUT  CONTROLLINO_RELAY_01
bool fan_out_power;
#define HEATER   CONTROLLINO_D5
int  heater_pwm;

#define V1 CONTROLLINO_RELAY_05
bool v1_power;
#define V2 CONTROLLINO_RELAY_04
bool v2_power;
#define V3 CONTROLLINO_RELAY_03
bool v3_power;

#define VS3 CONTROLLINO_RELAY_07
bool vs3_power;

#define CLOSE 0
#define OPEN  1

RunningMedian _p11(5), _p31(5);

int accurateADC_P11(){ for(byte i=0;i<5;i++) _p11.add(analogRead(P11)); return _p11.getMedian(); }
int accurateADC_P31(){ for(byte i=0;i<5;i++) _p31.add(analogRead(P31)); return _p31.getMedian(); }

// ---------------- Modbus Helpers ----------------
float readFloatFromHoldingReg(uint8_t slaveId, uint16_t startReg){
  uint16_t readBuf[2] = {0, 0};
  modbus_t request; request.u8id=slaveId; request.u8fct=3; request.u16RegAdd=startReg; request.u16CoilsNo=2; request.au16reg=readBuf;
  ControllinoModbusMaster.query(request);
  unsigned long t0 = millis();
  while (ControllinoModbusMaster.getState() != COM_IDLE) {
    ControllinoModbusMaster.poll();
    if (millis() - t0 > MODBUS_TIMEOUT) { Serial.println("Modbus read timeout!"); return NAN; }
  }
  union { float f; uint16_t w[2]; } converter;
  converter.w[1] = readBuf[0]; converter.w[0] = readBuf[1];
  return converter.f;
}

bool writeFloatToHoldingReg(uint8_t slaveId, uint16_t startReg, float value){
  union { float f; uint16_t w[2]; } converter; converter.f = value;
  uint16_t writeBuf[2]; writeBuf[1] = converter.w[0]; writeBuf[0] = converter.w[1];
  modbus_t request; request.u8id=slaveId; request.u8fct=16; request.u16RegAdd=startReg; request.u16CoilsNo=2; request.au16reg=writeBuf;
  ControllinoModbusMaster.query(request);
  unsigned long t0 = millis();
  while (ControllinoModbusMaster.getState() != COM_IDLE) {
    ControllinoModbusMaster.poll();
    if (millis() - t0 > MODBUS_TIMEOUT) { Serial.println("Modbus write timeout!"); return false; }
  }
  return true;
}

float read_flow(uint8_t MFC_addr)            { return readFloatFromHoldingReg(MFC_addr, 0x0000); }
float read_gastemperature(uint8_t MFC_addr)  { return readFloatFromHoldingReg(MFC_addr, 0x0002); }
float read_totalisator(uint8_t MFC_addr)     { return readFloatFromHoldingReg(MFC_addr, 0x0004); }
float read_flow_setpoint(uint8_t MFC_addr)   { return readFloatFromHoldingReg(MFC_addr, 0x0006); }

void reset_totalisator(uint8_t MFC_addr){ if (!writeFloatToHoldingReg(MFC_addr, 0x0004, 0.0f)) Serial.println("Fehler Reset Totalisator!"); }
void set_flow(uint8_t MFC_addr, float flow){ if (!writeFloatToHoldingReg(MFC_addr, 0x0006, flow)) Serial.println("Fehler Sollwert!"); }

// ---------------- Sensor Mapping ----------------
inline float mapPT100(int adc) { return (adc) *0.44-50; }   // °C (wie bisher)
inline float mapPx1(int adc)   { return (adc) / 0.130172; } // kPa (wie bisher)

// ---------------- Schutzfunktionen ----------------
void protect_sensors(){
  int p31 = analogRead(P31);
  if (p31 > 0){ digitalWrite(VS3, CLOSE); vs3_power = CLOSE; }
  if (p31 < 0){ digitalWrite(VS3, OPEN);  vs3_power = OPEN;  }
}

bool no_gas_leak(){ int s = analogRead(LEAK_SENSOR); if (s < 900) return true; Serial.print("Warnung: Gasleck-Sensorwert: "); Serial.println(s); return false; }

// =======================================================
//  NEU: Binäres Status-Frame senden (gemäß Vorgabe)
// =======================================================
void sendStatusFrame(bool GasLeakFlag,
                     bool V1b, bool V2b, bool V3b, bool FanInb, bool FanOutb,
                     int16_t P11_adc, int16_t P31_adc,
                     double FlowCO2, double TotCO2,
                     double FlowN2,  double TotN2,
                     double FlowBut, double TotBut)
{
  uint8_t payload[STATUS_PAYLOAD_LEN];
  uint8_t i = 0;

  uint8_t flags = 0;
  flags |= (GasLeakFlag ? 1 : 0) << 0;
  flags |= (V1b        ? 1 : 0) << 1;
  flags |= (V2b        ? 1 : 0) << 2;
  flags |= (V3b        ? 1 : 0) << 3;
  flags |= (FanInb     ? 1 : 0) << 4;
  flags |= (FanOutb    ? 1 : 0) << 5;
  payload[i++] = flags;

  auto put16 = [&](uint16_t v){ payload[i++] = v & 0xFF; payload[i++] = (v >> 8) & 0xFF; };
  auto put32 = [&](uint32_t v){ payload[i++] = v & 0xFF; payload[i++] = (v >> 8) & 0xFF; payload[i++] = (v >> 16) & 0xFF; payload[i++] = (v >> 24) & 0xFF; };

  put16((uint16_t)P11_adc);
  put16((uint16_t)P31_adc);
  put16(floatToU16_100(FlowCO2));  put32(floatToU32_100(TotCO2));
  put16(floatToU16_100(FlowN2));   put32(floatToU32_100(TotN2));
  put16(floatToU16_100(FlowBut));  put32(floatToU32_100(TotBut));

  // Padding auf 25 Bytes, falls die obigen Felder nur 23 Bytes belegen
  while (i < STATUS_PAYLOAD_LEN) payload[i++] = 0x00;

  Serial.write(SYNC0);
  Serial.write(SYNC1);
  Serial.write(STATUS_PAYLOAD_LEN);
  Serial.write(payload, STATUS_PAYLOAD_LEN);
  uint8_t cs = sum_checksum(payload, STATUS_PAYLOAD_LEN);
  Serial.write(cs);
}

// =======================================================
//  NEU: SET-Befehle verarbeiten (framed, DATA26|ADDR6)
// =======================================================
void applySet(uint8_t addr6, uint32_t data26){
  switch (addr6) {
    case A_RequestStatus: {
      // Live-Werte erfassen
      const bool gasLeak = !no_gas_leak();
      const int16_t p11 = accurateADC_P11();
      const int16_t p31 = accurateADC_P31();
      // Modbus-Werte (können einige ms benötigen)
      const double flowCO2 = read_flow(MFC_CO2);
      const double totCO2  = read_totalisator(MFC_CO2);
      const double flowN2  = read_flow(MFC_N2);
      const double totN2   = read_totalisator(MFC_N2);
      const double flowBut = read_flow(MFC_BUTAN);
      const double totBut  = read_totalisator(MFC_BUTAN);
      sendStatusFrame(gasLeak, v1_power, v2_power, v3_power, fan_in_power, fan_out_power,
                      p11, p31,
                      flowCO2, totCO2, flowN2, totN2, flowBut, totBut);
    } break;

    case A_V1:    digitalWrite(V1,    (data26 & 1u) ? OPEN : CLOSE); v1_power = (data26 & 1u) ? OPEN : CLOSE; break;
    case A_V2:    digitalWrite(V2,    (data26 & 1u) ? OPEN : CLOSE); v2_power = (data26 & 1u) ? OPEN : CLOSE; break;
    case A_V3:    digitalWrite(V3,    (data26 & 1u) ? OPEN : CLOSE); v3_power = (data26 & 1u) ? OPEN : CLOSE; break;
    case A_FanIn: digitalWrite(FAN_IN,(data26 & 1u) ? OPEN : CLOSE); fan_in_power  = (data26 & 1u) ? OPEN : CLOSE; break;
    case A_FanOut:digitalWrite(FAN_OUT,(data26 & 1u) ? OPEN : CLOSE); fan_out_power = (data26 & 1u) ? OPEN : CLOSE; break;

    case A_FlowCO2: { float v = from_u26_to_float100(data26); set_flow(MFC_CO2, v); } break;
    case A_TotCO2:  { float v = from_u26_to_float100(data26); writeFloatToHoldingReg(MFC_CO2, 0x0004, v); } break;

    case A_FlowN2:  { float v = from_u26_to_float100(data26); set_flow(MFC_N2, v); } break;
    case A_TotN2:   { float v = from_u26_to_float100(data26); writeFloatToHoldingReg(MFC_N2, 0x0004, v); } break;

    case A_FlowBut: { float v = from_u26_to_float100(data26); set_flow(MFC_BUTAN, v); } break;
    case A_TotBut:  { float v = from_u26_to_float100(data26); writeFloatToHoldingReg(MFC_BUTAN, 0x0004, v); } break;

    default: break;
  }
}

void pollSetCommands(){
  uint8_t cmd,len,payload[4],cs;
  while (readSetFrame(Serial, cmd, len, payload, cs)) {
    if (cmd != CMD_SET) continue;
    uint32_t word = le32(payload);
    uint8_t addr6 = word & 0x3F;       // untere 6 Bit
    uint32_t data26 = word >> 6;       // obere 26 Bit
    applySet(addr6, data26);
  }
}

// =======================================================
//  setup / loop
// =======================================================
void setup(){
  Serial.begin(115200);

  ControllinoModbusMaster.begin(9600);
  ControllinoModbusMaster.setTimeOut(500);

  pinMode(LEAK_SENSOR, INPUT);
  pinMode(PT100_STORAGE, INPUT); pinMode(PT100_VACUUM, INPUT); pinMode(PT100_HOUSING, INPUT);
  pinMode(P11, INPUT); pinMode(P30, INPUT); pinMode(P31, INPUT);

  pinMode(FAN_IN, OUTPUT); pinMode(FAN_OUT, OUTPUT); pinMode(HEATER, OUTPUT);
  pinMode(V1, OUTPUT); pinMode(V2, OUTPUT); pinMode(V3, OUTPUT); pinMode(VS3, OUTPUT);

  // Anfangszustände
  digitalWrite(V1, CLOSE); v1_power = CLOSE;
  digitalWrite(V2, CLOSE); v2_power = CLOSE;
  digitalWrite(V3, CLOSE); v3_power = CLOSE;
  digitalWrite(VS3, CLOSE); vs3_power = CLOSE;

  digitalWrite(FAN_IN, OPEN);  fan_in_power  = OPEN;
  digitalWrite(FAN_OUT, OPEN); fan_out_power = OPEN;
  analogWrite(HEATER, 0);      heater_pwm    = 0;
}

void loop(){
  protect_sensors();
  // Nur noch framed-SET einlesen / verarbeiten
  pollSetCommands();
}