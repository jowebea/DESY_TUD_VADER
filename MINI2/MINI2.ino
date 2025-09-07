/*
  Controllino-SPS – Druckregelung (State Machine; EVAC entfernt)

  Diese Version nutzt das 32‑Bit Mini2‑Protokoll (Big‑Endian). Der Gerätezustand
  wird NICHT mehr über ein MODE‑Feld gesetzt, sondern aus der Art des zuletzt
  empfangenen Befehls abgeleitet:

    - SETPOINT (BASE+1)         -> STATE = AUTOMATIC
    - VP1/VP2 (BASE+0)          -> STATE = MANUAL
    - RAMP_* (BASE+5)           -> STATE = RAMP (Startdruck = aktueller Messwert)

  Mini2-Protokoll (dieses Gerät nutzt einen 6‑Bit Basis‑Kanal BASE_ADDR):
    Python  -> Arduino (SET):
      BASE+0: MODE(2) @25..24, VP1(8) @23..16, VP2(8) @15..8  // MODE wird ignoriert
      BASE+1: SETPOINT(16, 0.1 kPa) @25..10
      BASE+5: RAMP_SPEED(16, 0.01 kPa/s) @25..10, RAMP_END(10, kPa) @9..0  // Startdruck = aktueller Messwert
      BASE+6: COMMANDS: SEND_STATUS(1) @0  // Wenn Bit0=1 empfangen wird, sendet Gerät sofort Status

    Arduino -> Python (STATUS):
      BASE+2: P20 (16, 0.1 kPa) @25..10
      BASE+3: P21 (16, 0.1 kPa) @25..10
      BASE+4: MODE(2) @25..24, VP1(8) @23..16, VP2(8) @15..8  // MODE spiegelt internen STATE (0=MANUAL,1=RAMP,2=AUTOMATIC)
*/

#include <Controllino.h>
#include <PWM.h>
#include <RunningMedian.h>
#include <Arduino.h>
#include <stdint.h>
#include <math.h>

/* --------------------------- Mini2: Helpers ------------------------------- */
static inline uint32_t makeWord(uint8_t addr6, uint32_t payload26) {
  addr6 &= 0x3F;
  payload26 &= 0x03FFFFFFUL;
  return ((uint32_t)addr6 << 26) | payload26;
}

static inline void splitWord(uint32_t word, uint8_t &addr6, uint32_t &payload26) {
  addr6 = (word >> 26) & 0x3F;
  payload26 = word & 0x03FFFFFFUL;
}

static inline void u32ToBytesBE(uint32_t w, uint8_t out[4]) {
  out[0] = (w >> 24) & 0xFF;
  out[1] = (w >> 16) & 0xFF;
  out[2] = (w >> 8)  & 0xFF;
  out[3] = (w)       & 0xFF;
}

static inline uint32_t bytesToU32BE(const uint8_t in[4]) {
  return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) | ((uint32_t)in[2] << 8) | in[3];
}

struct Mini2Set {
  uint16_t setpoint_raw; // 0,1 kPa Auflösung
  uint8_t  mode2;        // 0..3 (2 Bit, nur zur Info)
  uint8_t  vp1PWM;       // 0..255
  uint8_t  vp2PWM;       // 0..255
  uint16_t ramp_speed_raw16; // Einheiten 0.01 kPa/s
  uint16_t ramp_end10;       // 10 Bit gültig (0..1023 kPa)
};

static bool applySetFrame(uint8_t baseAddr, uint32_t word, Mini2Set &state) {
  uint8_t addr; uint32_t payload;
  splitWord(word, addr, payload);

  if (addr == (uint8_t)(baseAddr + 0)) {
    state.mode2  = (payload >> 24) & 0x03;
    state.vp1PWM = (payload >> 16) & 0xFF;
    state.vp2PWM = (payload >> 8)  & 0xFF;
    return true;
  } else if (addr == (uint8_t)(baseAddr + 1)) {
    state.setpoint_raw = (payload >> 10) & 0xFFFF;
    return true;
  } else if (addr == (uint8_t)(baseAddr + 5)) {
    state.ramp_speed_raw16 = (payload >> 10) & 0xFFFF;
    state.ramp_end10       =  payload        & 0x03FF;
    return true;
  }
  return false;
}

struct Mini2Status {
  uint16_t p20_raw; // 0,1 kPa
  uint16_t p21_raw; // 0,1 kPa
  uint8_t  mode2;   // 0..3
  uint8_t  vp1PWM;  // 0..255
  uint8_t  vp2PWM;  // 0..255
};

static void makeStatusFrames(uint8_t baseAddr, const Mini2Status &s, uint32_t outWords[3]) {
  uint32_t payload2 = ((uint32_t)(s.p20_raw & 0xFFFF) << 10);
  outWords[0] = makeWord(baseAddr + 2, payload2);

  uint32_t payload3 = ((uint32_t)(s.p21_raw & 0xFFFF) << 10);
  outWords[1] = makeWord(baseAddr + 3, payload3);

  uint32_t payload4 = ((uint32_t)(s.mode2 & 0x03) << 24)
                    | ((uint32_t)s.vp1PWM << 16)
                    | ((uint32_t)s.vp2PWM << 8);
  outWords[2] = makeWord(baseAddr + 4, payload4);
}

/* --------------------------- Hardware ------------------------------------- */
#define P20          CONTROLLINO_A1
#define P21          CONTROLLINO_A0
#define LEAK_SENSOR  CONTROLLINO_A3
#define VP1_PIN      CONTROLLINO_D5
#define VP2_PIN      CONTROLLINO_D1
#define VS2_PIN      CONTROLLINO_D4

#define CLOSE 0
#define OPEN  1

/* --------------------------- Globale Variablen ---------------------------- */
bool  ERROR = false;
float target_p      = 0;
int   max_pwm_step  = 1;

bool P20_online = false;
unsigned long last_control_time_us = 0;

RunningMedian _p21(5), _p20(5);
static int pwmVP1 = 0, pwmVP2 = 0;
byte vs2State = OPEN;

/* --------------------------- State Machine -------------------------------- */
enum ControlState {
  STATE_MANUAL     = 0,
  STATE_RAMP       = 1,
  STATE_AUTOMATIC  = 2
};

ControlState ctrl_state = STATE_AUTOMATIC;

// MANUAL: externe PWM-Vorgaben
int manual_pwmVP1 = 0;
int manual_pwmVP2 = 0;

/* --------------------------- Rampen-Parameter ----------------------------- */
float ramp_speed_kpa_s = 0.0f;
float ramp_start_kpa   = 0.0f;
float ramp_end_kpa     = 0.0f;
int   ramp_dir         = +1;
unsigned long ramp_t0_ms = 0;

/* --------------------------- Sensor-Kalibrierung -------------------------- */
inline float mapP21(int adc) { return (adc + 3.24f) / 0.279f; }
inline float mapP20(int adc) { return (adc + 5.5f ) / 1.54f ; }

/* --------------------------- ADC-Hilfsfunktionen -------------------------- */
int fastADC_P21() { int v = analogRead(P21); _p21.add(v); return v; }
int accurateADC_P21(){ for(byte i=0;i<5;i++) _p21.add(analogRead(P21)); return _p21.getMedian(); }
int accurateADC_P20(){ for(byte i=0;i<5;i++) _p20.add(analogRead(P20)); return _p20.getMedian(); }

/* --------------------------- Schutzventil VS2 ----------------------------- */
void protect_sensors(){
    float p21 = mapP21(fastADC_P21());

    if (p21 > 191.0f) {
        digitalWrite(VS2_PIN, CLOSE);
        vs2State = CLOSE;
        P20_online = false;
    } else if (p21 < 170.0f) {
        digitalWrite(VS2_PIN, OPEN);
        vs2State = OPEN;
        P20_online = true;
    }
}

float readPressure() {
    return (P20_online) ? mapP20(accurateADC_P20()) : mapP21(accurateADC_P21());
}

/* --------------------------- Druckregelung -------------------------------- */
const float Kp_increase = 0.0f;
const float Kp_decrease = 0.0f;
const float KI_per_100ms = 1.0f;
const int OFFSET_INCREASE = 98;
const int OFFSET_DECREASE = 116;
const int TOLERANCE = 2;

float integral_increase = 0;
float integral_decrease = 0;

void control_pressure() {
  unsigned long now_us = micros();
  float delta_ms = (now_us - last_control_time_us) / 1000.0f;
  if (delta_ms <= 0) return;
  last_control_time_us = now_us;

  float pressure = readPressure();
  float error = target_p - pressure;

  if (fabs(error) <= TOLERANCE) {
    integral_increase = 0;
    integral_decrease = 0;
    analogWrite(VP1_PIN, 0);  pwmVP1 = 0;
    analogWrite(VP2_PIN, 0);  pwmVP2 = 0;
    return;
  }

  float integral_factor = (delta_ms / 100.0f) * KI_per_100ms;

  if (error > 0) {
    integral_decrease = 0;
    float P = Kp_increase * error;
    integral_increase += error * integral_factor;
    float control = P + integral_increase;
    int pwm = constrain((int)lroundf(OFFSET_INCREASE + control), 0, 255);
    analogWrite(VP1_PIN, pwm); pwmVP1 = pwm;
    analogWrite(VP2_PIN, 0);   pwmVP2 = 0;
  } else {
    integral_increase = 0;
    float epos = -error;
    float P = Kp_decrease * epos;
    integral_decrease += epos * integral_factor;
    float control = P + integral_decrease;
    int pwm = constrain((int)lroundf(OFFSET_DECREASE + control), 0, 255);
    analogWrite(VP1_PIN, 0);   pwmVP1 = 0;
    analogWrite(VP2_PIN, pwm); pwmVP2 = pwm;
  }
}

/* --------------------------- Rampen-Logik --------------------------------- */
inline void update_ramp_target_if_active() {
  if (ctrl_state != STATE_RAMP) return;

  if (ramp_speed_kpa_s <= 0.0f || fabs(ramp_end_kpa - ramp_start_kpa) < 1e-6f) {
    target_p = ramp_end_kpa;
    ctrl_state = STATE_AUTOMATIC;
    return;
  }

  unsigned long elapsed_ms = millis() - ramp_t0_ms;
  float elapsed_s = elapsed_ms / 1000.0f;
  float candidate = ramp_start_kpa + ramp_dir * ramp_speed_kpa_s * elapsed_s;

  if ((ramp_dir > 0 && candidate >= ramp_end_kpa) ||
      (ramp_dir < 0 && candidate <= ramp_end_kpa)) {
    target_p = ramp_end_kpa;
    ctrl_state = STATE_AUTOMATIC;
  } else {
    target_p = candidate;
  }
}

/* --------------------------- Mini2 I/O ------------------------------------ */
static const uint8_t BASE_ADDR = 0x05;
static Mini2Set g_set = {0, 2, 0, 0, 0, 0};

static bool readWordFromSerial(uint32_t &w) {
  if (Serial.available() < 4) return false;
  uint8_t b[4];
  size_t n = Serial.readBytes(b, 4);
  if (n != 4) return false;
  w = bytesToU32BE(b);
  return true;
}

static bool status_requested = false;

static void handle_comms_mini2(){
  uint32_t w;
  while (readWordFromSerial(w)) {
    uint8_t a; uint32_t p; splitWord(w, a, p);
    if (applySetFrame(BASE_ADDR, w, g_set)) {
      if (a == (uint8_t)(BASE_ADDR + 1)) {
        // Setpoint empfangen -> AUTOMATIC
        target_p = (float)g_set.setpoint_raw * 0.1f;
        ctrl_state = STATE_AUTOMATIC;
      }
      else if (a == (uint8_t)(BASE_ADDR + 0)) {
        // PWM empfangen -> MANUAL
        manual_pwmVP1 = (int)g_set.vp1PWM;
        manual_pwmVP2 = (int)g_set.vp2PWM;
        ctrl_state = STATE_MANUAL;
      }
      else if (a == (uint8_t)(BASE_ADDR + 5)) {
        // Rampenparameter empfangen -> RAMP
        float current_p = readPressure();
        ramp_start_kpa   = current_p;
        ramp_end_kpa     = (float)g_set.ramp_end10;
        ramp_speed_kpa_s = (float)g_set.ramp_speed_raw16 * 0.01f;
        ramp_dir         = (ramp_end_kpa >= ramp_start_kpa) ? +1 : -1;
        target_p         = ramp_start_kpa;
        ramp_t0_ms       = millis();
        ctrl_state       = STATE_RAMP;
      }
      else if (a == (uint8_t)(BASE_ADDR + 6)) {
        // SEND_STATUS
        if (p & 0x1) status_requested = true;
      }
    }
  }

  if (status_requested) {
    send_status_mini2_once();
    status_requested = false;
  }
}

static void send_status_mini2_once(){
  float p20_kpa = mapP20(accurateADC_P20());
  float p21_kpa = mapP21(accurateADC_P21());

  Mini2Status s;
  s.p20_raw = (uint16_t)constrain((int)lroundf(p20_kpa * 10.0f), 0, 65535);
  s.p21_raw = (uint16_t)constrain((int)lroundf(p21_kpa * 10.0f), 0, 65535);
  s.mode2   = (uint8_t)ctrl_state;
  s.vp1PWM  = (uint8_t)constrain(pwmVP1, 0, 255);
  s.vp2PWM  = (uint8_t)constrain(pwmVP2, 0, 255);

  uint32_t words[3];
  makeStatusFrames(BASE_ADDR, s, words);

  uint8_t buf[4];
  for (int i=0;i<3;i++){
    u32ToBytesBE(words[i], buf);
    Serial.write(buf, 4);
  }
}

/* --------------------------- State Steps ---------------------------------- */
inline void step_manual(){
    analogWrite(VP1_PIN, manual_pwmVP1); pwmVP1 = manual_pwmVP1;
    analogWrite(VP2_PIN, manual_pwmVP2); pwmVP2 = manual_pwmVP2;
}

inline void step_ramp(){
    update_ramp_target_if_active();
    for (int i=0; i<5; ++i){ control_pressure(); }
}

inline void step_automatic(){
    for (int i=0; i<5; ++i){ control_pressure(); }
}

/* --------------------------- Setup / Loop --------------------------------- */
void setup(){
    Serial.begin(115200);
    pinMode(P20,INPUT); pinMode(P21,INPUT); pinMode(LEAK_SENSOR,INPUT);
    pinMode(VP1_PIN,OUTPUT); pinMode(VP2_PIN,OUTPUT); pinMode(VS2_PIN,OUTPUT);

    protect_sensors();
    target_p = 0;
    last_control_time_us = micros();

    analogWrite(VP1_PIN, 0); pwmVP1 = 0;
    analogWrite(VP2_PIN, 0); pwmVP2 = 0;
}

void loop(){
    protect_sensors();
    handle_comms_mini2();

    switch (ctrl_state){
      case STATE_MANUAL:     step_manual();     break;
      case STATE_RAMP:       step_ramp();       break;
      case STATE_AUTOMATIC:  step_automatic();  break;
      default:               step_automatic();  break;
    }

    // Status nur auf Anfrage (BASE+6 Bit0)
}
