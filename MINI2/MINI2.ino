/*
  Controllino-SPS – Druckregelung                           (Debounce entfernt)
  + Rampenfunktion (STATE_MACHINE: STATE_NORMAL / STATE_RAMP)
*/

#include <Controllino.h>
#include <PWM.h>
#include <RunningMedian.h>

/* --------------------------- Kommunikation -------------------------------- */
struct control_message { int command, address; long value; } cm;  // value jetzt 32-bit

#define NO_COMMAND    0
#define SEND_STATUS   1
#define SET           2
#define ADR_TARGET_P  3
#define ADR_ERR      13
#define ADR_MAXSTEP   6
#define ADR_SET_RAMP  4   // NEU: "set ramp" -> 2; 4; {params}
#define ADR_SET_EVAC  5   // Both VP1, VP2 = 255

/* --------------------------- Hardware ------------------------------------- */
#define P20          CONTROLLINO_A1
#define P21          CONTROLLINO_A0
#define LEAK_SENSOR  CONTROLLINO_A3
#define VP1_PIN      CONTROLLINO_D5     // Druck ↑
#define VP2_PIN      CONTROLLINO_D1     // Druck ↓
#define VS2_PIN      CONTROLLINO_D4     // Sensor-Schutz (NC)

#define CLOSE 0
#define OPEN  1
#define PWM_MAX_CMD          225
#define PWM_MIN_OFFSET_VP1   116
#define PWM_MIN_OFFSET_VP2    98

/* --------------------------- Globale Variablen ---------------------------- */
bool  ERROR = false;
float target_p      = 0;          // Sollwert [kPa]
int   max_pwm_step  = 1;          // Rampe 1…255

bool P20_online = false;
unsigned long last_control_time_us = 0; // in Mikrosekunden

RunningMedian _p21(5), _p20(5);
static int pwmVP1 = 0, pwmVP2 = 0;
byte vs2State = OPEN;             // aktueller Ventilzustand

const float DEADBAND_KPA        = 0.5;
const float LOW_PRESSURE_LIMIT  = 150.0;

/* --------------------------- State Machine (NEU) -------------------------- */
enum ControlState { STATE_NORMAL = 0, STATE_RAMP = 1, STATE_EVAC = 2 };
ControlState ctrl_state = STATE_NORMAL;

float ramp_speed_kpa_s = 0.0f;   // Betrag in kPa/s (Vorzeichen folgt Richtung start→end)
float ramp_start_kpa   = 0.0f;
float ramp_end_kpa     = 0.0f;
int   ramp_dir         = +1;     // +1: aufwärts, -1: abwärts
unsigned long ramp_t0_ms = 0;

/* --------------------------- Sensor-Kalibrierung -------------------------- */
inline float mapP21(int adc) { return (adc + 3.24) / 0.279; }   // kPa
inline float mapP20(int adc) { return (adc + 5.5 ) / 1.54 ; }   // kPa

/* --------------------------- ADC-Hilfsfunktionen -------------------------- */
int fastADC_P21() { int v = analogRead(P21); _p21.add(v); return v; }
int accurateADC_P21(){ for(byte i=0;i<5;i++) _p21.add(analogRead(P21));
                       return _p21.getMedian(); }
int accurateADC_P20(){ for(byte i=0;i<5;i++) _p20.add(analogRead(P20));
                       return _p20.getMedian(); }

/* --------------------------- Schutzventil VS2 ----------------------------- *
   Debounce vollständig entfernt – nur Hysterese 191↘170 kPa bleibt.         */
void protect_sensors()
{
    float p21 = mapP21(fastADC_P21());

    if (p21 > 191.0) {
        digitalWrite(VS2_PIN, CLOSE);
        vs2State = CLOSE;
        P20_online = false;
    }
    else if (p21 < 170.0) {
        digitalWrite(VS2_PIN, OPEN);
        vs2State = OPEN;
        P20_online = true;
    }
}

float readPressure() {
    return (P20_online) 
            ? mapP20(accurateADC_P20())
            : mapP21(accurateADC_P21());
}

/* --------------------------- Druckregelung -------------------------------- */
// Reglerparameter
const float Kp_increase = 0.0;
const float Kp_decrease = 0.0;
const float KI_per_100ms = 1.0;            // Integralzuwachs bei 100 ms
const int OFFSET_INCREASE = 98;            // Basis für Druck erhöhen
const int OFFSET_DECREASE = 116;           // Basis für Druck reduzieren
const int TOLERANCE = 2;                   // Sollwert-Toleranz (Einheit wie Druck)

// Interne Zustände
float integral_increase = 0;
float integral_decrease = 0;


void control_pressure() {
  unsigned long now_us = micros();
  float delta_ms = (now_us - last_control_time_us) / 1000.0; // in ms
  if (delta_ms <= 0) return; // Schutz
  last_control_time_us = now_us;

  float pressure = readPressure();
  float error = target_p - pressure;

  // Sollwert erreicht?
  if (abs(error) <= TOLERANCE) {
    // Beide Regler zurücksetzen und Ausgänge null setzen
    integral_increase = 0;
    integral_decrease = 0;
    analogWrite(VP1_PIN, 0);
    pwmVP1 = 0;
    analogWrite(VP2_PIN, 0);
    pwmVP2 = 0;
    return;
  }

  // Skalierungsfaktor für Integral: (delta_ms / 100ms) * KI_per_100ms
  float integral_factor = (delta_ms / 100.0) * KI_per_100ms;

  if (error > 0) {
    // Istwert zu niedrig -> Druck erhöhen aktivieren
    integral_decrease = 0; // anderer Regler resetten

    float P = Kp_increase * error;
    // Integral auf Basis vergangener Zeit
    integral_increase += error * integral_factor;
    float I = integral_increase;

    float control = P + I;
    float raw_output = OFFSET_INCREASE + control;

    int pwm = constrain((int)round(raw_output), 0, 255);
    analogWrite(VP1_PIN, pwm);
    pwmVP1 = pwm; 
    analogWrite(VP2_PIN, 0);
    pwmVP2 = 0;
  } 
  else {
    // error < 0 -> Istwert zu hoch -> Druck reduzieren aktivieren
    integral_increase = 0;

    float error_pos = -error; // positiv machen
    float P = Kp_decrease * error_pos;
    integral_decrease += error_pos * integral_factor;
    float I = integral_decrease;

    float control = P + I;
    float raw_output = OFFSET_DECREASE + control;

    int pwm = constrain((int)round(raw_output), 0, 255);
    analogWrite(VP1_PIN, 0);
    pwmVP1 = 0;
    analogWrite(VP2_PIN, pwm);
    pwmVP2 = pwm;

  }

}

/* --------------------------- Rampen-Logik (NEU) --------------------------- */
inline void update_ramp_target_if_active() {
  if (ctrl_state != STATE_RAMP) return;

  // Sonderfälle: keine Bewegung notwendig
  if (ramp_speed_kpa_s <= 0.0f || fabs(ramp_end_kpa - ramp_start_kpa) < 1e-6) {
    target_p = ramp_end_kpa;
    ctrl_state = STATE_NORMAL;
    return;
  }

  unsigned long elapsed_ms = millis() - ramp_t0_ms;
  float elapsed_s = elapsed_ms / 1000.0f;

  float candidate = ramp_start_kpa + ramp_dir * ramp_speed_kpa_s * elapsed_s;

  // Begrenzen und State beenden, wenn Ziel erreicht/überschritten
  if ((ramp_dir > 0 && candidate >= ramp_end_kpa) ||
      (ramp_dir < 0 && candidate <= ramp_end_kpa)) {
    target_p = ramp_end_kpa;
    ctrl_state = STATE_NORMAL;
  } else {
    target_p = candidate;
  }
}

/* --------------------------- Kommunikation -------------------------------- */
control_message read_serial_command()
{
    control_message r = {NO_COMMAND,0,0};
    while (Serial.available()) {
        r.command =  Serial.readStringUntil(';').toInt(); Serial.read();
        r.address =  Serial.readStringUntil(';').toInt(); Serial.read();
        r.value   =  Serial.readStringUntil('\n').toInt();   // 32-bit Wert
    }
    return r;
}

void send_status()
{
    Serial.println("---");
    Serial.print("P21 (kPa): ");       Serial.print(mapP21(accurateADC_P21()));
      Serial.print(" (");Serial.print(accurateADC_P21());Serial.println(")");
    Serial.print("P20 (kPa): ");       Serial.print(mapP20(accurateADC_P20()));    
      Serial.print(" (");Serial.print(accurateADC_P20());Serial.println(")");
    Serial.print("VS2: ");           Serial.println(vs2State==CLOSE?"CLOSE":"OPEN");
    Serial.print("Setpoint: ");      Serial.println(target_p);
    Serial.print("PWM1: ");          Serial.println(pwmVP1);
    Serial.print("PWM2: ");          Serial.println(pwmVP2);
    // Optional: State-Ausgabe
    Serial.print("STATE: ");         Serial.println(ctrl_state==STATE_RAMP ? "RAMP" : "NORMAL");
}

void start_ramp_from_params(uint32_t params)
{
    // params: [8 bit speed][10 bit start][10 bit end]
    uint8_t  speed8   = (params >> 20) & 0xFF;
    uint16_t start10  = (params >> 10) & 0x3FF;
    uint16_t end10    =  params        & 0x3FF;

    ramp_speed_kpa_s = (float)speed8;   // Betrag; Richtung separat
    ramp_start_kpa   = (float)start10;
    ramp_end_kpa     = (float)end10;

    ramp_dir         = (ramp_end_kpa >= ramp_start_kpa) ? +1 : -1;

    target_p         = ramp_start_kpa;  // sofort auf Startwert setzen
    ramp_t0_ms       = millis();
}

void set(int adr, long val){
    switch(adr){
        case ADR_TARGET_P:
            target_p   = (float)val;
            ctrl_state = STATE_NORMAL;       // Rampenmodus verlassen, wie gefordert
            break;
        case ADR_MAXSTEP :
            max_pwm_step = constrain((int)val,1,255);
            break;
        case ADR_ERR     :
            ERROR = (bool)val;
            break;
        case ADR_SET_RAMP: {                 // NEU: "set ramp"
            uint32_t params = (uint32_t)val;
            ctrl_state       = STATE_RAMP;
            start_ramp_from_params(params);
            break;
        }
        case ADR_SET_EVAC: { 
            ctrl_state = STATE_EVAC;
            analogWrite(VP1_PIN, 255);
            pwmVP1 = 255; 
            analogWrite(VP2_PIN, 255);
            pwmVP2 = 255;
            break;
        }
    }
}

/* --------------------------- Setup / Loop --------------------------------- */
void setup()
{
    Serial.begin(115200);
    pinMode(P20,INPUT); pinMode(P21,INPUT); pinMode(LEAK_SENSOR,INPUT);
    pinMode(VP1_PIN,OUTPUT); pinMode(VP2_PIN,OUTPUT); pinMode(VS2_PIN,OUTPUT);

    protect_sensors();         // initialer VS2-Status
    target_p = 0;              // Start-Sollwert
    last_control_time_us = micros();
}

void loop()
{
    protect_sensors();

    cm = read_serial_command();
    if      (cm.command == SEND_STATUS) send_status();
    else if (cm.command == SET){
      set(cm.address, cm.value);
      }         

    if (ctrl_state != STATE_EVAC){
      // Rampen-Ziel ggf. fortschreiben
      update_ramp_target_if_active();
      for (int i=0;i<5;i++){ protect_sensors(); control_pressure(); }
    }
}