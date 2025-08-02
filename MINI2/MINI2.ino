/*
 ──────────────────────────────────────────────────────────────────────────────
  Controllino-SPS – Druckregelung                           (Debounce entfernt)
 ──────────────────────────────────────────────────────────────────────────────
  Änderungen 2025-06-06-b
  • Debounce-Zeit für Schutzventil VS2 entfernt
    → Ventil kann sofort schließen, sobald 191 kPa überschritten werden
      bzw. sofort wieder öffnen, wenn der Druck unter 170 kPa fällt
 ──────────────────────────────────────────────────────────────────────────────
*/

#include <Controllino.h>
#include <PWM.h>
#include <RunningMedian.h>

/* --------------------------- Kommunikation -------------------------------- */
struct control_message { int command, address, value; } cm;

#define NO_COMMAND   0
#define SEND_STATUS  1
#define SET          2
#define ADR_TARGET_P 3
#define ADR_ERR     13
#define ADR_MAXSTEP  6

/* --------------------------- Hardware ------------------------------------- */
#define P20          CONTROLLINO_A1
#define P21          CONTROLLINO_A0
#define LEAK_SENSOR  CONTROLLINO_A3
#define VP1_PIN      CONTROLLINO_D5     // Druck ↑
#define VP2_PIN      CONTROLLINO_D1     // Druck ↓
#define VS2_PIN      CONTROLLINO_D4     // Sensor-Schutz (NC)

#define CLOSE 0
#define OPEN  1
#define PWM_MAX_CMD     225
#define PWM_MIN_OFFSET_VP1   116
#define PWM_MIN_OFFSET_VP2   98

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
const float Kp = 2.0;
const float KI_per_100ms = 1.0;            // Integralzuwachs bei 100 ms
const int OFFSET_INCREASE = 98;            // Basis für Druck erhöhen
const int OFFSET_DECREASE = 116;           // Basis für Druck reduzieren
const int TOLERANCE = 2;                   // Sollwert-Toleranz (in denselben Einheiten wie Druck)

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

    float P = Kp * error;
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
    float P = Kp * error_pos;
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

/* --------------------------- Kommunikation -------------------------------- */
control_message read_serial_command()
{
    control_message r = {NO_COMMAND,0,0};
    while (Serial.available()) {
        r.command =  Serial.readStringUntil(';').toInt(); Serial.read();
        r.address =  Serial.readStringUntil(';').toInt(); Serial.read();
        r.value   =  Serial.readStringUntil('\n').toInt();
    }
    return r;
}

void send_status()
{
    Serial.println("---");
    Serial.print("P (kPa): ");       Serial.println(mapP21(accurateADC_P21()));
    Serial.print("VS2: ");           Serial.println(vs2State==CLOSE?"CLOSE":"OPEN");
    Serial.print("Setpoint: ");      Serial.println(target_p);
    Serial.print("PWM1: ");          Serial.println(pwmVP1);
    Serial.print("PWM2: ");          Serial.println(pwmVP2);
}

void set(int adr,int val){
    switch(adr){
        case ADR_TARGET_P: target_p = val; break;
        case ADR_MAXSTEP : max_pwm_step = constrain(val,1,255); break;
        case ADR_ERR     : ERROR = val;    break;
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
    else if (cm.command == SET)         set(cm.address, cm.value);

    for (int i=0;i<200;i++){ protect_sensors(); control_pressure(); }
}
