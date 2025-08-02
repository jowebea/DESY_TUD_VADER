#include <Controllino.h>
#include "ModbusRtu.h"
#include <RunningMedian.h>

// Modbus Master-Objekt (Adresse=0, UART=RS485Serial=3, DE/RE-Pin=0)
#define MasterModbusAdd  0
#define RS485Serial      3
Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 0);

// Slave-Adressen deiner MFCs
#define MFC_BUTAN  1
#define MFC_CO2    2
#define MFC_N2     3

// Timeout für Modbus-Abfragen in Millisekunden
#define MODBUS_TIMEOUT 500

// =======================================================
// ==========   Steuerung   =======
// =======================================================

// Analoge Eingänge
// (Hinweis: Hier wurde LEAK_SENSOR auf A0 gelegt, um Kollision mit MFC_CO2_FLOW_SENSOR auf A3 zu vermeiden)
#define LEAK_SENSOR            CONTROLLINO_A3 //checked

#define PT100_STORAGE          CONTROLLINO_A7 //checked
#define PT100_VACUUM           CONTROLLINO_A6 //checked
#define PT100_HOUSING          CONTROLLINO_A8 //checked

#define P11                    CONTROLLINO_A9 //checked
#define P30                    CONTROLLINO_A5 //checked
#define P31                    CONTROLLINO_A4 //checked

// Digitale Ausgänge (Relais) und PWM
#define FAN_IN                 CONTROLLINO_RELAY_00 //checked
bool fan_in_power;
#define FAN_OUT                CONTROLLINO_RELAY_01 //checked
bool fan_out_power;
#define HEATER                 CONTROLLINO_D5 //checked
int heater_pwm;

#define V1                     CONTROLLINO_RELAY_05
bool v1_power;
#define V2                     CONTROLLINO_RELAY_04
bool v2_power;
#define V3                     CONTROLLINO_RELAY_03
bool v3_power;
#define V4                     CONTROLLINO_RELAY_02
bool v4_power;

#define VS3                    CONTROLLINO_RELAY_07
bool vs3_power;

// Ventil-Logik
#define CLOSE 0
#define OPEN  1

// Struktur für eingehende Steuerkommandos
struct control_message {
    int command;
    int address;
    int value;
};

control_message cm;

unsigned short command;

// Steuerbefehle
#define NO_COMMAND    0
#define SEND_STATUS   1
#define SET           2

// Adressen für die SET-/GET-Befehle
#define NO_ADDRESS     0
#define ADDRESS_V1     1
#define ADDRESS_V2     2
#define ADDRESS_V3     3
#define ADDRESS_V4     4

// Wir mappen die drei MFC-Adressen auf 5..7
#define ADDRESS_BUTAN_SET_POINT  5  // MFC_BUTAN (ID=1)
#define ADDRESS_CO2_SET_POINT    6  // MFC_CO2   (ID=2)
#define ADDRESS_N2_SET_POINT     7  // MFC_N2    (ID=3)

#define ADDRESS_FAN_IN 8
#define ADDRESS_FAN_OUT 9
#define ADDRESS_TEMP   10
#define ADDRESS_HEATER 11
#define ADDRESS_VS3    12
#define ADDRESS_ERROR  13


#define ADDRESS_MFC_BUTAN_FLOW  20
#define ADDRESS_MFC_BUTAN_TOTAL  21

#define ADDRESS_MFC_CO2_FLOW  30
#define ADDRESS_MFC_CO2_TOTAL  31

#define ADDRESS_MFC_N2_FLOW  40
#define ADDRESS_MFC_N2_TOTAL  41

bool ERROR = false;
RunningMedian _p11(5), _p31(5);

int accurateADC_P11(){ for(byte i=0;i<5;i++) _p11.add(analogRead(P11));
                       return _p11.getMedian(); }
int accurateADC_P31(){ for(byte i=0;i<5;i++) _p31.add(analogRead(P31));
                       return _p31.getMedian(); }

// ---------------------------------------------------------
// Hilfsfunktionen zum Lesen/Schreiben eines Float-Wertes
// (32-Bit Float in 2 Holding-Registern)
// ---------------------------------------------------------
float readFloatFromHoldingReg(uint8_t slaveId, uint16_t startReg)
{
  uint16_t readBuf[2] = {0, 0};

  // Abfrageobjekt anlegen
  modbus_t request;
  request.u8id       = slaveId;        // MFC-Slave-ID
  request.u8fct      = 3;              // Read Holding Registers
  request.u16RegAdd  = startReg;       // Startadresse
  request.u16CoilsNo = 2;              // Anzahl Register
  request.au16reg    = readBuf;

  // Anfrage starten
  ControllinoModbusMaster.query(request);

  // Warten auf Abschluss oder Timeout
  unsigned long t0 = millis();
  while (ControllinoModbusMaster.getState() != COM_IDLE) {
    ControllinoModbusMaster.poll();
    if (millis() - t0 > MODBUS_TIMEOUT) {
      Serial.println("Modbus read timeout!");
      return NAN;
    }
  }

  // Ergebnis in Float umwandeln
  union {
    float f;
    uint16_t w[2];
  } converter;

  // Achtung: Reihenfolge je nach Endian-Layout evtl. tauschen
  converter.w[1] = readBuf[0];
  converter.w[0] = readBuf[1];

  return converter.f;
}

bool writeFloatToHoldingReg(uint8_t slaveId, uint16_t startReg, float value)
{
  union {
    float f;
    uint16_t w[2];
  } converter;
  converter.f = value;

  uint16_t writeBuf[2];
  writeBuf[1] = converter.w[0];
  writeBuf[0] = converter.w[1];

  modbus_t request;
  request.u8id       = slaveId;
  request.u8fct      = 16;  // Write Multiple Holding Registers
  request.u16RegAdd  = startReg;
  request.u16CoilsNo = 2;
  request.au16reg    = writeBuf;

  ControllinoModbusMaster.query(request);

  // Warten auf Abschluss oder Timeout
  unsigned long t0 = millis();
  while (ControllinoModbusMaster.getState() != COM_IDLE) {
    ControllinoModbusMaster.poll();
    if (millis() - t0 > MODBUS_TIMEOUT) {
      Serial.println("Modbus write timeout!");
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------
// Lese-/Schreibfunktionen für spezifische Register
// ---------------------------------------------------------
float read_flow(uint8_t MFC_addr)                 { return readFloatFromHoldingReg(MFC_addr, 0x0000); }
float read_gastemperature(uint8_t MFC_addr)       { return readFloatFromHoldingReg(MFC_addr, 0x0002); }
float read_totalisator(uint8_t MFC_addr)          { return readFloatFromHoldingReg(MFC_addr, 0x0004); }
float read_flow_setpoint(uint8_t MFC_addr)        { return readFloatFromHoldingReg(MFC_addr, 0x0006); }

void reset_totalisator(uint8_t MFC_addr)
{
  bool ok = writeFloatToHoldingReg(MFC_addr, 0x0004, 0.0f);
  if (!ok) {
    Serial.println("Fehler beim Reset des Totalisators!");
  }
}

void set_flow(uint8_t MFC_addr, float flow)
{
  bool ok = writeFloatToHoldingReg(MFC_addr, 0x0006, flow);
  if (!ok) {
    Serial.println("Fehler beim Schreiben des Sollwertes!");
  }
}
/**
 *  Liest die „Bezeichnung Medium (lang)“ aus 0x6022..0x603A (25 Register).
 *  Jedes Register enthält 2 ASCII-Zeichen. 
 *  Rueckgabe als String (kann Leerzeichen / 0-Bytes enthalten).
 *
 *  @param slaveId  Modbus-Slave-Adresse (z.B. 1 = Butan, 2 = CO2, etc.)
 *  @return         Ausgelesene Zeichenkette oder ein leerer String bei Timeout.
 */

/**
 *  Liest die Einheit (Zeichenkette) aus 0x6046..0x6049 (4 Register).
 *  Jedes Register enthält 2 ASCII-Zeichen.
 *
 *  @param slaveId  Modbus-Slave-Adresse (z.B. 1 = Butan, 2 = CO2, etc.)
 *  @return         Zeichenkette mit der Einheit oder leer bei Timeout.
 */
String read_mfc_unit_string(uint8_t slaveId)
{
  const uint16_t START_REG = 0x6046;
  const uint16_t END_REG   = 0x6049;
  const uint16_t NUM_REGS  = (END_REG - START_REG) + 1; // 4

  uint16_t readBuf[NUM_REGS] = {0};

  // Modbus-Anfrage vorbereiten (FC=3, Read Holding Registers)
  modbus_t request;
  request.u8id       = slaveId;
  request.u8fct      = 3;
  request.u16RegAdd  = START_REG;
  request.u16CoilsNo = NUM_REGS;
  request.au16reg    = readBuf;

  // Anfrage starten
  ControllinoModbusMaster.query(request);

  // Warten auf Abschluss oder Timeout
  unsigned long t0 = millis();
  while (ControllinoModbusMaster.getState() != COM_IDLE) {
    ControllinoModbusMaster.poll();
    if (millis() - t0 > MODBUS_TIMEOUT) {
      Serial.println("Modbus read timeout (Einheit)!");
      return String("");
    }
  }

  // Die gelesenen Register in eine ASCII-Zeichenkette umwandeln
  char text[2 * NUM_REGS + 1];
  for (int i = 0; i < NUM_REGS; i++) {
    uint16_t regVal = readBuf[i];
    char c1 = (char)((regVal >> 8) & 0xFF); // High-Byte
    char c2 = (char)(regVal & 0xFF);       // Low-Byte
    text[2*i]   = c1;
    text[2*i+1] = c2;
  }
  text[2 * NUM_REGS] = '\0';

  return String(text);
}
// ====================================================
// Liest aus dem Serial den nächsten Befehl
// (Format: command;address;value\n )
// ====================================================
control_message read_serial_command(){
  control_message r;
  unsigned short cmd = 0;
  unsigned short addr = 0;
  int val = 0;

  while (Serial.available() > 0) {
    cmd  = Serial.readStringUntil(';').toInt();
    Serial.read(); // das ";" entfernen
    addr = Serial.readStringUntil(';').toInt();
    Serial.read(); // das ";" entfernen
    val  = Serial.readStringUntil('\n').toInt();
  }
  r.command = cmd;
  r.address = addr;
  r.value   = val;
  return r;
}

// Solltemperatur (in 1/100 °C)
int temp_housing_target = 3200;

// Beispiel-PT100-Lesefunktion (noch rudimentär)
int read_PT100_m50_100(int PIN){
    // Hier ggf. Kalibrierung / Umrechnung in °C/°F
    // Derzeit nur roher ADC-Wert
    return analogRead(PIN);
}
inline float mapPT100(int adc) { return (adc) *0.44-50; }   // °C

int read_px1(int PIN){
    // Beispiel: druck 0..10 bar -> 0..1.000.000 Pa
    // => Hier nur roher ADC-Wert
    return analogRead(PIN);
}
inline float mapPx1(int adc) { return (adc) / 0.130172; }   // kPa

int read_px0(int PIN){
    // Beispiel: druck -1..+1 bar -> ...
    // => Hier nur roher ADC-Wert
    return analogRead(PIN);
}

// Beispiel einer Automatikfunktion
void auto_vac(){
    // Einfaches Beispiel: schließe V4 über 100 ADC-Schwelle
    if (analogRead(P31) > 40) {
        digitalWrite(V4, CLOSE);
        v4_power = CLOSE;
    }
    if (analogRead(P31) < 33) {
        digitalWrite(V4, OPEN);
        v4_power = OPEN;
    }
}

// ====================================================
// send_status(): gibt Zustand aller relevanten I/Os aus
// ====================================================
void send_status(){
    Serial.println("======================");
    Serial.println("--- system status ---");
    Serial.println("======================");
    
    Serial.println("--- activ components ---");
    Serial.print("v1_power       [1]: "); Serial.println(v1_power);
    Serial.print("v2_power       [2]: "); Serial.println(v2_power);
    Serial.print("v3_power       [3]: "); Serial.println(v3_power);
    Serial.print("v4_power       [4]: "); Serial.println(v4_power);
    Serial.print("fan_in_power   [8]: "); Serial.println(fan_in_power);
    Serial.print("fan_out_power  [9]: "); Serial.println(fan_out_power);
    Serial.print("heater_pwm    [11]: "); Serial.println(heater_pwm);
    Serial.print("vs3_power     [12]: "); Serial.println(vs3_power);

    Serial.println("----- sensors -----");
    Serial.print("leak_sensor:   "); Serial.println(analogRead(LEAK_SENSOR));
    
    int pt100_storage_raw = read_PT100_m50_100(PT100_STORAGE);
    int pt100_vacuum_raw = read_PT100_m50_100(PT100_VACUUM);
    int pt100_housing_raw = read_PT100_m50_100(PT100_HOUSING);
    Serial.print("pt100_storage: "); Serial.print(mapPT100(pt100_storage_raw), 2);
      Serial.print(" °C ("); Serial.print(pt100_storage_raw); Serial.println(")");
    Serial.print("pt100_vacuum: "); Serial.print(mapPT100(pt100_vacuum_raw), 2);
      Serial.print(" °C ("); Serial.print(pt100_storage_raw); Serial.println(")");
    Serial.print("pt100_housing: "); Serial.print(mapPT100(pt100_housing_raw), 2);
      Serial.print(" °C ("); Serial.print(pt100_storage_raw); Serial.println(")");
    
    int p11_raw = accurateADC_P11();
    Serial.print("p11: "); Serial.print(mapPx1(p11_raw), 2);
      Serial.print(" kPa ("); Serial.print(p11_raw); Serial.println(")");
    int p31_raw = accurateADC_P31();
    Serial.print("p31: "); Serial.print(mapPx1(p31_raw), 2);
      Serial.print(" kPa ("); Serial.print(p31_raw); Serial.println(")");

    Serial.println("------- MFCs -------");
    Serial.println("Butan/Methan MFC");
    Serial.print("   unit:                  ");  Serial.println(read_mfc_unit_string(MFC_BUTAN));
    Serial.print("   flow Butan:            ");  Serial.println(read_flow(MFC_BUTAN));
    Serial.print("   flow (set point) [20]: ");  Serial.println(read_flow_setpoint(MFC_BUTAN));
    Serial.print("   temperature:           ");  Serial.println(read_gastemperature(MFC_BUTAN));
    Serial.print("   totalisator      [21]: ");  Serial.println(read_totalisator(MFC_BUTAN));
    Serial.println("CO2 MFC");
    Serial.print("   unit:                  ");  Serial.println(read_mfc_unit_string(MFC_CO2));
    Serial.print("   flow CO2:              ");  Serial.println(read_flow(MFC_CO2));
    Serial.print("   flow (set point) [30]: ");  Serial.println(read_flow_setpoint(MFC_CO2));
    Serial.print("   temperature:           ");  Serial.println(read_gastemperature(MFC_CO2));
    Serial.print("   totalisator      [31]: ");  Serial.println(read_totalisator(MFC_CO2));
    Serial.println("N2/Argon MFC");
    Serial.print("   unit:                  ");  Serial.println(read_mfc_unit_string(MFC_N2));
    Serial.print("   flow N2:               ");  Serial.println(read_flow(MFC_N2));
    Serial.print("   flow (set point) [40]: ");  Serial.println(read_flow_setpoint(MFC_N2));
    Serial.print("   temperature:           ");  Serial.println(read_gastemperature(MFC_N2));
    Serial.print("   totalisator      [41]: ");  Serial.println(read_totalisator(MFC_N2));

}

// Hilfsfunktionen
void evacuate_gas_storage(){
    // V1, V2, V3, V4 öffnen
    digitalWrite(V1, OPEN); v1_power = OPEN;
    digitalWrite(V2, OPEN); v2_power = OPEN;
    digitalWrite(V3, OPEN); v3_power = OPEN;
    digitalWrite(V4, OPEN); v4_power = OPEN;
}

void protect_sensors(){
    // Beispiel: VS3 schließen, wenn P31 zu hoch wird
    int p31 = read_px1(P31);
    if (p31 > 0){ // analog ab welcher Schwelle?
        digitalWrite(VS3, CLOSE);
        vs3_power = CLOSE;
    }
    if (p31 < 0){
        digitalWrite(VS3, OPEN);
        vs3_power = OPEN;
    }
}

void break_experiment(){
    Serial.println("break_experiment!");
    evacuate_gas_storage();
    // Heizung aus
    set(ADDRESS_HEATER, 0);
    // Lüfter an
    set(ADDRESS_FAN_IN, OPEN);
    set(ADDRESS_FAN_OUT, OPEN);
    ERROR = true;
}

bool no_gas_leak(){
    int s = analogRead(LEAK_SENSOR);
    if (s < 900) {
        return true;
    } else {
        Serial.print("Warnung: Gasleck-Sensorwert: ");
        Serial.println(s);
        return false;
    }
}



// ====================================================
// set(): steuert die IOs anhand address / value
// ====================================================
void set(int address, int value){
    Serial.print("set() called -> address: ");
    Serial.print(address);
    Serial.print(" , value: ");
    Serial.println(value);

    switch (address) {
        case ADDRESS_V1:
            digitalWrite(V1, value);
            v1_power = value;
            break;
        case ADDRESS_V2:
            digitalWrite(V2, value);
            v2_power = value;
            break;
        case ADDRESS_V3:
            digitalWrite(V3, value);
            v3_power = value;
            break;
        case ADDRESS_V4:
            digitalWrite(V4, value);
            v4_power = value;
            break;
        case ADDRESS_FAN_IN:
            digitalWrite(FAN_IN, value);
            fan_in_power = value;
            break;
        case ADDRESS_FAN_OUT:
            digitalWrite(FAN_OUT, value);
            fan_out_power = value;
            break;
        case ADDRESS_TEMP:
            temp_housing_target = value;
            break;
        case ADDRESS_HEATER:
            analogWrite(HEATER, value);
            heater_pwm = value;
            break;
        case ADDRESS_VS3:
            digitalWrite(VS3, value);
            vs3_power = value;
            break;
        case ADDRESS_ERROR:
            ERROR = value;
            break;
        case ADDRESS_MFC_BUTAN_FLOW:
            set_flow(MFC_BUTAN, value);
            break;
        case ADDRESS_MFC_BUTAN_TOTAL:
            reset_totalisator(MFC_BUTAN);
            break;
        case ADDRESS_MFC_CO2_FLOW:
            set_flow(MFC_CO2, value);
            break;
        case ADDRESS_MFC_CO2_TOTAL:
            reset_totalisator(MFC_CO2);
            break;
        case ADDRESS_MFC_N2_FLOW:
            set_flow(MFC_N2, value);
            break;
        case ADDRESS_MFC_N2_TOTAL:
            reset_totalisator(MFC_N2);
            break;
        default:
            Serial.println("Unbekannte Adresse in set()!");
            break;
    }
}

// Einfache Stellgrößenberechnung (noch Platzhalter)
#define __Kp 30
#define __Ki 0.7
#define __Kd 200
void control_temp(){
    // Sehr rudimentär – nur als Beispiel
    if(read_PT100_m50_100(PT100_HOUSING) < temp_housing_target){
      set(ADDRESS_HEATER, 255);
    } else {
      set(ADDRESS_HEATER, 0);
    }
}

// ====================================================
// setup()
// ====================================================
void setup()
{
    Serial.begin(115200);

    // (1) Modbus Master starten (aus Programm 1)
    ControllinoModbusMaster.begin(9600);     // Baudrate anpassen
    ControllinoModbusMaster.setTimeOut(500); // internes Timeout

    // (2) Pinmodes und Startwerte (aus Programm 2)
    pinMode(LEAK_SENSOR, INPUT);


    pinMode(PT100_STORAGE, INPUT);
    pinMode(PT100_VACUUM, INPUT);
    pinMode(PT100_HOUSING, INPUT);

    pinMode(P11, INPUT);
    pinMode(P30, INPUT);
    pinMode(P31, INPUT);

    pinMode(FAN_IN, OUTPUT);
    pinMode(FAN_OUT, OUTPUT);
    pinMode(HEATER, OUTPUT);

    pinMode(V1, OUTPUT);
    pinMode(V2, OUTPUT);
    pinMode(V3, OUTPUT);
    pinMode(V4, OUTPUT);
    pinMode(VS3, OUTPUT);

    // Anfangszustände
    set(ADDRESS_V1, CLOSE);
    set(ADDRESS_V2, CLOSE);
    set(ADDRESS_V3, CLOSE);
    set(ADDRESS_V4, OPEN);
    set(ADDRESS_VS3, CLOSE);

    set(ADDRESS_FAN_IN, OPEN);
    set(ADDRESS_FAN_OUT, OPEN);
    set(ADDRESS_HEATER, 0);
    
}

// ====================================================
// loop()
// ====================================================
void loop()
{   
    protect_sensors();
    //auto_vac();
    //control_temp();

    // Kommandos von Serial einlesen
    cm = read_serial_command();

    switch (cm.command) {
      case NO_COMMAND:
        // kein Kommando
        break;
      case SEND_STATUS:
        send_status();   // Ausgabe aller Stati
        break;
      case SET:
        set(cm.address, cm.value);
        break;
      default:
        // unbekannter Befehl
        break;
    }
}
