from time import sleep
import csv
import json
import serial
import datetime
import threading
import click
import sys

# --- Gemeinsame Konstanten für die Mini2-Adressen/Kommandos ---
SET = 2
ADR_TARGET_P = 3
ADR_SET_RAMP = 4

# --------- Hilfsfunktionen für Mini2-Kommandos ---------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def pack_ramp_params(ramp_speed, start_pressure, end_pressure):
    """
    params: [8 bit ramp_speed][10 bit start][10 bit end]
    - ramp_speed: 0..255 (kPa/s), Richtung ergibt sich aus start/end
    - start/end : 0..1023 (kPa)
    """
    speed8 = clamp(int(ramp_speed), 0, 255) & 0xFF
    start10 = clamp(int(start_pressure), 0, 1023) & 0x3FF
    end10 = clamp(int(end_pressure), 0, 1023) & 0x3FF
    return (speed8 << 20) | (start10 << 10) | end10

def send_set_pressure(ser_mini2, kpa):
    msg = f"{SET}; {ADR_TARGET_P}; {int(kpa)}\n".encode("utf-8")
    ser_mini2.write(msg)

def send_set_ramp(ser_mini2, ramp_speed, start_pressure, end_pressure):
    params = pack_ramp_params(ramp_speed, start_pressure, end_pressure)
    msg = f"{SET}; {ADR_SET_RAMP}; {params}\n".encode("utf-8")
    ser_mini2.write(msg)

# --------- JSON-Programm lesen ---------
def read_program_from_json(filename):
    """
    Erwartet eine JSON-Liste von Schritten, z.B.:
    [
      {"type":"Normal","target_pressure":120},
      {"type":"Ramp","start_pressure":0,"end_pressure":120,"ramp_speed":50},
      {"type":"delay","delay_time":5000}
    ]
    """
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    # einfache Validierung/Normalisierung
    program = []
    for i, step in enumerate(data):
        t = step.get("type")
        if t == "Normal":
            program.append({"type":"Normal", "target_pressure": int(step["target_pressure"])})
        elif t == "Ramp":
            program.append({
                "type":"Ramp",
                "start_pressure": int(step["start_pressure"]),
                "end_pressure":   int(step["end_pressure"]),
                "ramp_speed":     int(step["ramp_speed"])
            })
        elif t == "delay":
            program.append({"type":"delay", "delay_time": int(step["delay_time"])})
        else:
            raise ValueError(f"Unbekannter Step-Typ in JSON an Position {i}: {t}")
    return program

# --------- Datenlogger bleibt wie gehabt ---------
def log_data(path="temp.csv"):
    print("log_data called")
    ADDRESS_MINI1 = '/dev/ttyACM0'
    t = threading.current_thread()
    with open(path, 'a+') as csv_file:
        wr = csv.writer(csv_file, quoting=csv.QUOTE_ALL)
        with serial.Serial(ADDRESS_MINI1, 115200, timeout=2) as mini1:
            while getattr(t, "do_run", True):
                try:
                    raw = mini1.readline()
                    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                    wr.writerow([current_time, raw.decode(errors="ignore").strip()])
                except Exception:
                    pass

# --------- Server, der das JSON-Programm ausführt ---------
@click.command()
@click.option('--data_path', default="my_DataStore.csv", help='Pfad für geloggte Daten (.csv)')
@click.option('--program_path', default="vader_program.json", help='Pfad zum JSON-Programm')
def run_server(program_path, data_path):
    ADDRESS_MINI2 = '/dev/ttyACM2'
    ADDRESS_MAXI  = '/dev/ttyACM1'

    # Logger starten
    data_logger = threading.Thread(target=log_data, args=(data_path,))
    data_logger.start()

    # Programm laden (JSON)
    vader_program = read_program_from_json(program_path)

    with serial.Serial(ADDRESS_MINI2, 115200, timeout=0.1) as mini2:
        with serial.Serial(ADDRESS_MAXI, 115200, timeout=0.1) as maxi:
            sleep(2)
            # Beispiel: Vakuum-Ventil öffnen + ein paar Default-Settings (wie gehabt)
            # maxi.write(b'2; 4; 1\n')   # open vacuum vent V4
            maxi.write(b'2; 3; 0\n')   # (falls noetig) irgendein Target am MAXI
            maxi.write(b'2; 30; 10\n') # (falls noetig) irgend ein MAXI-Param

            # --- Programm abspielen ---
            for step in vader_program:
                print("new step:", step)
                st = step["type"]

                if st == "Normal":
                    # MINI2 set_pressure -> target_pressure
                    send_set_pressure(mini2, step["target_pressure"])

                elif st == "Ramp":
                    # MINI2: zuerst Startdruck setzen, dann Rampe konfigurieren
                    send_set_ramp(mini2, step["ramp_speed"], step["start_pressure"], step["end_pressure"])
                    # Keine feste Wartezeit hier: Die Rampe läuft autonom im MINI2
                    # Falls du "blockierend" bis zum Enddruck warten willst, kannst du optional
                    # die erwartete Rampendauer abschaetzen:
                    duration_s = abs(step["end_pressure"]-step["start_pressure"]) / max(1, step["ramp_speed"])
                    sleep(duration_s)

                elif st == "delay":
                    # Warten (ms)
                    sleep(step["delay_time"] / 1000.0)

            # Logger stoppen
            data_logger.do_run = False

if __name__ == "__main__":
    csv.field_size_limit(sys.maxsize)
    run_server()