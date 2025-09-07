from time import sleep
import csv
import json
import datetime
import threading
import click
import sys

# --- Treiber importieren: bitte Modulnamen anpassen ---
# Beispiel: from drivers.vader import VaderDeviceDriver
from your_driver_module import VaderDeviceDriver  # <-- ANPASSEN!

# --------- JSON-Programm lesen ---------
def read_program_from_json(filename):
    """
    Erwartet eine JSON-Liste von Schritten, z.B.:
    [
      {"type":"Normal","target_pressure":120},
      {"type":"Ramp","start_pressure":0,"end_pressure":120,"ramp_speed":50},
      {"type":"delay","delay_time":5000}
    ]

    Bedeutungen:
      - Normal: target_pressure in kPa (Ganzzahl)
      - Ramp:   start_pressure (kPa), end_pressure (kPa), ramp_speed (kPa/s)
      - delay:  delay_time in ms
    """
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    program = []
    for i, step in enumerate(data):
        t = step.get("type")
        if t == "Normal":
            program.append({"type": "Normal", "target_pressure": int(step["target_pressure"])})
        elif t == "Ramp":
            program.append({
                "type": "Ramp",
                "start_pressure": int(step["start_pressure"]),
                "end_pressure":   int(step["end_pressure"]),
                "ramp_speed":     float(step["ramp_speed"])
            })
        elif t == "delay":
            program.append({"type": "delay", "delay_time": int(step["delay_time"])})
        else:
            raise ValueError(f"Unbekannter Step-Typ in JSON an Position {i}: {t}")
    return program

# --------- Datenlogger: loggt MINI1-Druck in CSV ---------
def log_data(driver: VaderDeviceDriver, path="my_DataStore.csv", period_s: float = 0.1):
    """
    Schreibt Zeitstempel + aktuellen MINI1-Druck in eine CSV.
    Läuft, bis thread.do_run=False gesetzt wird.
    """
    t = threading.current_thread()
    with open(path, 'a+', newline='') as csv_file:
        wr = csv.writer(csv_file, quoting=csv.QUOTE_MINIMAL)
        # optional Kopfzeile
        if csv_file.tell() == 0:
            wr.writerow(["timestamp", "pressure_kpa"])
        while getattr(t, "do_run", True):
            try:
                p = driver.mini1.pressure
                ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                wr.writerow([ts, "" if p is None else f"{p:.3f}"])
                csv_file.flush()
            except Exception:
                # Logging stillhalten – Produktionscode könnte hier warnen
                pass
            sleep(period_s)

# --------- Server, der das JSON-Programm ausführt ---------
@click.command()
@click.option('--mini1_port', default="/dev/ttyACM0", help='Serieller Port MINI1')
@click.option('--mini2_port', default="/dev/ttyACM2", help='Serieller Port MINI2')
@click.option('--maxi_port',  default="/dev/ttyACM1", help='Serieller Port MAXI')
@click.option('--data_path', default="my_DataStore.csv", help='Pfad für geloggte Daten (.csv)')
@click.option('--program_path', default="vader_program.json", help='Pfad zum JSON-Programm')
@click.option('--log_period_ms', default=100, help='Logging-Periode MINI1 in Millisekunden')
def run_server(program_path, data_path, mini1_port, mini2_port, maxi_port, log_period_ms):
    """
    Führt die Programm-Schritte aus JSON mit dem neuen High-Level-Treiber aus.
    """
    # Programm laden (JSON)
    vader_program = read_program_from_json(program_path)

    driver = None
    data_logger = None
    try:
        # Treiber initialisieren
        driver = VaderDeviceDriver(mini1_port=mini1_port, mini2_port=mini2_port, maxi_port=maxi_port)

        # Logger starten
        data_logger = threading.Thread(
            target=log_data,
            args=(driver, data_path, max(0.01, float(log_period_ms) / 1000.0)),
            daemon=True
        )
        data_logger.start()

        # ganz kurzer settle
        sleep(0.2)

        # --- Programm abspielen ---
        for step in vader_program:
            print("new step:", step)
            st = step["type"]

            if st == "Normal":
                # MINI2: Ziel-Druck setzen
                target = step["target_pressure"]
                driver.setpoint_pressure(target)

            elif st == "Ramp":
                # MINI2: optional Startdruck anfahren, dann Rampe setzen
                start_p = step["start_pressure"]
                end_p   = step["end_pressure"]
                speed   = float(step["ramp_speed"])  # kPa/s

                # Startdruck anfahren (kurz setzen, Firmware übernimmt Start für Rampe aus aktuellem Druck)
                driver.setpoint_pressure(start_p)
                sleep(0.2)  # kurze Stabilisierung

                driver.set_ramp(speed_kpa_s=speed, end_kpa=end_p)

                # Optional: grob abwarten, bis Ende erreicht sein sollte
                approx_duration = abs(end_p - start_p) / max(1e-6, speed)
                sleep(min(approx_duration, 3600))  # Schutzkappe 1h

            elif st == "delay":
                sleep(step["delay_time"] / 1000.0)

        print("Programm abgeschlossen.")

    except KeyboardInterrupt:
        print("Abbruch durch Benutzer.")
    finally:
        # Logger stoppen
        if data_logger is not None:
            data_logger.do_run = False
            sleep(0.15)
        # Treiber sauber schließen
        if driver is not None:
            driver.close()

if __name__ == "__main__":
    csv.field_size_limit(sys.maxsize)
    run_server()