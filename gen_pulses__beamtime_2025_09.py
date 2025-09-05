import csv

def generate_pressure_csv(filename="pressure_program.csv"):
    """
    Erzeugt ein CSV mit Zeit (ms) und Druck (kPa).
    
    - 9 Programme mit steigender Geschwindigkeit (10...90 kPa/s)
    - Druck integer (1 kPa Schritte)
    - Zeit float (ms)
    """
    rows = [("time [ms]", "target pressure [kPa]")]
    
    for i in range(1, 10):
        speed = i * 10  # kPa/s
        dt_per_kPa = 1000.0 / speed  # ms pro 1 kPa Schritt
        
        # 1) Hold 120s at 0 kPa
        hold_time = 120 * 1000
        rows.append((int(hold_time), 0))
        
        # 2) Ramp up to 120 kPa
        for p in range(1, 121):  # von 1 bis 120 kPa
            rows.append((int(dt_per_kPa), p))
        
        # 3) Hold at 120 kPa for 60s
        hold_time = 60 * 1000
        rows.append((int(hold_time), 120))
        
        # 4) Ramp down to 0 kPa
        for p in range(119, -1, -1):  # von 119 zurück bis 0
            hold_time = dt_per_kPa
            rows.append((int(dt_per_kPa), p))

    # Schreiben ins CSV
    with open(filename, mode="w", newline="") as f:
        writer = csv.writer(f, delimiter=";")
        writer.writerows(rows)
    
    print(f"CSV '{filename}' erfolgreich erzeugt!")


if __name__ == "__main__":
    generate_pressure_csv()