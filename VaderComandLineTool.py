#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import csv
import signal
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Any, List, Optional

import click

from VaderDeviceDriver import VaderDeviceDriver  

# ---------------------------------------
# Zeit-Helfer
# ---------------------------------------
def iso_utc(ts: float) -> str:
    """UTC-ISO8601 mit 'Z'-Suffix, menschenlesbar und timezone-aware."""
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def ts_ms(ts: float) -> int:
    """Millisekunden seit Unix Epoch."""
    return int(ts * 1000)


# ---------------------------------------
# Pfad-Helfer
# ---------------------------------------
def ensure_unique_path(path: Path) -> Path:
    if not path.exists():
        return path
    parent = path.parent
    stem = path.stem
    suffix = path.suffix
    i = 1
    while True:
        candidate = parent / f"{stem}_{i:03d}{suffix}"
        if not candidate.exists():
            return candidate
        i += 1


def resolve_csv_path(data_path: Path) -> Path:
    if data_path.suffix.lower() == ".csv":
        data_path.parent.mkdir(parents=True, exist_ok=True)
        return ensure_unique_path(data_path)
    data_path.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw = data_path.joinpath(f"mini1_log_{ts}.csv")
    return ensure_unique_path(raw)


def derive_event_log_path(from_data_csv: Path) -> Path:
    parent = from_data_csv.parent
    stem = from_data_csv.stem
    event_csv = parent / f"{stem}_log.csv"
    return ensure_unique_path(event_csv)


# ---------------------------------------
# CSV-Logger für MINI1 (nur Druckdaten)
# ---------------------------------------
class Mini1Logger:
    """
    Loggt kontinuierlich MINI1-Daten in eine CSV.
    Spalten: timestamp_ms, pressure_kpa
    """
    def __init__(self, drv: VaderDeviceDriver, csv_path: Path, sample_hz: float = 10.0):
        self.drv = drv
        self.csv_path = csv_path
        self.sample_dt = max(0.001, 1.0 / float(sample_hz))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fh = None
        self._writer: Optional[csv.writer] = None

    def start(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        # Nur ms + Druckwert
        self._writer.writerow(["timestamp_ms", "pressure_kpa"])
        self._fh.flush()
        self._thread = threading.Thread(target=self._run, name="Mini1Logger", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ts = time.time()
            ms = ts_ms(ts)
            p = self.drv.mini1.pressure
            self._writer.writerow([ms, "" if p is None else f"{float(p):.6f}"])
            self._fh.flush()

            end = ts + self.sample_dt
            while time.time() < end:
                if self._stop.is_set():
                    break
                time.sleep(min(0.01, self.sample_dt))

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._fh:
            self._fh.flush()
            self._fh.close()


# ---------------------------------------
# Event-Logger (Doppelte Zeitangabe)
# ---------------------------------------
class EventLogger:
    """
    Schreibt je Programmschritt:
    step_index, timestamp_iso, timestamp_ms, json_step
    """
    def __init__(self, csv_path: Path):
        self.csv_path = csv_path
        self._fh = None
        self._writer: Optional[csv.writer] = None

    def __enter__(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(["step_index", "timestamp_iso", "timestamp_ms", "json_step"])
        self._fh.flush()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._fh:
            self._fh.flush()
            self._fh.close()

    def log_step(self, idx: int, step: Dict[str, Any]):
        ts = time.time()
        iso = iso_utc(ts)
        ms = ts_ms(ts)
        raw = json.dumps(step, ensure_ascii=False, separators=(",", ":"))
        self._writer.writerow([idx, iso, ms, raw])
        self._fh.flush()


# ---------------------------------------
# Programmausführung
# ---------------------------------------
def _sleep_ms(ms: int, cancel_event: threading.Event):
    remaining = max(0.0, ms / 1000.0)
    step = 0.05
    while remaining > 0 and not cancel_event.is_set():
        t = min(step, remaining)
        time.sleep(t)
        remaining -= t


def execute_step(step: Dict[str, Any], drv: VaderDeviceDriver, cancel_event: threading.Event):
    t = (step.get("type") or "").lower()

    if t == "delay":
        _sleep_ms(int(step.get("delay_time_ms", 0)), cancel_event)
        return
    if t == "set_co2":
        drv.set_flow_co2(float(step["flow_nl_min"]))
        return
    if t == "set_n2":
        drv.set_flow_n2(float(step["flow_nl_min"]))
        return
    if t == "set_butan":
        drv.set_flow_butan(float(step["flow_nl_min"]))
        return
    if t == "normal":
        drv.setpoint_pressure(float(step["target_pressure_kPa"]))
        return
    if t == "ramp":
        speed = float(step.get("ramp_speed_kP_sec", step.get("ramp_speed_kpa_sec", 0)))
        end_kpa = float(step["end_pressure_kPa"])
        drv.set_ramp(speed_kpa_s=speed, end_kpa=end_kpa)
        return
    if t == "eject_high_pressure_volume":
        val = bool(step.get("value", False))
        drv.storage_volume(val)
        return
    if t == "open_vp1":
        pwm = int(step["PWM"])
        drv.set_state_manual()
        drv.set_manual_PWM_in(pwm)
        return
    if t == "open_vp2":
        pwm = int(step["PWM"])
        drv.set_state_manual()
        drv.set_manual_PWM_out(pwm)
        return

    click.echo(f"[WARN] Unbekannter Schritt-Typ: {step.get('type')}. Schritt wird übersprungen.", err=True)


# ---------------------------------------
# CLI
# ---------------------------------------
@click.command(context_settings=dict(help_option_names=["-h", "--help"]))
@click.option("--data_path", type=click.Path(path_type=Path), required=True,
              help="Pfad (Datei .csv oder Verzeichnis) für MINI1-Daten. Existiert die Datei, wird '_001' usw. angehängt.")
@click.option("--program_path", type=click.Path(path_type=Path, exists=True), required=True,
              help="Pfad zur JSON-Programmdatei (Liste von Schritten).")
@click.option("--mini1-port", envvar="VADER_MINI1_PORT", default="/dev/ttyACM0", show_default=True,
              help="Serieller Port für MINI1.")
@click.option("--mini2-port", envvar="VADER_MINI2_PORT", default="/dev/ttyACM2", show_default=True,
              help="Serieller Port für MINI2.")
@click.option("--maxi-port", envvar="VADER_MAXI_PORT", default="/dev/ttyACM1", show_default=True,
              help="Serieller Port für MAXI.")
@click.option("--sample-hz", type=float, default=10.0, show_default=True,
              help="Abtastrate für MINI1-Logging in Hz.")
def main(data_path: Path, program_path: Path, mini1_port: str, mini2_port: str, maxi_port: str, sample_hz: float):
    try:
        program: List[Dict[str, Any]] = json.loads(program_path.read_text(encoding="utf-8"))
        if not isinstance(program, list):
            raise ValueError("Programm muss eine Liste von Schritten sein.")
    except Exception as e:
        raise click.ClickException(f"Programmdatei ungültig: {e}")

    data_csv = resolve_csv_path(data_path)
    event_csv = derive_event_log_path(data_csv)

    click.echo(f"[INFO] MINI1-Logging: {data_csv}")
    click.echo(f"[INFO] Event-Log:     {event_csv}")

    cancel_event = threading.Event()

    def _sig_handler(signum, frame):
        click.echo("\n[INFO] Abbruch angefordert – fahre sauber herunter ...")
        cancel_event.set()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        with VaderDeviceDriver(mini1_port, mini2_port, maxi_port) as drv:
            mini1_logger = Mini1Logger(drv, data_csv, sample_hz=sample_hz)
            mini1_logger.start()

            with EventLogger(event_csv) as elog:
                click.echo(f"[INFO] Programm mit {len(program)} Schritt(en) starten ...")

                for idx, step in enumerate(program, 1):
                    if cancel_event.is_set():
                        click.echo("[INFO] Abbruch vor Schritt-Ausführung.")
                        break

                    elog.log_step(idx, step)
                    click.echo(f"[STEP {idx}/{len(program)}] {step}")

                    try:
                        execute_step(step, drv, cancel_event)
                    except KeyError as ke:
                        click.echo(f"[ERROR] Fehlende Felder im Schritt {idx}: {ke}", err=True)
                    except Exception as ex:
                        click.echo(f"[ERROR] Ausführung in Schritt {idx} fehlgeschlagen: {ex}", err=True)
                        break

                click.echo("[INFO] Programm beendet.")
                time.sleep(0.2)

            mini1_logger.stop()

    except click.ClickException:
        raise
    except Exception as e:
        raise click.ClickException(f"Laufzeitfehler: {e}")


if __name__ == "__main__":
    main()