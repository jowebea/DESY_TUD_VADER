#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vader_cli.py — Click-Interface zur Bedienung des VaderDeviceDriver

Spiegelung der Tango-Funktionalität:
- Attribute/Reads: Drücke (Mini1, P11, P31, P20/P21), Leck, Ventile, Flüsse
- Writes/Kommandos: Setpoint, Ventile schalten, Flüsse setzen, Vakuum/Storage, Purge, CSV-Programm
- Logging: kontinuierliches CSV-Logging (Periodendefault 5 ms)
"""

import os
import csv
import json
import time
import click
from contextlib import contextmanager

# >>> Import anpassen, falls Driver in separatem Modul:
# from vader_driver import VaderDeviceDriver
from __main__ import VaderDeviceDriver  # wenn Driver & CLI in einer Datei sind

DEFAULT_MINI1 = "/dev/cu.usbmodem1133101"
DEFAULT_MINI2 = "/dev/cu.usbmodem1133301"
DEFAULT_MAXI  = "/dev/cu.usbmodem1133201"

# ----------------- Infrastruktur -----------------

@contextmanager
def driver_ctx(mini1, mini2, maxi, max_samples):
    drv = VaderDeviceDriver(mini1, mini2, maxi, max_samples)
    try:
        yield drv
    finally:
        drv.close()

def print_status(st: dict, pretty=True):
    if not pretty:
        click.echo(json.dumps(st, indent=2, default=str))
        return
    m1, m2, mx = st.get("mini1", {}), st.get("mini2", {}), st.get("maxi", {})
    click.echo("=== Probenvolumen (MINI1) ===")
    click.echo(f"  P (kombiniert): {m1.get('latest_combined_pressure')} kPa")
    click.echo(f"  P2.1 (hoch):    {m2.get('p21_kpa')} kPa")
    click.echo(f"  P2.0 (niedrig): {m2.get('p20_kpa')} kPa")
    click.echo("=== High/Low Pressure ===")
    click.echo(f"  P1.1 (HP): {mx.get('p11')} kPa")
    click.echo(f"  P3.1 (LP): {mx.get('p31')} kPa")
    click.echo("=== MINI2-Regler ===")
    click.echo(f"  Setpoint: {m2.get('setpoint_kpa')} kPa  |  Ist: {m2.get('pressure_kpa')} kPa")
    click.echo(f"  PWM1(vp1): {m2.get('pwm1')}  PWM2(vp2): {m2.get('pwm2')}  Kp: {m2.get('kp')}")
    click.echo("=== MAXI Aktoren ===")
    for k in ("v1_power","v2_power","v3_power","v4_power","fan_in_power","fan_out_power","vs3_power","heater_pwm"):
        if k in mx:
            click.echo(f"  {k}: {mx[k]}")
    click.echo("=== MFCs ===")
    for g in ("mfc_butan","mfc_n2","mfc_co2"):
        if g in mx:
            gg = mx[g]
            click.echo(f"  {g}: flow={gg.get('flow')}  setpoint={gg.get('setpoint')}  temp={gg.get('temperature')}")
    click.echo(f"Leak: {mx.get('leak_sensor')}")

# ----------------- Root-Gruppe -----------------

@click.group(context_settings=dict(help_option_names=["-h", "--help"]))
@click.option("--mini1-port", default=DEFAULT_MINI1, show_default=True, envvar="VADER_MINI1_PORT")
@click.option("--mini2-port", default=DEFAULT_MINI2, show_default=True, envvar="VADER_MINI2_PORT")
@click.option("--maxi-port",  default=DEFAULT_MAXI,  show_default=True, envvar="VADER_MAXI_PORT")
@click.option("--mini1-max-samples", type=int, default=300_000, show_default=True)
@click.pass_context
def cli(ctx, mini1_port, mini2_port, maxi_port, mini1_max_samples):
    """CLI zur Bedienung des VADER-Geräts (entspricht der Tango-Oberfläche)."""
    ctx.ensure_object(dict)
    ctx.obj.update(dict(mini1=mini1_port, mini2=mini2_port, maxi=maxi_port, max_samples=mini1_max_samples))

# ----------------- Status & Drücke -----------------

@cli.command("status")
@click.option("-n", "--mini1-last-n", type=int, default=1, show_default=True)
@click.option("--raw/--pretty", default=False, show_default=True)
@click.pass_context
def status_cmd(ctx, mini1_last_n, raw):
    """Gesamtstatus anzeigen (Drücke, Aktoren, Flüsse)."""
    with driver_ctx(**ctx.obj) as drv:
        st = drv.get_all_status(mini1_last_n=mini1_last_n)
        print_status(st, pretty=not raw)

@cli.group("pressure")
def pressure_grp():
    """Proben-/HP-/LP-Drücke lesen & Sollwert setzen."""

@pressure_grp.command("read")
@click.pass_context
def pressure_read(ctx):
    """P_proben (kombiniert), P2.1, P2.0, P1.1, P3.1 lesen."""
    with driver_ctx(**ctx.obj) as drv:
        st = drv.get_all_status(mini1_last_n=1)
        m1, m2, mx = st.get("mini1", {}), st.get("mini2", {}), st.get("maxi", {})
        click.echo(json.dumps({
            "sample_pressure_kpa": m1.get("latest_combined_pressure"),
            "P21_kpa": m2.get("p21_kpa"),
            "P20_kpa": m2.get("p20_kpa"),
            "P11_kpa": mx.get("p11"),
            "P31_kpa": mx.get("p31"),
        }, indent=2, default=str))

@pressure_grp.command("setpoint")
@click.argument("kpa", type=float)
@click.pass_context
def pressure_setpoint(ctx, kpa):
    """Druck-Sollwert (kPa) für Probenvolumen setzen."""
    with driver_ctx(**ctx.obj) as drv:
        drv.setpoint_pressure(kpa)
        click.echo(f"Setpoint -> {kpa} kPa")

# ----------------- Ventile & Aktoren -----------------

@cli.group("valve")
def valve_grp():
    """Ventile/Aktoren schalten (V1..V4, fan_in/out, vs3, heater)."""

@valve_grp.command("set")
@click.argument("name", type=click.Choice(["v1","v2","v3","v4","fan_in","fan_out","vs3"], case_sensitive=False))
@click.argument("state", type=click.Choice(["on","off"], case_sensitive=False))
@click.pass_context
def valve_set(ctx, name, state):
    """Ventil/Aktor schalten."""
    name = name.lower()
    on = (state.lower() == "on")
    mapping = {"v1":1,"v2":2,"v3":3,"v4":4,"fan_in":8,"fan_out":9,"vs3":12}
    with driver_ctx(**ctx.obj) as drv:
        if name.startswith("v"):
            drv.maxi.set_valve(mapping[name], on)
        else:
            drv.maxi.set_component(mapping[name], 1 if on else 0)
        click.echo(f"{name} -> {state}")

@valve_grp.command("heater")
@click.argument("pwm", type=int)
@click.pass_context
def heater_pwm(ctx, pwm):
    """Heater PWM (Adresse 11) setzen."""
    with driver_ctx(**ctx.obj) as drv:
        drv.maxi.set_component(11, int(pwm))
        click.echo(f"Heater PWM -> {pwm}")

# ----------------- MFC-Flüsse -----------------

@cli.group("flow")
def flow_grp():
    """MFC-Flüsse setzen/lesen (nl/min)."""

@flow_grp.command("set")
@click.argument("gas", type=click.Choice(["butan","n2","co2"], case_sensitive=False))
@click.argument("value", type=float)
@click.pass_context
def flow_set(ctx, gas, value):
    """Flow eines MFC in nl/min setzen (direkt)."""
    gas = gas.lower()
    with driver_ctx(**ctx.obj) as drv:
        if gas == "butan": drv.maxi.set_flow_butan(value)
        elif gas == "n2":  drv.maxi.set_flow_n2(value)
        else:              drv.maxi.set_flow_co2(value)
        click.echo(f"Flow {gas} -> {value} nl/min")

@flow_grp.command("use-gas")
@click.argument("n2", type=float)
@click.argument("co2", type=float)
@click.argument("butan", type=float)
@click.pass_context
def flow_usegas(ctx, n2, co2, butan):
    """Gasverwendung 0..100 (intern /10 -> nl/min)."""
    with driver_ctx(**ctx.obj) as drv:
        drv.use_gas(n2, co2, butan)
        click.echo(f"use-gas: N2={n2} CO2={co2} Butan={butan}")

# ----------------- Prozess-Kommandos -----------------

@cli.command("storage-volume")
@click.argument("state", type=click.Choice(["open","close"], case_sensitive=False))
@click.pass_context
def storage_volume(ctx, state):
    """V1+V2 gemeinsam öffnen/schließen (Probe <-> High Pressure Volume)."""
    with driver_ctx(**ctx.obj) as drv:
        drv.storage_volume(state.lower() == "open")
        click.echo(f"storage-volume -> {state}")

@cli.command("activate-vac")
@click.argument("state", type=click.Choice(["on","off"], case_sensitive=False))
@click.pass_context
def activate_vac(ctx, state):
    """Vakuumpumpe (V4) ein/aus."""
    with driver_ctx(**ctx.obj) as drv:
        drv.activate_vac(state.lower() == "on")
        click.echo(f"activate-vac -> {state}")

@cli.command("vac-all")
@click.option("--vac-kpa", type=float, required=True, help="Ziel-Vakuum in kPa im Probenvolumen")
@click.option("--timeout", "timeout_s", type=int, default=120, show_default=True)
@click.pass_context
def vac_all(ctx, vac_kpa, timeout_s):
    """Evakuieren bis Druck erreicht oder Timeout."""
    with driver_ctx(**ctx.obj) as drv:
        click.echo(f"vac-all: Ziel={vac_kpa} kPa, Timeout={timeout_s}s")
        drv.vac_all(vac_kpa, timeout_s)
        click.echo("vac-all: fertig")

@cli.command("purge-hp")
@click.pass_context
def purge_hp(ctx):
    """Gaswechsel im High Pressure Volume: MFCs zu, V1/V2 zu, V3 3 s offen."""
    with driver_ctx(**ctx.obj) as drv:
        drv.maxi.set_flow_butan(0.0); drv.maxi.set_flow_n2(0.0); drv.maxi.set_flow_co2(0.0)
        drv.maxi.set_valve(1, False); drv.maxi.set_valve(2, False)
        drv.maxi.set_valve(3, True); time.sleep(3.0); drv.maxi.set_valve(3, False)
        click.echo("purge-hp: abgeschlossen")

# ----------------- CSV-Programm -----------------

@cli.command("run-program")
@click.argument("csv_path", type=click.Path(exists=True, dir_okay=False))
@click.pass_context
def run_program(ctx, csv_path):
    """
    CSV abfahren: Haltezeit_ms,Druck_kPa,Butan_nlmin,N2_nlmin,CO2_nlmin
    (läuft synchron im CLI; für langen Betrieb lieber in Screen/tmux).
    """
    with driver_ctx(**ctx.obj) as drv, open(csv_path, newline="") as f:
        rdr = csv.DictReader(f)
        cols = {c.lower(): c for c in rdr.fieldnames or []}
        req = ["haltezeit_ms","druck_kpa","butan_nlmin","n2_nlmin","co2_nlmin"]
        for r in req:
            if r not in cols:
                raise click.ClickException(f"CSV fehlt Spalte: {r}")
        click.echo(f"Starte Programm: {csv_path}")
        for row in rdr:
            hold_ms = float(row[cols["haltezeit_ms"]])
            p_kpa   = float(row[cols["druck_kpa"]])
            butan   = float(row[cols["butan_nlmin"]])
            n2      = float(row[cols["n2_nlmin"]])
            co2     = float(row[cols["co2_nlmin"]])
            drv.setpoint_pressure(p_kpa)
            drv.maxi.set_flow_butan(butan)
            drv.maxi.set_flow_n2(n2)
            drv.maxi.set_flow_co2(co2)
            click.echo(f"  set: P={p_kpa} kPa | Bu={butan} N2={n2} CO2={co2} (halte {hold_ms} ms)")
            t_end = time.perf_counter() + hold_ms/1000.0
            while time.perf_counter() < t_end:
                time.sleep(0.01)
        click.echo("Programm beendet.")

# ----------------- Logging -----------------

@cli.group("log")
def log_grp():
    """Kontinuierliches CSV-Logging lokaler Messdaten."""

@log_grp.command("start")
@click.option("--dir", "log_dir", type=click.Path(file_okay=False), default="/tmp", show_default=True)
@click.option("--period-ms", type=int, default=5, show_default=True)
@click.option("--fields", default="", help="Komma-liste zusätzlicher Felder aus status.json-Pfaden (z.B. maxi.mfc_co2.flow)")
@click.pass_context
def log_start(ctx, log_dir, period_ms, fields):
    """Startet CSV-Logger (läuft im Vordergrund bis CTRL+C)."""
    os.makedirs(log_dir, exist_ok=True)
    fn = os.path.join(log_dir, time.strftime("vader_%Y%m%d_%H%M%S.csv"))
    extra = [p.strip() for p in fields.split(",") if p.strip()]
    period_s = max(1, int(period_ms)) / 1000.0

    def get_path(d, path):
        cur = d
        for p in path.split("."):
            if isinstance(cur, dict) and p in cur:
                cur = cur[p]
            else:
                return None
        return cur

    with driver_ctx(**ctx.obj) as drv, open(fn, "w", newline="") as f:
        base_fields = [
            "ts","P_sample","P21","P20","P11","P31",
            "Setpoint","FlowBu_set","FlowN2_set","FlowCO2_set",
            "FlowBu","FlowN2","FlowCO2","Leak"
        ]
        all_fields = base_fields + extra
        w = csv.DictWriter(f, fieldnames=all_fields); w.writeheader()
        click.echo(f"Logging -> {fn} (alle {period_ms} ms). Abbruch: CTRL+C")
        next_t = time.perf_counter()
        try:
            while True:
                st = drv.get_all_status(mini1_last_n=1)
                mx, m2, m1 = st.get("maxi",{}), st.get("mini2",{}), st.get("mini1",{})
                row = {
                    "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "P_sample": m1.get("latest_combined_pressure"),
                    "P21": m2.get("p21_kpa"),
                    "P20": m2.get("p20_kpa"),
                    "P11": mx.get("p11"),
                    "P31": mx.get("p31"),
                    "Setpoint": m2.get("setpoint_kpa"),
                    "FlowBu_set": (mx.get("mfc_butan",{}) or {}).get("setpoint"),
                    "FlowN2_set": (mx.get("mfc_n2",{}) or {}).get("setpoint"),
                    "FlowCO2_set": (mx.get("mfc_co2",{}) or {}).get("setpoint"),
                    "FlowBu": (mx.get("mfc_butan",{}) or {}).get("flow"),
                    "FlowN2": (mx.get("mfc_n2",{}) or {}).get("flow"),
                    "FlowCO2": (mx.get("mfc_co2",{}) or {}).get("flow"),
                    "Leak": mx.get("leak_sensor"),
                }
                for p in extra:
                    row[p] = get_path(st, p)
                w.writerow(row)

                next_t += period_s
                sleep = next_t - time.perf_counter()
                if sleep > 0: time.sleep(sleep)
        except KeyboardInterrupt:
            click.echo("\nLogging beendet.")

# ----------------- Entry -----------------

if __name__ == "__main__":
    cli(obj={})