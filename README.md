# Vader Device Control

This repository provides tools to operate the **Vader device**, a system that controls gas flows, pressures, and valves in a programmable way.  
It supports two usage modes:

1. **Command Line Tool (CLI)** – run JSON-defined programs and log pressure data.  
2. **TANGO Device Server (pyTANGO)** – integrate into a TANGO control system with full device attributes and commands.  

---

## Program JSON Format

Device operation is defined by a list of steps stored in a JSON file.  
Each step is an object with a `type` and additional parameters depending on the action.

### Supported Step Types

- **Gas Flow Control**
  ```json
  {"type": "set_co2", "flow_nl_min": 5.0}
  {"type": "set_n2", "flow_nl_min": 3.0}
  {"type": "set_butan", "flow_nl_min": 1.5}
  ```
  → Sets the flow rate of CO₂, N₂, or Butan in NL/min.

- **Pressure Control**
  ```json
  {"type": "Normal", "target_pressure_kPa": 200}
  {"type": "Ramp", "end_pressure_kPa": 500, "ramp_speed_kPa_sec": 100}
  ```
  → Sets target pressure directly or ramps pressure at a defined rate.

- **Delays**
  ```json
  {"type": "delay", "delay_time_ms": 10000}
  ```
  → Waits the given time in milliseconds before continuing.

- **High Pressure Volume**
  ```json
  {"type": "eject_high_pressure_volume", "value": true}
  ```
  → Opens/closes the eject valve of the high-pressure volume.

- **Manual Valves (PWM Control)**
  ```json
  {"type": "open_vp1", "PWM": 255}
  {"type": "open_vp2", "PWM": 128}
  ```
  → Sets PWM values (0–255) for manual valves controlling pressure increase/decrease.

---

### Example Program (from this repository)

```json
[
  {"type": "set_co2",  "flow_nl_min": 5.00},
  {"type": "delay",  "delay_time_ms": 10000},
  {"type": "Normal", "target_pressure_kPa": 0},
  {"type": "delay",  "delay_time_ms": 10000},
  {"type": "Ramp",   "end_pressure_kPa": 0, "ramp_speed_kP_sec": 100},
  {"type": "Normal", "target_pressure_kPa": 0},
  {"type": "set_co2",  "flow_nl_min": 0},
  {"type": "Normal", "target_pressure_kPa": 0},
  {"type": "eject_high_pressure_volume", "value": true},
  {"type": "delay",  "delay_time_ms": 10000},
  {"type": "eject_high_pressure_volume", "value": false},
  {"type": "delay",  "delay_time_ms": 300},
  {"type": "set_butan",  "flow_nl_min": 5},
  {"type": "delay",  "delay_time_ms": 10000},
  {"type": "Normal", "target_pressure_kPa": 200},
  {"type": "delay",  "delay_time_ms": 5000},
  {"type": "Normal", "target_pressure_kPa": 0},
  {"type": "set_n2",  "flow_nl_min": 0},
  {"type": "delay",  "delay_time_ms": 5000},
  {"type": "open_vp1",  "PWM": 255},
  {"type": "open_vp2",  "PWM": 255},
  {"type": "delay",  "delay_time_ms": 5000},
  {"type": "delay",  "delay_time_ms": 5000},
  {"type": "delay",  "delay_time_ms": 5000}
]
```

---

## Command Line Tool

The CLI allows you to execute JSON programs while logging device data.

### Usage

```bash
python VaderComandLineTool.py   --data_path ./data   --program_path ./program.json   --mini1-port /dev/ttyACM0   --mini2-port /dev/ttyACM2   --maxi-port /dev/ttyACM1   --sample-hz 10
```

### Options

- `--data_path` : Path to output CSV files (pressure log + event log).  
- `--program_path` : Path to JSON program file.  
- `--mini1-port`, `--mini2-port`, `--maxi-port` : Serial ports for hardware units.  
- `--sample-hz` : Logging rate for MINI1 pressure (Hz).  

### Output

- **Pressure data** (CSV):  
  ```
  timestamp_ms,pressure_kpa
  1712345678901,101.325
  ```
- **Event log** (CSV):  
  ```
  step_index,timestamp_iso,timestamp_ms,json_step
  1,2025-09-17T12:34:56.789Z,1712345678901,"{"type":"set_co2","flow_nl_min":5.0}"
  ```

---

## pyTANGO Device Server

The repository also includes a TANGO Device Server for integration into control systems.  
This exposes device state, attributes, and commands.

### Start the Server

```bash
python Vader.py
```

### Key Attributes

- **Flows**:  
  - `setpoint_flow_Butan_nl_per_min`  
  - `setpoint_flow_CO2_nl_per_min`  
  - `setpoint_flow_N2_nl_per_min`

- **Pressure**:  
  - `setpoint_pressure_kPa`  
  - `pressure_kPa` (read-only from MINI1)  

- **Valves & Volume**:  
  - `large_high_pressure_volume` (V1 & V2)  
  - `eject_high_pressure` (V3)  
  - `valve_increase_pressure` (PWM → VP1)  
  - `valve_reduce_pressure` (PWM → VP2)  

### Key Commands

- **Run a Program**  
  ```
  RunProgram("/path/to/program.json")
  ```
- **Stop Current Program**  
  ```
  StopProgram()
  ```
- **Export MAXI Log**  
  ```
  ExportMaxiLog("/tmp/maxi_log.jsonl")
  ```

---

## Logging

- **MINI1 Pressure** is logged continuously.  
- **Event Logs** record each program step with timestamps.  
- **MAXI Status** (I/O and ADC signals) can be logged to file in JSONL format.  

---

## Safety Notes

- Ensure that no air is in the system, if you work with flammable gases! 
- Ensure Input Pressure is lower than 10 bar.