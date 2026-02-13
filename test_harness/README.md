# Step/Dir Signal Analyzer — Test Harness

A standalone ESP32-S3 firmware that monitors and analyzes step/dir signals from the 6DOF Stewart Platform controller board. Two-stage test approach:

- **Stage 1**: ESP-to-ESP direct jumper wires — validates firmware logic (no extra hardware)
- **Stage 2**: SN75175N receiver daughter board — validates full differential signal chain

See [hardware design doc](../docs/hardware/step_dir_analyzer.md) for detailed wiring, BOM, and test procedures.

## Stage 1 Quick Start (ESP-to-ESP)

Just 13 jumper wires between two ESP32-S3 DevKitC boards + shared GND.

| Controller GPIO | Signal | Analyzer GPIO |
|----------------|--------|---------------|
| GPIO4 | STEP_0 | GPIO4 |
| GPIO5 | STEP_1 | GPIO5 |
| GPIO6 | STEP_2 | GPIO6 |
| GPIO7 | STEP_3 | GPIO7 |
| GPIO8 | STEP_4 | GPIO8 |
| GPIO9 | STEP_5 | GPIO9 |
| GPIO10 | DIR_0 | GPIO10 |
| GPIO11 | DIR_1 | GPIO11 |
| GPIO12 | DIR_2 | GPIO12 |
| GPIO13 | DIR_3 | GPIO13 |
| GPIO14 | DIR_4 | GPIO14 |
| GPIO17 | DIR_5 | GPIO17 |
| GND | — | GND |

> For single-motor testing, only 3 wires needed: STEP_0, DIR_0, GND.

## Build & Flash

```bash
cd test_harness
idf.py set-target esp32s3
idf.py build
idf.py -p COMX flash monitor
```

Replace `COMX` with the analyzer ESP32's COM port (different from the controller).

## Commands

Send via serial monitor (append `X` or newline as terminator):

| Command | Description |
|---------|-------------|
| `STATUS` | JSON dump of all motor stats |
| `SUMMARY` | Human-readable table |
| `RESET` | Clear all counters and timing stats |
| `STREAM:1` | Enable continuous JSON output at 2 Hz |
| `STREAM:0` | Stop streaming |
| `MONITOR:N` | Focus output on motor N (0–5), `-1` for all |
| `HELP` | List commands |

## JSON Output

```json
{
  "t_ms": 12345,
  "motors": [
    {
      "id": 0,
      "pos": 1523,
      "steps": 3046,
      "rate": 48230.5,
      "dir": 1,
      "min_us": 4,
      "max_us": 52,
      "avg_us": 6,
      "dir_chg": 12,
      "idle_ms": 0
    }
  ]
}
```

Event notifications (when streaming):
```json
{"event":"active","motor":0}
{"event":"idle","motor":0,"pos":1523,"steps":3046}
```

## What It Measures

- **Position**: Net step count (up/down based on DIR level)
- **Total steps**: Absolute count regardless of direction
- **Step rate**: Pulses per second (windowed over 500ms)
- **Timing**: Min/max/avg microseconds between step pulses
- **Direction changes**: Number of DIR reversals
- **Idle detection**: Reports when a motor starts/stops stepping
- **Ghost pulse detection**: Any steps appearing when controller is idle

## Stage 2

Adds SN75175N receiver daughter board + voltage dividers. Same firmware, same pin map — just validates the differential signal chain. See the [hardware design doc](../docs/hardware/step_dir_analyzer.md#stage-2--full-pcb-test-with-sn75175n-receivers) for BOM, wiring diagrams, and additional test procedures.
