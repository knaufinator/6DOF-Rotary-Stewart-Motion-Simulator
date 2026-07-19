# `mini/` — Mini-6DOF Firmware Variant

Desktop-scale **mini** variant of the Stewart-platform firmware: an **ESP32** driving
**6 hobby servos via LEDC PWM**. This is one of two firmware variants in this repo — see
[`../FIRMWARE.md`](../FIRMWARE.md) for the variant map. Both variants share the
single [`stewart-core`](../stewart-core) submodule at the repo root (IK / AxisScaling /
MotionCueing — single source of truth).

| | This variant (`mini/`) | `Controller/` (normal) |
|---|---|---|
| **Role** | desktop / dev / education | full-scale motion sim |
| **MCU** | ESP32 (DevKitC) | ESP32 (PCBv1) / ESP32-S3 (PCBv2) |
| **Actuator** | LEDC PWM → hobby servos | MCPWM step/dir → AASD-15A AC servos |
| **Servo rate** | analog 50 Hz (runtime-switchable to digital 250 Hz via `SERVO:RATE`) | 250 Hz cue loop |
| **Wireless** | BLE (Android phone-as-controller) | W5500 Ethernet / WiFi |
| **Workspace** | ~30 mm | ~350 mm |

> This tree was migrated from the standalone **Mini-6DOF** repo (its `Controller/`, Phase-3
> firmware) into the main app repo as part of the firmware-unification effort. The mini's former
> `components/stewart-core` submodule was dropped in favor of the root submodule.

## Build

Requires ESP-IDF v5.5 (matches the rest of the repo).

```bash
# from repo root, one-time: fetch the shared core
git submodule update --init --recursive

cd mini
idf.py set-target esp32   # first time only
idf.py build
idf.py -p COM6 flash monitor
```

`mini/CMakeLists.txt` points at the root submodule via
`set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../stewart-core")` (mirrors
`Controller/CMakeLists.txt`); `main/CMakeLists.txt` resolves it through `PRIV_REQUIRES stewart-core`.

On this machine, the headless build is driven by the **`6dof-esp-build`** skill
(`idf_build.ps1 -ProjDir mini -Clean`), which encodes the ESP-IDF v5.5 env setup that avoids the
MSYS/venv/stderr traps. CI builds it the same way (see the "Build Mini Firmware" job in
`.github/workflows/ci.yml`).

## Project structure

```
mini/
├── main/
│   ├── main.cpp              # app_main, CueTask, SOURCE modes, command API, LEDC servo output
│   ├── CobsTransport.cpp     # COBS-framed serial transport (CH_DATA18 / CH_DATA_RAW / CH_CMD)
│   ├── BleTransport.cpp      # BLE GATT server (motion + accel characteristics)
│   ├── helpers.cpp           # mapfloat / rate-limit utilities
│   ├── laps123_moderate.m6p  # baked 50 Hz motion-cued laps 1-3 (EMBED_FILES) for boot demo
│   └── CMakeLists.txt        # component build (PRIV_REQUIRES stewart-core)
├── include/                  # BleTransport.h, CobsTransport.h, cobs.h, helpers.h, version.h, debug_uart.h
├── partitions.csv            # 4 MB no-OTA table: 2 MB app + raw seq partition
├── sdkconfig.defaults        # ESP32 config
└── CMakeLists.txt            # project-level, wires EXTRA_COMPONENT_DIRS → ../stewart-core
```

IK / AxisScaling / MotionCueing are **not** in this tree — they come from `../stewart-core`.
The only mini-specific code is the LEDC servo backend + BLE transport + pin map.

## Hardware

- **MCU**: ESP32 DevKitC (original ESP32 — not ESP32-S3)
- **Servos**: 6 × hobby servos (SG90/MG996R class; ordering Feetech/ANNIMOS DS3218 for the digital upgrade)
- **PWM**: LEDC at 50 Hz (analog default), 16-bit resolution
- **Pulse range**: 800–2200 µs (center 1500 µs)
- **Power**: dedicated **5 V 8 A** servo rail (do not run servos off USB)
- **Servo enable**: GPIO 27 controls the servo-rail power relay/MOSFET

### Servo pinout

| Servo | GPIO | LEDC ch | Inverted |
|-------|------|---------|----------|
| 0 | 15 | 0 | Yes |
| 1 | 14 | 1 | No |
| 2 | 4  | 2 | Yes |
| 3 | 32 | 3 | No |
| 4 | 33 | 4 | Yes |
| 5 | 5  | 5 | No |

| Function | GPIO |
|----------|------|
| Servo enable | 27 |
| E-Stop | 22 |

Inverted servos are mounted mirrored; the firmware applies the sign flip automatically.

### Platform geometry (mm)

| Param | Value | Description |
|-------|-------|-------------|
| RD | 15.75 | Base radius |
| PD | 16.00 | Platform radius |
| L1 | 7.25 | Servo horn length |
| L2 | 28.50 | Connecting rod length |
| H | 25.517 | Neutral platform height |
| θ_r | 10° | Base rotation angle |
| θ_p | 30° | Platform rotation angle |

3D-printed mounts + seat rails are in [`../cad/mini/`](../cad/mini).

## Servo rate profile (analog / digital)

The firmware supports **both** an analog 50 Hz carrier (SG90/MG996R class — default) and a digital
250 Hz carrier (DS3218 class), runtime-switchable via `SERVO:RATE` (persisted in NVS). `SERVO:RATE`
sets the LEDC carrier and `cueLoopHz` together, so a servo swap needs no reflash. Staying on analog
50 Hz now; the digital-servo swap later flips the profile to 250 Hz and unlocks the high-fidelity
mini cue loop. See `6dof/MINI_SERVO_UPGRADE.md` for the servo spec.

## Communication

- **Serial**: COBS-framed over USB — `CH_DATA18` (0x06, baked 18-byte motion), `CH_DATA_RAW`
  (0x07, raw 6×float32 live telemetry), `CH_CMD` (0x02, ASCII commands). Legacy 15-byte
  `0xAA/0x55` binary + CSV also accepted.
- **BLE**: GATT server (device name `Mini6DOF`) — Motion char `0xFF01` (12 B, 6×uint16 LE),
  Accel char `0xFF03` (24 B, 6×float32 LE) for phone-as-controller.

### Selected serial commands

| Command | Description |
|---------|-------------|
| `VERSION?` / `FINGERPRINT?` | Version + identity handshake (reports `platform=mini-6dof`, `caps=raw`) |
| `CONFIG?` / `CONFIG:key=value` | Geometry dump / set (auto-recomputes axis scales) |
| `SCALE?` / `BITS?` / `BITS:N` | Axis scales / input bit depth |
| `SERVO:RATE=...` | Analog 50 Hz ↔ digital 250 Hz profile (NVS-persisted) |
| `SERVO:CENTER=c0..c5` / `SERVO:PULSE=v` | Servo center + pulse-per-radian calibration |
| `SOURCE:OFF\|DEMO\|LIVE` | Motion source select (`OFF` homes = one-tap kill) |
| `PLAY:BOOT=0\|1` | Disable/enable boot auto-play (runtime, no reflash) |
| `ZERO` / `ESTOP:SOFT` / `DBG:1`\|`DBG:0` | Home / soft e-stop / debug toggle |

Runtime serial control without reflashing: see the `6dof-mini-serial` skill.

## Config-follows-flashed-features (direction)

The intended architecture is that **nothing about a device's capabilities is hardcoded per
connection** — the device advertises what it is (variant, PCB version, actuator type, servo mode,
axis count, `caps=raw`, partition/OTA presence) via the **FINGERPRINT handshake**, and the app /
Linux bridge / Android client **derive** their UI/config/param-set from that report. Build-time
feature flags determine the flashed feature set → the firmware reports it at runtime → the config
follows. FINGERPRINT already reports `platform=mini-6dof` and `caps=raw`; extending it to the full
capability set is the follow-up (the handshake is **not** being re-architected now).

## License

MIT — see [`../LICENSE`](../LICENSE).
