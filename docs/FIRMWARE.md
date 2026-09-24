# Firmware variants

Paths and commands in this guide start at the repository root unless noted otherwise.

> **6DOF 2 PCBv2 prototype status — 2026-09-23:** r13 RC4 routed PCB files are complete
> for prototype review, but nothing is ordered or hardware-validated. Firmware
> changes are deliberately deferred. The current full-size firmware must not be
> treated as a safe drive-connected commissioning image. The actionable backlog is
> [6DOF2_FIRMWARE_TODO.md](../hardware/pcb/6DOF2_FIRMWARE_TODO.md); the current package
> state is [6DOF2_BOARD.md](../hardware/pcb/6DOF2_BOARD.md).

This repo carries **one firmware codebase in two build-time variants**, both sharing the single
[`stewart-core`](../controller/stewart-core) submodule under `controller/` (IK / AxisScaling / MotionCueing —
the single source of truth, canonical `atan2f`). The variant is selected at build time via the
IDF target + the actuator backend. Shared math does not mean identical application
behavior: full-size `controller/` and `controller/mini/` transport, playback, cueing, command
and persistence features differ. Do not assume a mini feature exists in the full controller.

| Variant | Directory | Actuator backend | Boards / MCU | Servo rate | Build |
|---------|-----------|------------------|--------------|-----------|-------|
| **normal** (full system) | [`controller/`](../controller) | MCPWM step/dir → AASD-15A AC servos (MCP23S17 SPI expander on PCBv1; direct GPIO on PCBv2). W5500 Ethernet. | PCBv1 = ESP32, PCBv2 = ESP32-S3 | 250 Hz cue loop | `set-target esp32`\|`esp32s3` |
| **mini** (desktop) | [`controller/mini/`](../controller/mini) | LEDC PWM → hobby servos | ESP32 DevKitC | analog 50 Hz (runtime-switchable to digital 250 Hz via `SERVO:RATE`) | `set-target esp32` |

Both directories are separate project-level ESP-IDF projects. The main project
uses `set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/stewart-core")`;
the mini project uses `"${CMAKE_CURRENT_LIST_DIR}/../stewart-core"`.
Both resolve it through `PRIV_REQUIRES stewart-core` in their component build.
There is **one shared math submodule**, not a separate copy per variant.

```text
<repo root>/
├── controller/       Full-system firmware, main/ + include/
│   ├── mini/         Hobby-servo firmware project
│   ├── test_harness/ Step/dir analyzer firmware project
│   ├── stewart-core/ Single shared IK/scaling/cueing Git submodule
│   ├── tests/        Host-side regression tests
│   └── tools/        Diagnostics
├── app/              Desktop SIL/HIL simulator
│   └── bridge/       Optional Linux HIL bridge
├── hardware/         PCB packages and mechanical/mini printed parts
└── docs/             Shared documentation
```

## Building

Requires ESP-IDF v5.5 (current dependency-lock baseline: 5.5.2). On a fresh clone,
fetch the shared core first. For an existing checkout, preserve any local core
edits; see the [build guide](BUILD.md) for submodule and relocated-checkout guidance.

```bash
git submodule update --init --recursive
```

Each example below starts from the repository root; choose one target.

```bash
# mini variant
cd controller/mini && idf.py set-target esp32 && idf.py build
```

```bash
# normal variant — PCBv2 (ESP32-S3)
cd controller && idf.py set-target esp32s3 && idf.py build
```

```bash
# normal variant — PCBv1 (ESP32)
cd controller && idf.py set-target esp32 && idf.py build
```

**PCBv2 E-stop (2026-09-05):** J2 takes a normally-closed loop with a 10 k pull-up, so the input is LOW while closed (run) and HIGH when opened or a wire breaks (stop). `helpers.h` sets `ESTOP_ACTIVE_STATE 1` for `PCB_VERSION == 2` (PCBv1 keeps 0), the monitor task tests edges against that constant, and monitoring is enabled on both boards. Board details: [`../hardware/pcb/6DOF2_BOARD.md`](../hardware/pcb/6DOF2_BOARD.md).

Per-variant hardware notes, pinouts, and serial commands: [`controller/mini/README.md`](../controller/mini/README.md)
and [`controller/README.md`](../controller/README.md).

## Validation and commissioning

A successful build or desktop simulation does not validate physical motion. The
[firmware and commissioning requirements](../hardware/pcb/6DOF2_FIRMWARE_TODO.md)
are the authority for the current r13 prototype; older backend measurements are
not acceptance evidence for this board/firmware combination.

Use the [analyzer firmware guide](../controller/test_harness/README.md) for the
independent measurement tool and the [single-axis test gate](../hardware/pcb/single_motor_test_plan.md)
for first-motion prerequisites. Pulse counts, direction setup/hold, idle output,
stop/re-arm behavior and timing under load require measurement, not just firmware
self-reports. Keep drives disconnected until the applicable gates are satisfied.
