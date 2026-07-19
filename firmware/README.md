# Firmware variants

This repo carries **one firmware codebase in two build-time variants**, both sharing the single
[`stewart-core`](../stewart-core) submodule at the repo root (IK / AxisScaling / MotionCueing —
the single source of truth, canonical `atan2f`). The variant is selected at build time via the
IDF target + the actuator backend; the shared motion pipeline (COBS transport, playback, CueTask,
command handler, NVS config) is identical.

| Variant | Directory | Actuator backend | Boards / MCU | Servo rate | Build |
|---------|-----------|------------------|--------------|-----------|-------|
| **normal** (full system) | [`../Controller/`](../Controller) | MCPWM step/dir → AASD-15A AC servos (MCP23S17 SPI expander on PCBv1; direct GPIO on PCBv2). W5500 Ethernet. | PCBv1 = ESP32, PCBv2 = ESP32-S3 | 250 Hz cue loop | `set-target esp32`\|`esp32s3` |
| **mini** (desktop) | [`../mini/`](../mini) | LEDC PWM → hobby servos | ESP32 DevKitC | analog 50 Hz (runtime-switchable to digital 250 Hz via `SERVO:RATE`) | `set-target esp32` |

Both directories are project-level ESP-IDF projects that point at the root submodule with
`set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../stewart-core")` and resolve it through
`PRIV_REQUIRES stewart-core` in their `main/CMakeLists.txt`. There is **no per-variant copy** of
the core math and **no `components/stewart-core` submodule** inside either variant.

```
<repo root>/
├── Controller/       normal (full-system) firmware variant  → ../stewart-core
├── mini/             mini (desktop) firmware variant         → ../stewart-core
├── stewart-core      single shared submodule (IK/AxisScaling/MotionCueing)
├── cad/mini/         mini 3D-printed parts (servo mounts, seat rails)
├── app/              desktop SIL/HIL simulator (also consumes stewart-core)
├── bridge/           Linux HIL bridge service
├── pcb/  tests/  docs/
```

## Building

Requires ESP-IDF v5.5. Fetch the shared core first:

```bash
git submodule update --init --recursive
```

```bash
# mini variant
cd mini && idf.py set-target esp32 && idf.py build

# normal variant — PCBv2 (ESP32-S3)
cd Controller && idf.py set-target esp32s3 && idf.py build
# normal variant — PCBv1 (ESP32)
cd Controller && idf.py set-target esp32 && idf.py build
```

Per-variant hardware notes, pinouts, and serial commands: [`../mini/README.md`](../mini/README.md)
and [`../Controller/README.md`](../Controller/README.md).

## Config-follows-flashed-features (direction)

The intended architecture: **nothing about a device's capabilities is hardcoded per connection.**
Build-time feature flags determine the flashed feature set; the device then **advertises what it is**
via the FINGERPRINT handshake (variant, PCB version, actuator type, servo mode, axis count,
`caps=raw`, partition/OTA presence, …); the app / Linux bridge / Android client **derive** the
UI / config / param-set from that report. FINGERPRINT already reports `platform=…` and `caps=raw`;
extending it to the full capability set is the follow-up. **The handshake is not being
re-architected now** — this section only records the direction.

## Planned follow-up refactor (NOT done here)

The current layout adds `mini/` as a sibling of the existing `Controller/`, the lowest-risk first
step. The intended end state groups by variation and factors the shared firmware out:

```
firmware/
├── common/          transport, playback, CueTask, motion pipeline, command handler, config
└── variants/
    ├── normal/      (today's Controller/)
    └── mini/        (today's mini/)
```

Deferred deliberately: moving `Controller/` → `normal/` and extracting `firmware/common/` is
disruptive to the app's CMake wiring and CI, so it is a separate follow-up once both variants are
proven in-tree. Do not fold it into this change.
