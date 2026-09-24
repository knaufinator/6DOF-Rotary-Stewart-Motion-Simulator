# 6DOF Motion Simulator Controller

[Repository overview](../README.md) · [Desktop app](../app/README.md) · [Build guide](../docs/BUILD.md) · [Firmware variants](../docs/FIRMWARE.md)

Paths and commands in this guide start at the repository root unless noted otherwise.
`controller/main/` and `controller/include/` are the full-size firmware project;
`controller/mini/` and `controller/test_harness/` are separate ESP-IDF projects.

> **PCBv2 / 6DOF 2: UNTESTED, firmware work pending (2026-09-23).** The routed r13
> RC4 prototype files are complete, not a motion-ready release. Nothing is ordered.
> No r13 firmware has been implemented or validated. Native USB input, Ethernet,
> pulse direction guards, physical stop enforcement, alarm handling and the new
> hardware-enable contract must be completed before connecting servo drives.
> [Required work and acceptance tests](../hardware/pcb/6DOF2_FIRMWARE_TODO.md) ·
> [Hardware status](../hardware/pcb/6DOF2_BOARD.md).

ESP-IDF v5.5 firmware for the Stewart motion platform. Supports two hardware variants from a single codebase:

| | **PCBv1** (ESP32 DevKit) | **PCBv2** (ESP32-S3 "6DOF 2" board, see [`hardware/pcb/6DOF2_BOARD.md`](../hardware/pcb/6DOF2_BOARD.md)) |
|---|---|---|
| **MCU** | ESP32 (CP2102/CH340 USB-UART) | ESP32-S3 (native USB CDC) |
| **Motor I/O** | MCP23S17 SPI GPIO expander | Direct GPIO; selected shared MCPWM backend uses software counts, not independent output measurement |
| **Step/Dir** | 6 step + 6 dir on MCP23S17 ports A/B | 6 STEP GPIOs + 6 DIR GPIOs |
| **E-Stop** | GPIO 22, active LOW | GPIO 21, active HIGH (NC loop opened / broken = stop) |
| **Baud** | 115200 (UART0) | 115200 (USB CDC — nominal) |

The build system auto-detects the target chip and sets `PCB_VERSION` accordingly.

## Project Structure

```text
controller/
├── main/                Full-size application, transports and ESP-IDF component
├── include/             Controller I/O, pulse engines, protocol and platform headers
├── CMakeLists.txt       Full-size ESP-IDF project
├── sdkconfig.defaults*  Common and target-specific configuration defaults
├── build-esp-idf.ps1     Local Windows build helper
├── mini/                Separate ESP32 hobby-servo firmware project
├── test_harness/        Separate ESP32-S3 step/dir analyzer project
├── stewart-core/        Shared Git submodule: IK, axis scaling and motion cueing
├── tests/               Host-side regression tests
└── tools/               Serial diagnostics and signal validation scripts
```

Shared math implementations and headers live in `stewart-core/src/` and
`stewart-core/include/`, not in the main firmware's `main/` or `include/`.
The desktop app also builds this same core; changes require both builds.


## Hardware

### PCBv1 — ESP32 DevKit

- **MCU**: ESP32 DevKit V1 (CP2102 or CH340 USB-UART)
- **Motor I/O**: MCP23S17 on HSPI (SPI2_HOST)
- **Servo drivers**: 6 × AASD-15A
- **E-Stop**: GPIO 22, active LOW with internal pull-up

#### MCP23S17 SPI Wiring

| MCP23S17 Pin | ESP32 GPIO |
|---|---|
| CS | 15 |
| MOSI (SI) | 13 |
| MISO (SO) | 12 |
| SCLK (SCK) | 14 |

#### Motor Pin Map (MCP23S17 ports)

| Motor | STEP (Port A) | DIR (Port B) |
|---|---|---|
| 1 | GPA0 (bit 0) | GPB0 (bit 0) |
| 2 | GPA1 (bit 1) | GPB1 (bit 1) |
| 3 | GPA2 (bit 2) | GPB2 (bit 2) |
| 4 | GPA3 (bit 3) | GPB3 (bit 3) |
| 5 | GPA4 (bit 4) | GPB4 (bit 4) |
| 6 | GPA5 (bit 5) | GPB5 (bit 5) |

### PCBv2 — ESP32-S3 ("6DOF 2" board)

- **MCU**: ESP32-S3-WROOM-1-N8R2 module on the custom 6DOF 2 board (8 MB flash, 2 MB PSRAM); not a socketed DevKit
- **Motor I/O**: Direct GPIO (shared MCPWM pulse generation; independent pulse validation pending)
- **Servo drivers**: 6 × AASD-15A
- **Interface**: Three AM26LS31 quad differential line drivers; the SN75174N breadboard interface is historical
- **E-Stop**: GPIO 21, active HIGH — J2 normally-closed loop with 10 k pull-up; monitoring enabled, boot holds paused if the loop is open
- **Status LED / r13 arm**: the GPIO48 WS2812 implementation belongs to the earlier DevKit setup. r13 uses GPIO48 for `ARM_PULSE` and GPIO47 for `MCU_INHIBIT`; remove the conflicting LED behavior before using r13. The earlier r12 unconnected-pin statement does not apply to r13.

#### STEP/DIR GPIO mapping (not a commissioning acceptance test)

These GPIO assignments match the r12 schematic reviewed September 22 and are
retained in the saved r13 schematic. The selected shared MCPWM backend's counters are software
counters, not independent PCNT/RMT evidence. `PINTEST` currently bypasses stop
gating; do not run it with drives attached.

| Motor / board port | STEP | DIR |
|---|---|---|
| 1 / M0 | GPIO 4 | GPIO 10 |
| 2 / M1 | GPIO 5 | GPIO 11 |
| 3 / M2 | GPIO 6 | GPIO 12 |
| 4 / M3 | GPIO 7 | GPIO 13 |
| 5 / M4 | GPIO 8 | GPIO 14 |
| 6 / M5 | GPIO 9 | GPIO 17 |

> ⚠️ **GPIO wiring note**: DevKitC header placement can vary by revision and is not strictly sequential by GPIO number. Always wire by printed GPIO labels (or continuity check), not by assumed physical order.

#### LED Status Indicator

**Legacy DevKit hardware only.** This section describes firmware support for an
external/DevKit WS2812; it does not mean the custom 6DOF 2 PCB contains that LED.

The onboard WS2812 RGB LED on GPIO 48 provides at-a-glance system status using color and blink patterns. A priority system ensures the most urgent state is always shown.

> ⚠️ **Hardware note**: On the ESP32-S3-DevKitC-1-N8R2, the RGB LED requires a **solder bridge on the RGB junction pad** (located near the LED on the PCB) to connect GPIO 48 to the LED data line. Without this bridge, the LED will not respond even though the driver initializes successfully.

| State | Color | Pattern | Meaning |
|---|---|---|---|
| `BOOT` | White | Solid | Initializing |
| `READY` | Green | Breathe | Idle, waiting for commands |
| `COMMS_ACTIVE` | Blue | Slow blink (1Hz) | Receiving serial/WiFi/BLE data |
| `MOTORS_ACTIVE` | Cyan | Solid | Motors stepping |
| `CONFIG` | Purple | Breathe | Configuration/setup mode |
| `WARN_POSITION` | Yellow | Double pulse | Position error / missed steps |
| `WARN_COMMS` | Blue | Fast blink (5Hz) | Communication timeout |
| `ESTOP` | Red | Fast blink (5Hz) | Emergency stop active |
| `ERROR` | Red | Triple pulse | Hardware or fatal error |

Multiple states can be active simultaneously — the highest-priority state (lowest in the table) always wins. Use from firmware code:

```cpp
#include "LedStatus.h"

led_status_set(LED_STATE_MOTORS_ACTIVE);    // cyan solid
led_status_clear(LED_STATE_MOTORS_ACTIVE);  // revert to next highest
led_status_set(LED_STATE_ESTOP);            // red fast blink (overrides all)
led_status_reset();                         // clear all, back to READY
```

## Building

Requires [ESP-IDF v5.5](https://docs.espressif.com/projects/esp-idf/en/v5.5/get-started/index.html).

```bash
cd controller

# PCBv1 (ESP32 DevKit):
idf.py set-target esp32
idf.py build
idf.py -p COM6 flash monitor

# PCBv2 (ESP32-S3 DevKit):
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor
```

> **Note**: The ESP32 DevKit may require manual boot mode for flashing (hold BOOT, press EN, release both) if auto-reset via DTR/RTS is unreliable.

The build system automatically:
- Sets `PCB_VERSION=1` for ESP32, `PCB_VERSION=2` for ESP32-S3
- Includes MCP23S17 driver only for PCBv1, MCPWM + EthernetTransport only for PCBv2
- Applies target-specific `sdkconfig.defaults.esp32` or `sdkconfig.defaults.esp32s3`

### Build Options

- **Debug UART** (disabled by default for safety):
  ```cmake
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ENABLE_DEBUG_UART=1)
  ```
- **Optimization**: `-O2 -ffast-math` set in `main/CMakeLists.txt`

### NVS Geometry Persistence

Platform geometry (`CONFIG:key=value` commands) is automatically saved to NVS flash. On boot, the firmware loads saved geometry if present, otherwise uses factory defaults. This means the desktop app only needs to push geometry once — it persists across reboots and reconnects.

## Communication Protocol

Intended transport interfaces. September 22 source review found two integration
defects: native USB is an output-only secondary console while reception uses UART0;
Ethernet's 12-byte payload is rejected by the 18-byte motion decoder. The documented
interfaces below require TODO F06–F08 before use as a commissioning contract.

### Transport A: Serial (native USB receive fix pending)

921600 baud over USB Serial JTAG / COBS-framed stream (native USB CDC — baud rate is nominal).

**COBS Binary (preferred):**

| Channel | Payload | Notes |
|---|---|---|
| `CH_DATA18` (`0x06`) | 18 bytes = `6 × uint24` LE (low 18 bits used) | Motion data (all bit depths use this single wire format) |
| `CH_CMD` (`0x02`) | ASCII command text | Runtime commands (`BITS`, `CONFIG`, etc.) |

COBS framing adds a `0x00` delimiter between encoded frames and self-recovers after line noise.

All runtime commands (`FINGERPRINT?`, `CONFIG?`, `BITS?`, etc.) are sent on `CH_CMD`.

**COBS metadata diagnostics (visible in app HIL panel):**

- Delimiters seen (`delim`)
- Decode success/fail counters (`ok`, `fail`)
- Per-channel receive counters (`tel`, `resp`, `log`)
- Serial byte totals (`RX`, `TX`) and telemetry quality (`Hz`, `seq`, rejected count)

### Transport B: UDP over Ethernet (compile-time opt-in)

The custom PCB includes W5500. Enable with `-DENABLE_ETHERNET=1` after repairing
its MAC initialization and payload dispatch. The following legacy packet format
is not compatible with the current motion decoder; update sender and receiver together.

- **Port**: 4210 (configurable via `ETH_UDP_PORT`)
- **Packet**: 12 raw bytes (6 × `uint16_t` LE) — no sync/checksum needed, UDP provides framing
- **Also accepts**: 15-byte framed packets (same as serial binary) for sender simplicity
- **DHCP**: automatic IP assignment
- SimTools "Network Interface" sends UDP to the ESP32's IP:4210

#### W5500 Wiring

| W5500 Pin | ESP32-S3 GPIO |
|-----------|---------------|
| MOSI | 35 |
| MISO | 37 |
| SCLK | 36 |
| CS | 38 |
| INT | 39 |
| VCC | 3.3 V |
| GND | GND |

### Debug Telemetry (10 Hz, when enabled)

```
DEBUG,<timestamp_us>,<a0>,<a1>,<a2>,<a3>,<a4>,<a5>,<tX>,<tY>,<tZ>,<rX>,<rY>,<rZ>
```

## FreeRTOS Tasks

| Task | Core | Priority | Purpose |
|------|------|----------|---------|
| `EStopMonitorTask` | 0 | MAX−1 | Debounced E-stop with GPTimer pause |
| `InterfaceMonitorTask` | 0 | 2 | UART RX → COBS frame processing |
| `UDPListenerTask` | 0 | 2 | W5500 UDP RX (when `ENABLE_ETHERNET`) |
| `GPIOLoopTask` | 1 | 3 | 100 µs deterministic motor update |
| `LedStatusTask` | 0 | 1 | RGB LED status indicator (PCBv2 only) |

## Desktop App

A native C++/OpenGL/ImGui desktop application for SIL simulation, HIL ESP32 control, test signal generation, capture playback, and spectrogram analysis. See the [app README](../app/README.md#desktop-app) for full feature list.

```bash
cd app
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/Release/stewart-platform
```

### Serial Commands (Runtime Axis Scaling)

The firmware supports live axis scale adjustment over serial:

- **`SCALE:8,8,7,30,30,30`** — set all 6 axis scales
- **`SCALE?`** — query current scales (responds `SCALE:8.00,8.00,7.00,30.00,30.00,30.00`)

## Troubleshooting

```bash
idf.py --version              # verify v5.5.x
idf.py fullclean && idf.py build   # clean rebuild
idf.py -p COM6 flash          # explicit port (PCBv1 on COM6)
idf.py -p COM3 flash          # explicit port (PCBv2 on COM3)
idf.py monitor --print-filter '*:V'  # verbose log filter
```

**ESP32 DevKit flashing issues**: If auto-reset doesn't work, manually enter boot mode (hold BOOT, press EN, release both) before running `idf.py flash`. Use `--before no_reset` with esptool if needed.

**App won't connect**: Check `app_debug.log` next to the executable for serial RX trace. The app disables DTR/RTS to avoid resetting the ESP32 on connect.

## Links

- [Firmware variants and validation](../docs/FIRMWARE.md)
- [Single Motor Test Plan](../hardware/pcb/single_motor_test_plan.md)
- [Firmware and commissioning requirements](../hardware/pcb/6DOF2_FIRMWARE_TODO.md)
- [Platform Geometry](../hardware/mechanical/platform_geometry.md)
- [Desktop App](../app/)
- [ESP-IDF v5.5 Docs](https://docs.espressif.com/projects/esp-idf/en/v5.5/)

## License

MIT — see [LICENSE](../LICENSE).
