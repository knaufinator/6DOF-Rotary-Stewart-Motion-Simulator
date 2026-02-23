# 6DOF Motion Simulator Controller

ESP-IDF v5.5 firmware for the Stewart motion platform. Supports two hardware variants from a single codebase:

| | **PCBv1** (ESP32 DevKit) | **PCBv2** (ESP32-S3 DevKit) |
|---|---|---|
| **MCU** | ESP32 (CP2102/CH340 USB-UART) | ESP32-S3 (native USB CDC) |
| **Motor I/O** | MCP23S17 SPI GPIO expander | Direct GPIO (MCPWM + RMT/PCNT hardware counting) |
| **Step/Dir** | 6 step + 6 dir on MCP23S17 ports A/B | 6 STEP GPIOs + 6 DIR GPIOs |
| **E-Stop** | GPIO 22, active LOW | GPIO 20 (disabled — conflicts with USB D+) |
| **Baud** | 115200 (UART0) | 115200 (USB CDC — nominal) |

The build system auto-detects the target chip and sets `PCB_VERSION` accordingly.

## Project Structure

```
Controller/
├── main/
│   ├── main.cpp              # app_main, FreeRTOS tasks, command API
│   ├── MCP23S17.cpp          # SPI GPIO expander driver (PCBv1 only)
│   ├── InverseKinematics.cpp # Stewart platform IK solver
│   ├── AxisScaling.cpp       # Per-axis scaling + mapRawToPosition()
│   ├── MotionCueing.cpp      # MCA washout/smoothing filters
│   ├── helpers.cpp           # mapfloat, rateLimit utilities
│   ├── WifiTransport.cpp     # WiFi STA + UDP transport (opt-in)
│   ├── EthernetTransport.cpp # W5500 UDP transport (opt-in, PCBv2)
│   ├── LedStatus.cpp         # RGB LED status indicator (PCBv2)
│   └── CMakeLists.txt        # Component build — auto PCB_VERSION
├── include/
│   ├── InverseKinematics.h   # StewartConfig + IK API + drive train
│   ├── AxisScaling.h         # AxisScaleConfig + scaling API
│   ├── MotionCueing.h        # MCA config + biquad filters
│   ├── helpers.h             # Conditional pin defs (#if PCB_VERSION)
│   ├── MCP23S17.h            # SPI GPIO expander (PCBv1)
│   ├── MCPWMMotorControl.h   # MCPWM motor control (PCBv2)
│   ├── GPTimerScheduler.h    # 100 µs deterministic timer
│   ├── LedStatus.h           # LED status indicator API
│   ├── version.h             # FW version, build date, platform ID
│   └── debug_uart.h          # Compile-time debug gating
├── sdkconfig.defaults        # Common config (both targets)
├── sdkconfig.defaults.esp32  # PCBv1: BLE-only, IRAM optimizations
├── sdkconfig.defaults.esp32s3 # PCBv2: USB CDC, full BT stack
└── CMakeLists.txt            # Top-level ESP-IDF project
```

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

### PCBv2 — ESP32-S3 DevKit

- **MCU**: ESP32-S3-DevKitC-1-N8R2 (8MB flash, 2MB PSRAM)
- **Motor I/O**: Direct GPIO (MCPWM pulse generation + hardware counting)
- **Servo drivers**: 6 × AASD-15A
- **Interface**: SN75174N quad RS-422 line drivers
- **E-Stop**: GPIO 20 (currently disabled — conflicts with USB D+)
- **Status LED**: WS2812 RGB on GPIO 48 (see [LED Status Indicator](#led-status-indicator) below)

#### Pinout (validated with PINTEST)

| Motor | STEP | DIR | Control |
|---|---|---|---|
| 1 | GPIO 4 | GPIO 10 | MCPWM + PCNT |
| 2 | GPIO 5 | GPIO 11 | MCPWM + PCNT |
| 3 | GPIO 6 | GPIO 12 | MCPWM + PCNT |
| 4 | GPIO 7 | GPIO 13 | MCPWM + PCNT |
| 5 | GPIO 8 | GPIO 14 | MCPWM + RMT |
| 6 | GPIO 9 | GPIO 17 | MCPWM + RMT |

> ⚠️ **GPIO wiring note**: DevKitC header placement can vary by revision and is not strictly sequential by GPIO number. Always wire by printed GPIO labels (or continuity check), not by assumed physical order.

#### LED Status Indicator

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
cd Controller

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

Two transports, both feeding the same binary packet handler:

### Transport A: Serial (always active)

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

Requires a W5500 SPI Ethernet module. Enable with `-DENABLE_ETHERNET=1`.

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

A native C++/OpenGL/ImGui desktop application for SIL simulation, HIL ESP32 control, test signal generation, capture playback, and spectrogram analysis. See the [root README](../README.md#desktop-app) for full feature list.

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

- [Step/Dir Optimization Roadmap](../docs/firmware/esp32s3_step_dir_roadmap.md)
- [Single Motor Test Plan](../docs/hardware/single_motor_test_plan.md)
- [HIL Test Plan](../docs/hardware/hil_test_plan.md)
- [Platform Geometry](../docs/platform_geometry.md)
- [Desktop App](../app/)
- [ESP-IDF v5.5 Docs](https://docs.espressif.com/projects/esp-idf/en/v5.5/)

## License

MIT — see [LICENSE](../LICENSE).
