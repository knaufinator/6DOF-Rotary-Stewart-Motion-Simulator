# 6DOF Motion Simulator Controller

ESP-IDF v5.2.0 firmware for the ESP32-S3 driving 6 AASD-15A servo drivers via step/direction signals.

## Project Structure

```
Controller/
├── main/
│   ├── main.cpp              # app_main entry point, FreeRTOS tasks
│   ├── InverseKinematics.cpp # Stewart platform IK solver
│   ├── AxisScaling.cpp       # Per-axis scaling + mapRawToPosition()
│   ├── helpers.cpp           # mapfloat, rateLimit utilities
│   ├── EthernetTransport.cpp # W5500 UDP transport (opt-in)
│   └── CMakeLists.txt        # Component build config
├── include/
│   ├── InverseKinematics.h   # StewartConfig struct + IK API + drive train params
│   ├── AxisScaling.h         # AxisScaleConfig struct + scaling API
│   ├── helpers.h             # Pin defs, timing constants
│   ├── MCPWMMotorControl.h   # MCPWM hardware-timed motor control (6 channels)
│   ├── GPTimerScheduler.h    # 100 µs deterministic timer
│   ├── EthernetTransport.h   # W5500 SPI + UDP API
│   └── debug_uart.h          # Compile-time debug gating
└── CMakeLists.txt            # Top-level ESP-IDF project
```

## Hardware

- **MCU**: ESP32-S3-DevKitC-1
- **Servo drivers**: 6 × AASD-15A
- **Interface**: SN75174N quad RS-422 line drivers — see [single motor test plan](../docs/hardware/single_motor_test_plan.md)
- **E-Stop**: GPIO 20, active LOW with internal pull-up

### Pinout

> ⚠️ **UNTESTED** — verify with oscilloscope before running on physical hardware.

| Motor | STEP | DIR | Control |
|-------|------|-----|---------|
| 1 | GPIO 4 | GPIO 10 | RMT CH0 (TX) |
| 2 | GPIO 5 | GPIO 11 | RMT CH1 (TX) |
| 3 | GPIO 6 | GPIO 12 | RMT CH2 (TX) |
| 4 | GPIO 7 | GPIO 13 | RMT CH3 (TX) |
| 5 | GPIO 8 | GPIO 14 | GPIO (pending RMT migration) |
| 6 | GPIO 9 | GPIO 17 | GPIO (pending RMT migration) |

Motors 5–6 currently use direct GPIO bit-bang. Migrating to IDF v5 RMT API for all 6 channels is a planned optimization — see [esp32s3_step_dir_roadmap.md](../docs/firmware/esp32s3_step_dir_roadmap.md).

## Building

Requires [ESP-IDF v5.2.0](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/get-started/index.html).

```bash
cd Controller
idf.py set-target esp32s3    # first time only
idf.py build
idf.py flash monitor
```

### Build Options

- **Debug UART** (disabled by default for safety):
  ```cmake
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ENABLE_DEBUG_UART=1)
  ```
- **Optimization**: `-O2 -ffast-math` set in `main/CMakeLists.txt`

## Communication Protocol

Two transports, both feeding the same binary packet handler:

### Transport A: Serial (always active)

115200 baud, 8N1 over USB Serial JTAG (native USB CDC — baud rate is nominal).

**Binary (preferred — 15 bytes):**

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `0xAA` sync |
| 1 | 1 | `0x55` sync |
| 2 | 12 | 6 × `uint16_t` LE (surge, sway, heave, pitch, roll, yaw) 0–4094 |
| 14 | 1 | XOR checksum of bytes 2–13 |

**Legacy CSV (backward-compatible):** `<v0>,…,<v5>X` — also handles `DBG:1X` / `DBG:0X`.

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
| `InterfaceMonitorTask` | 0 | 2 | UART RX → binary/CSV parser |
| `UDPListenerTask` | 0 | 2 | W5500 UDP RX (when `ENABLE_ETHERNET`) |
| `GPIOLoopTask` | 1 | 3 | 100 µs deterministic motor update |

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
idf.py --version              # verify v5.2.0
idf.py fullclean && idf.py build   # clean rebuild
idf.py -p COM3 flash          # explicit port (Windows)
idf.py monitor --print-filter '*:V'  # verbose log filter
```

## Links

- [Step/Dir Optimization Roadmap](../docs/firmware/esp32s3_step_dir_roadmap.md)
- [Single Motor Test Plan](../docs/hardware/single_motor_test_plan.md)
- [HIL Test Plan](../docs/hardware/hil_test_plan.md)
- [Platform Geometry](../docs/platform_geometry.md)
- [Desktop App](../app/)
- [ESP-IDF v5.2 Docs](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/)

## License

MIT — see [LICENSE](../LICENSE).
