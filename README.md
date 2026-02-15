# 6DOF Rotary Stewart Motion Simulator Platform

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)

> A high-performance 6 Degrees of Freedom motion simulator platform powered by AC servo motors with AASD-15A drivers.

⚠️ **SAFETY WARNING**: This is a DANGEROUS project. Improper assembly or operation can result in serious injury or death. Ensure emergency stop systems are in place before any operation.

## Demo Videos

<div align="center">
  <a href="http://www.youtube.com/watch?feature=player_embedded&v=fEcdGIq_Jzc" target="_blank">
    <img src="http://img.youtube.com/vi/fEcdGIq_Jzc/0.jpg" alt="Motion Sim Demo 1" width="400"/>
  </a>
  <a href="http://www.youtube.com/watch?feature=player_embedded&v=1WXx59tWYc4" target="_blank">
    <img src="http://img.youtube.com/vi/1WXx59tWYc4/0.jpg" alt="Motion Sim Demo 2" width="400"/>
  </a>
</div>

<div align="center">
  <a href="http://www.youtube.com/watch?feature=player_embedded&v=_NR_MUGvmUo" target="_blank">
    <img src="http://img.youtube.com/vi/_NR_MUGvmUo/0.jpg" alt="Motion Sim Demo 3" width="400"/>
  </a>
  <a href="http://www.youtube.com/watch?feature=player_embedded&v=CdDkL8X6qOE" target="_blank">
    <img src="http://img.youtube.com/vi/CdDkL8X6qOE/0.jpg" alt="Motion Sim Demo 4" width="400"/>
  </a>
</div>

## Features

- 6 × 750 W AC servo motors with AASD-15A drivers and 50:1 planetary gears
- ESP32-S3 controller running native ESP-IDF v5.2.0 with 10 kHz deterministic control loop
- RMT peripheral for hardware-accelerated step pulse generation
- SN75174N RS-422 differential line drivers for STEP/DIR signaling to AASD-15A servo drivers
- SimTools compatible via UDP over Ethernet (W5500 SPI module)
- Native desktop control app (C++/OpenGL/ImGui) — single executable, no server, no browser
  - SIL simulation with real-time 3D visualization and servo readout
  - HIL mode: serial bridge to ESP32 at up to 1000 Hz TX rate
  - Multi-entity test bench: run SIL and HIL side-by-side with independent configs
  - Test signal generator with per-axis waveforms, smooth parameter interpolation
  - Motion capture recording and playback with speed control
  - Rolling spectrogram heatmap with multi-entity/axis overlay
  - SimTools UDP input with configurable bit depth
  - Shared C source with ESP32 firmware (IK, axis scaling, motion cueing)

## Repository Layout

| Directory | Contents |
|-----------|----------|
| `app/` | Native desktop app (C++/OpenGL/ImGui) — see [Desktop App](#desktop-app) below |
| `Controller/` | ESP32-S3 firmware (ESP-IDF v5.2) — see [Controller/README.md](Controller/README.md) |
| `test_harness/` | Step/dir signal analyzer firmware — see [test_harness/README.md](test_harness/README.md) |
| `docs/` | [App Guide](docs/APP_GUIDE.md), [Architecture](docs/ARCHITECTURE_ROADMAP.md), [IK research](docs/IK_RESEARCH.md), [Platform geometry](docs/platform_geometry.md) |
| `docs/firmware/` | [Step/Dir optimization roadmap](docs/firmware/esp32s3_step_dir_roadmap.md) |
| `docs/hardware/` | [Single motor test plan](docs/hardware/single_motor_test_plan.md), [HIL test plan](docs/hardware/hil_test_plan.md) |

## Quick Start

```bash
# 1. Clone
git clone https://github.com/knaufinator/6DOF-Rotary-Stewart-Motion-Simulator.git
cd 6DOF-Rotary-Stewart-Motion-Simulator/Controller

# 2. Build & flash (requires ESP-IDF v5.2.0)
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

See **[BUILD.md](BUILD.md)** for detailed step-by-step build instructions for both the firmware and desktop app, including prerequisites, troubleshooting, and build options.

See [Controller/README.md](Controller/README.md) for firmware architecture, pin mappings, and communication protocol.

## Desktop App

The desktop app is a single native C++ executable that replaces the former Python/browser dashboard. It compiles the same C firmware modules (IK, axis scaling, motion cueing) directly — no CFFI bridge, no JSON serialization on the hot path.

```bash
cd app
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/Release/stewart-platform      # Linux/macOS
.\build\Release\stewart-platform.exe  # Windows
```

**Requires**: CMake 3.20+, C++17 compiler (MSVC, GCC, or Clang), OpenGL 3.3+

### App Features

- **Multi-entity test bench** — run multiple SIL/HIL entities with independent pipeline configs
- **Input sources** — manual sliders, SimTools UDP, test signal generator, capture playback
- **Test signals** — per-axis sine waves with configurable frequency/amplitude/phase, smooth live parameter changes, S-curve ramp envelope, presets
- **Recording & playback** — capture any input source, save to library, play back with speed control and looping
- **HIL serial bridge** — auto-detect COM port, auto-connect, binary protocol at up to 1000 Hz, real-time telemetry
- **3D visualization** — per-entity OpenGL Stewart platform renderer with orbit camera
- **Spectrogram** — rolling waterfall heatmap with multi-entity/axis lane selection
- **Data streams** — time-series charts, frequency analysis, recording library management
- **Console** — per-entity filtered log with configurable rate
- **Auto-save** — settings persist automatically including input source, entity configs, and HIL ports

## Communication Protocol

Two transports, both feeding the same binary packet handler. See [Controller/README.md](Controller/README.md) for full details.

### Serial (always active) — 115200 baud, 8N1 (USB CDC)

15-byte binary packet: `[0xAA] [0x55] [6 × uint16 LE] [XOR checksum]`

Legacy CSV also supported: `<v0>,…,<v5>X`

### UDP over Ethernet (opt-in) — W5500 SPI module

Compile with `-DENABLE_ETHERNET=1`. Sends 12 raw bytes (6 × `uint16_t` LE) to UDP port 4210. No sync/checksum needed — UDP provides framing. DHCP for IP assignment.

### SimTools Setup

![SimTools Network Setup](documentation/images/simtools_network_setup.png)

| Setting | Value |
|---------|-------|
| Interface Type | Network |
| IP Address | `127.0.0.1` (SIL) or ESP32 IP (hardware) |
| Port | `4123` (SIL) or `4210` (hardware UDP) |
| Output - Bit Range | `12` |
| Output - Type | Decimal |
| Output Rate | 20 ms (SIL testing) / 1 ms (production target: 1000 Hz) |
| Axis Mapping | `x, y, z, Ry, Rx, Rz` |

For serial: Baud Rate `115200`, 8N1 (USB CDC — baud is nominal).

## AASD-15A Servo Settings

```
pn002 - Control Mode: "002"
pn003 - Servo enable: "001"
pn098 - Gear: "80"
pn109 - Position command deceleration mode: "002"
pn110 - Position command filtering time constant: "050"
pn111 - S-shaped filtering time constant Ta: "50"
pn112 - Position instruction Ts S-shaped filtering: "50"

Homing:
pn033 - Power on homing: 3
pn034 - Direction: 0 (CW) or 1 (CCW)
pn036 - Coarse position: +/-11 ×1000 pulses
pn037 - Fine position: +/-5000
pn038 - Initial speed: 100
pn039 - Return speed: 100
```

## Geometry Calibration & Home Height

Getting the platform geometry right is one of the most important — and most commonly overlooked — steps when setting up a Stewart platform. If the dimensions entered in the app don't match the physical hardware, the inverse kinematics (IK) will produce incorrect servo angles. The most critical parameter is **Home Height**, and this section explains why.

### What is Home Height?

Home Height (`z_home`) is the vertical distance from the base plate to the platform plate when all six servo arms are at **0 degrees** (perfectly horizontal) and the platform is **level and parallel** to the base. It is not an arbitrary number — it is a direct consequence of the other geometry parameters.

<!-- IMAGE: Annotated side-view diagram of a Stewart platform at home position, showing z_home as the vertical distance between base and platform plates, with servo arms horizontal and connecting rods at an angle. -->
<!-- File: documentation/images/home_height_diagram.png -->

### How Home Height is Determined

For a given set of geometry parameters (base radius, platform radius, servo arm length, connecting rod length, and joint angles), there is exactly **one** correct home height. It is derived from the physical constraint that the connecting rod (L2) must reach from the servo arm tip to the platform joint:

```
At home (servo angle = 0°):
  arm_tip  = base_joint + L1 × [cos(β), sin(β), 0]     (arm horizontal)
  plat_joint = platform_joint_pos + [0, 0, z_home]       (platform level)

  |arm_tip − plat_joint| = L2                             (rod length constraint)

Solving for z_home:
  z_home = √( L2² − horizontal_distance² )

where horizontal_distance is the XY distance between the arm tip and platform joint.
```

For a symmetric 3-pair Stewart platform, all six actuators yield the **same** z_home. If your geometry is truly symmetric, this value is unique and exact.

<!-- IMAGE: Top-down 2D schematic showing base joints (B_k), arm tips at 0°, and platform joints (P_k), with horizontal_distance labeled between arm tip and platform joint for one actuator. -->
<!-- File: documentation/images/home_height_topdown.png -->

### What Happens When Home Height is Wrong

When z_home doesn't match the geometry, the IK must compensate by computing **non-zero** servo angles just to keep the platform at its "home" position. Because the platform has 3-pair symmetry (not 6-fold symmetry), these compensation angles are **different** for each actuator pair. This causes several visible problems:

1. **Platform not level at home** — Even with zero input, the platform tilts slightly because the six servos are at different angles.

2. **Cross-axis coupling** — A pure surge (forward/backward) input produces unwanted roll or pitch motion. The asymmetric home offsets bias the geometry so that linear translation creates rotational side effects.

3. **Asymmetric workspace** — The platform can travel further in one direction than another because some servos start closer to their limits.

4. **Visual distortion in 3D view** — The connecting rods in the visualization appear to stretch or compress because the geometric constraints can't be satisfied with the wrong z_home.

<!-- IMAGE: Side-by-side comparison of the 3D platform visualization: left shows distorted/stretched rods with wrong z_home (92mm), right shows correct geometry with computed z_home (~99mm). Both at the same input pose. -->
<!-- File: documentation/images/home_height_comparison.png -->

### Example: Mini-6DOF with Wrong Home Height

The Mini-6DOF platform has the following approximate geometry:

| Parameter | Value |
|-----------|-------|
| RD (base radius) | 74 mm |
| PD (platform radius) | 74 mm |
| L1 (servo arm) | 31 mm |
| L2 (connecting rod) | 116 mm |
| Theta R | 7° |
| Theta P | 30° |

With these dimensions, the **correct** home height is approximately **99.2 mm**. If you enter 92 mm (a 7% error), the IK produces home angles ranging from -4° to +6° across the six servos instead of all zeros. During a pure surge test signal, this manifests as a visible rocking/wobbling of the platform — the pure linear input couples into rotational motion.

<!-- IMAGE: Screenshot of the app running a pure surge sine test signal with the wrong z_home, showing the platform rocking in the 3D view or the output angles being unequal at home. -->
<!-- File: documentation/images/home_height_wrong_surge.png -->

### Using the Auto-Compute Button

The app includes an **Auto** button next to the Home Height field in the Geometry tab. This button computes the mathematically correct z_home from the current geometry parameters.

**How to use it:**

1. Open the entity's **Geometry** tab
2. Enter your measured values for RD, PD, L1, L2, Theta R, and Theta P
3. Look at the **Auto** button next to Home Height — if it's **orange**, your current z_home is off by more than 0.5 mm
4. Hover over the Auto button to see the computed value and the difference
5. Click **Auto** to set the correct home height

<!-- IMAGE: Screenshot of the Geometry tab showing the Home Height field with the orange Auto button, and the tooltip showing "Computed: 99.2 mm (current: 92.0 mm, diff: 7.2 mm)". -->
<!-- File: documentation/images/auto_home_height_button.png -->

After clicking Auto, the 3D visualization should show a level platform at home with no rod stretching, and pure single-axis test signals should produce motion in only that axis.

<!-- IMAGE: Screenshot of the Geometry tab after clicking Auto, showing the corrected Home Height value and the 3D view with a level, non-distorted platform. -->
<!-- File: documentation/images/auto_home_height_result.png -->

### When to Re-Compute Home Height

You should click **Auto** (or manually verify z_home) any time you change:

- **RD** or **PD** (base or platform radius)
- **L1** or **L2** (servo arm or connecting rod length)
- **Theta R** or **Theta P** (joint angles)

These parameters all feed into the z_home calculation. Changing any of them without updating z_home will reintroduce the asymmetry.

### Measuring Your Physical Geometry

If you're setting up a new platform or calibrating an existing one, measure each parameter as follows:

- **RD (base radius)** — Distance from the center of the base plate to the center of a base joint (servo shaft). Measure at least two opposing pairs and average.
- **PD (platform radius)** — Same measurement on the platform plate to the center of a platform ball joint.
- **L1 (servo arm)** — Center-to-center distance from the servo shaft to the rod end at the tip of the servo arm.
- **L2 (connecting rod)** — Center-to-center distance between the two rod end ball joints on the connecting rod (Panhard bar).
- **Theta R / Theta P** — Angular spread of the joint pairs. See [docs/platform_geometry.md](docs/platform_geometry.md) for details.

<!-- IMAGE: Photo or annotated diagram of a real platform with measurement callouts for RD, PD, L1, L2, showing where to measure each dimension. -->
<!-- File: documentation/images/geometry_measurement_guide.png -->

> **Tip**: It's better to let the app compute z_home from your other measurements than to try to measure the home height directly. Direct measurement of the vertical gap is prone to error because the platform must be perfectly level and all servos must be exactly at 0° — which is the very state you're trying to calibrate.

## Hardware

- **Base**: 31″ diameter × ½″ steel plate
- **Motors**: 6 × 750 W AC servos + 50:1 planetary gears + couplers
- **Linkage**: 12 × ½″ Panhard bar kits with rod ends and high-misalignment spacers
- **Interface**: SN75174N quad RS-422 line drivers — see [single motor test plan](docs/hardware/single_motor_test_plan.md)
- **Geometry**: See [docs/platform_geometry.md](docs/platform_geometry.md)

## Testing

```bash
# Firmware build
cd Controller && idf.py build

# Desktop app
cd app && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --config Release

# Step/dir signal analyzer (ESP-to-ESP testing)
cd test_harness && idf.py build
```

## License

MIT — see [LICENSE](LICENSE). By using any part of this project you acknowledge you do so **at your own risk**.
