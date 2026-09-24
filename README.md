# 6DOF Rotary Stewart Motion Simulator

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Controller: ESP32 / ESP32-S3](https://img.shields.io/badge/Controller-ESP32%20%2F%20ESP32--S3-blue.svg)](controller/README.md)

A DIY, open-source motion simulator built around a **six-axis rotary Stewart
platform**: six servo-driven arms and connecting rods move a platform in surge,
sway, heave, roll, pitch and yaw. The project brings together the mechanical
build, custom electronics, ESP32 controller firmware and a desktop app for
simulation, motion cueing and testing.

It began as a full-size racing-simulator build and has grown into a development
platform of its own, including a smaller hobby-servo version. This repository
shares the build history and videos as well as the code, design files and lessons
learned along the way—not just the latest PCB package.

[Watch the videos](#demo-and-build-videos) · [Project history](#a-bit-of-project-history) ·
[Desktop app](#desktop-app) · [Get started](#getting-started) · [Support the project](#support-the-project)

<div align="center">
  <a href="https://www.paypal.me/knaufinator">
    <img src="https://img.shields.io/badge/Support_This_Project-PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Donate via PayPal"/>
  </a>
</div>

## Current development status

> **UNTESTED PCB PROTOTYPE — 2026-09-23. DO NOT ORDER.** The r13 RC4 design
> package is complete for review; actual JLC stackup, processed allocation and
> complete assembly approval remain pending. Nothing has been ordered or paid for.
> The board has not been manufactured or validated as a working controller.
> Firmware corrections and physical testing are required before drive-connected
> operation. Do not use the current firmware/PCB combination on a powered platform.

Heavy rotating machinery can cause serious injury or death. Independent emergency
stop hardware must be in place before powered operation; the app's E-STOP button
is not a substitute for a physical safety system. Earlier working builds and
signal captures do **not** validate the current PCB/firmware combination.

## Demo and build videos

Watch the original platform demonstrations and controller build videos below.
Click a thumbnail to open YouTube. These document earlier builds, not an assembled
or tested r13 board.

### Platform demonstrations

<div align="center">
  <a href="https://www.youtube.com/watch?v=_NR_MUGvmUo">
    <img src="https://img.youtube.com/vi/_NR_MUGvmUo/0.jpg" alt="Original motion-platform demonstration" width="400"/>
  </a>
  <a href="https://www.youtube.com/watch?v=CdDkL8X6qOE">
    <img src="https://img.youtube.com/vi/CdDkL8X6qOE/0.jpg" alt="Additional motion-platform demonstration" width="400"/>
  </a>
</div>

[Platform demo 1](https://www.youtube.com/watch?v=_NR_MUGvmUo) ·
[Platform demo 2](https://www.youtube.com/watch?v=CdDkL8X6qOE)

### Controller PCB build videos

<div align="center">
  <a href="https://www.youtube.com/watch?v=fEcdGIq_Jzc">
    <img src="https://img.youtube.com/vi/fEcdGIq_Jzc/0.jpg" alt="Original controller PCB video, part 1" width="400"/>
  </a>
  <a href="https://www.youtube.com/watch?v=1WXx59tWYc4">
    <img src="https://img.youtube.com/vi/1WXx59tWYc4/0.jpg" alt="Original controller PCB video, part 2" width="400"/>
  </a>
</div>

[PCB video — part 1](https://www.youtube.com/watch?v=fEcdGIq_Jzc) ·
[PCB video — part 2](https://www.youtube.com/watch?v=1WXx59tWYc4)

## A bit of project history

**2020–2021 — the original rig.** The repository started in January 2020 around
a full-size rotary Stewart platform, six AC servos and an ESP32 controller taking
SimTools input. Development included custom controller and magnetic-sensor PCBs,
dual-core Arduino firmware, MCP23S17 I/O expansion, Android BLE controls and a
.NET platform tester. The build and demo videos above come from this early period.

**2025 — the “Phoenix” modernization.** Work resumed with Python visualization,
an ESP32-S3 migration and, later that year, a move from Arduino to ESP-IDF. The
controller and its development tools evolved alongside the mechanical project.

**2026 — a shared simulation and hardware workbench.** A native C++ desktop app
arrived in February, bringing 3D visualization, motion cueing, recording and
software/hardware-in-the-loop workflows together. In July, the shared math was
extracted into `stewart-core`, and the desktop-scale Mini-6DOF hobby-servo firmware
joined this repository. The current September work includes the r13 controller
PCB prototype, a clearer hardware/controller/app split, and illustrated app
documentation with repeatable, hardware-disabled captures.

The project has evolved through several hardware and software generations. Old
videos, timing measurements and design files are preserved as history; use the
[current hardware status](hardware/pcb/6DOF2_BOARD.md) and
[firmware TODO](hardware/pcb/6DOF2_FIRMWARE_TODO.md) for today's implementation
and testing requirements.

## The hardware behind the project

The original full-size build described in this repository uses:

- A **31-inch-diameter, ½-inch steel base plate**.
- **Six 750 W AC servo motors**, 50:1 planetary gearboxes and couplers.
- **AASD-15A servo drives** with step/direction signaling.
- Twelve ½-inch Panhard bar kits with rod ends and high-misalignment spacers for the linkage.

These describe the original build, not a complete shopping list or a claim that
all historical controller revisions are interchangeable.

The newer **“6DOF 2” controller** is an ESP32-S3 PCB design with W5500 Ethernet,
six matching DB25 servo connectors, differential STEP/DIR outputs, conditioned
alarm inputs and a supervised stop/inhibit circuit. Its r13 revision remains an
**untested prototype**, with hardware-specific firmware work still required.

The [Mini-6DOF variant](controller/mini/README.md) uses an ESP32 and six hobby
servos for desktop-scale development. Mechanical assets live under
[hardware/mechanical/](hardware/mechanical/); the
[geometry guide](hardware/mechanical/platform_geometry.md) explains platform
dimensions and calibration.

## Desktop app

<div align="center">
  <img src="docs/images/app/overview.png" alt="Native desktop app with a simulated Stewart platform, motion-cueing controls and six-axis plots" width="1100"/>
  <br><em>Actual app screenshot using synthetic SIL data in hardware-disabled documentation mode—not r13 hardware telemetry.</em>
</div>

The native C++/OpenGL/ImGui app is both a simulator and a development workbench:

- **3D platform views** with servo-angle readouts and editable geometry.
- **Independent entities** for software-in-the-loop (SIL) experiments and hardware-in-the-loop (HIL) development.
- **Motion cueing and filtering** with per-axis controls and frequency-response views.
- **Input plugins** for Assetto Corsa, SimTools, manual controls and test signals.
- **Recording and playback**, time-series plots, spectrograms and diagnostic logs.
- **Shared math** for inverse kinematics, axis scaling and motion cueing, used by the desktop app and controller firmware.
- **MCP automation** for repeatable UI setup and genuine app screenshots, including an isolated mode that blocks hardware access.

Start with the [illustrated app guide](docs/APP_GUIDE.md). The
[app README](app/README.md) contains deeper technical detail, and
[app automation](docs/APP_AUTOMATION.md) explains the MCP interface and its
current coverage. Software tests and synthetic demonstrations are not physical
hardware acceptance tests.

## Getting started

1. **Explore the software:** follow the [build guide](docs/BUILD.md), then use the
   [app guide](docs/APP_GUIDE.md). For an explicitly hardware-disabled session,
   use the [documentation-mode workflow](docs/APP_AUTOMATION.md).
2. **Choose the right controller:** read the [firmware variant map](docs/FIRMWARE.md)
   before working with the full-size, Mini or analyzer firmware. Their outputs
   and hardware requirements differ.
3. **Review the PCB:** use the [board overview](hardware/pcb/6DOF2_BOARD.md),
   [exact RC4 files and remaining gates](hardware/pcb/6dof2_r13/r13-rc4-2026-09-23/README.md),
   and [firmware/commissioning TODO](hardware/pcb/6DOF2_FIRMWARE_TODO.md).
   Buying and bench-use guidance is indexed in [Hardware](hardware/README.md);
   the current **do-not-order** status still applies.

## Repository layout

| Start here | Contents |
| --- | --- |
| [Hardware](hardware/README.md) | `hardware/pcb/` design packages, buying/bench-use guides and `hardware/mechanical/` models |
| [Controller](controller/README.md) | Full-size firmware, Mini firmware, shared math, diagnostics and tests |
| [App](app/README.md) | Native desktop simulator/control app, MCP tools and optional Linux HIL bridge |
| [Documentation](docs/README.md) | Build instructions, illustrated app usage, firmware, architecture and automation references |

For source paths, submodule setup and build directories, see the
[build guide](docs/BUILD.md#repository-layout).

## Support the project

If you enjoy the project or find the code, designs and build videos useful,
you can support its continued development: **[Donate via PayPal](https://www.paypal.me/knaufinator)**.

Bug reports, documentation corrections and reproducible test results are welcome
too. When reporting hardware behavior, include the board revision and firmware
version so results from different generations do not get mixed together.

## License

MIT — see [LICENSE](LICENSE). Use at your own risk.
