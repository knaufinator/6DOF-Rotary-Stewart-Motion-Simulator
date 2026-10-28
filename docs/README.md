# Documentation

[Repository overview](../README.md) · [Controller](../controller/README.md) · [App](../app/README.md) · [Hardware](../hardware/README.md)

Controller firmware and the desktop application are separate root projects.
Shared build instructions, user guides and developer references live here.

| Guide | Purpose |
| --- | --- |
| [Build guide](BUILD.md) | Desktop and ESP-IDF prerequisites, project paths and fresh builds |
| [Firmware variants](FIRMWARE.md) | Full-size controller, mini firmware and shared math |
| [App guide](APP_GUIDE.md) | Desktop controls, entities, motion pipeline and recordings |
| [App automation](APP_AUTOMATION.md) | Repository-local MCP setup, isolated documentation mode and real UI captures |
| [Architecture](ARCHITECTURE.md) | Implemented software boundaries, data flow and shared kinematics |
| [Firmware simulation plan](FIRMWARE_SIMULATION_PLAN.md) | Proposed host integration, ESP32-S3 emulation, fault injection and physical-test boundaries |

R13 one-axis bench evidence and the recovered firmware are documented in the
[R14 firmware status](../hardware/pcb/6dof2_r14/FIRMWARE_STATUS.md). The
[current R14 release](../hardware/pcb/6dof2_r14/README.md) is ordered, with PCB and
schematic drawings, design decisions and a pending hardware-test plan. [Design philosophy](../hardware/pcb/6dof2_r14/DESIGN_PHILOSOPHY.md)
explains pulse permission, holding torque and the new hardware checks.
Board-specific build/buy/bench guidance stays with the hardware package.

Unless stated otherwise, command examples start at the repository root. The app
normally runs with `app/` as its working directory to retain its existing settings
and recordings. Documentation automation instead uses an isolated empty session;
see the automation guide. Firmware projects are `controller/`, `controller/mini/` and
`controller/test_harness/`; the shared core is `controller/stewart-core/`.
