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

**The r13 PCB is untested. DO NOT ORDER.** Documentation and successful software
builds do not establish safe drive-connected operation. Read the
[current board status](../hardware/pcb/6DOF2_BOARD.md) and
[firmware/commissioning TODO](../hardware/pcb/6DOF2_FIRMWARE_TODO.md).
Board-specific build/buy/bench guidance stays with the hardware package, not here.

Unless stated otherwise, command examples start at the repository root. The app
normally runs with `app/` as its working directory to retain its existing settings
and recordings. Documentation automation instead uses an isolated empty session;
see the automation guide. Firmware projects are `controller/`, `controller/mini/` and
`controller/test_harness/`; the shared core is `controller/stewart-core/`.
