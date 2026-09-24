# Hardware

[Repository overview](../README.md) · [Controller](../controller/README.md) · [App](../app/README.md) · [Docs](../docs/README.md)

**r13 RC4 is an untested PCB prototype. DO NOT ORDER.** Nothing has been ordered
or paid for. Vendor allocation, stackup and complete assembly approval remain
pending; firmware corrections and physical tests are required before connecting
drives. A clean DRC result and complete file package do not establish a working
motion controller.

| Location | Contents |
| --- | --- |
| [PCB overview](pcb/6DOF2_BOARD.md) | Current revision, interfaces, power/stop contract and test gates |
| [r13 handoff](pcb/6dof2_r13/README.md) | Current package index and prototype instructions |
| [RC4 release](pcb/6dof2_r13/r13-rc4-2026-09-23/README.md) | Exact frozen manufacturing files, source, drawings and evidence |
| [Mechanical models](mechanical/) | Platform and mini printed-part assets moved from `cad/` |
| [Platform geometry](mechanical/platform_geometry.md) | Dimensions, motor layout and calibration derivation |

## Current PCB package

Read the [release status](pcb/6dof2_r13/r13-rc4-2026-09-23/STATUS.md) before using
these files. For a future authorized quotation, the three JLC inputs are separate:

| Input | File |
| --- | --- |
| PCB fabrication | [Gerber/drill ZIP](pcb/6dof2_r13/r13-rc4-2026-09-23/fabrication/6DOF2_r13_Gerber.zip) |
| Assembly population | [Normalized JLC BOM](pcb/6dof2_r13/r13-rc4-2026-09-23/assembly/6DOF2_r13_BOM_JLC.csv) |
| Assembly placement | [Normalized JLC CPL](pcb/6dof2_r13/r13-rc4-2026-09-23/assembly/6DOF2_r13_CPL_JLC.csv) |

The release also contains [editable source](pcb/6dof2_r13/r13-rc4-2026-09-23/source/),
[drawings](pcb/6dof2_r13/r13-rc4-2026-09-23/drawings/),
[STEP/mechanical review](pcb/6dof2_r13/r13-rc4-2026-09-23/mechanical/) and
[audit evidence](pcb/6dof2_r13/r13-rc4-2026-09-23/evidence/).
Use its README, manifest and SHA-256 sums as the file authority. Do not mix
revisions or use the complete project archive as the fabrication upload.

## Prototype preparation

- [How to buy](pcb/6dof2_r13/HOW_TO_BUY.md): full SMT/THT assembly and processed-order checks.
- [How to use](pcb/6dof2_r13/HOW_TO_USE.md): disconnected-drive bench preparation.
- [Firmware and commissioning TODO](pcb/6DOF2_FIRMWARE_TODO.md): required corrections and acceptance tests.
- [Single-axis test gate](pcb/single_motor_test_plan.md): prerequisites for controlled first motion.

Earlier r13 checkpoints, [r12](pcb/6dof2_r12/) and [legacy experiments](pcb/legacy/README.md)
are preserved as history. **Do not reorder r12.** Frozen packages and their
internal paths remain unchanged; current navigation lives outside those packages.
