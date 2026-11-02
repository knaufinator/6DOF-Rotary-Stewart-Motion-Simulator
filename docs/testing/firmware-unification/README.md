# Unified firmware project — 2 November 2026

The full-size and Mini controllers now build from one ESP-IDF project at `controller/`. A shared application entry and command router select exactly one hardware runtime at configure time. Console/COBS, BLE, identity formatting, geometry setters, helpers and the shared math have one implementation. The former Mini project and duplicate transport/header copies are removed; `controller/mini/README.md` remains the hardware/command guide.

Hardware runtime policies remain separate modules within that application: STEP/DIR owns the pulse engine, drive alarms and safety sequencing; Mini owns PWM scheduling, servo calibration and playback. This consolidation preserves those behaviors rather than granting Mini features to the AC-drive profiles. Mini's automatic demo startup cannot be linked into R14. See the [profile/build guide](../../FIRMWARE.md).

## Checks

| Evidence | Result and scope |
| --- | --- |
| Controller CTest | 62 entries pass locally: the existing unit, board-runtime, plant and composed-safety lanes plus two new real shared-protocol/profile executables |
| Profile configuration | Three Python test methods exercise all four profiles, wrong chips, unknown names, unsafe flag combinations and rejection of a reused hardware build directory |
| Existing source contracts | Four E-stop and six R14 source checks pass against the relocated STEP/DIR runtime; guard assertions were retained |
| Native ESP-IDF builds | R14, Mini and local-only R13 bench images compile and link; source/flag/image binding checks pass |
| CI coverage | Linux/Windows host tests and sensitivity controls; native R14, Mini and legacy profile builds; actual linked-source and safety-flag verification before artifact packaging |
| Documentation | Owned README local targets and anchors pass |
| Preserved source | [Recorded SHA-256 comparisons](SOURCE_PRESERVATION.json) retain the pulse engine, admission/quiescence/seek guards, board adapter, bench code, Mini partition table and embedded sequence; `stewart-core` remains at `118e6b0965cee674994bd66d7b9fb7f07c7420e8` |

The shared-protocol executables compile the actual `Application.cpp`, `CobsTransport.cpp` and shared handshake/geometry code against narrow SDK substitutes. They check the selected startup hook, fragmented stream decoding/recovery, exact existing fingerprints, source tags, sequence accounting, profile identity, geometry updates and rejection of other-profile motion formats. Oversized command frames are now rejected whole instead of truncated into a different command. Mini keeps protocol 1 / 12-byte baked / 24-byte raw; STEP/DIR keeps protocol 2 / DATA18 / DATA18_SEQ.

The build-binding tool inspects ESP-IDF's actual main-component source inventory and compile commands, then checks emitted BIN/ELF/bootloader/partition files. It rejects a Mini image containing STEP/DIR or R14 board I/O and an R14 image missing its lock/monitor flags. Separate per-chip dependency locks and per-build SDK configurations prevent Mini/S3 builds from overwriting one another's target settings.

The new legacy native-build lane exposed inherited references to S3-only tick/soft-limit and RGB-status diagnostics. Those commands and NVS tick restoration are now gated by the actual backend. Legacy explicitly rejects unsupported `TICKRATE`/`LED` commands, does not initialize nonexistent GPIO48, and does not report an S3 software soft limit. R14/R13 behavior is unchanged; GPIO48 remains reserved for the arm latch and is never initialized as an LED.

## Physical boundary

No device was flashed, no serial connection was opened, and no new physical motor or Hall test was performed. A newly linked binary is not the previously tested R13 image. Pulse-engine source preservation and passing host models do not establish electrical timing or physical acceptance of this refactor. Recheck console/handshake/BLE behavior on each hardware profile before relying on a new image.

R14 remains `MotionAdmission<false>` with its early GPIO inhibit, independent hardware clear path and diagnostic-only field control. Its ordered PCB packages remain unchanged. Physical qualification follows [R14 bring-up](../../../hardware/pcb/6dof2_r14/BRINGUP.md). The independent signal-analyzer firmware remains a separate measuring instrument.
