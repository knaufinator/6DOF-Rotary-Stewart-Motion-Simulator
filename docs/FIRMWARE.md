# Unified controller firmware

One ESP-IDF project at [`controller/`](../controller/CMakeLists.txt) builds every controller profile. Select the hardware at configure time; there is one application entry point, shared console/COBS and BLE implementations, shared handshake formatting, geometry setters and the single `stewart-core` math submodule. Each profile produces a separate flashable image for its chip and wiring.

**2 November 2026:** R14C2 is ordered and awaiting physical testing. R13 single-axis evidence and its pulse engine are retained. The R14 profile keeps normal motion locked. See [current hardware status](../hardware/pcb/6dof2_r14/README.md) and [firmware evidence](../hardware/pcb/6dof2_r14/FIRMWARE_STATUS.md).

| Profile | Chip | Outputs and behavior | Build from `controller/` |
| --- | --- | --- | --- |
| `r14` (default) | ESP32-S3 | Shared MCPWM STEP/DIR, W5500, MCP23017/INA226 diagnostics; motion locked | `idf.py -B build-r14 -D FIRMWARE_PROFILE=r14 build` |
| `r13-bench` | ESP32-S3 | Retained console-only one-axis bench mode; six-axis gate locked; no R14 board I/O | `idf.py -B build-r13-bench -D FIRMWARE_PROFILE=r13-bench build` |
| `mini` | ESP32 | LEDC hobby servos, analog 50 Hz or configured digital rate, BLE accelerometer, raw cueing and embedded playback | `idf.py -B build-mini -D FIRMWARE_PROFILE=mini build` |
| `legacy` | ESP32 | Original MCP23S17 STEP/DIR board and its existing behavior | `idf.py -B build-legacy -D FIRMWARE_PROFILE=legacy build` |

Use ESP-IDF 5.5.2 and initialize `git submodule update --init --recursive`. Each build directory owns its SDK configuration. A chip/profile mismatch, conflicting safety options or reuse of a build directory for another profile fails configuration. These commands choose the chip through the profile; separate `set-target` commands are unnecessary. The bench profile remains prohibited in CI/release builds. All binaries are named `stewart_platform.bin` inside their own build directory; CI retains the existing external Mini/main artifact names.

`PROFILE?` reports the compiled profile and actuator. There is no command to switch hardware profiles at runtime. Use the corresponding build directory with ESP-IDF for any later flash operation; this refactor does not flash devices or migrate their saved settings.

## Shared implementation and hardware policies

[`Application.cpp`](../controller/main/Application.cpp) is the only `app_main()` and the common command router. It immediately enters the selected hardware runtime, preserving R14's inhibit setup as the first hardware action. [`StepDirRuntime.cpp`](../controller/main/StepDirRuntime.cpp) owns STEP/DIR scheduling, drive alarms and board safety; [`MiniRuntime.cpp`](../controller/main/MiniRuntime.cpp) owns PWM scheduling, servo calibration, source transitions and playback. Exactly one runtime is linked into each image. Shared code lives in `controller/main/` and `controller/include/`; there are no separate Mini transport/header copies or second Mini ESP-IDF project.

The hardware policies deliberately retain their existing behavior. Mini's `DEMO` boot default, 50/250 Hz PWM choices, servo centers, stale-target return and NVS namespace/partition offsets remain Mini-only. The full controller retains its drive gearing, alarm/stop path and R14 admission lock. Sharing code does not grant every profile every capability. [Mini hardware and commands](../controller/mini/README.md) and [STEP/DIR hardware](../controller/README.md) remain the profile guides.

## Protocol compatibility

| Contract | Mini | STEP/DIR profiles |
| --- | --- | --- |
| Baked serial motion | Channel `0x01`, 12-byte six-uint16 payload | Channel `0x06`, 18-byte six-uint24 payload; sequenced `0x08` preserved |
| Raw telemetry input | Channel `0x07`, 24-byte six-float payload | Not admitted |
| Handshake | Existing protocol 1, `platform=mini-6dof,caps=raw` | Existing protocol 2 fingerprint and sequence statistics |
| Configuration storage | Existing Mini servo/geometry/source keys and partition offsets | Existing geometry/drive keys |

The shared transport dispatches only the selected profile's motion formats. This corrects older prose that implied Mini used DATA18. Command/source identity, framing and profile identity have host regressions, alongside the existing R14 board/plant/composition lanes. The analyzer at [`controller/test_harness/`](../controller/test_harness/README.md) remains an independent measuring instrument, not a motion-controller profile.

## Qualification

The [unification verification record](testing/firmware-unification/README.md) describes the profile/packet regressions, native builds and preserved-source hashes.

The recovered R13 pulse-engine source and `stewart-core` revision are unchanged. Build and host checks cannot qualify physical timing, Hall wiring, stopping latency or a loaded platform. R14 remains `MotionAdmission<false>`. Follow [R14 bring-up](../hardware/pcb/6dof2_r14/BRINGUP.md); recheck the changed console, handshake and BLE paths on hardware before relying on the new image. Frozen ordered PCB packages are unchanged.
