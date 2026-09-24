# Architecture

This reference describes the implemented desktop and firmware boundaries. For
setup, see [Build](BUILD.md); for target selection, see [Firmware](FIRMWARE.md).
The [app guide](APP_GUIDE.md) documents the user interface.

## Components and entry points

| Component | Entry points and responsibility |
| --- | --- |
| Desktop application | [`app/src/main.cpp`](../app/src/main.cpp) creates the GLFW/OpenGL window and drives update/render; [`app.cpp`](../app/src/app.cpp) owns entities, input processing, HIL sessions, recordings and settings. |
| Desktop presentation | [`ui_panels.cpp`](../app/src/ui_panels.cpp) builds the Dear ImGui/ImPlot interface; [`platform_viz.cpp`](../app/src/platform_viz.cpp) renders the platform geometry. |
| Input plugins | [`plugin_api.h`](../app/src/plugin_api.h) defines the plugin interface; [`plugin_manager.cpp`](../app/src/plugin_manager.cpp) loads and runs plugins. Implementations live in [`app/plugins/`](../app/plugins/). |
| Shared math | [`controller/stewart-core/`](../controller/stewart-core/) contains inverse kinematics, axis scaling, filters and motion cueing. |
| Full-size firmware | [`controller/main/main.cpp`](../controller/main/main.cpp), `app_main()`: ESP-IDF application, commands, input selection, step targets and telemetry. |
| Mini firmware | [`controller/mini/main/main.cpp`](../controller/mini/main/main.cpp), `app_main()` and `CueTask()`: hobby-servo output, live input and embedded playback. |
| Signal analyzer | [`controller/test_harness/main/main.c`](../controller/test_harness/main/main.c): a separate ESP-IDF application for measuring step/direction signals. |
| Optional network bridge | [`app/bridge/`](../app/bridge/): a separate process relaying desktop motion/control traffic to a serial-connected device. |

The desktop compiles the three shared math implementations into `stewart_math`
in [`app/CMakeLists.txt`](../app/CMakeLists.txt). Both controller projects consume
the same submodule as an ESP-IDF component through `EXTRA_COMPONENT_DIRS`.
Hardware drivers, transport handlers, commands and persistence remain in their
respective applications; sharing math does not make firmware variants equivalent.

## Desktop execution and data flow

`main.cpp::doFrame()` calls `App::update()`, builds the UI and renders it, then
drains control requests before swapping buffers. Processing in `App::update()`
follows the frame cadence; it is not a fixed-rate real-time control scheduler.
Filter coefficients are adjusted for the observed application sample rate.

Input plugins, manual controls and recording playback feed a six-channel input
bus. Each enabled entity has its own geometry, cueing configuration and state:

```text
Input percentages -> optional input LP/notch -> optional MCA washout/tilt
                  -> intensity, per-axis gain/invert -> output percentages
SIL: output percentages -> physical pose -> generalized IK -> plots/3D view
HIL baked: output percentages -> integer motion packet -> device IK/output
HIL raw: input percentages -> float motion packet -> device cueing/IK/output
```

SIL means software-in-the-loop: the desktop computes actuator angles locally.
HIL means hardware-in-the-loop: displayed output angles come from device
telemetry. A connected HIL entity must complete its geometry/scaling/bit-depth
handshake before the motion transmitter uses it. Raw streaming also requires
the device's `caps=raw` capability and the entity's raw-mode setting.

[`transport.h`](../app/src/transport.h) defines `ITransport` for direct serial
and network HIL. [`serial_port.cpp`](../app/src/serial_port.cpp) implements COBS
framing over serial; [`udp_transport.cpp`](../app/src/udp_transport.cpp) sends
motion over UDP and control over TCP through the bridge. These are distinct
from the controller's direct Ethernet/WiFi motion listeners.

The application uses background HIL transmission and transport worker threads,
mutexes and queues. Thread count depends on active transports and connections.
The optional [`control_server.cpp`](../app/src/control_server.cpp) endpoint
accepts newline-delimited JSON on localhost TCP; it queues commands for execution
on the render thread. [`app/tools/stewart_mcp.py`](../app/tools/stewart_mcp.py)
exposes that endpoint through MCP. Screenshots read the actual OpenGL framebuffer.

## Firmware output paths

The full-size COBS path decodes `CH_DATA18` as six little-endian 24-bit values
with 18 usable bits. `process_binary_packet()` maps those already-cued counts
to a physical pose; `applyMotionValues()` applies position slew while its
rate-limiting state is active, and
`setPos()` runs compact IK and converts angles to motor step targets. This path
does not apply motion cueing again on the device.

[`controller/main/CMakeLists.txt`](../controller/main/CMakeLists.txt) selects
PCBv1 for ESP32 and PCBv2 for ESP32-S3. PCBv1 emits step/direction through the
MCP23S17 expander. PCBv2 uses the [`StepDriver`](../controller/include/StepDriver.h)
interface; its selected backend is `STEP_DRIVER_SHARED_MCPWM`. These build names
identify firmware configurations, not acceptance of a particular PCB revision.

Mini producers publish their latest target; `CueTask()` is the runtime servo
writer. Baked counts bypass cueing. Raw `CH_DATA_RAW` packets contain six
little-endian float32 percentages and run input filtering, MCA and output gain
on the device before conversion to pose. `driveServos()` applies position slew,
compact IK, angle limiting and LEDC PWM updates. Mini also implements source
selection, embedded playback and stale-input handling. Those behaviors must not
be inferred for the full-size application. See the [controller reference](../controller/README.md).

## Coordinates, geometry and shared math

| Boundary | Representation |
| --- | --- |
| Desktop input and raw HIL | Signed percentages in surge, sway, heave, roll, pitch, yaw order. |
| IK pose | `[x, y, z, roll, pitch, yaw]`; translation in millimeters, rotation in radians. `z` is displacement from home height. |
| Axis scale configuration | Millimeters for translation, degrees for rotation; conversion to radians occurs before IK. |
| Compact `StewartConfig` | Lengths in millimeters; `theta_r`, `theta_p` and `theta_s[]` in degrees. |
| Generalized `PlatformDef` | Per-actuator base/platform joint coordinates and arm/rod lengths in millimeters; `beta` and servo limits in radians. |
| IK output | Servo angles in radians; full-size firmware converts through degrees and `steps_per_degree` to step targets. |

The desktop swaps surge/sway into IK X/Y order before SIL IK and before encoding
baked HIL packets. Raw HIL retains app order; Mini swaps those channels after
cueing. Preserve this boundary when adding sources or transport formats.

[`InverseKinematics.cpp`](../controller/stewart-core/src/InverseKinematics.cpp)
contains two implementations: desktop SIL calls `calcAllActuatorAngles()` with
`PlatformDef`; both current firmware actuator paths call
`calculateAllServoAngles()` with compact `StewartConfig`. The generalized solver
rotates platform joints using `Rz(yaw) * Ry(pitch) * Rx(roll)`, adds translation
and home height, and solves each arm angle from the resulting leg vector.
Sharing the source library does not establish equivalence between these solvers.

`buildPlatformFromConfig()` expands the symmetric three-pair configuration.
Its current implementation uses `RD/theta_r` for platform joint positions and
`PD/theta_p` for base joint positions. Some UI labels describe the radii the other
way around; check this mapping when changing geometry or importing measurements.
`computeHomeHeight()` averages the heights implied by horizontal arms using
the XY joint offsets. It does not validate a general asymmetric/non-planar build.

[`AxisScaling.cpp`](../controller/stewart-core/src/AxisScaling.cpp) probes positive
single-axis motion from home using compact IK, takes the smallest translation
and rotation limits within each group, applies the requested margin and rounds
down. These are input scaling estimates, not a validated combined-pose workspace.
IK clamps its inverse-sine argument and output angles; `validatePositionV2()`
flags angles at the configured limits. Neither is a collision, load, singularity
or complete reachability check.

## Runtime and hardware safety boundary

Software stop controls, command gating, numerical limits and telemetry belong to
the control implementation. They do not replace independent physical stopping
and drive-enable systems. The r13 PCB/firmware combination remains **untested**;
build success, SIL output and earlier signal captures do not authorize powered
platform operation. Consult the current status in the [controller reference](../controller/README.md).

For isolated software work, set `STEWART_DOCUMENTATION_MODE=1` before process
launch. [`automation.cpp`](../app/src/automation.cpp) caches this policy, which
guards serial enumeration/opening, network HIL initialization, external plugin
loading and the HIL transmit thread. The app can render synthetic SIL data and
disconnected HIL views in this mode; those images are not device measurements.
