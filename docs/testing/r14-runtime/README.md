# R14 runtime integration and ESP32-S3 boot probe

Implemented 28 October 2026. This is the first milestone of the [simulation plan](../../FIRMWARE_SIMULATION_PLAN.md). Production firmware source, the recovered R13 pulse engine, SDK options and R14 motion admission are unchanged.

## Production runtime exercised on the host

`controller/tests` now builds `r14-runtime-scenarios` from the actual `R14Hardware.cpp` and `CobsTransport.cpp`, rather than a rewritten controller. SDK headers are substituted **only for this executable**. The tests drive encoded/fragmented console input through the production stream decoder, frame dispatcher, R14 diagnostic handler, atomic field-request queue, actual monitor task, actual I²C adapter and `BoardIo`, then decode production COBS response frames.

The monitor runs on a host thread, paused at each `vTaskDelay` rendezvous. A test advances virtual time by the requested delay; modeled failed I²C transfers consume their configured 3 ms timeout. This gives reproducible ordering, not target scheduling or physical response-time evidence. MCP23017 BANK recovery, direction/latch/readback behavior and driven arm-clock edges are modeled. The INA226 model provides its checked IDs, voltage conversion and conversion-ready behavior for the configuration actually used. Unsupported register/configuration accesses fail the model instead of acknowledging arbitrary success.

| Scenario | Checked behavior |
| --- | --- |
| Cold boot | Safe latches before output direction, actual input decoding and 5 V voltage conversion; field feed can switch without an arm edge |
| Retained outputs, BANK=0 | MCU-only reset clears retained field/arm/clear states without a new arm edge |
| Retained outputs, BANK=1 | Actual IOCON recovery reaches the same safe initialization |
| Bus failure after feed request | Invalid health, one fault callback, direct alarm callback despite I²C failure; healthy reads do not clear the latched callback history |
| Bus unavailable at startup | Initialization fails and is not retried into automatic health |
| Missing INA226 | Device identity/read failure blocks monitor health |
| Remote/unknown requests | Ethernet/WiFi/BLE cannot issue local feed requests; arm/home remain rejected by the R14 handler |
| Six alarms while the bus fails | Every direct GPIO alarm reaches its callback, independently of failed I²C transfers |
| Stale snapshot | Status reports stale data when virtual sample age exceeds the existing 50 ms threshold |
| Malformed COBS input | A truncated frame is discarded and the following valid status command resynchronizes |

All ten scenarios pass locally under MSVC, alongside the existing 85 C++ unit tests. CTest contains eleven entries: the unit-suite executable plus ten runtime scenarios. CI runs these on Linux and Windows and uploads JSON event traces. Each scenario uses a fresh process so production file-scope state is not reset by hidden test hooks.

The test-only SDK substitutes GPIO, I²C, console USB/VFS, synchronization and scheduling. It also supplies the response/fault callbacks normally implemented in `main.cpp`. Therefore this milestone does **not** exercise the entire `process_data()` routing, `main.cpp` stop side effects, the real desktop transport, OS USB/DTR/RTS, FreeRTOS scheduling, emitted MCPWM edges or real electrical gates. Its `motion_qualified=0` response is the production diagnostic text; it is not an independent pulse measurement. Existing source-contract and admission tests continue to cover the production motion lock.

## Negative controls

`tools/check_r14_runtime_sensitivity.py` creates and compiles isolated copies of the production monitor with two intentional defects: suppress the I/O-fault callback, or remove the command-source restriction. The corresponding integration cases must exit unsuccessfully for the intended reason. Both controls passed locally; [receipt](2026-10-28/sensitivity.json). Source in `controller/main` is never rewritten by this experiment. CI repeats the experiment and uploads the results.

## Unchanged-image QEMU experiment

The [GitHub CI artifact from run 40730790850](https://github.com/knaufinator/6DOF-Rotary-Stewart-Motion-Simulator/actions/runs/40730790850), associated with PR head `cc79d9e`, was assembled into a private 8 MB flash file without changing its bootloader, partition table or application bytes. The artifact names contain the workflow's synthetic PR merge-build ID `6422d64`; that is distinct from the PR head. Input binary/ELF hashes are retained in the [probe receipt](2026-10-28/probe.json).

Espressif QEMU `esp_develop_9.2.2_20250817` / 9.2.2 ran the image for a bounded 15-second attempt with no host network backend, no physical serial port and no real GPIO. A loopback-only GDB connection captured the target stack. The application reached `app_main()`; the second CPU was in the FreeRTOS idle path. The observed application stack was:

```text
app_main → ethernet_transport_init → esp_eth_driver_install
         → emac_w5500_init → w5500_reset → eth_spi_write
         → spi_device_polling_transmit → spi_device_polling_end
         → spi_hal_usr_is_done
```

The probe ended at its host timeout. This identifies an incomplete emulated W5500/SPI path in the observed environment; it does not qualify a complete startup or demonstrate that the real PCB fails. No W5500 substitute, OpenCores network driver or firmware bypass was introduced. The [GDB snapshot](2026-10-28/gdb-stop.txt), [ROM console](2026-10-28/console.log) and [emulator stderr](2026-10-28/emulator-stderr.log) are retained. The receipt reports an attempted boot, not a simulation pass.

| Interface | Evidence in this milestone | Remaining coverage |
| --- | --- | --- |
| CPU, flash, FreeRTOS, application entry | Target stack reaches `app_main()` and a FreeRTOS idle task | Boot completion, long-run stability and scheduler fidelity |
| W5500 / general-purpose SPI | Pending polling transfer during W5500 reset in captured stack | Working emulated device/backend; physical SPI/IRQ behavior |
| Native USB Serial/JTAG | Host COBS tests use an explicit console substitute; QEMU ROM logs are captured on UART | Real USB behavior and application-level QEMU console round trip |
| MCP23017 / INA226 | Production bus/monitor code runs against declared host device models | Real I²C and complete target-side emulation |
| Alarm GPIO | Production polling and dispatch exercised through host GPIO substitute | Electrical polarity/latency and target-side I/O |
| MCPWM STEP/DIR | Existing logic tests retained; pulse engine bytes untouched | Peripheral model, independent edge captures and physical timing |
| Desktop transport | Not part of this delivery | Planned headless app/controller round trip |

## Reproduce

From a checkout with the shared-core submodule initialized:

```sh
cmake -S controller/tests -B build-runtime -DCMAKE_BUILD_TYPE=Release
cmake --build build-runtime --config Release
ctest --test-dir build-runtime -C Release --output-on-failure --no-tests=error
python tools/check_r14_runtime_sensitivity.py --output /absolute/temporary/runtime-mutants
```

Use a writable, separate temporary directory for generated mutants/builds. Traces are written to the normal CMake build directory; they contain virtual timestamps and callback/bus events. SDK replacements cannot be included by the production ESP-IDF project because only the host test target adds that include directory.

The optional target boot probe requires the original CI binary/ELF artifacts and the ESP-IDF-pinned QEMU/GDB installation:

```sh
python tools/probe_r14_qemu.py --qemu /path/to/qemu-system-xtensa \
  --artifact-directory /path/to/downloaded-firmware-artifact \
  --source-commit cc79d9eb246237e6fdfa06c1074d0eef5fc2dc45 \
  --gdb /path/to/xtensa-esp32s3-elf-gdb \
  --output /absolute/temporary/qemu-probe --seconds 15
```

This recording utility deliberately does not turn a timeout or expected unsupported-device failure into a functional qualification pass. It is not a required hosted CI acceptance check. A later emulator-specific build must declare every adapter and its different image hash; the [remaining plan](../../FIRMWARE_SIMULATION_PLAN.md) still applies.
