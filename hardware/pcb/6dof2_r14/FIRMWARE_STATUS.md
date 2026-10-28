# R14 firmware and R13 evidence

Updated 2026-10-26. This documents source and build evidence; no R14 image was
flashed and no new motor test was performed during repository preparation.

## Provenance

The laptop capture is `DESKTOP-9LF81EF-20261025T2000`, based on commit
`7cc29d43fafb2de6d7da733621495c489b70ed64`, branch `r13-bench-one-axis`, with
uncommitted tested source retained. Receiver verification checked 352 file hashes,
the source ZIP CRCs and three Git bundles. The shared core was clean at
`118e6b0965cee674994bd66d7b9fb7f07c7420e8`.

The retained final binary SHA-256 is
`cc4f5eba9003e93ab9ab4efafaf1e90a2118d54fbf8b02334be1cddb343a6089`.
Its embedded ELF digest agrees with the retained ELF. Six final verification legs
and a five-minute wave test followed the final cleanup/flash. Earlier hour-long
tests used a binary that was subsequently overwritten; those runs must not be
attributed byte-for-byte to the final image.

Ten verified source/host files were integrated, with before/after hashes retained
in the preparation evidence. Their exact recovered bytes are also retained in the
[repository evidence](review/r13-recovered-source/README.md). The complete capture stays outside Git at
`G:/My Drive/projects/6dof/firmware/handoffs/2026-10-25-r13-tested-baseline/`.
This machine-specific location is provenance, not a portable documentation link.

## Current implementation

| Feature | State and remaining evidence |
| --- | --- |
| Shared MCPWM engine | Recovered R13 timing/accounting and DIR guard integrated; only axis 0 physically demonstrated in the retained test evidence |
| Drive scaling | Default electronic gear 2:1 restored in the main application; finite/range validation added; shared-core generic defaults preserved |
| Stop/resume | New quiescence protocol drains accounted periods before stopping the timer; three quiet periods on resume; new code requires pad-level validation |
| Native USB tools | DTR asserted, RTS deasserted; bench GPIO47/48 readback retained; PCBv2 does not initialize the old GPIO48 RGB LED |
| MCP23017 | New register adapter sets torque-clear/clock/feed outputs to inactive before directions; verifies readback; GPA7/GPB7 used only as outputs |
| Hall inputs | GPA0–5 decoded NC/closed away from the magnet; an open signal is not accepted as proof of home at boot |
| Field diagnostics | GPA6 reads FIELD_VGOOD; GPB4 reads active-low field eFuse fault; these are distinct from auxiliary-boost health |
| Driver 5 V | INA226 bus-voltage conversion and freshness checked; this wiring does not measure current |
| Drive alarms | Direct MCU inputs GPIO1/2/15/16/18/40 sampled independently of I²C; new alarms request common pulse stop |
| Operator request | Explicit software request replaces obsolete physical hold-to-run admission; health, calibration and bounded seek remain required |
| Normal motion/homing | **Not physically qualified.** Default R14 build keeps motion admission locked; diagnostics cannot arm torque or start homing |

## Build variants

Use separate build directories and SDKCONFIG files. ESP-IDF 5.5.2 is the checked
baseline. Do not reuse a bench build directory for the R14 image.

```text
# Run from controller/ in an ESP-IDF environment; configure esp32s3 first.
idf.py -B build-r14 -D R13_BENCH_ONE_AXIS=OFF -D R14_BOARD_IO=ON -D R14_FAIL_CLOSED=ON build
idf.py -B build-r13-bench -D R13_BENCH_ONE_AXIS=ON -D R14_BOARD_IO=OFF -D R14_FAIL_CLOSED=ON build
```

`R14_BOARD_IO` and `R13_BENCH_ONE_AXIS` cannot be enabled together. The R14 adapter
requires `R14_FAIL_CLOSED=ON`. Separate bench tooling is not a production admission
path. Local USB diagnostics are `R14:STATUS` and `R14:FIELD=0|1`; the latter only
queues sensor-feed control while torque remains clear and pulse admission locked.

## Acceptance boundary

Host tests exercise decoded inputs, stale/missing conversions, warm expander state,
seek limits, explicit requests, scale rejection and stop-state transitions. ESP-IDF
builds check compilation and linking. They cannot establish interrupt timing,
pin-level pulse counts, cable dropout behavior, simultaneous-axis timing, motor
direction, mechanical bottom position, SRV-ON behavior or a loaded platform's safety.
Follow [BRINGUP.md](BRINGUP.md) and retain exact image hashes with each physical run.

## Repository verification retained with this revision

The integrated source passed 85 host C++ tests, 10 bench-host Python tests and
9 source-contract checks. Separate R14 and recovered-R13 ESP-IDF builds passed.
The Windows MSVC Release application and documentation-safety build passed;
`documentation_transport_guards` passed. Native ESP32-S3 USB Serial/JTAG
VID303A/PID1001 uses DTR enabled and RTS disabled; legacy USB-UART policy is retained.
No actual serial port was opened by that app verification.

The [corrected hardware review](review/2026-10-26-10V/README.md) is now complete
for static CAD/CAM checks. Its U41 pulse-clear path is hardware-enforced; that does
not qualify subsequent MCPWM/ISR or adapter changes against the older R13 soak.
The normal R14 build still reports `motion_qualified=0` and denies motion admission.

## Ordered hardware checkpoint

The owner confirmed R14 ordered on 26 October 2026. The
[order record](ORDER_RECORD.json) binds the current manufacturing package and
this firmware source checkpoint. Physical R14 testing is pending; normal motion
admission remains locked. See [BRINGUP.md](BRINGUP.md) for the acceptance record.
