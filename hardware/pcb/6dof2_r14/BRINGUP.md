# R14 bench qualification plan

**R14 was ordered on 26 October 2026, as confirmed by the owner.**
The [ordered package](exports/2026-10-26/README.md) is the design baseline.
All hardware acceptance steps below remain NOT RUN on R14. Record the actual
hardware/package hash, firmware image/ELF hash, instrument and setup for each result.
Keep motion admission locked until the applicable gates pass.

1. With drive cables disconnected, inspect population, polarity, pin-1 orientation,
   solder joints, plated slots, shorts and connector labels against the final BOM,
   drawings and assembly preview. Confirm the actual cable pin-to-pin wiring.
2. Use a current-limited supply. Record J9 input, VINRAW, protected field output,
   remote-collector voltage, 5 V and 3.3 V throughout startup, shutdown, brownout,
   field short and recovery. Sweep the final specified supply/cable/load envelope.
   Use nominal 12 V, maximum 12.6 V. Measure the actual 15 ft cable hot complete
   loop including contacts: ≤2.0 Ω; additional board loop ≤0.4 Ω. Verify the U42/Q11
   bypass and retained D1 startup path. Record both monitor trips (nominal falling
   11.046 V / rising 11.2 V), current at 12 V and U41-to-U11 inhibit latency.
   Verify the remote sensors remain ≥10 V in the admitted steady envelope and
   that startup/fault transients cannot admit motion. The calculated 0.257935 A
   screen is not a proven 12 V hard current ceiling.
3. Confirm auxiliary-boost regulation, over/undervoltage trips, eFuse RTN behavior,
   inrush and backfeed on power removal. Validate startup order and that a returning
   rail cannot re-arm the pulse gate or torque latches.
4. Boot the locked R14 diagnostic image. Record image/ELF hashes, reset cause and
   `R14:STATUS`. Expander initialization must clear torque requests, keep the arm
   clock low and leave the field feed off. Prove missing/stuck I²C or ADC data denies
   admission. `R14:FIELD=1` may energize only the sensor branch.
5. Check six Hall channels electrically: closed away from the magnet, open at the
   intended point, unplugged, shorted and with field power removed. Verify no open
   input at boot is accepted as home. Check each drive alarm polarity separately.
6. With an independent analyzer, measure STEP and DIR at the physical outputs.
   Verify direction setup/hold, stop/resume boundaries, maximum pulse rate, all six
   axes together, serial traffic, reconnects, reset, field loss and every alarm.
   Compare physical pad counts with engine counts before restoring motion permission.
7. Qualify servo-on polarity with one unloaded drive, then each axis individually.
   Verify a fault stops common pulses while healthy-axis torque behaves as intended.
   Test global release and emergency stop separately; do not rely on software alone.
8. Only after the electrical gates pass, implement and qualify bounded homing from
   a verified initial position. Establish count/time limits and calibrated direction
   per axis. Requalify after scaling, cable, drive parameter or pulse-engine changes.

R14 testpoint meanings must come from the final schematic/netlist: **TP7 is drive
enable and TP8 is stop** in the reviewed baseline. Do not copy the earlier R13
testpoint interpretation. GPIO47/48 ownership also differs from old DevKit LED code.

Retain dated measurements, instruments, supply settings, cable identity, drive
parameters, exact hardware package hash and firmware binary hash for every run.
See [firmware status](FIRMWARE_STATUS.md) for what the recovered R13 tests established.

## Results to record

Create a dated test report with a row per case. Use NOT RUN, PASS, FAIL or BLOCKED;
record measured values and the evidence path, rather than changing a status alone.

| Case | Acceptance evidence | R14 result |
| --- | --- | --- |
| Population/contact mapping | Exact BOM, polarity, slots, six DB25 cables, field/network jack identification | NOT RUN |
| Input and USB sequencing | Rail waveforms, U42/Q11/D1 startup/removal/reverse behavior, no unintended enable | NOT RUN |
| Sensor cable and voltage | Hot complete loop ≤2.0 Ω, board-loop allowance ≤0.4 Ω, actual 12 V current and remote voltage ≥10 V within admitted envelope | NOT RUN |
| Monitor and inhibit | Both trip voltages, hysteresis, fault transient and last-pulse latency; intentional rearm only | NOT RUN |
| Auxiliary power | Regulation, UV/OV, RTN, inrush, shutdown/backfeed and fault recovery | NOT RUN |
| I²C/ADC faults | Missing/stuck bus, warm reset, stale conversion/readback all deny admission | NOT RUN |
| Six Hall channels | Away/magnet/unplugged/open/short/power-off states and repeatability | NOT RUN |
| Six drive alarms | Polarity, own-latch clearing, common pulse stop and healthy-axis hold behavior | NOT RUN |
| Physical pulse outputs | Independent STEP count/last edge/DIR setup and hold under six-axis traffic and reconnect/reset | NOT RUN |
| Drive enable and homing | Unloaded one-axis SRV-ON checks, calibrated bounded seek, then all six axes | NOT RUN |

A later firmware/cable/scaling change must identify which accepted cases need
requalification. No loaded-platform acceptance follows automatically from a bench pass.
