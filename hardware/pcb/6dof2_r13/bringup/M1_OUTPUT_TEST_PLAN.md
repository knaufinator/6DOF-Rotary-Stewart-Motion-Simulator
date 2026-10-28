# R13 motor-1 bench test sequence

2026-10-11, revision 2. One r13 first article, one unloaded AASD drive and motor, bench supply,
multimeter, two-channel oscilloscope. Follows the one-motor handoff
(`docs/R13_ONE_MOTOR_SIM_HANDOFF_20261011.md`, branch `r13-single-motor-sim-20261011`,
commit 6ce5d7e). No reply to that handoff had arrived when this was written.

Everything here is recorded in `BRINGUP_LOG.md` as it happens: readings, scope captures,
part numbers, what was shorted and how. A step without a recorded reading is not done.

## How to read this document

Each step has the same shape: **Purpose** (what question it answers and why that question
comes before the next one), **Setup**, **Do**, **Measure**, **Pass**, and **If it fails**.
Steps are ordered so that every failure has only one new variable to blame. Skipping a step
removes that property.

Axis naming: the board, the probe firmware and the controller count axes M0..M5.
"Motor 1" on the bench is **M0**: STEP GPIO4 / DIR GPIO10, nets `M0S_G` / `M0D_G`.

## Why the motor did not spin: four gates in series

| Gate | What blocks it | Released in |
|---|---|---|
| Controller firmware | `R14_FAIL_CLOSED` is ON by default (`controller/main/CMakeLists.txt`). The step timer never starts, STEP pins are held low, `ESTOP:RESET` is refused, and nothing in the controller arms GPIO47/48. By design. | Steps 0-11 use the probe firmware instead. Controller bench mode is step 12. |
| Hardware output latch U11 | AM26LS31 pin 4 (G) is driven by `DRIVE_ENABLE` = U11 Q. Q sets only on a rising edge of GPIO48 with GPIO47 low, the J2 loop closed, and supervisor U13 seeing `VCC5` above about 4.96 V. `VCC5` comes only from the 12 V buck, so **USB power alone can never put STEP on a DB25**. Disabled drivers are high-impedance. | Steps 2-4. |
| Drive parameters | r13 leaves DB25 pin 6 (SRV-ON) unconnected, so the drive must self-enable: `Pn003=1`, `Pn096=0` (pulse + direction), `Pn033=0` (no power-on origin return). | Step 8. |
| Cable | DB25 3/14 STEP+/-, 4/5 DIR+/-, 10 COM, 23 ALARM. Untested. | Step 6. |

## Standing rules

- Motor unloaded, no arm, drive and motor clamped so the shaft cannot touch anything.
- 12 V bench supply current-limited to 0.6 A for the board alone (steps 2-7). Drive power
  is a separate supply and stays off until step 8.
- DB25 pin 23 (ALARM) does not reach the drive until step 7 has passed. The reason is in
  step 2c and step 7.
- Every powered-motion step starts with a bounded pulse count (`pulse`), never a tone.
  The operator keeps one hand on the J2 link.
- The probe firmware never arms by itself; only the typed `arm` command pulses GPIO48.

## Probe points and expected values

All points are numbered TP1-TP14 in `TEST_POINTS.md`, with the circles on the schematic
pages in `testpoints/schem_pNN_testpoints.png` and on the board in
`testpoints/pcb_testpoints.png`. Easy-access points only: screw terminals, header pins,
DB25 pins, electrolytic leads, SOIC/SOT pins. Summary:

| TP | Net | Primary point | Expect |
|---|---|---|---|
| TP1 | GND | J9 pin 2 (12V IN "GND"), also J2.2, J1.1, DB25 pin 10 | meter black lead |
| TP2 | VIN_J | J9 pin 1 (12V IN "+") | 12.0 V |
| TP3 | VINRAW | C1 positive lead | VIN_J minus about 0.3 V. Decides the alarm hold (step 2c) |
| TP4 | VCC5 | C2 positive lead | USB only 0 V, D4 dark. 12 V on: 4.96-5.1 V, D4 lit |
| TP5 | LOGIC_5V | U7 pin 3 | USB only 4.4-4.8 V. 12 V on: about 5.0 V |
| TP6 | VCC3V3 | U7 pin 4 (tab) | 3.30 +-0.1 V |
| TP7 | ESTOP | J2 pin 1 (E-STOP terminal) | J2 open 3.3 V, shorted 0 V |
| TP8 | DRIVE_ENABLE | U3 pin 4 (alt R63) | 0 V disarmed, 3.3 V armed. No MCU readback |
| TP9 | STEP0 logic | R46 pin 1 | 3.3 V pulses |
| TP10 | M0 STEP +/- | J3 pins 3 / 14 | armed: complementary 0 / about 3.4 V; disarmed: float |
| TP11 | M0 DIR +/- | J3 pins 4 / 5 | `dir 0 1`: 4 high, 5 low |
| TP12 | M0 ALARM_DRV | J3 pin 23 | jumper to pin 10 = emulated alarm |
| TP13 | M0_ALARM_LED | U9 pin 1 | (TP3 - TP13) / R30 = LED current |
| TP14 | M0_ALARM (MCU) | U9 pin 16 | 3.3 V idle, under 0.4 V with the opto conducting |

GPIO47/48/21 need no meter: the probe firmware reads them back (`probe.py cmd in`).

Scope: channels on J3 pins 3 and 14, ground on J3 pin 10, 1 V/div, 200 us/div for a 1 kHz
tone. Meter: DC volts, continuity, and DC volts across R30 via TP3 and TP13.

---

## Step 0 - build and flash the probe firmware

**Purpose.** Separate "cannot flash" from "flashed but dead". Nothing else can be tested
until a known-good image runs, and the probe image is the one with bench commands
(`inhibit`, `arm`, `pulse`, `tone`, `mon`, `in`). The flashing trouble reported at the
bench matches the handoff's own note that the IDF Python environment "is missing at
idf5.5_py3.12_env": the IDF `export.ps1` script picks a venv matching the default Python
(3.12) while the installed venv is `idf5.5_py3.11_env`.

**Setup.** Board on USB (J11) to the PC that will flash it. Nothing else connected.

**Do.** Use the helper that bypasses `export.ps1`:

```powershell
& 'g:\My Drive\projects\.claude\skills\6dof-esp-build\idf_build.ps1' `
  -ProjDir 'C:\Users\Chris\Documents\GitHub\6DOF-Rotary-Stewart-Motion-Simulator\hardware\pcb\6dof2_r13\bringup\probe_fw' -Clean
& 'g:\My Drive\projects\.claude\skills\6dof-esp-build\idf_build.ps1' `
  -ProjDir '<same probe_fw path>' -Action '-p COM10 flash'
```

**Measure.** `IDF_EXIT=0`, a fresh timestamp on `build/*.bin`, then
`probe.py --port COM10 boot` prints the self-test banner.

**Pass.** Banner shows 0 failures and `probe.py cmd info` returns `OK`.

**If it fails.** In this order: (1) stale binary, check the timestamp; (2) wrong target,
run `idf.py set-target esp32s3` once in a fresh build dir, and for the controller repo
`git submodule update --init --recursive`; (3) port, Device Manager must show
`USB\VID_303A&PID_1001` and no terminal may hold it; (4) esptool cannot sync, hold BOOT,
tap RESET, release BOOT, flash again; (5) flashes but no banner, the application is not
running: ROM flashing proves nothing about the app. Note that RESET does not reset the
W5500 (RC-only RSTn); only a power cycle does.

## Step 1 - board on USB only

**Purpose.** Establish the baseline the whole latch design depends on: with no 12 V, the
driver rail VCC5 must be absent and the latch must be impossible to arm. If this step
passes, every later "it armed" result is attributable to 12 V being present, and every
"USB-only can never drive a motor" claim in the design is confirmed on real hardware.

**Setup.** USB only. J2 open. No 12 V. Nothing on any DB25.

**Do.** `probe.py cmd selftest`, then `probe.py cmd in`.
Negative test: `probe.py run "inhibit 0" "arm"` (expect refusal, loop open). Then short J2
and run `arm` once more. Afterwards `safe` and remove the J2 short.

**Measure.** Meter TP6, TP5, TP4, TP8 after that last `arm`.

**Pass.** Self-test 0 failures; `in` shows `estop.gpio21=1`, alarms all 1,
`inhibit.gpio47=1 arm.gpio48=0`; TP4 = 0 V with D4 off; TP8 stays 0 V even after
an `arm` with the loop shorted.

**If it fails.** DRIVE_ENABLE 3.3 V here means the supervisor is not holding the latch
clear without VCC5: check U13 pin 6 (should be low) and R67/R68. Stop; the safety
contract is broken and nothing downstream is trustworthy.

## Step 2 - 12 V on J9: rails, current, and the alarm feed check

**Purpose.** Bring up the only supply that can enable the drivers, confirm it clears the
supervisor threshold, and settle the R30-R35 question with one meter reading before any
alarm wire exists.

**Setup.** Bench supply 12.0 V, 0.6 A limit, polarity per the J9 silkscreen. USB still
connected. J2 open.

**Do.** Switch on the 12 V. Wait two minutes.

**Measure.**
- 2a. D4 lights. TP4, TP5, TP2. Supply current. Buck and U7 temperature
  by touch.
- 2b. `probe.py cmd in` unchanged; TP8 still 0 V.
- 2c. **TP3, VINRAW at the C1 positive lead.** This is the check behind the alarm hold.
  The released schematic feeds the six LTV-847 LEDs from VINRAW through 220 Ω, 0603,
  0.1 W resistors (R30-R35). At 12 V that is about 49 mA through the LED and 0.53 W in a
  0.1 W resistor; at 13.8 V, 57 mA and 0.72 W, past the LED's 50 mA maximum. If the
  meter reads about 12 V here, the hold is confirmed and step 7 is mandatory before pin
  23 is ever connected. If it reads about 5 V, the hold was wrong, step 7 is skipped and
  the note in `BRINGUP_LOG.md` is withdrawn.

**Pass.** TP4 4.96-5.1 V, no current-limit trip, cool regulators, and a recorded TP3
figure.

**If it fails.** VCC5 low: buck, its feedback, or the J9 polarity. Current limit trip:
stop and look for a short before raising the limit.

## Step 3 - stop loop

**Purpose.** J2 is the only operator stop on this bench. Prove the MCU sees it, and that it
is clean, before it is relied on to end motion in step 9.

**Setup.** As step 2. Make a wire link for J2 and record which pins it bridges. The loop is
normally closed: shorted means run.

**Do.** `probe.py mon 20`, fit and remove the link a few times during the 20 s.

**Measure.** `estop` field in the monitor; meter TP7.

**Pass.** `estop=0` while shorted, `estop=1` when open, no bounce visible beyond the
1 k/100 n filter; meter 0 V shorted, 3.3 V open.

**If it fails.** Wrong J2 pins, or R1/C filter. Do not proceed; the arm sequence refuses
without this.

## Step 4 - arm the latch, then prove every clear is one-way

**Purpose.** This is the first time DRIVE_ENABLE should ever go high. The step proves the
arm path works and, more importantly, that each of the three conditions that must clear it
does clear it and does not re-arm by itself. A latch that re-arms on its own would make
step 9's operator stop meaningless.

**Setup.** 12 V on, J2 shorted.

**Do.** `probe.py run "inhibit 0" "arm"`. Scope single-shot on GPIO48 during `arm`.
Then, re-arming between each: (1) pull the J2 link, then refit it; (2) `inhibit 1`, then
`inhibit 0`; (3) switch off the 12 V, then switch it back on.

**Measure.** TP8 (U3 pin 4) after the arm, and after each of
the three clears and their restores.

**Pass.** 3.3 V after arm; one 50 us rising edge on GPIO48; 0 V after each clear and
**still 0 V after the condition is restored**. A fresh `inhibit 0` + `arm` is needed each
time.

**If it fails.** Never arms with 12 V on: VCC5 below threshold, J2 not truly shorted, or
U13 release (pin 6) low. Re-arms on restore: U14/U11 clear wiring, stop.

## Step 5 - STEP and DIR at the DB25, no cable, no drive

**Purpose.** Prove the entire on-board signal path for M0 with the scope, where nothing
external can be blamed: MCU pin, line driver, enable gating, connector. Also identify
which physical DB25 is M0 instead of trusting the silkscreen.

**Setup.** Armed per step 4. Scope on TP10 (J3 pins 3 and 14), ground TP1 at J3 pin 10.

**Do and measure.**
- `tone 0 1000 3000`: 1 kHz on STEP0 for 3 s. Expect two complementary square waves at the
  levels in the table, 50 % duty, 1.000 kHz.
- `tone 1 1000 3000`: nothing on this connector. Find where it does appear; that is M1.
- `dir 0 1` then `dir 0 0`: pins 4 and 5 swap levels.
- `pulse 0 500 200`: exactly 200 pulses in a 0.4 s burst, counted on the scope.
- `inhibit 1`, then `tone 0 1000 2000`: no drive on pins 3/14; note what they float to.
  That is what the drive's receiver sees when the board is disarmed.
- Re-arm; tone at 10 kHz and 100 kHz: clean edges, note rise time.

**Pass.** Differential STEP and DIR present only while armed; pulse count exact; M0
connector identified and labelled on the bench.

**If it fails.** STEP at GPIO4 but not at pin 3/14: U3 pin 4 (needs 3.3 V), pin 16
(VCC5), pin 12 (LOGIC_5V). No STEP at GPIO4: firmware command, rerun `selftest`.

## Step 6 - the one cable

**Purpose.** Remove the cable as a variable before the drive is involved. Pair integrity
matters as much as continuity: a swapped + and - inverts the signal and a split pair
invites noise.

**Setup.** Cable disconnected at both ends. Later, board end plugged into M0, armed.

**Do and measure.** Continuity, mating faces: pins 3, 14, 4, 5, 10, 23 each to their own
number and to nothing else; shield to shell; 3/14 and 4/5 each a twisted pair. For this
step and the next, pin 23 is **lifted at the drive end** (or use a breakout with it
disconnected). Then plug the board end only, arm, `tone 0 1000 3000`, scope at the drive
end of the cable.

**Pass.** No pin swap, and the same waveform at the far end.

**If it fails.** Re-pin or replace the cable; do not compensate in firmware.

## Step 7 - alarm input path: rework and emulated alarm

**Purpose.** Make the alarm input safe to connect, then prove it works without the drive,
so that when the drive's alarm output is finally wired (step 10) the only unknown is the
drive itself. Also measure the two things the resistor review could not: the real LED
current at the real VINRAW, and whether that current actually pulls the 3.3 V input low.
Do this step only if step 2c read about 12 V. If it read about 5 V, skip to step 8.

**Setup.** 12 V off. Board on the bench with R30-R35 accessible.

**Do.**
- 7a. Rework: replace R30-R35 with 1.5 kΩ to 2.0 kΩ. 0603 parts are 0.1 W, too small for
  margin; use 1206 0.25 W stood across the 0603 pads, or lift the resistor and wire a
  through-hole part. The R14 schematic already uses YAGEO AC1206FR-071K5L 1.5 kΩ, which at
  12 V gives about 7 mA and 0.08 W. Record the part used.
- 7b. Emulated alarm: 12 V on, J2 open, disarmed. On the M0 DB25 (no cable, or the cable
  with nothing at the far end), jumper pin 23 to pin 10. This does exactly what the drive's
  alarm transistor does.

**Measure.** TP3 and TP13 (current = (TP3 - TP13) / R30), TP13 to TP12 (LED forward
voltage), TP14 with the jumper in and out, `probe.py cmd in` alarm bit for axis 0, and resistor
temperature after five minutes with the jumper in.

**Pass.** 5-8 mA, TP14 under 0.4 V with the jumper and 3.3 V without, `in` shows alarm0
0 then 1, resistor warm at most.

**If it fails.** GPIO1 not pulled low at 5-7 mA: opto CTR too low for the 10 k pull-up,
drop the resistor toward 1.5 kΩ or note it for R14 (the review flags this as unproven).
Resistor hot: wrong value fitted.

## Step 8 - drive powered, parameters set, cable connected, board disarmed

**Purpose.** Get the drive into a state where it will follow pulses, and prove that state
with readback across a power cycle, before any pulse is sent. Also confirm the drive and
board can share a ground without the board resetting.

**Setup.** Cable board end in M0, drive end in the drive. Pin 23 connected only if step 7
passed; otherwise still lifted. Board disarmed. Drive supply on.

**Do.** Record the drive nameplate and the exact manual revision used. On the keypad set,
power-cycle, and read back: `Pn003 = 1` (servo enabled with no SRV-ON input),
`Pn096 = 0` (pulse + direction), `Pn033 = 0` (no power-on origin return; never command
an origin search, there is no Hall sensor). Write down, from the manual, the electronic
gear ratio and pulse-input filter values. They decide how far 200 pulses turns the shaft.

**Measure.** Drive display: no alarm, servo enabled, shaft resists hand rotation.
Scope VCC3V3 at the moment the drive enables. `probe.py cmd in`.

**Pass.** Parameters read back as set after the power cycle, no alarm, board did not
reset, `in` unchanged (alarm0 = 1 if pin 23 is connected and the drive reports no alarm;
see step 10 about polarity).

**If it fails.** Drive alarm at power-up: manual, usually encoder or power. Board reset on
enable: ground bounce on the shared COM; scope it, stop.

## Step 9 - first motion, bounded

**Purpose.** Prove motion in both directions with a known pulse count, then prove the
operator stop ends motion and does not let it restart. Bounded counts first so a wrong
gear ratio produces a small surprise, not a fast one.

**Setup.** Armed per step 4. Scope on the drive end of the cable. Hand on the J2 link.

**Do and measure.**
1. `pulse 0 500 200`: shaft moves a small, repeatable angle and holds. Mark the shaft.
2. `dir 0 1`, `pulse 0 500 200`: returns to the mark. Record which DIR level is which
   rotation.
3. `pulse 0 2000 2000`, then `pulse 0 5000 5000`: smooth, no alarm, angle scales with
   count.
4. `tone 0 2000 3000`: steady rotation for 3 s, stops by itself.
5. Operator stop: `tone 0 2000 10000`, pull the J2 link after 2 s. Motor stops at once,
   pins 3/14 go high-Z on the scope, DRIVE_ENABLE reads 0 V, refitting the link does not
   restart it.
6. Observation only: does the drive hold torque after pulses stop, and after the latch
   clears? Pulse removal is not torque-off; that remains open (F05).

**Pass.** Motion both ways with counts that make sense against the gear ratio, stop-loop
ends motion, no auto re-arm.

**If it fails.** Pulses at the drive but no motion and no alarm: Pn003/Pn096, gear ratio
so large the move is invisible, or pulse width under the drive's filter. Alarm during
motion: code against the manual, usually over-speed from the gear ratio; retry at 500 Hz.
One twitch then nothing: DIR changing inside a train, scope DIR against STEP.

## Step 10 - real drive alarm

**Purpose.** Learn the drive alarm output's polarity and confirm the board reads it,
because the controller's alarm latch (F04) cannot be written correctly without knowing
whether "conducting" means healthy or faulted. Many servo drives close the ALM output in
the healthy state, which would mean continuous LED current in normal operation; that is
why the resistor power in step 7 matters even when nothing is wrong. Needs step 7 passed
and pin 23 connected.

**Setup.** As step 8, disarmed, no motion commanded.

**Do.** Observe the alarm bit in a healthy state. Then provoke a benign, recoverable alarm
the manual allows, for example power the drive with the encoder cable unplugged. Clear it
per the manual.

**Measure.** `probe.py mon 30` throughout; TP14 on the meter in both states; current
through R30 (TP3 - TP13) in both states.

**Pass.** The healthy and faulted states are distinguishable at GPIO1, the polarity is
written down, and R30 current never exceeds 8 mA in either state.

**If it fails.** Both states read the same: the drive's output may need its own supply or
is not an open collector; check the manual's ALM output drawing before any firmware work.

## Step 11 - when a step fails, where to look

| Symptom | Most likely | Check |
|---|---|---|
| `arm` runs, DRIVE_ENABLE stays 0 V with 12 V on | VCC5 below threshold, or J2 not shorted | VCC5, DRIVE_5V_SENSE, ESTOP_MCU 0 V, U13 pin 6 high |
| STEP at GPIO4, nothing at DB25 | Latch not set, or U3 unpowered | U3 pin 4 3.3 V, pin 16 VCC5, pin 12 LOGIC_5V |
| STEP at DB25, not at the drive end | Cable | step 6 |
| Pulses at the drive, no motion, no alarm | Pn003/Pn096, gear ratio, pulse filter | drive pulse counter if present, raise count |
| Pulses at the drive, alarm | Over-speed from gear ratio, encoder, power | alarm code, retry at 500 Hz |
| Motor twitches once | DIR changed mid-train | scope DIR vs STEP, set DIR then wait |
| Board resets on drive enable | Shared COM ground bounce | scope VCC3V3 at enable |
| Alarm input never changes | Opto not conducting, or ALM output polarity | R30 current, step 10 |

## Step 12 - controller firmware bench mode (the firmware change)

**Purpose.** Move from the probe image to the real controller without opening its
six-axis motion gate. `R14_FAIL_CLOSED` must stay ON: clearing it enables all six axes and
the network path at once. A separate, console-only, one-axis option gives the same bench
evidence with the production step engine, which also exercises the DIR-setup and
final-pulse behaviour listed under F03.

**Do.** Add build option `R13_BENCH_ONE_AXIS`, default OFF, that refuses to build with a
release target, and that:
- keeps boot disarmed and every network motion path exactly as now;
- adds `BENCH:ARM` (refuses unless ESTOP_MCU is low; GPIO47 low, 300 ms wait, 50 us pulse
  on GPIO48; replies that DRIVE_ENABLE has no readback), `BENCH:PULSE <hz> <n>` on axis 0
  only with hz capped at 5000 and n at 10000, and `BENCH:STOP` (GPIO47 high, engine stop,
  emergency clear), all accepted on the USB console only, never from Ethernet;
- routes transport reconnect, stop-loop edge and alarm reports through the existing
  `r14_report_*` paths, which already drive GPIO47 high;
- logs the firmware SHA, option state and every bench command.

**Measure.** Repeat steps 5 and 9 with the controller image. Compare scope captures with
the probe captures.

**Pass.** Identical levels and identical counts; stop-loop behaviour identical.

## Step 13 - report

Write into the shared handoff: firmware SHA for both images, COM port and flash method,
the J2 link pins, VCC5 and DRIVE_ENABLE readings, VINRAW at R30 and the resistor fitted,
which DB25 is M0, scope captures from steps 5, 9 and 10, drive nameplate and parameter
readbacks, DIR-to-rotation mapping, alarm output polarity, and stop-loop timing. Commit
`BRINGUP_LOG.md` and this file with the results.

## Step 12 implementation (2026-10-11, second workstation)

Implemented on `r13-single-motor-sim-20261011` as specified above:

- Build: `controller\tools\idf_build.ps1 -ProjDir controller -Action "-B build-bench -DR13_BENCH_ONE_AXIS=ON build"`.
  Run `-B build-bench set-target esp32s3` once first. The option is OFF by default and
  is refused when `R14_FAIL_CLOSED` is OFF, when `CI` is set, or for a `refs/tags/` build.
  The git SHA, with `-dirty` if the tree has local changes, is compiled in as `FW_GIT_SHA`.
- Code: `controller/include/R13BenchAxis.h`, `controller/main/R13BenchAxis.cpp`,
  `controller/include/R13BenchPulse.h` (pure schedule, host tests in
  `controller/tests/test_r13_bench_pulse.cpp`). Hooks in `controller/main/main.cpp` are
  guarded by `R13_BENCH_ONE_AXIS && R14_FAIL_CLOSED`:
  - the console command handler, which checks the source tag
  - `handleStepDirection`, which leaves the engine alone while the bench is armed
  - the E-stop raw-open path
  - `r14_report_transport_reconnect`
  - `r14_report_normalized_drive_alarm`
  - `ESTOP:FULL`
- Commands, from the USB console only:
  - `BENCH:ARM` refuses while GPIO21 is high, then GPIO47 low, 300 ms wait, 50 µs on GPIO48.
  - `BENCH:PULSE <hz> <n>` drives axis 0 only, hz 1–5000, |n| 1–10000, sign = direction.
    It runs on a 200 µs esp_timer schedule (one step per tick at 5 kHz) through the shared
    MCPWM engine.
  - `BENCH:STOP` sets GPIO47 high, stops the engine and clears targets.
  - `BENCH:STATUS` reports state.
  - Every bench command is echoed with the source and SHA. Commands from Ethernet, Wi-Fi
    or BLE get `BENCH:ERR console only`.
- Host console: `python controller/tools/cobs_console.py --port COMx -i`. An empty line
  sends `BENCH:STOP`.
- Verified on the second workstation: host tests pass (79/79). Bench and default
  firmware images both build with ESP-IDF v5.5.2. **Not yet run on hardware.**
  Measurement is still step 5 and step 9 with this image, compared against the probe
  captures.
