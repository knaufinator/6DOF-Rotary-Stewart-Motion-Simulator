# R13 full bring-up plan

2026-10-12. Extends `M1_OUTPUT_TEST_PLAN.md` (motor-1 sequence, steps 0–13) to the whole
first-article board, and assigns every step to the bench tool `r13bench_app.py`. Test points
are in `TEST_POINTS.md`. Results go in `BRINGUP_LOG.md`; the tool writes paste-ready
markdown in `logs/`.

**Rules unchanged from the motor-1 plan:**
- Motor unloaded.
- 12 V current-limited to 0.6 A for the board alone.
- DB25 pin 23 off the drive until step 7 passes.
- Bounded pulse counts before any tone.
- Hand on the J2 link.
- Never arm automatically.

The probe firmware is used until phase H. Bench "motor 1" is axis 0 = **J3, silkscreen
`MOTOR 0`**.

## Status at the start of this plan

| Area | State | Evidence |
|---|---|---|
| USB enumeration, ROM, probe self-test | PASS | BRINGUP_LOG §1–2, step 0 |
| Ethernet W5500, link benchmarks, controller + app over Ethernet | PASS | BRINGUP_LOG §3, 5–7 |
| Native USB command/reply, controller image | PASS | second-workstation report in the repo handoff |
| USB-only rails (step 1) | partial: VCC3V3 3.29 V, LOGIC_5V 4.0 V | BRINGUP_LOG step 1 |
| 12 V up, D4 lit, loop closed, latch armed (probe) | done, DB25 readings pending | BRINGUP_LOG shortcut |
| Second-workstation flashing | qualified 2026-10-12: 5/5 connects, 4/4 1 MB write/verify, with and without 12 V | repo handoff |
| Drive parameters | Pn033=0, 1:1 gear, bench filter values set; Pn003/Pn096 readback pending | `docs/AASD15A_DRIVE_SETTINGS.md` |
| Steps 2–10 measurements | **open** | — |

## Phases

| Phase | Goal | Steps (tool catalogue id) | Exit criterion |
|---|---|---|---|
| A. Power and safety baseline | USB-only cannot arm; VCC5 valid on 12 V; J2 seen; latch arms and every clear is one-way | 0, 1, 2, 3, 4 | TP4 4.96–5.1 V; TP8 3.3 V armed and 0 V after each clear and restore |
| B. Axis-0 signal chain | Differential STEP/DIR at J3, exact counts, cable proven | 5, 6 | Scope captures saved; M0 = J3 confirmed |
| C. Alarm path | Resistor hold resolved by TP3; reworked R30–R35; emulated alarm on all six | 7, F2 | 5–8 mA; each alarm bit follows only its own connector |
| D. Drive and first motion | Drive readback, bounded motion both ways, operator stop proven | 8, 9, 10 | Motion scales with count; J2 pull stops with no re-arm; ALM polarity recorded |
| E. All axes | Axes 1–5 at their own DB25 with exact counts | F1 | Five captures, no cross-talk |
| F. Interfaces | W5500, UART J1 loopback, Wi-Fi scan, temperature | F3 | All pass |
| G. Soak | 30 min at 12 V, one axis toning | F4 | VCC5 in range, no hot parts |
| H. Controller bench parity | Production step engine gives the same evidence | F5 (plan step 12) | Same levels, counts and stop behaviour as the probe |
| I. Platform firmware gates | F01–F06 in `6DOF2_FIRMWARE_TODO.md` | — | Separate plan. **Nothing here authorises six-axis or loaded motion.** |

Run phases in order. A phase starts only after the previous phase's steps are recorded
PASS (or SKIP with a reason) in the log.

## Using the bench tool

```
<IDF venv>\Scripts\python.exe hardware\pcb\6dof2_r13\bringup\r13bench_app.py --operator <name>
```

1. **Connect.** The tool auto-selects the ESP32-S3 port and detects the firmware: probe
   text console, or the controller `R13_BENCH_ONE_AXIS` image (COBS `BENCH:*`).
2. **Bring-up tab:** pick a step.
   - It shows purpose, setup and pass criteria, with one-click actions.
   - Motion actions ask for confirmation, and every action is checked against the caps.
   - Type each meter reading into its field; it turns PASS or FAIL against the expected
     range. **Record PASS/FAIL/SKIP** writes the table to `logs/r13_session_*.md`; paste
     that into `BRINGUP_LOG.md`.
3. **Drive connected switch,** on by default:
   - On: pulse ≤ 5000 Hz / 10 000 pulses, tone ≤ 5000 Hz / 10 s.
   - Switch it off only for scope work with no cable fitted (steps 5 and F1).
4. **STOP / Esc** sends `safe` (probe) or `BENCH:STOP` (bench image). The probe runs one
   command at a time, so during a probe tone or pulse STOP takes effect when that command
   ends. **The immediate stop is the J2 link.**
5. **Headless** (agents, scripts): `python r13bench_core.py --port COMx cmd in`,
   `... steps`, `... ports`. Add `--drive-connected` to apply drive caps.

Host tests: `python test_r13bench_core.py` (fake serial; no board needed).

## Known issues carried into this plan

| # | Issue | Where it bites | Handling |
|---|---|---|---|
| K1 | Native-USB flashing on the second workstation dropped at erase/write with the drive powered and cabled; clean without the drive | Flashing during phase D | Flash with the drive unpowered or uncabled. Log any recurrence with drive state. R14 item U3 |
| K2 | One manual BOOT entry did not register (`boot:0xb`) | Recovery flashing | Recheck SW/GPIO0 continuity in phase A |
| K3 | LOGIC_5V 4.0 V on USB only (expected 4.4–4.8) | USB-only operation | Acceptable for the LDO. R14 item P2 |
| K4 | R30–R35 220 Ω 0603 at VINRAW: about 49 mA, 0.53 W | Pin 23 to a drive | Step 2c/7 hold; rework to 1.5–2 k |
| K5 | Silkscreen `MOTOR 0–5` vs bench "motor 1" naming | Cable on the wrong socket (happened twice) | Label the bench cable "J3 / axis 0". R14 item S1 |
| K6 | DRIVE_ENABLE has no MCU readback | Every arm/clear | Meter TP8. R14 item D1 |
| K7 | Probe console blocks during tone/pulse, so STOP is delayed | Phase D | J2 is the stop; probe enhancement E1 |

## R13 enhancements (near term, no board respin)

| Id | Area | Enhancement | Why |
|---|---|---|---|
| E1 | Probe firmware | Run `tone`/`pulse` in a task so `safe` and `inhibit 1` are processed mid-train; add `stop` | Makes the software STOP immediate (K7) |
| E2 | Probe firmware | `id` command with build git SHA; print it in the boot banner | Log traceability like the bench image |
| E3 | Probe firmware | PCNT-count STEP0 at the GPIO and report it after `pulse` | Independent count without a scope (F03 evidence) |
| E4 | Bench image | Optional axis argument (`BENCH:PULSE <axis> ...`) behind a second option, for phase E with the production engine | All-axis parity |
| E5 | Bench image | Report alarm GPIO states and loop-open timestamps in `BENCH:STATUS` | F04 preparation, stop timing evidence |
| E6 | Bench image | Jerk-limited S-curve option for `BENCH:PULSE` | Lets the drive filter (Pn109–112) go to minimum for cueing tests |
| E7 | Tool | Sequence runner for F1/F2: loop axes with operator prompts | Fewer clicks, fewer mistakes |
| E8 | Tool | Export a session straight into `BRINGUP_LOG.md` format with SHA, port and drive Pn table | Report step 13 |
| E9 | Tool | Optional SCPI scope capture (frequency, count) into measurements | Removes manual entry errors in steps 5/9 |
| E10 | Bench rig | DB25 breakout/loopback with LED dummy loads on STEP±/DIR± and a pin-23 switch, one per connector | Dummy-load acceptance from `inhibit_design.md`; faster F1/F2 |
| E11 | Bench rig | Real NC mushroom switch on J2, keyed connector | Replaces the wire link as the operator stop |
| E12 | Bench rig | Shielded DB25 cable with shell bonded; ferrite on USB; separate drive mains/earth plan | K1 interference suspicion |
