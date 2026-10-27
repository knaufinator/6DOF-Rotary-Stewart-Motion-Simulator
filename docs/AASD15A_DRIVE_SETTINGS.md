# AASD-15A drive settings ? tested R13 configuration and history


## Verified capture update ? 2026-10-25

The recovered final R13 bench capture uses **Pn098=2, Pn102=1 and Pn051=3000**.
With the 2,500-line encoder and 50:1 gearbox this is 5,000 input pulses per motor
revolution, 694.444 pulses per arm degree, and **?50,000 pulses for ?72?**. The
current main firmware defaults agree. This is one-axis bench evidence, not a
six-axis or loaded-platform acceptance result.

R14 adds externally controlled SRV-ON. Do not carry the historical automatic
power-on servo-enable setting into R14: its exact drive polarity/parameter
configuration must be qualified with the new latch circuit. Use the
[R14 firmware evidence](../hardware/pcb/6dof2_r14/FIRMWARE_STATUS.md) and
[qualification plan](../hardware/pcb/6dof2_r14/BRINGUP.md).

## Historical October 11 trial

The table and procedure below describe the earlier 1:1 trial. They are retained
for history and must not override the captured 2:1 configuration above.

Status: **bench trial values, 11 October 2026.** For one unloaded drive on the R13
board, used with the bring-up sequence in
`hardware/pcb/6dof2_r13/bringup/M1_OUTPUT_TEST_PLAN.md`. That means the probe firmware
for steps 0–11, then the controller `R13_BENCH_ONE_AXIS` build for step 12
(`controller/include/R13BenchAxis.h`). Values marked *Target* are where we expect to
land after bench tuning; they are not yet verified on hardware.

**Wiring hold from the bring-up plan:** DB25 pin 23 (ALARM) must not reach the drive
until step 7 passes. Bench "motor 1" is axis 0 = DB25 **J3, silkscreen `MOTOR 0`**.
The silkscreen is zero-based.

Manuals (Drive-synced): `G:\My Drive\projects\6dof\docs\`
- `AASD_Servo_Drive_Technical_Manual_cncdrive.pdf` — parameter detail pages used here
- `AASD-15A_manual_aqtronic.pdf` — second translation; clearer wording for Pn110
- `AASD-15A_Installation_Manual_60ST-M01330.pdf` — numbering differs in places; do not
  use it for Pn numbers

The parameter *summary* table in the cncdrive PDF is mis-aligned when extracted (names
shifted against numbers). Use the per-parameter detail pages. Always confirm the value
range shown on the drive display matches the table below before writing.

## How to set and save

1. `MODE` until `Pn` is shown, arrows to the number, `SET` to open, edit, hold `SET`
   ~1 s to confirm.
2. **Save:** `MODE` to `Fn`, run **`Fn001`** (writes all Pn to EEPROM, ~3 s). Without
   this the drive reverts at the next power-up.
3. Power-cycle and read the changed values back. Record before/after values.

## Settings

| Param | Meaning (manual) | Original 2020 | **Bench start** | Target | Notes |
|---|---|---|---|---|---|
| Pn002 | Control mode: 0 torque, 1 speed, **2 position** | 2 | **2** | 2 | Step/dir position mode |
| Pn003 | Servo enable: 0 = SigIn SON, **1 = auto-enable at power-on** | 1 | **1** | 1 | R13 leaves DB25 pin 6 (SRV-ON) unconnected, so this is required. Motor holds torque whenever drive power is on; the drive power switch is the torque-off |
| Pn033 | Origin-return trigger: **0 off**, 1 GOH level, 2 GOH edge, 3 power-on auto | 3 | **0** ✅ set | 0 | No Hall/home sensor on the bench |
| Pn052 | SigIn1 function | 22 (homing trigger GOH) | 1 (default) | 1 | 22 is harmless while Pn033=0, but would start a homing search if Pn033 were ever 1/2 |
| Pn096 | Pulse input: **0 pulse+direction**, 1 CW/CCW, 2 quadrature | 0 | **0** | 0 | Board sends STEP + DIR |
| Pn097 | Direction logic: 0 = CCW for forward | 0 | **0** | — | Flip to 1 if direction is reversed |
| Pn098 | Electronic gear numerator 1 | 80 | **1** | 1 | 1:1 = 10000 pulses per motor rev (2500-line encoder ×4) |
| Pn102 | Electronic gear denominator | 1 | **1** | 1 | Ratio must stay within 1/127 … 127 |
| Pn109 | Command filter: 0 none, 1 first-order smoothing, **2 S-curve** | 2 | **2** | 2, or 1 once firmware is jerk-limited | |
| Pn110 | First-order filter time (used only when Pn109=1) | 50 | leave | 5–10 ms if Pn109=1 | Ignored in S-curve mode — the old 50 had no effect |
| Pn111 | S-curve **Ta**, straight (ramp) part | 300 | **20 ms** | 5–10 ms | Smaller = faster acceleration |
| Pn112 | S-curve **Ts**, arc (rounding) part | 150 | **10 ms** | 5 ms | Larger = smoother, less jolt |
| — | Total filter delay T = Ta + Ts | 450 ms | **30 ms** | 10–15 ms | ≈2–4 command periods at the 250 Hz cue loop |
| Pn113 | Position-loop feed-forward gain | 0 | **0** | 50–80 % | Cuts following error; raise in steps while watching for overshoot |
| Pn114 | Feed-forward filter time | 5 (default) | 5 | 2–5 ms | |
| Pn103 | Position-deviation alarm (×1000 pulses) | 50 (default) | **20** | 10–20 | 20 = 2 motor revs at 1:1; trips sooner on a bind or runaway |
| Pn051 | Motor top-speed limit (r/min) | 3000 (default) | **300** | 1500 | Drive-side cap. 1500 r/min = 250 kHz at 1:1, the firmware ceiling |
| Pn115/116 | Position-loop gains | default | default | tune under load | Gains tuned on a bare motor do not carry over to the arm |

The `app/README.md` table that listed Pn110/111/112 = 50/50/50 was a transcription error
(introduced 2026-02-24); the original 2020 values were 50/300/150 as above.

## Speed and resolution at 1:1

The R13 step engine tops out at **250 kHz per motor** (4 µs period, 2 µs pulse); the
AASD differential input accepts up to 500 kHz.

| Electronic gear | Pulses / motor rev | Motor speed at 250 kHz | Arm speed (50:1) | Arm resolution |
|---|---|---|---|---|
| **1:1 (earlier trial)** | 10 000 | 1500 r/min | 180 °/s | 0.00072 ° |
| 2:1 | 5 000 | 3000 r/min (rated) | 360 °/s | 0.0014 ° |
| 80:1 (2020) | 125 | 3000 r/min at 6.25 kHz | 360 °/s | 0.058 ° |

At 1:1 the motor cannot reach its rated 3000 r/min (that would need 500 kHz). If the
platform needs more than 180 °/s at the arm, switch to 2:1.

Platform firmware soft limits are ±100 000 steps per motor, ≈ ±72 ° at the arm at 1:1.
This is the earlier 1:1 count calculation. The captured 2:1 configuration uses ?50,000 pulses. Do not increase limits to bypass a calibration or mechanical-travel mismatch.

## Differential STEP/DIR — nothing to set

The ESP32-S3 drives one GPIO per signal (motor 1: STEP = GPIO4, DIR = GPIO10). The
AM26LS31 line drivers produce each ± pair. The drive's PP/PD pulse inputs accept
differential directly (the manual recommends AM26LS31). Board-side DB25:

| DB25 | Signal |
|---|---|
| 3 / 14 | STEP+ / STEP− |
| 4 / 5 | DIR+ / DIR− |
| 10 | COM (circuit ground) |
| 23 | ALARM (not yet used by firmware) |
| 6 | SRV-ON — unconnected on R13 |

Verify the cable against the drive's CN2 pinout; do not wire the drive's open-collector /
24 V pulse inputs.

## Bench tuning procedure

Run this only after bring-up plan steps 0–9 have passed with the probe firmware.

1. Board: J2 link in, wire clamped across both screws of the `E-STOP LOOP` terminal
   (pulling it is the hardware stop). 12 V applied, USB on J11.
2. Controller built with `-DR13_BENCH_ONE_AXIS=ON`. Run
   `python controller/tools/cobs_console.py --port COMx -i`, then `BENCH:STATUS`
   (expect `loop=CLOSED`), then `BENCH:ARM`.
3. `BENCH:PULSE 1000 1000` (0.1 motor rev at 1:1) to confirm direction, then
   `BENCH:PULSE 1000 -1000` back. Limits: ≤5000 Hz, ≤10 000 pulses; the sign is the
   direction.
4. Repeat at `BENCH:PULSE 5000 10000` (1 motor rev in 2 s), both directions. Listen and
   watch: grinding, buzz, or overshoot at the end of a move means the drive filter is
   too sharp.
5. Step Pn111 → 10 → 5 and Pn112 → 5, saving with Fn001 each time. Then raise Pn113 in
   ~20 % steps. Watch position deviation on the drive `dn` monitor: it should stay small
   and settle quickly.
6. Record each combination tried and the result in this file. `BENCH:STOP` ends a run.

At 1:1 the bench cap of 5000 Hz is 30 r/min (3.6 °/s at the arm). Profiles faster
than that need the controller's platform path, after the bench gates pass.

## Trial log

| Date | Pn111 | Pn112 | Pn113 | Speed / test | Result |
|---|---|---|---|---|---|
| | | | | | |
