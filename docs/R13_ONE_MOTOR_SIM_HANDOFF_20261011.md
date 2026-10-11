# R13 one-motor simulator handoff — 11 October 2026

Branch: `r13-single-motor-sim-20261011`, based on `phoenix` at `b53712f`.
This branch shares ESP32-S3 controller and desktop direct-HIL transport work
with the separate simulator instance. Use full `controller/` firmware, not
`controller/mini/` or the historical serial bridge.

## Bench configuration and immediate task

The remote bench has one unloaded motor with no arms, Hall sensor, or E-stop
switch. Motor motion is an intended bench test. First build the branch and
establish controller-to-desktop communication; then develop an explicitly
scoped one-axis bench mode. This commit is a development handoff, not a
motor-motion-ready firmware image.

The R13 board's J2 normally-closed stop loop and output-enable latch still
matter with no external switch fitted. Record how J2 is closed and verify
the physical enable and STEP outputs. Current PCBv2 code boots disarmed, its
output latch has no qualified arming sequence, and `R14_FAIL_CLOSED` prevents
the STEP engine starting. Disabling that flag alone does not complete a
one-axis bench mode. A network handshake does not prove motor pulses.

## Drive origin setting

The candidate [AASD series manual](https://aqtronic.com/wp-content/uploads/2025/04/AASD-15A-manual.pdf)
identifies `Pn033=0` as power-on origin return disabled and `Pn033=3` as
power-on automatic origin return. Confirm the actual drive model and manual,
set `Pn033=0` on the physical drive, power-cycle if required, and read it back.
Record values before and after. STEP/DIR cannot set this parameter. With no
Hall sensor, do not command a drive origin search. The app README's `Pn033=3`
line is marked historical, not a bench setting.

PCBv2 firmware rejects `ESTOP:SOFT` (formerly a target-zero move) and `ZERO`
(which could mark an unreferenced position as home). `ESTOP:FULL` discards
commanded motion. Software zero is not proof of physical home.

## Source included

- `app/src/` and `app/README.md`: direct UDP HIL protocol-2 transport,
  `DATA18_SEQ` handling, packet diagnostics, and drive-origin note.
- `controller/include/`, `controller/main/`, and
  `controller/sdkconfig.defaults.esp32s3`: direct Ethernet/COBS transport,
  E-stop fail-closed changes, and R14 admission/alarm logic present in shared
  controller source. R14 logic is a development dependency, not a claim that
  this R13 bench has six drives or Hall sensors.
- `controller/tests/`: host tests for transport and safety logic.
- This handoff. Local recordings, UI settings, workspaces, generated PCB
  files, and the R14 PCB worktree are excluded.

## Pickup on the remote simulator instance

```text
git fetch origin r13-single-motor-sim-20261011
git switch --track origin/r13-single-motor-sim-20261011
```

1. Read this file, `app/README.md`, and
   `hardware/pcb/single_motor_test_plan.md`. Record the fetched commit SHA.
2. Build the full controller and desktop app. Verify protocol-2 handshake
   and packet counters with motor power off.
3. Record drive nameplate, `Pn033` readback, DB25 continuity, J2 loop state,
   output-enable state, and M0 STEP/DIR idle levels.
4. Implement and review a bench-only, one-axis enable path for the unloaded
   M0 motor, separate from six-axis platform firmware. Give it an operator
   stop and bounded pulse count before powered motion. Confirm actual stop
   behavior with the motor unloaded.
5. Report firmware SHA, handshake, transport losses, drive readback, and
   bench-mode changes in the shared project handoff.

## Verification on source workstation

- Desktop Release build succeeded; desktop CTest passed 2/2.
- Controller Visual Studio host test build succeeded; CTest passed 1/1.
- `git diff --check` passed.
- ESP-IDF firmware build not verified: local IDF 5.5 Python environment is
  missing at `C:\Users\Chris\.espressif\python_env\idf5.5_py3.12_env`.

R13 physical waveforms, cable mapping, J2/enable behavior, and one-axis
motion remain commissioning items. R14 PCB order readiness is separate.
