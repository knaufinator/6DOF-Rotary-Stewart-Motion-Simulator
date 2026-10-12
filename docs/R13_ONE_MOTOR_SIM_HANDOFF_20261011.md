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

0. **Bring the ESP-IDF Python environment to spec before anything else.** The
   required spec is IDF v5.5 with a venv created by IDF's own installer for the
   Python that is first on your PATH. Run `<IDF_PATH>\install.ps1 esp32s3` from
   PowerShell, then `<IDF_PATH>\export.ps1` must succeed without "Python
   environment is missing". If it still names a venv you do not have, your
   default `python` and the installed venv disagree: either install the matching
   venv (preferred, that is the spec) or build through
   `controller\tools\idf_build.ps1`, which picks an existing IDF 5.5 venv and
   bypasses `export.ps1`. Record the venv name, Python version and IDF version
   in your report. A build that was never verified is not a handoff.
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
- ESP-IDF firmware build: the first attempt failed with "Python environment is
  missing at `...\python_env\idf5.5_py3.12_env`". That is not a missing install.
  IDF's `export.ps1` names the venv after whatever `python` is first on PATH
  (3.12 here) while the venv that `install.ps1` created is `idf5.5_py3.11_env`.
  Fix, either: (a) run `controller\tools\idf_build.ps1 -ProjDir <project> -Clean`
  from PowerShell (not Git Bash); it finds the existing venv and never calls
  `export.ps1`; or (b) run `<IDF_PATH>\install.ps1 esp32s3` once so a venv for
  the default Python exists, then `export.ps1` works. With (a) the r13 probe
  firmware built and flashed clean on the source workstation on 2026-10-11
  (`IDF_EXIT=0`, fresh `build/r13probe.bin`, self-test 0 failures on COM10).
  That probe project, its host driver, the bench log and the motor-1 test
  sequence with circled test points are on this branch under
  `hardware/pcb/6dof2_r13/bringup/` (start with `M1_OUTPUT_TEST_PLAN.md`
  and `TEST_POINTS.md`). Flash with
  `-Action '-p COMx flash'`. Confirm success by the binary timestamp and
  "Project build complete", not by the exit code alone.

R13 physical waveforms, cable mapping, J2/enable behavior, and one-axis
motion remain commissioning items. R14 PCB order readiness is separate.

## Second-workstation report and plan (2026-10-11 evening)

Second workstation (user `knauf`, board enumerated as COM9). The owner was present for all
powered work.

### Do this first when the board is plugged in on the source workstation

1. **Reflash before anything else.** The board still carries a local experimental image
   (`MOTOR_TESTER`, not on this branch). It **holds GPIO21/ESTOP_MCU low in firmware**,
   which means the J2 stop loop is ignored. The owner approved that for one session,
   but it contradicts the standing rules in `M1_OUTPUT_TEST_PLAN.md`. Flash the probe
   image (bring-up step 0) or the step-12 bench image before any other test.
2. Resume `BRINGUP_LOG.md` at "Resume tomorrow": probe image, the DB25-first static
   test at **J3 (silkscreen `MOTOR 0`)**, pin 23 measured at the board, then the drive
   end of the cable.
3. Step 12 is now implemented. Build and test details are in
   `M1_OUTPUT_TEST_PLAN.md` § "Step 12 implementation". After step 9 passes on the probe
   image:
   `idf_build.ps1 -ProjDir controller -Action "-B build-bench -DR13_BENCH_ONE_AXIS=ON build"`
   then flash, then:
   `python controller/tools/cobs_console.py --port COMx BENCH:STATUS BENCH:ARM "BENCH:PULSE 1000 10"`.
   Verified image SHA `45f99185c615` (clean tree). Host tests 79/79. Not yet run on
   hardware.

### Drive state (owner, at the drive panel)

- AASD-15A on bench motor 1.
- `Pn033=0` set and read back.
- Owner also applied the bench values in `docs/AASD15A_DRIVE_SETTINGS.md` except
  `Pn051`, which stays at 3000:
  - `Pn098=1` / `Pn102=1` (1:1 = 10 000 pulses per motor rev)
  - `Pn111=20` / `Pn112=10`
  - `Pn103=20`
  - `Pn052=1`
- `Pn003=1` and `Pn096=0` are carried over from the original setup. Confirm their
  readback in step 8.
- That doc also corrects the old README 50/50/50 transcription of Pn110–112; the original
  2020 values were 50/300/150.

### What happened

- Experimental image arms (GPIO47 low, 250 ms, 20 µs GPIO48). It ran JOG ±10 and
  OSC ±1000 ×3 with correct firmware positions, but **the motor did not move**.
- **Likely cause:** the motor cable was probably on **J4 (`MOTOR 1`)** while axis 0
  drives **J3 (`MOTOR 0`)**. The silkscreen is zero-based; the same mix-up appears in
  `BRINGUP_LOG.md`.
- **Other possible causes,** not proven:
  - the U11 latch not setting (VCC5 or DRIVE_ENABLE not measured)
  - DB25 pin 23 connected through to the drive, contrary to the step-7 hold
- **The owner's first J2 attempt** linked the J1 `UART 3.3V` TX/RX pins instead of the
  `E-STOP LOOP` screw terminal. That is harmless: it loops the unused UART0 back on
  itself.

### Flashing trouble here (second workstation only)

The owner reports the source workstation flashes this board without problems, so treat
these as second-workstation symptoms, not a board defect, until they are reproduced
there.

- esptool auto-reset flashing over native USB repeatedly dropped at erase or the first
  write. Errors seen: "Lost connection", pySerial "Write timeout", "device not
  functioning", "ClearCommError". It was worse with the drive and 12 V powered.
- Once the bootloader region was rewritten (verified) and the app region erased, the
  board boot-looped with `No bootable app partitions` until a later flash completed.
  The latch stayed inhibited throughout.
- **What worked:** 12 V unplugged, manual download mode (hold BOOT while plugging USB),
  esptool `--no-stub --no-compress`, with the app split into 64 KB chunks
  (re-run the remaining chunks after a drop). Then press RESET (EN): after a
  USB-JTAG `hard_reset` the chip stayed in download mode, because the strap latched
  from the manual entry persists until a real chip reset.
- After USB-JTAG resets, the experimental image's console sometimes did not answer until
  a physical RESET.
- One manual BOOT attempt booted normally (`boot:0xb`); recheck the BOOT button
  (GPIO0) on the source workstation.
- Environment step 0 on this machine:
  - Python 3.12.10, venv `idf5.5_py3.12_env`, ESP-IDF v5.5.2
  - `export.ps1` passes
  - `idf_build.ps1` verified

### Update 2026-10-12 (second workstation)

- **Resolved: the board no longer has the bypass image.** The probe image (`r13probe.bin`,
  built from this branch) was flashed normally:
  `idf_build.ps1 -Action "-p COM9 flash"`, all three hashes verified, about 5 s.
- Probe readback: `estop.gpio21=0` (J2 shorted by the owner, loop genuinely closed),
  `inhibit=1`, `arm=0`, STEP/DIR all low, alarms idle.
- Second-workstation link qualification, with the board on a root USB port, 12 V and drive
  state as the owner left them:
  - drivers: `usbser` (CDC) and `WinUSB` (JTAG)
  - esptool v4.12.0, pyserial 3.5, IDF v5.5.2, venv `idf5.5_py3.12_env`
  - 5/5 stub connect and reset cycles
  - 1 MB read_flash
  - 4/4 full-speed 1 MB erase/write/verify rounds at the unused offset `0x600000`
  - All passed. Yesterday's drops happened with the drive powered and the board cabled to
    it, so suspect drive EMI or ground coupling during that session rather than this PC.

### Bench tool and plans (2026-10-12)

- `hardware/pcb/6dof2_r13/bringup/r13bench_app.py` is a Tkinter bench app.
  - It auto-detects the probe text console or the controller `R13_BENCH_ONE_AXIS` COBS console.
  - Guided steps 0–10 plus the full-board phases F1–F5, with meter-entry fields checked
    against expected ranges.
  - Motion caps follow a "Drive connected" switch; STOP is on Esc.
  - Paste-ready session logs go to `logs/`, which is git-ignored.
- Headless use: `r13bench_core.py --port COMx cmd in`. Host tests:
  `test_r13bench_core.py`, 10/10.
- Verified live on COM9 with the probe image (read-only commands): detection, `in`
  parsing, cap refusal, GUI live status.
- Plans:
  - `R13_FULL_BRINGUP_PLAN.md`: phases A–I mapped to tool steps, known issues K1–K7,
    and near-term enhancements E1–E12.
  - `R14_TEST_ENHANCEMENTS.md`: P1–P3 DFT, safety, USB and firmware requests for the R14
    teams. Top items: DRIVE_ENABLE readback, VCC5 ADC sense, silkscreen axis naming,
    SRV-ON, USB ESD and choke, BOOT button.
