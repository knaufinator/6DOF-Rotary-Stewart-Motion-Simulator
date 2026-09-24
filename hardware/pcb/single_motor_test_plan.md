# 6DOF 2 single-axis commissioning gate — not yet executable

**UNTESTED HARDWARE / FIRMWARE BLOCKED**, 2026-09-22. This replaces the historical
ESP32 DevKit + SN75174N breadboard plan. That plan used a different pulse backend,
swapped DB25 STEP-/DIR- contacts and identified the wrong COM contact. Do not use
old copies to connect the custom 6DOF 2 board.

The r13 hardware update is intended for a prototype order and controlled bench
evaluation, not a proven motion controller. Complete the
[firmware and commissioning TODO](6DOF2_FIRMWARE_TODO.md) and verify the final
[hardware contract](6DOF2_BOARD.md) before drive-connected tests.

## Controller-side connector reference

| DB25 contact | Signal |
|---|---|
| 3 | STEP+ / PULS+ |
| 14 | STEP- / PULS- |
| 4 | DIR+ |
| 5 | DIR- |
| 10 | COM / circuit GND |
| 23 | ALARM |
| 6 | SRV-ON: unconnected in r12; verify final r13/external enable contract |

These are board-side assignments, not a declaration that any generic DB25 cable
or AASD-labelled drive uses that mapping. Verify mating-face/solder-side numbering,
continuity, conductor pairing, shield and exact drive model/manual. Preserve each
differential pair. The owner has six matching cables; continuity remains untested.

## Prerequisites before connecting a drive

- [ ] Final r13 package and physical board revision agree.
- [ ] Board-only supply, current, rail, temperature, reset/brownout and supply-order
  checks pass with current-limited bench power and no drive connections.
- [ ] Corrected firmware starts disarmed; stop loop, output enable and every
  command/diagnostic obey one verified inhibit policy.
- [ ] Native USB and/or Ethernet reception is verified without motion.
- [ ] Independently measured STEP/DIR waveforms pass on every axis: idle, single
  pulse, final pulse, reversal, sustained rate, stop, reset and lost link.
  Firmware-reported step count is not independent evidence of emitted pulses.
- [ ] Six alarm inputs and disconnected/faulted-drive behavior are verified.
- [ ] The exact drive manual establishes electrical requirements, pulse timing,
  enable/stop behavior, alarm polarity and electronic gearing. Do not reuse legacy
  Pn settings or enable-on-power-up instructions without verification.
- [ ] An independent stop/inhibit and brake/holding strategy is documented;
  dropping torque does not automatically hold a loaded platform safely.

## Later single-axis procedure

After all prerequisites have recorded passes, use one mechanically unloaded,
secured motor/drive with the platform disconnected, a guarded work area and a
competent operator. Begin with bounded low-rate, low-count commands and verify
direction, independent pulse count, alarm response and stopping before increasing
rate. Observe the actual drive's timing/voltage limits. Test fault and link-loss
recovery without automatic re-arming.

Record board serial/revision, firmware hash, drive model/parameters, cable mapping,
power conditions, instruments, waveforms and measured pass/fail limits. Only then
extend these tests to all six channels and a supported, unoccupied mechanism.
This document does not authorize powered-platform or occupied-platform operation.
