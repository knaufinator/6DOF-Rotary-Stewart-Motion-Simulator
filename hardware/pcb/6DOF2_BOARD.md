# 6DOF 2 controller board — untested prototype

**Status, 2026-09-23: r13 RC4 manufacturing handoff; placed/routed/labeled, native DRC empty. UNTESTED PCB.**
The owner intends to order assembled prototypes for testing shortly. Nothing has
been ordered in this update; no working-controller validation is claimed.
Firmware is intentionally deferred; [required work](6DOF2_FIRMWARE_TODO.md) must
be completed before drive-connected operation. Do not use the current firmware/PCB
combination on a powered motion platform.

## Files and revision authority

- [r13 package/status](6dof2_r13/README.md) and
  [RC4 release handoff](6dof2_r13/r13-rc4-2026-09-23/README.md): fabrication, assembly,
  mechanical, source, drawings and evidence are separated. Use only the files/hashes
  named there. Actual JLC processed allocation and complete factory assembly remain
  order gates; catalog stock is not an allocation. Do not mix RC1/RC2/RC3 or r12 exports.
- `6dof2_r12/`: historical Gerber, BOM/CPL, audits, render, review and enclosure.
  Its September 5 release-ready wording is superseded by the September 22
  functional review and this corrective revision. **Do not reorder r12.**
- [Firmware/commissioning TODO](6DOF2_FIRMWARE_TODO.md): software, stop,
  power, harness, bench and factory-test acceptance work.
- [Single-axis test gate](single_motor_test_plan.md): replaces the old breadboard
  plan and incorrect DB25 wiring table.
- [Hardware overview](../README.md): current package paths, three separate JLC
  inputs and superseded revisions.

Development source: EasyEDA Pro **6DOF 2**, project
`16231355292e4948ba37840dcdf926b0`, PCB `bc0fa261f3c54e66b7be29928385553a`,
schematic page `fd2c3589563945538936811f609de096`; local tooling/handoff:
`G:/My Drive/projects/6dof/pcb/`. The repository carries the portable handoff,
not an automatically synchronized live EasyEDA project.

The cancelled/refunded r9 order is historical, not an r12/r13 order. A DRC pass
or deposited ZIP does not mean an order has been submitted.

## Purpose and retained interfaces

This is a six-axis **command interface to external AASD servo drives**, not a motor
power stage. ESP32-S3 STEP/DIR outputs feed three AM26LS31 differential line drivers
and six vertical female DB25 ports. W5500 Ethernet and native USB provide host
interfaces. Six optocoupler-conditioned alarm inputs and a normally-closed stop-loop
sense input are provided. The interface is not wholly galvanically isolated:
drive COM and circuit GND are connected.

The persisted r13 retains six DB25 ports, Ethernet, four layers and connector
supports: 170 x 200 mm with six mounting holes. All 21 new parts are placed/routed;
U2's antenna overhang is intentional. Final inventory is 144 components / 654 pads /
967 tracks / 398 vias / 689 fills / eight regions / one native GND pour, with all
144 designators visible. PCB source checkpoint202 and native DRC203 include the
twelve DB25 flange-hole corrections to 3.20 mm; schematic checkpoint189 preserves
all 144 parts / 640 logical pins and cleans up the old frame/subtitle and U2 labels.
The source archive's separate imported-copy test restores the four-layer stack/rules;
the earlier checkpoint100 archive warning is historical, not the RC4 acceptance basis.
RC4 also corrects only EasyEDA's stale board display boundary from the already
correct outline; the gray left strip and oversized exterior are gone after reload.
All electrical geometry and all 144 parts / 640 logical pins remain unchanged.
Recheck physical cable/body/enclosure fit on the first articles. The C3 maximum-body
envelope and retained-model limitations below still apply.

## Baseline pin contract retained for r13 review

Retained assignments are verified in the final r13 source; use the released netlist
and drawings as authority. Firmware has not yet implemented the complete contract.

| Function | ESP32-S3 GPIO / board net |
|---|---|
| STEP M0..M5 | 4, 5, 6, 7, 8, 9 / `M0S_G`..`M5S_G` |
| DIR M0..M5 | 10, 11, 12, 13, 14, 17 / `M0D_G`..`M5D_G` |
| Alarm M0..M5 | 1, 2, 15, 16, 18, 40 / `M0_ALARM`..`M5_ALARM` |
| Stop-loop sense | 21 / `ESTOP_MCU`; HIGH = open/broken loop (stop) |
| W5500 SPI | MOSI35, MISO37, SCLK36, CS38, INT39 |
| Native USB | GPIO19 D-, GPIO20 D+ |
| Bench UART J1 | TX43, RX44, GND; 3.3 V logic |
| BOOT / RESET | GPIO0 / EN |
| r13 inhibit (implemented hardware; untested) | GPIO47, HIGH = inhibit/default stop |
| r13 arm (implemented hardware; untested) | GPIO48, fresh arm edge into supervised latch; remove legacy GPIO48 LED waveform in firmware |

### DB25 controller-side table

| Contact | Signal | Status |
|---|---|---|
| 3 | STEP+ / PULS+ | Differential output |
| 14 | STEP- / PULS- | Differential output |
| 4 | DIR+ | Differential output |
| 5 | DIR- | Differential output |
| 10 | COM | Circuit GND reference |
| 23 | ALARM | Conditioned feedback input |
| 6 | SRV-ON | Unconnected in r13; independent external drive enable/stop contract still required |

The owner has six matching cables; that does not prove continuity, pair assignments
or drive compatibility. Verify contact numbering/view and each used wire against
the exact drive manual. Do not use the old swapped-negative-line table or pin13 COM.

## Stop, power and programming

J2's NC loop reads LOW while closed and HIGH if opened/broken. It is a control/inhibit
input, **not a certified emergency-stop system**. Current firmware can clear its
pause with the loop open and has diagnostic bypasses. Hardware corrections do not
replace the TODO or an independent drive stopping/holding/brake strategy.

**USB-alone programming is required.** r12's USB-to-5 V path had a buck back-power/
headroom concern. r13's persisted schematic and routed PCB use D3 and D8 feeding `LOGIC_5V`, a
separate buck-derived driver 5 V rail and an LDL1117 replacement for U7. USB supports
logic/programming while motor-command outputs remain inhibited in USB-only operation.
This circuit is implemented but its behavior has not been measured. Rails, gating,
priority and components belong in the r13 release report. No r13 electrical test exists yet;
the old diode-sharing description does not prove safe supply coexistence.

J11 uses ESP32-S3 GPIO19/20 with BOOT/RESET recovery access. ROM download capability
is separate from application USB reception, which needs a firmware correction.
J1 is a 3.3 V bench UART, not a power input.

## What to buy or arrange

Detailed draft: [r13 buying instructions](6dof2_r13/HOW_TO_BUY.md). Ordering remains
blocked until the final r13 manufacturing release passes its gates.

1. Quote **fully assembled prototypes**, comparing two versus five assembled boards
   if useful. Include every required SMT and through-hole component; no hand-soldering
   of required connectors is intended. Do not substitute the obsolete C3110947 DB25
   merely because an old order page lists it.
2. Upload only matching final r13 Gerber/BOM/CPL. Confirm JLC's actual processed
   quantities, stock/attrition, service eligibility, DB25 pin-1/body orientation
   and complete delivered price. Retain the accepted quote and DFM preview.
3. Programming and powered functional testing require separately agreed services.
   With firmware deferred, order **assembled unprogrammed/untested prototypes**,
   not a plug-in working-controller package.
4. Arrange a current-limited regulated supply matching final input requirements,
   USB data cable, meter, logic analyzer and suitable differential measurement
   equipment. Continuity-check the existing six cables. Drive mains wiring,
   stop/inhibit/brake equipment and guarded mechanics are outside ordinary PCBA.
5. Do not buy/print a final enclosure from r12 assets until r13 dimensions, cable
   hood fit, connector access and thermal requirements are confirmed.

## How to use the first prototypes

Detailed draft: [r13 first-use plan](6dof2_r13/HOW_TO_USE.md). The guides describe
intended tests, not measured or validated hardware behavior.

Begin with no drive cables connected. Inspect workmanship/polarity and shorts;
measure rails, current, supply sequencing and reset behavior with current-limited
bench power. Verify USB programming/recovery, then implement and validate firmware.
Measure all differential outputs and alarm/stop/enable states with a fixture before
any drive connection. Only after the TODO gates pass may an unloaded, secured
single-axis test proceed. A supported, unoccupied mechanism comes later; no
occupied-platform validation is claimed.

## Order and test gates

The board may be manufactured for controlled bench evaluation after the r13
source/routing/CAM/assembly gates pass. This is not electrical or firmware validation.
Inspect actual processed BOM/SMT/THT inclusion, connector-body fit, stackup and Gerbers
before ordering. No validated factory firmware image or test fixture exists yet.
Continuity/DRC are not functional tests. r12 enclosure files remain reference assets
until confirmed against final r13 mechanical and thermal requirements.
