# R13 test points for the bench sequence

2026-10-11. Companion to `M1_OUTPUT_TEST_PLAN.md`. Every measurement in that plan refers to a
TP number from this table. The rule for choosing a point: **a screw terminal, header pin,
DB25 pin, electrolytic capacitor lead or a 2.54 mm / 1.27 mm IC pin before anything
smaller**. 0603 pads appear only as alternates. VSSOP and SC70 pins are never the primary
point. The r13 board has no dedicated test points, which is the first item in the r14 list
at the end.

Where the circles are: `testpoints/schem_pNN_testpoints.png` are the r13 schematic review
pages (`r13-rc4-2026-09-23/drawings/6DOF2_r13_Schematic_Review.pdf`, page NN) with each TP
circled in red (blue = ground or indicator). `testpoints/pcb_testpoints.png` is the RC4 PCB
review image with the same numbers at the physical part, placed from the JLC CPL
coordinates (`px = 573 + 4.78 x X_mm`, `py = 978 - 4.78 x Y_mm`, checked against J9, J11
and J3).

Nets and pins below come from `r13-rc4-2026-09-23/source/schematic_netlist.json`
(`pinInfoMap`), not from reading the drawing.

## The table

| TP | Net | Primary point (easy) | Alternates | Schematic page | Expect |
|---|---|---|---|---|---|
| TP1 | GND | **J9 pin 2** (12V IN "GND" screw terminal) | J2 pin 2, J1 pin 1 (UART header, GND), any DB25 pin 10 | 6, 8, 15 | meter black lead here |
| TP2 | VIN_J | **J9 pin 1** (12V IN "+") | F1 pin 1 | 6 | 12.0 V from the bench supply |
| TP3 | VINRAW | **C1 positive lead** (100 uF electrolytic, top right of board) | D1 cathode (SMA diode), R30 pin 1 | 6, 17 | VIN_J minus the SS34 drop, about 11.6-11.8 V. **This reading decides the alarm-resistor hold** (plan step 2c) |
| TP4 | VCC5 | **C2 positive lead** (electrolytic next to the buck) | U3 pin 16, D8 cathode, L1 output end | 7, 9 | USB only: 0 V, D4 dark. 12 V on: 4.96-5.1 V, D4 lit |
| TP5 | LOGIC_5V | **U7 pin 3** (SOT-223 input pin) | D8 anode, U3 pin 12 | 7, 9 | USB only: 4.4-4.8 V. 12 V on: about 5.0 V |
| TP6 | VCC3V3 | **U7 pin 4** (SOT-223 output tab) | C3, C7, U3 pin 12 is NOT this net | 7 | 3.30 +-0.1 V, D5 lit |
| TP7 | ESTOP | **J2 pin 1** (E-STOP screw terminal) | R1 pin 2, R60 pin 1 | 8 | J2 open: 3.3 V. J2 shorted: 0 V. ESTOP_MCU (after R60 1 k) follows it |
| TP8 | DRIVE_ENABLE | **U3 pin 4** (SOIC-16, left row, 4th from the pin-1 dot) | R63 pin 1 (0603, beside U3); origin is U11 pin 5 (VSSOP, do not probe) | 9, 30, 27 | 0 V disarmed, 3.3 V armed. No MCU readback exists |
| TP9 | M0S_I (STEP0 logic, after R20) | **R46 pin 1** (0603 pull-down beside U3) | U2 castellation pin 4 (GPIO4, before R20) | 9 | 3.3 V logic pulses from `tone 0` / `pulse 0` |
| TP10 | M0S_P / M0S_N (STEP0 differential) | **J3 pins 3 and 14** (MOTOR 0 DB25) | U3 pins 2 and 3 | 15 | armed: complementary 0 / about 3.4 V square waves. disarmed: float |
| TP11 | M0D_P / M0D_N (DIR0 differential) | **J3 pins 4 and 5** | U3 pins 6 and 5 | 15 | `dir 0 1`: pin 4 high, pin 5 low; `dir 0 0` reversed |
| TP12 | M0_ALARM_DRV (opto LED cathode, to drive) | **J3 pin 23** | U9 pin 2 | 15, 18 | open: about VINRAW minus LED drop. jumpered to pin 10: 0 V (LED conducts) |
| TP13 | M0_ALARM_LED (R30 far end, opto LED anode) | **U9 pin 1** (LTV-847S, 2.54 mm pitch wide SMD) | R30 pin 2 | 18, 17 | with pin 23 jumpered: TP3 minus this = drop across R30; current = drop / R30 |
| TP14 | M0_ALARM (MCU input) | **U9 pin 16** | R36 pin 2, U2 castellation pin 39 (GPIO1) | 18, 17 | no alarm current: 3.3 V. opto conducting: under 0.4 V |
| D4 | VCC5 indicator | LED near the top-right corner | | 8 | lit only with 12 V |
| D5 | VCC3V3 indicator | LED next to D4 | | 8 | lit on USB or 12 V |

DB25 pin numbering: J3-J8 are female sockets. The digits are moulded into the insulator
next to each hole; read them, do not count. Looking into the socket with the 13-hole row
on top, hole 1 is at the top right and hole 14 is at the bottom right, directly under hole 1.

Which DB25 is M0: the PCB silkscreen says "MOTOR 0" at J3 (bottom-left of the board,
CPL 6.4 mm, 35 mm). Plan step 5 confirms it with a tone on axis 0 and silence on axis 1.

## Measurements the plan takes at these points

| Plan step | What | Points |
|---|---|---|
| 1 | rails on USB only, latch cannot arm | TP6, TP5, TP4 (expect 0), TP8 (expect 0), D4 dark |
| 2 | 12 V up, supervisor threshold, alarm feed check | TP2, TP3, TP4, TP5, D4 lit; **TP3 decides step 7** |
| 3 | stop loop | TP7 with the J2 link in and out |
| 4 | arm, then one-way clears | TP8 after each action; scope single-shot on U2 pin 25 is optional, firmware reads GPIO48 back |
| 5 | STEP/DIR at the connector, no cable | scope TP10 (both pins) and TP11, ground TP1 at J3 pin 10; TP9 to split "no pulse at MCU" from "no pulse at driver" |
| 6 | cable | continuity from J3 pins 3, 14, 4, 5, 10, 23 to the far end |
| 7 | alarm rework and emulated alarm | jumper J3 pin 23 to pin 10; read TP3, TP13, TP14; current = (TP3 - TP13) / R30 |
| 8-10 | drive, motion, real alarm | scope at the drive end of the cable; TP14 and `probe.py mon` for the alarm |

## Why there is no TP on the ESP32 GPIOs

GPIO47 (inhibit), GPIO48 (arm) and GPIO21 (stop loop) are read back by the probe firmware
(`probe.py cmd in`), so a meter adds nothing. The one signal that has no readback is
DRIVE_ENABLE, which is why TP8 is on the plan at every arm and clear.

## Requests for r14 (so this sequence needs no fine probing)

1. Add through-hole or 1.5 mm pad test points, labelled on silkscreen, for: GND (two,
   one near the terminals and one near the drivers), VINRAW, VCC5, LOGIC_5V, VCC3V3,
   DRIVE_ENABLE, ESTOP_MCU, and one STEP/DIR logic pair (axis 0 before the line driver).
2. A ground loop or pin header for a scope ground spring near the DB25 bank.
3. Keep the screw terminals for 12 V and E-STOP; they were the two easiest points here.
4. Bring DRIVE_ENABLE to an LED, since it is the only latch state with no firmware readback.
5. Bring each alarm opto output to an LED, or at least a header, so alarm polarity can be
   seen without a meter.
6. The alarm LED resistor power problem (220 R at 12 V) is already in the r14 schematic as
   1.5 k 1206; sync it to the PCB.
