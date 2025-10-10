# Servo Driver Interface PCB

This document defines the dedicated interface board that sits between the ESP32-S3 controller and the AASD-15A servo drivers. The goals are:

- Translate ESP32 3.3 V logic to 5 V logic while preserving edge quality.
- Deliver balanced RS-422 style differential pairs for STEP\+/STEP\- and DIR\+/DIR\- on all six axes.
- Provide deterministic failsafe behaviour for the emergency-stop signal and driver enables.
- Maintain clear, safety-first build steps so the platform can be serviced and audited.

## Electrical Architecture

1. **Logic translation** – A single `SN74LVCH16T245` (or equivalent 16-channel level translator) shifts the 3.3 V GPIO outputs up to a clean 5 V logic domain. Direction pins (`DIR`) and step pins (`STEP`) are hard-configured as outputs.
2. **Differential line driving** – Three `AM26C31IDWR` RS-422 line drivers supply 12 differential outputs (six STEP pairs, six DIR pairs). Each driver generates four differential channels. Line drivers run from a regulated 5 V rail and share a solid ground reference with the servo drives.
3. **Termination** – 120 Ω line-to-line termination resistors are placed at the servo-driver end per AASD requirements. Series damping resistors (33 Ω) are placed close to the driver outputs to control edge rate.
4. **Emergency stop** – The ESTOP loop enters the board on a dedicated pluggable terminal. A latching safety relay (e.g., `Omron G7L-2A-BUBJ-CB`) interrupts the 24 V enable supply to every AASD driver. GPIO20 monitors the loop through an opto-isolator (`TLP2361`) so that the ESP32 sees a galvanically isolated, normally-closed contact.
5. **Power** – The PCB accepts 5 V @ 500 mA for the translators, line drivers, and opto outputs. A local buck regulator (if the cabinet only supplies 24 V) may be populated (`LMR33630` module shown in BOM).

## Bill of Materials

| Item | Qty | Reference | Description | Manufacturer PN | Notes |
|------|-----|-----------|-------------|------------------|-------|
| 1 | 1 | U1 | 16-bit dual-supply level translator, 1.2–5.5 V | Texas Instruments SN74LVCH16T245PWR | Fixed DIR pin low for ESP → driver direction |
| 2 | 3 | U2–U4 | Quad differential line driver | Texas Instruments AM26C31IDWR | Each provides STEP±/DIR± for two axes |
| 3 | 1 | U5 | Opto-isolator, high-speed, open collector | Toshiba TLP2361(TP,E | Interfaces ESTOP loop to ESP32 GPIO20 |
| 4 | 1 | K1 | Safety relay, 2-pole, 24 V coil | Omron G7L-2A-BUBJ-CB | Interrupts AASD drive enable supply |
| 5 | 1 | U6 | Buck regulator, 24 V → 5 V @ 3 A | Texas Instruments LMR33630ADDA | Optional if cabinet only offers 24 V |
| 6 | 12 | R1–R12 | 33 Ω 0603 resistors | Vishay CRCW060333R0FKEA | Series damping at differential outputs |
| 7 | 6 | R13–R18 | 120 Ω 0603 resistors | Vishay CRCW0603120RFKEA | Populate only if termination done on PCB |
| 8 | 1 | R19 | 2.4 kΩ 0805 resistor | Vishay CRCW08052K40FKEA | Limits ESTOP opto LED current at 24 V (9.5mA) |
| 9 | 6 | C1–C6 | 0.1 µF 0603 decoupling capacitors | Murata GRM188R71C104KA01 | Place adjacent to AM26C31 VCC pins |
| 10 | 2 | C7–C8 | 10 µF 1206 bulk capacitors | Murata GRM31CR71A106KA01 | Input/output bulk for 5 V rail |
| 11 | 7 | J1–J7 | 6-pin pluggable terminal (3.81 mm) | Phoenix Contact 1803577 | One per axis, plus one for ESTOP/5 V input |
| 12 | 1 | TP1 | Test loop for 5 V rail | Keystone 5003 | Allows clamp-meter verification |
| 13 | 1 | TP2 | Test loop for ESTOP return | Keystone 5003 | Use for routine safety checks |
| 14 | 1 | PCB | 4-layer FR-4, 1.6 mm, 2 oz | Custom fabrication | Controlled-impedance pairs on layers 1/3 |

> **Safety**: Substitute parts must match or exceed voltage, current, and isolation ratings. Maintain third-party certification where possible (UL/CE components).

## Assembly Procedure

1. **Pre-checks**
   - Print fabrication drawings and schematic. Verify copper weights and impedance notes with the PCB manufacturer.
   - Perform an incoming QC inspection on all safety-critical components (relay, opto, power module).
2. **Power stage**
   - Populate the buck regulator (U6) if 24 V input is used. Solder bulk capacitors C7/C8 and verify 5 V output with a bench supply before populating logic.
   - Install decoupling capacitors C1–C6 near U2–U4 and confirm continuity to ground.
3. **Logic section**
   - Solder U1 (level translator) and strap the DIR pin to ground to force A→B direction.
   - Place AM26C31 devices (U2–U4). Inspect for solder bridges under magnification.
   - Add 33 Ω series resistors and (if required) 120 Ω shunts. Leave the shunts open if termination will occur at the drive cabinet.
4. **Safety circuit**
   - Mount the opto-isolator U5 and its current-limiting resistor (calculated for 24 V loop → 10 mA LED current).
   - Install the safety relay and verify coil wiring against the schematic; label the NO/NC terminals clearly.
5. **Connectors**
   - Fit Phoenix terminal blocks J1–J7. Torque screws to manufacturer spec and apply thread-lock for vibration resistance.
   - Silk-screen each connector with axis number, STEP±, DIR±, and 0 V reference.
6. **Testing**
   - With 5 V applied, verify idle outputs sit low and the ESTOP opto reports closed (logic LOW on GPIO20 test pad).
   - Pulse each ESP32 output via firmware test mode and scope the differential pairs; confirm amplitude (±2.5 V) and clean edges.
   - Inject an ESTOP open condition and confirm the relay opens and the microcontroller reports the state within <5 ms.

## Wiring Map

| Axis | ESP32 Pins | Translator Output | Differential Driver | Servo Driver Terminals |
|------|------------|-------------------|---------------------|------------------------|
| 0 | STEP GPIO4 / DIR GPIO10 | U1 A0/A1 → B0/B1 | U2 OUT1± | STEP1+/STEP1- |
| 1 | STEP GPIO5 / DIR GPIO11 | U1 A2/A3 → B2/B3 | U2 OUT2± | STEP2+/STEP2- |
| 2 | STEP GPIO6 / DIR GPIO12 | U1 A4/A5 → B4/B5 | U2 OUT3± | STEP3+/STEP3- |
| 3 | STEP GPIO7 / DIR GPIO13 | U1 A6/A7 → B6/B7 | U2 OUT4± | STEP4+/STEP4- |
| 4 | STEP GPIO8 / DIR GPIO14 | U1 A8/A9 → B8/B9 | U3 OUT1± | STEP5+/STEP5- |
| 5 | STEP GPIO9 / DIR GPIO17 | U1 A10/A11 → B10/B11 | U3 OUT2± | STEP6+/STEP6- |
| ESTOP | GPIO20 | U5 collector → ESP32 | — | Safety relay feedback |

> Run the STEP/DIR pairs as shielded twisted pairs from the PCB to each AASD driver. Ground the shield at the drive end only to prevent ground loops.

## Commissioning Checklist

- [ ] Firmware `ENABLE_DEBUG_UART` left at `0` for production builds.
- [ ] ESTOP loop verified for open/short faults before energising actuators.
- [ ] Differential pairs scoped under load; rise/fall time < 40 ns, no overshoot > 10%.
- [ ] Servo enables tested with relay forced open and closed.
- [ ] All connectors torqued, labelled, and strain relieved.

Maintain this document with revision control. Any change to the BOM, layout, or assembly sequence must be reviewed under the project’s safety management workflow before deployment.
