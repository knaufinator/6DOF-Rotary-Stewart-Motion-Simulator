# r13 supervised, latched pulse-output inhibit — frozen contract

2026-09-22. Definitive hardware proposal for an **untested engineering prototype**.
This supersedes the earlier two-IC and alternate-supervisor discussions. The main
revision audit, not this proposal, owns implementation/routing/test status.

## Scope

Preserve all six DB25 pinouts and their mechanical supports. Add hardware output
inhibit for unconfigured MCU GPIOs, deliberate disarm, opening the NC stop loop,
and loss of the operating 5 V driver supply. Opening the stop loop or losing
operating power clears a latch; merely restoring either cannot re-arm it.

USB-alone programming is explicitly required. Root's power revision separates
buck-fed `VCC5` (line drivers) from diode-OR `LOGIC_5V` (USB or buck, feeding the
3.3 V regulator). USB must not feed the buck output/line-driver rail.

This is **not** machinery E-stop, STO, torque-off, a brake controller, or
personnel-protection circuitry. The drives can retain torque after pulse removal.
The independently engineered external stop/brake strategy remains necessary.

## Exact IC pin/net contract

All new logic ICs and the supervisor are powered by VCC3V3, not VCC5.
Module U2 is the existing ESP32-S3-WROOM-1 **N8R2** (3.3 V GPIO domain), confirmed
from the fresh r13 exported Manufacturer Part field. Earlier proposal wording
shortened this incorrectly to N8; do not use that shorthand in purchasing.

| Ref | Exact part; supplier code | Pin connections |
|---|---|---|
| U11 | TI SN74LVC1G74DCUR; **C70285**; VSSOP-8 | 1 CLK=ARM_PULSE; 2 D=VCC3V3; 3 /Q=NC; 4 GND; **5 Q=DRIVE_ENABLE**; 6 /CLR=ARM_CLEAR_N; 7 /PRE=VCC3V3; 8 VCC=VCC3V3 |
| U12 | TI SN74LVC1G98DCKR; **C485081**; SC70-6 | 1 IN1=ESTOP_MCU; 2 GND; 3 IN0=VCC3V3; 4 Y=CONTROL_PERMIT; 5 VCC=VCC3V3; 6 IN2=MCU_INHIBIT |
| U13 | TI TPS389001DSER; **C1509297**; WSON6 1.5 × 1.5 mm | 1 SENSE=DRIVE_5V_SENSE; 2 GND; 3 /MR=CONTROL_PERMIT; 4 VDD=VCC3V3; 5 CT=ARM_DELAY_CT; 6 /RESET=SUPERVISOR_RELEASE_N |
| U14 | TI SN74LVC1G98DCKR; **C485081**; SC70-6 | 1 IN1=VCC3V3; 2 GND; 3 IN0=GND; 4 Y=ARM_CLEAR_N; 5 VCC=VCC3V3; 6 IN2=SUPERVISOR_RELEASE_N |

The TI DFF non-inverted Q is **pin5**, not pin3. Verify symbol and footprint pin
maps, not merely device names. U13 must use the TI DSE 1.5 mm WSON drawing, not a
TPS3808 SOT23 footprint. Treat any exposed package pad according to the exact
manufacturer land drawing/library audit; no inferred signal pad.

MCU connections:

- U2 module pin24 / GPIO47 → MCU_INHIBIT.
- U2 module pin25 / GPIO48 → ARM_PULSE.
- Existing U2 pin23 / GPIO21 remains ESTOP_MCU.
- GPIO47/48 were unconnected in the r12 extracted netlist. They are not strapping
  pins, have input-enabled reset states, and are not among the datasheet's listed
  power-up-glitch pins. Do not silently substitute R8V/R16V modules: those variants
  have different GPIO47/48 power-domain behavior.

## Exact passive contract

| Ref | Value and exact sourcing | Connection |
|---|---|---|
| R61 | 1 kΩ 1% 0603, 0603WAF1001T5E, **C21190** | MCU_INHIBIT → VCC3V3 (default inhibited) |
| R62 | same 1 kΩ | ARM_PULSE → GND |
| R63 | same 1 kΩ | U3 pin4 / DRIVE_ENABLE → GND, local to U3 |
| R64 | same 1 kΩ | U4 pin4 / DRIVE_ENABLE → GND, local to U4 |
| R65 | same 1 kΩ | U5 pin4 / DRIVE_ENABLE → GND, local to U5 |
| R66 | 10 kΩ 1% 0603, 0603WAF1002T5E, **C25804** | SUPERVISOR_RELEASE_N → VCC3V3 |
| R67 | **32.0 kΩ 0.1%, 25 ppm/°C**, Yageo RT0603BRD0732KL, **C861325**, 0603 | VCC5 → DRIVE_5V_SENSE |
| R68 | **10.0 kΩ 0.1%, 25 ppm/°C**, Yageo RT0603BRD0710KL, **C95204**, 0603 | DRIVE_5V_SENSE → GND |
| C37 | 100 nF X7R 50 V 0603, CC0603KRX7R9BB104, **C14663** | U11 VCC3V3 → GND, local bypass |
| C38 | same 100 nF | U12 VCC3V3 → GND, local bypass |
| C39 | same 100 nF | U13 VCC3V3 → GND, local bypass |
| C40 | same 100 nF | U14 VCC3V3 → GND, local bypass |
| C41 | same 100 nF | ARM_DELAY_CT → GND, timing capacitor |
| R46–R57 | **change 10 kΩ to 1 kΩ C21190** | Retain the twelve existing STEP/DIR pull-down connections |

Each 1 kΩ MCU-control pull costs about 3.3 mA when driven against its default. This
is deliberate, robust bias and reuses an existing feeder SKU.

D8 (buck-to-LOGIC_5V OR diode) and U7 replacement are controlled by root's power
contract; this document does not assign or approve their exact parts.

## Existing AM26LS31 modifications

For **each** U3/U4/U5:

- Pin4 G: remove VCC5 connection; connect DRIVE_ENABLE.
- Pin12 /G: remove GND connection; connect **LOGIC_5V**.
- Pin16 VCC remains **VCC5**, the buck-only driver rail.
- Pin8 remains GND.
- Keep every STEP/DIR output and DB25 assignment unchanged.

The complementary enable inputs implement **OR**, not AND. Leaving /G low would
defeat the new G control. Using LOGIC_5V for /G keeps it high both during USB-only
operation and during buck operation before 3.3 V rises. In the absence of both
power sources the drivers are unpowered. AM26LS31 input absolute voltage limit is
7 V without a VCC+0.3 restriction; nevertheless verify real back-power current
with the actual assembled devices.

Old VCC5-to-G and GND-to-/G copper must be removed completely after schematic
sync, including same-coordinate contacts, stale vias and fills.

## Logic and sequencing

U12 is a Schmitt-input NOR:
`CONTROL_PERMIT = !(ESTOP_MCU || MCU_INHIBIT)`.

U13 asserts its open-drain reset when CONTROL_PERMIT is low or the measured buck
rail is below threshold. R66 supplies its high level. U14 is a non-inverting
Schmitt buffer and drives the DFF clear with a fast CMOS edge. No push-pull output
is tied to the open-drain reset node.

| Condition | Result |
|---|---|
| NC stop loop opens | ESTOP_MCU rises; hardware clears U11 independent of firmware |
| MCU_INHIBIT high or GPIO high-Z | R61 biases high; hardware clears U11 |
| Buck VCC5 absent/low, including USB-only operation | U13 clears U11; ARM pulses cannot set Q |
| Loop/power restored | Supervisor eventually releases clear; U11 stays cleared |
| Valid loop/power, INHIBIT low, new ARM rising edge | D=1 transfers into Q; outputs enable |
| ARM remains high across stop release | No new rising edge, so no re-arm |

With /G high, G low means both line-driver outputs are **high impedance**, not
actively driven STEP-low. Confirm the exact AASD receiver/cable idle behavior.
Disabling during a high STEP can truncate that pulse. Do not treat pulse gating
as a complete machine stop or promise position integrity through an emergency
disable.

## Threshold calculation and acceptance voltage

Nominal falling cutoff:
`1.15 V × (1 + 32.0 kΩ / 10.0 kΩ) = 4.83 V`.

Using the supervisor's ±1% accuracy, resistor initial ±0.1%, and a conservative
±100 nA SENSE-current allowance gives:

| Corner | Falling threshold | Maximum rising threshold including 0.825% hysteresis |
|---|---:|---:|
| 25°C resistor reference | 4.771–4.889 V | 4.929 V |
| -40 to +125°C, opposing 25 ppm/°C resistor drift | 4.753–4.908 V | 4.948 V |

Thus use **at least 4.96 V measured driver rail** as a conservative full-temperature
static qualification target; 4.94 V covers the tabulated room-temperature corner.
These figures exclude aging, contamination and unbounded transients. The margin
to the AM26LS31C 4.75 V operating minimum is deliberately conservative but small
at the most extreme resistor-temperature corner.

The existing XL1509 low-output-tolerance unit may legitimately refuse to arm.
**Do not bypass the supervisor to force operation.** Qualify actual rail voltage,
ripple and load regulation; if an assembled unit cannot reach the valid threshold,
correct its regulator design in a new reviewed revision. Failure to arm is safer
than asserting a fictitious full-tolerance compatibility claim.

A supervisor has finite propagation/glitch-rejection delay. Static threshold
qualification is not proof that a rapidly collapsing rail never spends time out
of specification while outputs remain enabled.

## Bias/load calculation

AM26LS31 low-input current is up to 0.36 mA at 0.4 V. A 1 kΩ +1% resistor gives
approximately 0.364 V at this current. The old 10 kΩ did not establish a valid
worst-case low. Include MCU leakage; a pull-down does not overpower an actively
driven MCU boot glitch.

With U11 unpowered, the three G pull-downs in parallel give
`(3×0.36mA + 10µA U11 Ioff) × (1010Ω/3) = 0.367 V`.
A broken enable bus still leaves a local pull-down at each driver.

When enabled, the three pulls plus three LS high-input currents draw under
11 mA at 3.6 V. U11 guarantees at least 2.4 V high at 3.0 V supply and 16 mA load,
above the LS 2.0 V VIH requirement. Do not add a large series resistor on this
bus without repeating that check.

Each STEP/DIR high drives about 3.2 mA through its existing 33 Ω series resistor
and new 1 kΩ pull-down. Twelve high inputs add approximately 38 mA of MCU GPIO
load; include this in rail/thermal budgeting.

## Placement and timing

- Place U11–U14 near U2 with their individual bypass capacitors and short ground
  returns. U13 divider and CT must be local, away from switch-node copper.
- U11's ordinary CMOS inputs require the published maximum 10 ns/V transition
  rate at 3.3 V. CLK comes directly from the MCU, with no RC. /CLR comes directly
  from U14. U14 is necessary because the supervisor's 10 kΩ pull-up creates a slow
  rise. Do not omit it based on another vendor's generic “Schmitt action” claim.
- U12 accepts the existing **filter-side ESTOP_MCU**. Existing R60=1 kΩ,
  C36=100 nF and loop pull-up R1=10 kΩ give an approximately millisecond-scale
  opening delay; measure it. This filter is not a surge clamp or industrial input.
- C41=100 nF produces about 107 ms nominal release delay, approximately 78–158 ms
  from the specified charge-current/threshold limits and ±10% capacitance before
  additional effects. Firmware should wait **250 ms** after all permits are valid
  before emitting ARM. Assertion of reset does not wait for CT release delay.
- Route DRIVE_ENABLE with ground reference to the three driver blocks and local
  G pull-downs. Avoid antenna exclusion and USB/Ethernet pair corridors.
- Add accessible test points or documented probe pads for DRIVE_ENABLE,
  ARM_CLEAR_N, SUPERVISOR_RELEASE_N, VCC5, LOGIC_5V, VCC3V3 and GND where practical.
- Use device output state below valid supply only as a bench-test question. Logic
  operation is not specified below 1.65 V; the static zero-supply pull calculation
  does not prove every intermediate rail state.

## Deferred firmware requirements

1. Reserve GPIO47 active-high MCU_INHIBIT and GPIO48 ARM_PULSE. Never enable GPIO
   hold through reset/deep sleep on them or repurpose them as peripheral clocks.
2. First initialization: assert INHIBIT; preload ARM low before configuring output.
   Set all STEP outputs low and establish DIR while inhibited.
3. Every boot/watchdog/reconnect/stop-reset starts **unarmed**; no demo motion or
   automatic arm.
4. Explicit arm requires stable closed loop, valid supply and alarm/readiness
   inputs, valid transport, and correct reference/homing state. Set INHIBIT low,
   wait at least 250 ms, then emit a clean ARM low-high-low pulse (at least 10 µs
   high). Begin STEP only after enable settling and direction setup.
5. Fault/disarm asserts INHIBIT before other changes. Stop closure and power
   restoration must never automatically create an ARM edge.
6. Disarm before software reset. Validate reset modes/watchdog behavior that might
   preserve GPIO output state; a high-Z assumption is not universal.
7. Repair the independent reviewed stop-bypass, direction-guard, alarm, Ethernet
   packet-length and native-USB defects before motion. Firmware work is explicitly
   deferred by the user; the PCB is not a working motion controller without it.

## Prototype acceptance, before a powered mechanism

Use dummy receiver loads and capture all STEP pairs and DRIVE_ENABLE. Test USB-only
programming, USB/12 V hot plug, cold/slow ramps, reset button, bootloader, watchdog,
brownout, buck loss while USB remains, forced 3.3 V loss with buck present, and
power removal. Repeat with ARM held high, unarmed, inhibited and armed. Verify
loop opening inhibits without software service and neither loop closure nor
restored buck supply re-arms.

Confirm the intended drive receiver does not count an unintended positive STEP
edge at enable/disable. Validate all six matching DB25 cables by continuity,
pairing, common/shield assignment and drive settings. Commission an unloaded drive
only after dummy-load tests; never use a person as a first test load.

## Sourcing evidence and references

2026-09-22 catalog screens (provisional, not allocation): C485081
gross9764/canPresale9583; C70285 13510/13327; C1509297 591/578; C861325
9656/9627; C95204 1768570/1106298. All parts are required. Refresh the processed
JLC BOM for actual build/attrition quantity and check footprints/orientation.

Primary references:

- [TI AM26LS31 Rev N](https://www.ti.com/lit/ds/symlink/am26ls31.pdf): enable truth
  table, IIL, Ioff behavior, supply range. Modern die POR descriptions do not prove
  the exact inventory die in an assembled prototype.
- [TI SN74LVC1G98 Rev L](https://www.ti.com/lit/ds/symlink/sn74lvc1g98.pdf): Schmitt
  input truth table, pinout and output drive.
- [TI SN74LVC1G74 Rev G](https://www.ti.com/lit/ds/symlink/sn74lvc1g74.pdf): pin5 Q,
  asynchronous clear, input-transition limit, output drive and Ioff.
- [TI TPS3890 Rev A](https://www.ti.com/lit/ds/symlink/tps3890.pdf): adjustable 1.15 V
  threshold, accuracy, hysteresis, reset timing and package.
- [Espressif ESP32-S3](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
  and [module datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf):
  GPIO reset states, power domains and module pin mapping.
- [Yageo RT series](https://yageogroup.com/content/datasheet/asset/file/pyu-rt_1-to-0-01_rohs_l):
  B tolerance=0.1%, D TCR=25 ppm/°C.
