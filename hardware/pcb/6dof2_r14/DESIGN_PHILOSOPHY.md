# R14 design philosophy and checkable boundaries

Pulse permission and healthy-axis holding torque have separate hardware paths.
An ordinary pulse stop need not release all healthy drives. The controller still
requires an independently designed machine emergency stop and mechanical support.

![Implemented pulse, torque and sensing boundaries](images/r14-permission-paths.svg)

## Implemented pulse inhibition

U11 stores common pulse permission; GPIO47 inhibits and GPIO48 supplies a deliberate
arm pulse. U41 is now fitted in the corrected master: its three inputs are
`ARM_CLEAR_N`, `FIELD_VGOOD_3V3` and `ALL_TORQUE_ENABLED`; its output is
`PULSE_CLEAR_N`, connected to U11 clear. Field-good or any torque-latch loss therefore
inhibits pulses independently of I²C polling. Returning power does not create an arm edge.
Propagation and physical last-pulse behavior remain prototype measurements.

U23–U28 retain six independent servo-on requests. Reset/release, auxiliary BAD and
the corresponding drive alarm control their clear paths. One drive alarm clears
that drive's request; healthy requests can remain asserted. A logic-Q aggregate
is evidence of latch state, not measured mechanical torque.

## A sensor budget includes the cable

![Corrected remote-sensor power and monitor path](images/r14-10v-power-path.svg)

The selected NC sensors require 10 V minimum. The old 9.522 V monitor could not
establish this. U18 is now TLV6700 with a 27 kΩ / 1 kΩ precision divider:
nominal falling trip 11.046 V and rising trip 11.2 V. R82 remains the series
input resistor; R83 provides the open-drain 3.3 V pullup and D9 is removed.
U42 LM74700-Q1 and Q11 DMN6075S reduce steady input loss; the original D1 SS34
retains its parallel startup/surge path.

The confirmed length is 15 ft Ethernet. The qualification contract requires a
**measured hot complete cable loop including contacts ≤2.0 Ω**, plus additional
board loop ≤0.4 Ω. Gauge and cable model are not confirmed; length alone is insufficient.
The corner calculation yields a minimum falling trip of 10.752752 V and a
conditional steady remote floor of 10.133708 V using a 0.257935 A current screen.
TPS2662 current-limit data used for that screen are specified at 24 V; they do not
prove a hard 12 V current ceiling. Cable/load/temperature, startup, trip latency
and recovery must be measured under [BRINGUP.md](BRINGUP.md).

Use nominal 12 V input, maximum 12.6 V. This preserves the existing auxiliary-boost
undervoltage separation; arbitrary low-input startup is not established. U17 is
TPS26621DRCR, the auto-retry variant. Power retry must never automatically restore
pulse permission or a previously invalid home.

## Feedback has a defined scope

Hall closed means the NC electrical path is present away from the magnet. Open can
mean the magnet, broken wire or lost field power; it cannot prove home at boot.
Bounded homing needs healthy power, calibrated direction, known initial mechanical
conditions and count/time limits. R14 motion admission remains locked until qualified.

INA226 measures driver-bus voltage, not shunt current. Software MCPWM counts are
not physical output counts. Independent pulse measurements, voltage measurements,
alarm checks and cable continuity establish different facts.

## Interfaces and manufacture

R14 uses corrected DB25 contacts and a verified straight-through logical cable map;
historical R13 crossover adapters are incompatible. Actual cable continuity,
SRV-ON polarity and drive parameters require disconnected-drive acceptance tests.
The field 8P8C jack carries power and must be prevented from mating with network
ports through keying/captive wiring and clear labels at both ends.

The [final review](review/2026-10-26-10V/README.md) checks native lands, exported
mask/paste, drills, labels and assembly identities. Factory stencil/rotation/process
approval and physical enclosure/hood fit remain separate. STEP is a CAD assembly,
with some conservative generic package models; it does not prove installed fit.

## Primary references

- [TI TLV6700 thresholds, input limits and DDC layout](https://www.ti.com/lit/ds/symlink/tlv6700.pdf).
- [TI LM74700-Q1 ideal-diode application](https://www.ti.com/lit/ds/symlink/lm74700-q1.pdf).
- [Diodes DMN6075S pinout and ratings](https://www.diodes.com/datasheet/download/DMN6075S.pdf).
- [TI TPS2662 family and auto-retry](https://www.ti.com/lit/ds/symlink/tps2662.pdf).
- [Microchip MCP23017 registers and pin restrictions](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP23017-Data-Sheet-DS20001952.pdf).
- [Original voltage feasibility study](R14_FIELD_VGOOD_12V_FEASIBILITY_FOLLOWUP_20261017.md), historical assumptions superseded by the current voltage contract.
- [Firmware evidence](FIRMWARE_STATUS.md) and [final voltage calculation](review/2026-10-26-10V/VOLTAGE_CONTRACT.json).
