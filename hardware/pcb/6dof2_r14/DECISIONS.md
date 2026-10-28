# R14 design decisions

## 27 October 2026: R14C1 housing-clearance correction

Retain required J12 and every selected part. Move and reroute the interfering Hall-bank components rather than omit the interface. Use manufacturer maximum/conservative body bounds with placement allowance, closed LINE-loop coverage and complete STEP pair screening as independent gates. Relocate labels and test points with the parts. Keep the original order tag immutable and publish a complete replacement Gerber/BOM/CPL set. Circuit nets, sensor monitor and recovered firmware remain unchanged. [Decision, measurements and factory disposition](MECHANICAL_CORRECTION.md); [final acceptance evidence and remaining limits](review/2026-10-27-clearance/README.md).

This record explains the ordered 26 October 2026 design. Reference numbers and
net names refer to the [current schematic](drawings/R14-schematic.pdf), not an
earlier experimental checkpoint. Static review is complete; hardware qualification
is pending under [BRINGUP.md](BRINGUP.md).

## Sensor supply: retain 12 V and supervise its usable range

The selected OMCH NJK-5002D NC sensors operate at 10–30 V DC. **The board still
supplies nominal 12 V; 10 V is the sensor minimum, not a new regulator output.**
The fuse, protection path, board copper, contacts and 15 ft cable reduce the
voltage delivered remotely. A power-good monitor must account for those losses.

The earlier R14 monitor asserted good down to about 9.522 V on the board. That
could not establish 10 V at the sensors. U18 is now TLV6700 with R80/R81
27 kΩ / 1 kΩ, 0.1%, 25 ppm/°C. R82 remains the series input resistor; R83 is
the open-drain pullup and D9 is removed. Nominal falling/rising trips are
11.046 V / 11.2 V. The signal is a 3.3 V good/bad indication, not an ADC reading.

The corner budget includes reference/hysteresis range, resistor tolerance/drift
and input bias. Its minimum falling trip is 10.752752 V. The required hot complete
cable loop is ≤2.0 Ω including contacts, with ≤0.4 Ω additional board loop.
At a 0.257935 A current screen, the conditional remote floor is 10.133708 V.
The current data used are specified at 24 V, so this is not a proven hard 12 V
current ceiling. Actual current, cable, temperature and dynamic behavior must be
measured. See the [calculation](review/2026-10-26-10V/VOLTAGE_CONTRACT.json).

| Alternative considered | Decision |
| --- | --- |
| Keep 12 V with the old 9.522 V good threshold | Rejected: approves voltage below the selected sensor specification before cable loss. |
| Raise the threshold without reducing upstream loss | Insufficient across the broad low-input/load assumptions; earlier divider proposals demanded more rail voltage than the protected input could establish. |
| Add a regulated 10 V sensor rail | Not selected: 10 V is the minimum at the remote sensor and leaves no cable-loss margin. |
| Change to a lower-voltage sensor | Possible future option, but changes the selected NC sensor and requires electrical, mechanical and magnet-gap qualification. |
| Lower the input loss and monitor at a higher threshold | Selected, with an explicit supply/cable envelope and bench qualification. |

## Input loss: ideal-diode bypass with the existing startup path

U42 LM74700-Q1 and Q11 reduce steady input loss. Q11 is oriented gate/source/drain
to VIN_IDEAL_GATE/VIN12/VINRAW; the controller VCAP capacitor returns to VIN12.
The original D1 SS34 remains in parallel, in its original pose and polarity,
as a startup/surge path. Nominal input remains 12 V, maximum 12.6 V.
Startup, reverse input, removal/backfeed, surge and thermal operation still need tests.

Q11 originally selected DMN6075S-13 / C3280148. The order reported seven short.
DMN6075S-7 / C155242 is the same device/package with a different reel size and
packing quantity. Both masters now contain that code. The
[stock correction](review/2026-10-26-10V/Q11_STOCK_REFRESH.md) proves unchanged
physical geometry and manufacturing streams. Y1 C9006 was retained; the owner
identified its warning as an additional assembly charge.

## Pulse permission and stored torque are separate

U11 stores common pulse permission. U41 now implements:

```text
PULSE_CLEAR_N = ARM_CLEAR_N AND FIELD_VGOOD_3V3 AND ALL_TORQUE_ENABLED
```

Field-good also goes to MCP23017 GPA6 for firmware diagnostics. The direct U41
path inhibits pulses without waiting for I²C polling. Returning health does not
create a new arm edge. U23–U28 retain independent servo-on requests, so an ordinary
pulse stop need not deliberately release every healthy axis. An individual alarm
clears its own latch; auxiliary BAD or global release/reset has its own clear path.
Logic-Q readback is not measured mechanical torque, and this is not a replacement
for an independent machine emergency stop or mechanical support.

## NC sensing and bounded homing

The NC choice allows an unplugged/unpowered circuit to fail an away-from-magnet
presence check. Open can also mean a magnet, so it cannot establish home at boot.
A stuck-low wire can pass static presence. Six independent Hall states are retained;
no complete cable-fault diagnostic coverage is claimed. Homing needs calibrated
direction, a verified initial mechanical sector and explicit count/time bounds.

## Power supervision and interfaces

U17 is TPS26621, the auto-retry variant. Power retry must not rearm motion or restore
an invalid home. INA226 measures the 5 V driver bus, not current. Auxiliary boost
regulates to about 16.4686 V despite the inherited AUX15V net name; its RTN/window
topology is retained. The 12.6 V input ceiling preserves the reviewed UV separation,
but startup/backfeed/RTN/transients still need qualification.

R14 uses corrected DB25 contacts and a straight-through logical cable map; the
old R13 crossover adapter is incompatible. Field and Ethernet 8P8C ports have
different functions; wrong-port mating must be prevented in the installed system.
Physical contact continuity, SRV-ON polarity and drive parameters remain bench gates.

## Preserve tested behavior without overstating R13 evidence

The recovered R13 engine and gear configuration are the integration baseline.
Generic shared-core defaults remain unchanged; the full application supplies the
2:1 drive override. Later R14 quiescent stop/resume, validation and I/O work is
identified separately. The new ISR/adapter code is not physically qualified by
the older R13 soak. Normal R14 motion stays locked until the acceptance tests pass.

## Source and library discipline

Verified placed lands take precedence over a catalog footprint refresh. Explicit
overrides preserve the TI DDC land geometry for U18/U37 and the existing C82 lands.
Equivalent capacitor Device aliases and Name expressions were reconciled until
native comparison and DRC were empty. No broad schematic import was applied over
the reviewed layout. The [review method](REVIEW_METHOD.md) describes the independent
checks and the [section report](review/2026-10-26-10V/README.md) records every section.

Primary sources: [OMCH sensor](https://www.omchsmps.com/product/njk-5002c/),
[TI TLV6700](https://www.ti.com/lit/ds/symlink/tlv6700.pdf),
[TI LM74700-Q1](https://www.ti.com/lit/ds/symlink/lm74700-q1.pdf),
[Diodes DMN6075S](https://www.diodes.com/datasheet/download/DMN6075S.pdf),
[TI TPS2662](https://www.ti.com/lit/ds/symlink/tps2662.pdf).
