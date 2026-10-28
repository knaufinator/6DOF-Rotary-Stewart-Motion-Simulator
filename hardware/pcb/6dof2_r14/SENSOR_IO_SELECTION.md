# R14 normally-closed Hall sensor and controller I/O selection

> Supporting design study. For the ordered 26 October revision, use
> [current decisions](DECISIONS.md), [review method](REVIEW_METHOD.md) and
> [hardware-test plan](BRINGUP.md). Dated proposals and open-status statements
> below retain their original context.

**Decision 2026-10-10:** select **OMCH NJK-5002D**, NPN normally closed,
as the six replacement 12-o'clock Hall sensors for R14 system design. Select
**Microchip MCP23017-E/SO**, JLC `C47023`, as the one 16-bit I2C expander for
the six conditioned Hall inputs. This is an architecture/part selection, not a
released Hall/IO circuit or permission for powered homing. U15 MCP23017 and
the two ESP I²C stubs are now saved in the R14 schematic; its support parts,
Hall conditioners and complete field circuit have not been entered. See
`R14_MCP23017_SCHEMATIC_CHECKPOINT.md` (historical working note)
and `R14_SENSOR_IO_PREFLIGHT.csv` for evidence and sourcing gates.

**2026-10-10 pin-capability correction:** Microchip's current
[DS20001952D data sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP23017-Data-Sheet-DS20001952.pdf)
states that MCP23017 `GPA7` and `GPB7` are **output-only**. The earlier
allocation of local-enable readback to GPA7 below is invalid. Use GPB2 and
GPB3 for the two proposed local hold-to-run contacts; GPB4 can read the
eFuse fault. GPA0–5 remain six Hall inputs and GPA6 field-good. GPA7/GPB7
must never be assigned sensor or motion-permission input roles. The corrected
capture contract is in
`R14_ELECTRICAL_IMPLEMENTATION_PROPOSAL.md` (historical working note).

**Owner acceptance update, 2026-10-10:** the required automatic startup check
is **electrical presence/continuity of each NC channel while its arm is away
from the 12-o'clock magnet**, followed by a measured maximum seek count and
time as the missed-edge backup. A pre-motion proof that the magnetic element
will switch is no longer an acceptance gate. After an accepted Hall edge,
referenced per-axis travel counts bound normal motion. This is a control
strategy, not a safety-rated sensor loop.
The owner reports all six arms normally settle **at bottom under gravity**
before a cold start, away from the Hall magnets. Use the measured spread of
these bottom positions as the first-seek starting sector; do not infer bottom
from an MCU-only reset while the drives remain powered.

## Sensor fit and electrical interface

OMCH lists `NJK-5002C` as NPN normally open and `NJK-5002D` as the NPN
normally closed choice in the same [NJK-5002 family](https://www.omchsmps.com/product/njk-5002c/).
The family lists a flush **M12 x 1** body, 10 mm +/-10% sensing distance,
0-8 mm setting distance, 10-30 VDC supply, 200 mA maximum load and 2 m cable.
Thus `NJK-5002D` matches the documented *thread and sensing family* of the
historical `NJK-5002C` without changing the collector's six-output contact
budget. The exact installed sensor markings, axial body length, cable exit,
thread engagement, magnet polarity/gap and lead colors have not been inspected;
these must match a physical `D` sample before claiming drop-in mechanical fit.
Order identity is **OMCH NJK-5002D**, no unnamed marketplace substitute.

The retained collector uses one supply, one ground and one signal per sensor,
then 8P8C contact 1 = common, 2 = +12 V, contacts 3-8 = six outputs in the
historical Git design. The `D` replacement is the same 3-wire NPN family,
but confirm its actual brown/blue/black lead function by manufacturer drawing
and bench meter before wiring; wire colors are not a contact-number proof.
At nominal 12 V, the 10-30 V OMCH rating is compatible **only after** the
field supply's worst-case minimum and maximum are measured. At the specified
maximum 15 mA per sensor, six sensors imply up to 90 mA steady draw, before
startup and cable-fault margins. Source branch protection to measured current.

Expected NPN-NC logic after a qualified low-voltage conditioner:

| Sensor condition | Raw output expectation | R14 decision |
|---|---|---|
| Powered, magnet absent | Sinks toward common | Healthy *electrical* away-state candidate |
| Powered, magnet at sensor | Opens | 12-o'clock edge candidate; stop approach, validate reference |
| Output wire open or sensor power absent | Often opens | Startup presence check fails when arm is away; inhibit all motion |
| Output short to common | Sinks even at magnet | Can falsely pass a static presence check; test supply-cycling diagnostic and retain bounded seek/alarm backup |

The last two rows are circuit expectations to verify on the actual device,
collector and conditioner. At startup, require field supply in range and all
six conditioned NC outputs in the **powered, magnet-away sinking state** for a
debounced interval. Because a normal cold start is at bottom, high/open is an
unexpected fault indication. The signal alone cannot distinguish a broken
channel from an arm somehow held at the magnet, so **do not move that arm or any
other arm** until the condition is resolved. If power interruption is proven
safe on a sample, switch only the protected sensor-feed branch off and on while
drives are inhibited; require each away-state output to follow low-high-low.
This extra test detects some shorts/stuck-low faults, but does not test magnet
pickup. Its coverage depends on the conditioner: if an optocoupler LED draws
from the switched sensor feed, an output short to common can still produce a
plausible off/on reading. Qualify the **whole input circuit** under injected
shorts before claiming the feed-cycle test detects them. Do not claim it
detects every cable or sensor failure.

The first seek is limited by an axis-specific pulse/time maximum and conservative
speed, plus the six-drive alarm-to-common-pulse-gate fault path. Measure the
gravity-settled bottom range and its minimum Hall-to-stop clearance before
assigning the count. The NC state alone proves only magnet-away, not bottom;
an MCU-only reset while arms are held elsewhere cannot reuse the bottom-start
assumption. On a missing edge or maximum
count/time, halt commands to **all six**, keep healthy drives holding, latch
fault and require deliberate recovery. The welded stop and drive alarm are
fallback faults, never routine homing events.

## Why an I2C expander, and actual ESP pin budget

The frozen R13 netlist identifies U2 as `ESP32-S3-WROOM-1-N8R2`. Its module
pin 34 / GPIO41 and pin 35 / GPIO42 have empty nets. Module pin 15 / GPIO3 is
also empty but is a [JTAG strapping pin](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf).
GPIO1/2/15/16/18/40 are drive alarms; 4-14/17 are six STEP/DIR pairs;
19/20 USB, 21 E-stop, 35-39 W5500 SPI/interrupt, 47/48 common inhibit/arm.
The six Hall lines therefore do not fit on direct free ESP inputs.

The earlier MCP23S17 suggestion is withdrawn for this firmware baseline.
`controller/main/EthernetTransport.cpp` initializes SPI3 on GPIO35/36/37 and
calls `spi_device_acquire_bus(s_spi, portMAX_DELAY)` for the W5500, explicitly
because another device does not share it. The existing `MCP23S17.cpp` is a
PCBv1 output driver; it initializes/uses a different bus model and is not a
drop-in R14 Hall input driver. Sharing SPI would require a measured Ethernet
bus refactor, including six-axis pulse timing under network load.

Use the [MCP23017](https://www.microchip.com/en-us/product/MCP23017) on a new
3.3 V I2C bus. It has 16 GPIOs, two interrupt outputs and 1.8-5.5 V supply.
The exact `MCP23017-E/SO` SOIC-28 part is [JLC `C47023`](https://jlcpcb.com/partdetail/MCP23017-E/SO/C47023),
listed for SMT assembly. EasyEDA's read-only public-library search returned
device `a62eb900bb2a42cf9b998d735c5543bd`, symbol
`1d42b3bbe78d44f9aa4b3e31373128cf`, footprint
`7c72ec7a49354a1199adc1e5b63ae042`; placed pads have not been audited.

| Resource | Proposed use | Capacity result |
|---|---|---|
| ESP GPIO41 (U2 pin 34) | I2C SDA, 3.3 V pull-up | One previously unconnected pin |
| ESP GPIO42 (U2 pin 35) | I2C SCL, 3.3 V pull-up | One previously unconnected pin |
| ESP GPIO3 (U2 pin 15) | Optional MCP23017 `INTA` interrupt, open drain/pull-up | Reserved, not required for six-channel count; boot/JTAG strap to bench-test |
| MCP23017 GPA0-GPA5 | Six conditioned NC Hall outputs | 6 of 16 GPIO |
| MCP23017 GPA6 | Conditioned field-supply-good status | 1 of 16 GPIO |
| MCP23017 GPA7 | Output-only on current Microchip data sheet; reserve/unconnected | 0 input capacity |
| MCP23017 GPB0 | W5500 reset control, if R14 reset improvement is adopted | 1 of 16 GPIO |
| MCP23017 GPB1 | Protected field-supply off/on diagnostic command, if qualified | 1 of 16 GPIO |
| MCP23017 GPB2-GPB3 | Two local hold-to-run contact readbacks; independent hardware gate still required | 2 input-capable GPIO |
| MCP23017 GPB4 | Field eFuse fault readback | 1 input-capable GPIO |
| MCP23017 GPB5-GPB6 | Reserved input-capable pins | 2 input-capable spares |
| MCP23017 GPB7 | Output-only on current Microchip data sheet; reserve/unconnected | 0 input capacity |

The corrected pinout uses **10 inputs + up to 2 optional outputs = 12 of 16**,
leaving two input-capable spares (`GPB5/6`) and two output-only reserves
(`GPA7/GPB7`). The six drive alarms stay on their direct U2 pins, so this
does not displace the alarm-to-all-axis stop requirement. The twelve requested
LEDs beside the field jack are driven by a six-channel Schmitt inverter tapped
from the conditioned Hall sense nodes, so they consume **zero** expander I/O;
see `HALL_INDICATOR_DESIGN.md` (historical working note). The indicator circuit
needs its own loaded `VCC5` rail, USB-only off-state, current-budget and
brightness check. The conditional AHCT14/LED/resistor choices are in the
indicator note; this bank is dark without field `VCC5`. I2C expanders and
their firmware are not safety-rated: losing communication, reset, stale data,
field supply or interrupts must close the independent common pulse gate and
invalidate motion permission. If `INTA` uses GPIO3, provide a deterministic
reset strap level and check JTAG/eFuse configuration on the actual module.
Without a qualified interrupt latency, use a conservative motion speed and
independent drive-side directional inhibition; I2C polling alone is not a
proven collision stop.

## Implementation and acceptance order

1. Read the installed sensor markings and measure the existing body/cable and
   magnet gap. Compare one genuine `NJK-5002D` sample, including its clear and
   magnet-present output truth table at minimum and maximum field voltage.
2. Continuity-trace all eight collector contacts and six sensor terminals.
   Confirm the `D` lead functions and verify field power/current and cable
   short behavior before connecting R14.
3. Freeze the 12 V-to-3.3 V input conditioner and field-feed monitor. Bench
   the power-off/on NC response, then implement the automatic six-channel
   away-state presence test (plus supply-cycle test if qualified). Inject open,
   short, unpowered, stuck-low and stuck-high faults. A channel high at startup
   blocks motion. In the normal bottom start this is a fault; the electronics
   alone cannot distinguish a broken channel from an arm at its magnet.
   Test magnet-miss as a **bounded-seek/global-fault** case, not as a required
   pre-motion magnetic proof.
4. Add MCP23017, I2C pins, input channels and optional interrupt/reset nets to
   the R14 schematic under an exact-project snapshot. Export and compare the
   complete netlist before PCB import. Audit the public footprint pin numbers
   and JLC assembly quantity, then place/route with connector clearances.
5. Implement an R14 I2C input driver with health timeout and common pulse-gate
   fault response. Test all six edges and disconnects during Ethernet/USB
   traffic; measure the last output STEP pulse, missed-edge maximum count/time,
   healthy-drive hold and no automatic restart.
