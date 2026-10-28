# R14 switch-field and six-drive fault interface contract

> Supporting design study. For the ordered 26 October revision, use
> [current decisions](DECISIONS.md), [review method](REVIEW_METHOD.md) and
> [hardware-test plan](BRINGUP.md). Dated proposals and open-status statements
> below retain their original context.

**Current status 2026-10-10:** the historical collector map is known and J12,
U17/U18 field-power cells and six Hall conditioners are saved/reopened in the
194-part/654-wire schematic checkpoint. The 12 indicators, all-axis alarm
gate, local hold, shield treatment and PCB synchronization/placement remain
open. The owner's physical collector, cable, installed sensors and drive
variants have not been traced. The earlier capture notes below document the
sequence; [current decisions](DECISIONS.md) and the [order record](ORDER_RECORD.json) govern current status.

**Schematic progress:** exact candidate jack `RJE231881317T` / `C5384839`
is now saved as J12 with pins 1–8 on the named field nets below. These are
isolated stubs; no protected feed, conditioner or shield circuit is connected.
The saved/reopened netlist comparison is in
`R14_J12_SCHEMATIC_CHECKPOINT.md` (historical working note).

## Jacks and cable identity

R13 J10 remains the W5500 network MagJack. R14 adds one **straight, plain 8P8C
modular jack without magnetics** for the original switch collector cable.
The two jacks must be separated and marked `NETWORK` and `12V SWITCH ARRAY`;
field-jack contact 2 carries 12 V in the historical design. Before selection,
check the chosen jack's mating orientation, numbered-contact drawing, shield,
footprint, assembly height and plug boot clearance. Document how misplugging
either cable is prevented or electrically tolerated. Do not connect this cable
to network magnetics, a switch, computer or Ethernet equipment.
An internal J12 alone is insufficient if the removable male plug at the
collector end can reach the external J10 network jack. Keep **both** ends of
the field cable captive/retained with no live loose 8P8C male able to reach
J10, or provide an equivalent non-interchangeable mechanical key and prove it
in the actual enclosure. The upstream field eFuse does **not** protect the
W5500/network jack from 12 V applied through a wrong-port insertion. Until the
physical mitigation is measured and reviewed, wrong-port survivability is an
open first-article order gate.
The exact Amphenol drawing and matching EasyEDA Standard/Pro library pad
numbering are captured in `FIELD_JACK_PAD_AUDIT.md` (historical working note).
Both original schematics agree on the contact numbers. Specify a
**straight-through cable, contact 1-to-1 through 8-to-8**. Use Pro pad 2 for
the protected sensor feed in the R14 schematic. Check placed-footprint
orientation in CAD and the actual retained cable during assembly/bring-up.

Place six labelled green/red indicator pairs on the **R14 PCB immediately beside
the field jack**, outside the mated collector plug, latch, boot, cable-bend and
hand-access envelope. Green means the conditioned NC line is low/closed; red
means high/open. Red can be a magnet edge, open conductor, unplugged collector
or lost field feed while `VCC5` remains present; with field `VCC5` off both
colors are dark, so neither LED color alone is a home or safety verdict.
Tap each protected 3.3 V Hall sense node with one channel of a six-channel
Schmitt inverter to drive the LED pair; do not load the raw 8P8C output or use
MCP23017 ports. See
`HALL_INDICATOR_DESIGN.md` (historical working note) for the conditional
`VCC5` driver, LED, 1.5 kΩ resistor and bypass candidates. The conditional
3.3 V-loss blanking circuit switches the common red supply and green return
separately; see `HALL_INDICATOR_BLANKING_REVIEW.md` (historical working note).
The AHCT14's 5 V ramp below 4.5 V remains an indication gate.
The owner confirmed this R14-side location. The original remote collector PCB
and its cable remain unchanged for compatibility.

| Historical contact | Candidate Pro symbol/footprint pad | Collector function | R14 proposed logical net |
|---:|---:|---|---|
| 1 | 1 | shared sensor ground | `FIELD_GND` |
| 2 | 2 | shared +12 V sensor feed | `FIELD_12V` |
| 3 | 3 | M1 Hall output | `HALL_0_FIELD` |
| 4 | 4 | M2 Hall output | `HALL_1_FIELD` |
| 5 | 5 | M3 Hall output | `HALL_2_FIELD` |
| 6 | 6 | M4 Hall output | `HALL_3_FIELD` |
| 7 | 7 | M5 Hall output | `HALL_4_FIELD` |
| 8 | 8 | M6 Hall output | `HALL_5_FIELD` |

This design map is supported by **both** the original collector and controller
schematics from Git commit `64332d7`, and by matching native Pro symbol and
footprint pin numbers. It assumes the retained cable is straight-through.
Amphenol's drawing does not label its contact tails; that omission does not
change the CAD net assignment. Record the actual cable's continuity and latch
orientation in `COLLECTOR_CONTACT_TRACE.csv` as assembly quality control before
applying 12 V. Do not infer contacts from wire color.

## Sensor interface that the R14 schematic must provide

- Protect and current-limit the sensor feed from the controller 12 V rail;
  measure aggregate six-sensor load, start surge and cable fault current before
  choosing the branch protection. Sense the **field-side** supply, not only the
  local 12 V rail, and keep the controller output gate disarmed on missing feed.
  A provisional latch-off eFuse candidate and current-limit screen are in
  `FIELD_FEED_PROTECTION_REVIEW.md` (historical working note); the
  protection row remains blocked pending actual load and fault measurements.
- Treat each six field output as a 12 V industrial signal until measured.
  Include input surge/ESD, polarity, filtering and 3.3 V translation or
  isolation; never wire a field signal or drive DI directly to an ESP32 pin.
  Choose conditioning from measured clear, triggered, unplugged, shorted and
  unpowered voltages/currents on the actual installed collector.
- Reserve six independent controller-readable states. The selected R14
  allocation is one 16-bit **MCP23017-E/SO I2C expander** on U2 GPIO41/42,
  with conditioned Hall 0-5 on GPA0-GPA5, field-supply-good on GPA6, two
  local hold-to-run readbacks on GPB2/3 and eFuse-fault readback on GPB4.
  Microchip's current DS20001952D marks GPA7 and GPB7 output-only.
  See [`SENSOR_IO_SELECTION.md`](SENSOR_IO_SELECTION.md). The earlier
  shared-SPI MCP23S17 proposal is withdrawn because the current W5500 firmware
  permanently acquires its SPI bus. Interrupt via GPIO3, boot-strapping,
  input latency, supply budget and an independent hardware stop remain gates.
  Sensor sampling alone is not a hardware stop path.
- The installed historical NJK-5002C is an NPN normally-open reference, not
  proof of current hardware. With a plain pull-up, inactive, open signal and
  unpowered remote sensor are indistinguishable. Therefore the old passive
  circuit alone cannot grant the owner's automatic pre-motion Hall-health
  permission. **OMCH NJK-5002D NPN-NC** is selected as the same-family M12 x 1
  replacement candidate; physical sample fit, lead functions and conditioner
  remain unverified. The owner accepts an automatic **electrical presence**
  check followed by a bounded seek. With each arm away from its magnet, the
  powered NC output must actively sink; an open/unpowered channel or missing
  field feed inhibits all motion. The owner reports the normal cold-start arm
  position is bottom under gravity, away from the 12 o'clock magnets. A
  high/open channel at that start is a fault and inhibits motion; electrically
  it remains ambiguous between a cable fault and an arm held at the magnet.
- A protected field-feed off/on low-high-low challenge may detect some
  stuck-low wiring faults, but only if that response is confirmed on the
  actual sensor, collector and **input circuit**. The input must keep sensing
  the field output while feed is off. If an optocoupler LED is powered by the
  switched feed, even an output short to ground can appear to follow an
  off/on challenge; do not count that topology as short detection without an
  independent observation. Electrical presence does not prove magnetic
  switching, magnet alignment, or a safe starting angle. Record these as
  accepted residual limits of the bounded-seek scheme, not as a claim of
  full pre-motion sensor proof.

## Drive alarms and global pulse stop

R13 has six optocoupler-conditioned drive alarm lines to GPIO1/2/15/16/18/40.
USB-only bring-up found all six high at rest, but with 12 V and drives absent;
the drive-side polarity and cable-loss behavior remain untested. R14 must
qualify every drive's alarm output, 12 V return and disconnected state before
using these inputs to permit motion. Review the R13 opto LED resistor current
at the actual 12 V range before drive connection.

Any one qualified drive alarm, Hall-health loss, invalid switch transition or
homing timeout must block new pulses to **all six** drives, latch the source,
cancel trajectories and invalidate the common home permission. The five
healthy drives remain enabled and hold; the alarmed drive follows its own
fault response. Keep pulse inhibition separate from `SRV-ON` torque enable.
Evaluate an alarm OR into a dedicated pulse gate so stopping does not depend
solely on a firmware task or shared SPI transaction. Such a gate must have a
defined power-up and MCU-reset state and be measured from alarm edge to last
STEP edge under maximum six-axis and network traffic. Its polarity, latch
and reset interaction with R13's existing U11–U14 logic need schematic proof.

## Local motion permission and step bounds

The existing E-stop/stop loop protects against an asserted stop; it does not
show that an operator is present. To meet the owner's unattended-motion
requirement, R14 needs a qualified local operator enable input in the common
pulse-permission chain, with an inactive/open/power-lost state that inhibits
STEP output. Choose the actual switch and contact scheme after the mechanical
control location and stop-loop wiring are confirmed. A software arm request
may only follow healthy startup diagnostics and this local permission; no
boot, network reconnect or fault clear may generate an arm edge. Releasing
local permission during movement latches a global command stop while the
healthy drives remain enabled to hold, subject to the measured drive scheme.
Keep the E-stop function independent of the local enable.

The ESP checks configured per-axis seek/time bounds during homing and signed
position bounds after a verified Hall reference. A count reaching a bound,
missing/implausible Hall edge or any drive alarm closes the common gate and
cancels six motion queues. These are commanded STEP counts. They cannot locate
an arm at first power-up or prove the servo followed the pulses. Set each
maximum seek count and time from the measured gravity-settled bottom range,
Hall-to-stop geometry and stopping margin. An MCU-only reset while powered
drives hold an arm elsewhere cannot establish bottom from the NC level alone;
the first seek remains inhibited until the bottom start is established. Record
the measured minimum stop clearance before authorizing mechanical homing.

## Bench acceptance matrix before R14 schematic freeze

| Injection or observation | Required evidence |
|---|---|
| Actual collector and cable continuity | All eight contact numbers, axis order, ground and supply match the recorded map; no shorts or swaps |
| Normal magnet-clear and magnet-present at each sensor | Raw field voltage/current, conditioned logic, response and repeatability at actual gap |
| Each of six green/red indicator pairs under clear, magnet, open and feed-loss states | With valid `VCC5`, green only when conditioned NC is low and red only when high/open; with `VCC5` absent both off, field-good inhibits motion; verify brightness and label visibility with plug mated |
| Lose 3.3 V while `VCC5` remains powered | Inhibit all motion; show that green does not falsely claim NC health, or qualify an added indicator-blanking circuit before schematic freeze |
| Feed absent, one sensor supply/ground/output open, whole cable unplugged | With arms at their gravity-settled bottom start, each fault makes a channel fail the away-state check; all axes remain inhibited |
| Each output shorted to ground and +12 V | Input protection survives; no false arming or false home |
| One drive alarm while six channels issue pulses | Last STEP edge timing, all-axis command halt, healthy drive hold and latched fault |
| Drive cable or drive power absent | Defined alarm/ready state; no unverified drive participates in homing |
| Controller reset, USB/12 V sequencing and W5500 traffic | Pulse gate stays inactive until all-six startup gates pass |
| Local enable open at boot, released mid-move or restored after a fault | No boot/reconnect auto-motion; all six STEP outputs stop, healthy drives hold, explicit recovery and new arm required |
| Homing or normal command just below/at/above each pulse bound | Out-of-range pulses never reach DB25; any live overrun latches a six-axis command stop |
| Magnet misses one Hall sensor during a slow seek | Axis reaches its measured pulse/time bound; all six command queues and STEP outputs stop, with fault source latched |

The exact sensor-health circuit, connector SKU, extra protective parts and
alarm gate remain open. Record selected values, footprints, simulation/bench
captures, source snapshot and DRC in a dated review before any future CAD promotion.
