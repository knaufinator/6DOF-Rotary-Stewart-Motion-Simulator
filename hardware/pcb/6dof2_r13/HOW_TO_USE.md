# First use of r13 prototypes

**HARDWARE UNTESTED; FIRMWARE NOT MOTION-READY.** RC4 routing, native DRC and local
manufacturing-file checks are complete; see the [final package status](r13-rc4-2026-09-23/STATUS.md).
Actual JLC allocation/assembly gates remain open. No r13 board has been ordered or
tested. This is a staged
bench-check plan, not permission to connect the existing firmware to servo drives.
Complete [the firmware TODO](../6DOF2_FIRMWARE_TODO.md) before drive-connected work.

## What this board does

The board sends differential STEP/DIR commands to six external AASD servo drives.
It does not supply motor power or replace the drives' protection, external stop
system or brakes. JLC assembly alone does not supply a working motion application.
The current DB25 SRV-ON contact is not a substitute for an independently reviewed
drive enable/stop/holding arrangement.

## Before power

- Match the physical PCB revision to its final release package and assembly record.
  Photograph both sides; inspect connector population/orientation and soldering.
- Keep **all six drive cables disconnected**. Leave the stop loop open so the
  board remains inhibited. Do not fit a run-loop jumper simply to make tests pass.
- Secure the PCB on nonconductive supports using its intended mounts. Do not press
  connector plugs into an unsupported board or allow loose screws to short copper.
- Check supply polarity, obvious shorts and component orientation using the released
  schematic. Never connect external power or a scope ground lead to an assumed pin.
- Use ESD precautions and protected/current-limited bench sources. Set supply/current
  limits from the final electrical test specification; arbitrary limits are not a
  substitute for that specification. Stop for unexpected current, rail voltage,
  heating, smell or visible damage; disconnect and investigate.

Required bench equipment includes a meter, suitable current-limited 12 V source,
a protected USB data/programming connection and a logic analyzer/scope appropriate
for the measured signals. Measure differential outputs with an appropriate method;
do not put an earth-referenced scope ground clip on an RS-422 signal output.

## Power modes — intended r13 contract, not yet measured

| Power applied | Intended capability | Mandatory output behavior |
|---|---|---|
| USB alone | Logic power, ROM programming/recovery; later application USB after firmware fixes | Driver supply unavailable; STEP/DIR interface inhibited |
| External 12 V alone | Logic and differential-driver supplies available | Disarmed by default; healthy inputs and deliberate fresh arm required |
| USB plus external 12 V | Programming/data with normal external power | No source backfeeding; same stop/arm rules |
| 12 V removed while USB stays | Logic may remain powered | Immediately inhibit and clear armed state; no automatic resume when 12 V returns |

**USB-alone programming must not require a 12 V supply.** Conversely, USB-only
operation is not a supported way to drive motors. The D3/D8 split/reverse-blocked
logic supply, driver-only buck rail and supervisory latch must pass supply-order,
power removal and reset measurements before this behavior is accepted as proven.

## USB programming and recovery

1. Keep the drives disconnected and the stop input open. Use USB data cable to J11;
   a charge-only cable cannot provide programming.
2. Verify expected logic power/current and ROM USB detection. If manual entry is
   needed, hold **BOOT**, press/release **RESET**, then release BOOT after entering
   download mode. Use the final programming guide for the exact board/tool settings.
3. Program only a reviewed **r13 bench image** with recorded version/hash and safe
   defaults. Such an image is not supplied yet; do not treat the repository's current
   production binary or a build-success report as one.
4. Application USB input needs the documented transport fix. Successful ROM flashing
   is not evidence that COBS commands or motion packets are received correctly.

The old GPIO48 WS2812 status driver conflicts with r13's planned ARM input. It must
be disabled/remapped before firmware is allowed to control this hardware. GPIO47
must initialize HIGH/inhibit and GPIO48 LOW; arming requires a qualified new edge,
not a stale state or LED waveform. Follow the final verified hardware contract if
the implementation differs from this design target.

## Board-only qualification before a drive cable

Use a suitable fixture and reviewed bench firmware, with external drive power absent:

1. Measure rails/current under USB only, 12 V only and both supplies, including
   connecting/removing them in either order and checking reverse current. Test
   USB programming at the planned worst-case USB supply/cable condition.
2. Verify reset, brownout, stop-open and lost-driver-supply all disable the outputs
   and latch disarmed. Restoring the loop/supply must not silently re-arm.
3. Verify the line-driver enable/idle state electrically; do not infer it only from
   software telemetry. Test a deliberate qualified arm and immediate inhibit with
   receiver loads, never a powered mechanism at this stage.
4. Independently count/measure all six STEP and DIR channels through the DB25s,
   including start, single/final pulse, reversal, idle, reset, asserted stop,
   communication loss and simultaneous network activity. Software counters alone
   cannot validate physical pulse output.
5. Exercise each alarm input with the final rated test interface and check polarity,
   qualification and fault latching. Do not apply an arbitrary external voltage.
6. Validate native USB reception, Ethernet identity/link/protocol, reconnect and
   command timeout. Current firmware has known defects in these paths.
7. Measure steady/transient loads and regulator temperatures under the planned
   worst case. A cold unloaded board does not establish thermal margin.

Record board serial/revision, component deviations, firmware hash, test setup,
measurements, limits and pass/fail results. Stop at any failed or unspecified gate.

## Only after board and firmware acceptance

Continuity-check the owner's six DB25 cables against the final table:
STEP+ = 3, STEP- = 14, DIR+ = 4, DIR- = 5, COM = 10, ALARM = 23.
Check actual connector numbering/view, conductor pairs/shields and the exact
drive-side manual. Matching plug shapes are not proof of electrical compatibility.
See [the canonical table](../6DOF2_BOARD.md) for the SRV-ON limitation.

Then follow the [single-axis commissioning gate](../single_motor_test_plan.md): one
secured, mechanically unloaded motor first, with guarded work area and reviewed
independent stop/inhibit/brake behavior. Establish direction, gearing, position and
travel limits at bounded low rate/count before advancing. The full mechanism must
remain mechanically supported and unoccupied until all six axes and fault responses
are validated. No ready-to-plug or occupied-platform safety claim is made.
