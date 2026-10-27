# 6DOF 2: firmware and commissioning TODO

> **Current status — 2026-10-26:** R14 is ordered, confirmed by the owner.
> See [current PCB/schematic and decisions](6dof2_r14/README.md) and
> [firmware evidence](6dof2_r14/FIRMWARE_STATUS.md). Hardware testing is pending.
> Dated sections below are history and do not establish current qualification.


Status: **UNIMPLEMENTED / UNTESTED**, 2026-09-23. Hardware r13 RC4 has a routed,
persisted manufacturing handoff for prototype testing; nothing has been ordered. The owner requested
hardware changes now and firmware later. No item below is satisfied merely by a
successful build, zero PCB DRC errors or factory assembly.

This backlog applies to full-size `controller/`, not the hobby-servo `controller/mini/` build.
It supersedes motion-ready claims in older README, pin reviews and test plans.
Hardware/package status: [6DOF2_BOARD.md](6DOF2_BOARD.md).

## Before any servo drive is connected

- [ ] **F01 — One motion-permission state machine.** In `controller/main/main.cpp`,
  make stop enforcement continuous, including a physical loop already open at
  boot. `ESTOP:RESET` must not resume with the loop open. Acknowledging a fault must
  remain distinct from explicit re-arming. Gate `PINTEST`, `SIGTEST`, `FREQTEST`,
  `MTEST`, normal targets and every direct STEP/DIR write through the same contract.
  Test every command with asserted stop, stopped-at-boot, reset, brownout and fault.
- [~] **F02 — Implement the r13 hardware-output enable.** 2026-10-08 minimal part done:
  GPIO47 HIGH / GPIO48 LOW are the first thing `app_main` does, and the GPIO48 WS2812
  `LedStatus` driver is compiled out for PCB_VERSION 2. No arm sequence exists; outputs stay
  inhibited. Everything else in this item remains.
- [ ] **F02 (original text) — Implement the r13 hardware-output enable.** The persisted r13 RC4 circuit
  has GPIO47 = HIGH inhibit/default stop, GPIO48 = fresh arm edge into a supervised
  hardware latch. Schematic/PCB implementation is audited; electrical operation and
  firmware behavior remain untested. Add the
  released GPIO polarity/timing to `helpers.h` and initialization. Keep outputs
  disabled through reset, boot, download, unconfigured state, fault and loss of
  command. Set stable idle STEP/DIR levels before enabling; disable before changing
  GPIO ownership or running disconnected-drive diagnostics. Initialize GPIO47
  HIGH and GPIO48 LOW before any peripheral takes ownership. **Disable/remove or
  remap the existing GPIO48 WS2812 `LedStatus` driver**: its waveform must never
  reach the new arm input. Firmware reset/stop must not re-arm by a stale HIGH;
  qualify all inputs and require a deliberate new arm edge. Confirm exact net names,
  latch timing and supply qualifications from the final source. Asserted stop must
  inhibit outputs independently of the software request; USB-only operation must
  remain inhibited. Test GPIO48 status traffic is absent at boot and every state.
- [ ] **F03 — Fix physical pulse generation and counting.** Note 2026-10-08: with no motion
  commanded, the step engine already loads core 1 enough that an SPI polling transaction
  there took ~1 ms instead of ~15 us; whatever ISR runs continuously should idle when there
  is nothing to step.
- [ ] **F03 (original text) — Fix physical pulse generation and counting.** In
  `controller/include/SharedMcpwmStepEngine.h`, actively suppress the waveform during
  direction-setup/hold guards, idle, final-step completion and stop. Confirm the IDF
  timer-empty versus timer-full event semantics and synchronize DIR updates to them.
  Use an independent counter/logic analyzer, not `totalSteps`, to prove zero extra
  pulses at start, stop, one-step moves and mid-move reversals on all six axes.
  Repeat under Ethernet/USB load, flash/cache/NVS work and reset/brownout conditions.
- [ ] **F04 — Six drive alarms.** Add GPIO1/2/15/16/18/40 mapping, measured polarity,
  startup qualification, filtering, fault latching and per-axis telemetry. Verify
  each actual drive/cable's alarm behavior and the effects of missing drive/12 V
  power. **Owner's R14 requirement:** an alarm from any one drive must stop
  commanded motion on all six axes. Inhibit the global STEP/DIR output gate,
  terminate all trajectories, reject new motion commands, record fault axis/time,
  invalidate home validity and require explicit reset plus a fresh arm sequence;
  never resume when the alarm merely clears. The owner specifies that the five
  healthy drives stay enabled and hold position; do not command them home or
  remove their drive enable as the ordinary alarm response. Exercise each alarm input while
  six axes are pulsing under host traffic; use an independent counter/logic
  analyzer to measure the last output pulse and response latency. Test alarm
  already asserted at boot, cable/power loss and reset. Alarm faults must
  participate in the same hardware-enable/arming contract. The R13 schematic
  has no direct alarm-to-gate connection and current firmware never arms the
  common latch; implement and prove both sides as specified in
  `6dof2_r14/ALARM_GLOBAL_STOP_DESIGN.md`.
- [ ] **F05 — External stop/drive-enable/brake contract.** The DB25 SRV-ON contact
  (pin 6) remains unconnected in r13; its latched STEP/DIR output inhibit does not
  independently command torque removal. Verify the released table against the actual
  drive/cable and provide the external drive stop/inhibit wiring.
  Define gravity/holding/brake behavior before any platform test; removal of torque
  is not automatically a safe state. A software pause is not a safety-rated stop.

## Transport and application correctness

- [~] **F06 — Bidirectional native USB.** 2026-10-08: console moved to USB-Serial-JTAG
  (`sdkconfig.defaults.esp32s3`), driver installed with 4 kB rings in `CobsTransport.cpp`,
  console writes bounded (0 ticks) so a closed port cannot stall the firmware. UART0/J1 is free.
  Still to test: app over COM10 (handshake + stream), reconnect, framing errors, sustained load.
- [ ] **F06 (original text) — Bidirectional native USB.** Fix `controller/sdkconfig.defaults.esp32s3`,
  generated configuration and `controller/main/CobsTransport.cpp`: current console
  input is UART0 while native USB is output-only secondary console. Use a primary
  USB Serial/JTAG or explicit USB receive path with binary-safe COBS framing. Test
  host-to-device and device-to-host, reconnect, framing errors, sustained load and
  manual BOOT/RESET recovery. ROM flashing does not prove application reception.
- [~] **F07 — One versioned Ethernet protocol.** 2026-10-08: Ethernet carries the USB wire
  format verbatim (one COBS frame per datagram, FW_PROTOCOL_VERSION 2) on a W5500 hardware UDP
  socket (`EthernetTransport.cpp`): same dispatcher as USB, DATA18 / DATA18_SEQ (0x08, 16-bit
  seq) / CMD in, RESP / LOG / TEL mirrored back to the last peer. esp_eth + lwIP run only until
  the DHCP lease, then stop (MACRAW socket 0 and the hardware stack cannot share one IP: ARP and
  ICMP split between them and every hardware SEND stalled). Link loss restarts lwIP for a new
  lease. Measured from a wired host: 2.0 ms command round trip, 10,000/10,000 frames at 1 kHz
  and at 2 kHz with zero gaps. The socket task must run on core 0: on core 1 the step engine's
  interrupt load stretched every SPI transaction to ~1 ms (see F03). Desktop app: direct mode
  in `UdpTransport` runs the FINGERPRINT handshake over UDP and streams DATA18_SEQ.
  `PEERMAC:<mac>` + W5500 SEND_MAC lets the board answer hosts that ignore ARP (Wi-Fi laptops);
  mirrored sends never block the receive path.
  Still to do: flood tolerance test, Wi-Fi/BLE wrappers untouched, bridge mode unchanged.
- [ ] **F07 (original text) — One versioned Ethernet protocol.** `EthernetTransport.cpp` currently
  delivers a 12-byte payload to `main.cpp`'s 18-byte / six-uint24 decoder. Update the
  host and all enabled network transports together, with exact length, byte order,
  axis order, range and version tests. Prove received packets change expected targets
  while disarmed observations cannot energize outputs. Wi-Fi/BLE wrappers need the
  same audit if built.
- [x] **F08 — Unique W5500 MAC.** 2026-10-08: `esp_read_mac(ESP_MAC_ETH)` is written to the
  driver before start (90:70:69:8d:49:63 on the first article). Multi-board test still open.
- [ ] **F08 (original text) — Unique W5500 MAC.** Derive/assign a valid unique Ethernet MAC from the
  ESP identity before netif attachment/start. Test DHCP, reconnect/link recovery,
  duplicate/invalid packets, and a multi-board network. Current code leaves this unset.
- [~] **F09 — Known position, arming and command-loss policy.** 2026-10-08: sequence tracking
  (gaps, lost, restarts) and command age are measured (`cobs_motion_stats`, `INPUT_STAT`); no
  timeout behaviour acts on them yet.
- [ ] **F09 (original text) — Known position, arming and command-loss policy.** Power up disarmed;
  qualify configuration, all required inputs, drive state and a measured home/known
  position before arming. `ZERO` only resets counters and is not homing. Add command
  age/sequence tracking, timeout behavior and no automatic resume after reconnect.
  The task watchdog is not a motion-command watchdog.
- [ ] **F10 — Motion limits and persistent configuration.** Implement and test
  bounded velocity/acceleration, per-axis inversion/home/travel/drivetrain parameters,
  versioned NVS with readback, finite/range validation and an invalid-workspace/IK
  policy. Current DRIVE settings do not persist, and default fixed count limits are
  not measured mechanical travel limits. Do not let an old active-source setting
  arm or move a newly booted board.
- [ ] **F11 — R14 controller-owned six-switch homing and horizontal offset.** The owner's
  retained collector has one 12 o'clock Hall end switch per arm and no physical
  opposite-end sensor (see `6dof2_r14/CONTROLLER_HOMING_DESIGN.md`). The R14
  controller is the recommended owner of the seek, Hall edge, step offset,
  position validity and global alarm state; disable/reconcile drive-internal
  auto-homing/offset settings first. On a verified R14 input path,
  implement a bounded, limited-speed seek and repeatable switch-edge capture,
  permit controlled retreat, then execute a measured signed step offset to each
  arm's horizontal start. Persist/version per-axis offsets, direction, switch
  polarity and measured travel limits with readback; decide whether the drive or
  controller owns the offset. Reject stuck/never-triggered/noisy switches and
  lost home validity; never treat `ZERO` as a physical home. No R13 hardware has
  these inputs, and no R14 homing behavior is implemented yet. The owner reports
  a welded physical stop beyond a missed Hall trigger, where contact **trips
  a drive alarm and the servo stops working**. Raw NPN-NO inactive readings cannot prove
  the collector is powered or a wire intact. The accepted R14 startup gate
  automatically checks valid field supply and all six magnet-clear NC channels
  actively sinking; any high/open or unpowered reading inhibits motion. No
  manual magnet test is required. A protected field-feed cycle may be added
  after bench qualification. Seek at limited speed with a measured maximum
  STEP count and time; a missed Hall edge closes the common pulse gate and
  stops all six command queues. Count limits cannot guarantee clearance from
  an unknown starting angle. The owner reports that all arms normally settle
  at bottom under gravity before a cold start. Measure each bottom range and
  clearance to the stop, then derive the seek bound from that sector. An
  MCU-only reset while powered drives hold position cannot assume bottom.
  Latch any drive stall alarm and prevent automatic
  retry; verify its code, latency and post-fault torque separately from a
  commanded stop. R13/R14 STEP counters are not motor encoder feedback; qualify
  actual-position evidence and safe axis order on the coupled platform. See
  `6dof2_r14/HOMING_FAULT_ANALYSIS.md`.

- [ ] **F09 R14 attendance gate.** Automatic startup may run sensor/drive
  diagnostics, but must never start arm motion. Boot, transport reconnect,
  fault clear and restored host frames require a fresh deliberate arm request.
  A network command does not prove an operator is physically present: if
  unattended movement must be impossible, implement and qualify a local
  enable or observed-operator interlock in the R14 motion-permission path.
  Releasing it during motion must close the common STEP/DIR gate and latch a
  non-resuming stop; keep healthy servo drives enabled to hold where qualified.
- [ ] **F11 R14 pulse-count acceptance.** Store versioned, signed per-axis
  approach direction, maximum seek/back-off/slow-edge pulses and time,
  Hall-to-horizontal offset, and minimum/maximum position counts relative to
  the verified Hall edge. Before scheduling any pulse from direct, queued or
  streamed commands, check the bound and platform workspace. Missing or
  implausible Hall edge, reached bound, alarm, lost input validity or lost local
  permission must close the common gate, cancel all six queues, latch fault and
  invalidate home. Prove one pulse below/at/above each limit, under host traffic
  and reset/reconnect. A first seek requires the gravity-settled bottom start
  assumed by the bound to be established; counting from an arbitrary held
  angle cannot guarantee the welded stop is avoided. Commanded pulse counts
  are not encoder feedback.
- [ ] **F11 R14 sensor/IO allocation, selected 2026-10-10.** Six OMCH
  `NJK-5002D` NPN-NC M12 x 1 sensors retain the original collector's six
  signal contacts. One MCP23017-E/SO (`C47023`) adds 16 I2C GPIO on free U2
  GPIO41/42; conditioned Hall 0-5 occupy GPA0-GPA5, field-good GPA6 and local
  enable readback GPA7. Two optional GPB outputs leave six spare. GPIO3 is
  reserved for optional expander interrupt only after JTAG boot-strap testing.
  Write a new PCBv2 I2C input driver, with timeout/stale-read detection and
  common-pulse-gate fault response. The PCBv1 MCP23S17 driver cannot simply be
  reused: current W5500 code permanently acquires SPI3 for its own throughput.
  Benchmark six Hall reads and interrupt-to-last-STEP latency under Ethernet
  load; a bus failure, input power failure or expander reset inhibits motion.
  A high/open channel at the normal gravity-settled bottom start is a fault
  and must inhibit motion; electrically it remains ambiguous between a broken
  channel and an arm held at the magnet. The NC swap does not prove magnetic
  switching before first movement; the owner accepts this residual limit with
  bounded seeking and global stop on missing edge, input loss or drive alarm.
  Exact table and open gates: `6dof2_r14/SENSOR_IO_SELECTION.md`.

## Reproducible bench / factory package

- [ ] Freeze source revision and dirty-change state, ESP-IDF version, PCB version,
  selected backend, flash/PSRAM settings and application protocol version.
- [ ] Produce a corrected, safe-default commissioning image with explicit offsets,
  merged image if used, SHA-256 hashes, programming/recovery instructions, version
  readback and known safe NVS defaults. No current binary is an r13 release image.
- [ ] Define a fixture and logged pass/fail limits: supply rails/input current,
  reset/brownout and power sequencing, temperature under worst-case measured load,
  USB, Ethernet, all 12 differential outputs, six alarm inputs and stop/enable.
  Exercise output receivers without energizing a motion platform.
- [ ] If JLC programming or functional testing is desired, obtain explicit service
  acceptance and a quote using this image/fixture/test specification. Ordinary PCBA
  does not include validated firmware or a working motion application.
- [ ] Continuity-test the owner's six matching DB25 cables against the released
  table, including pin numbering/view, shield, COM, pairing and drive-side assignments.
  Verify actual drive model/manual and settings before an unloaded single-axis test.
- [ ] Record independent waveform and unloaded-axis results; then verify all six
  axes on a mechanically supported, unoccupied mechanism. No occupied-platform use
  until the system-level stop/holding strategy and operating limits are validated.

## Review provenance

The September 22 review found these defects in the actual full-controller source,
including existing staged GPIO21 / active-HIGH E-stop changes. An isolated build with
ESP-IDF 5.5.2 succeeded, but no electrical or motor test was performed. Source review
and build success are not functional acceptance. Original detailed evidence is in
`G:/My Drive/projects/6dof/pcb/reviews/2026-09-22/` (`firmware_review.md`,
`electrical_review.md`, `REVIEW.md`); these local paths are not portable repository
deliverables. This TODO preserves the actionable requirements within the repository.

- [ ] **F-R14-EFUSE — emulate eFuse latch-off (2026-10-17).** R14 U17 is now TI TPS26621 (auto-retry) instead of
  TPS26620 (latch-off) for sourcing. On FIELD_EFUSE_FLT_N low (U15 input), drive FIELD_FEED_ENABLE_3V3 (U15 output →
  U17 SHDN) low and hold it until an explicit operator recovery; never re-enable automatically.
