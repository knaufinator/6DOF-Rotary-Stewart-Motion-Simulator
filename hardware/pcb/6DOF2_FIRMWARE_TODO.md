# 6DOF 2: firmware and commissioning TODO

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
- [ ] **F02 — Implement the r13 hardware-output enable.** The persisted r13 RC4 circuit
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
- [ ] **F03 — Fix physical pulse generation and counting.** In
  `controller/include/SharedMcpwmStepEngine.h`, actively suppress the waveform during
  direction-setup/hold guards, idle, final-step completion and stop. Confirm the IDF
  timer-empty versus timer-full event semantics and synchronize DIR updates to them.
  Use an independent counter/logic analyzer, not `totalSteps`, to prove zero extra
  pulses at start, stop, one-step moves and mid-move reversals on all six axes.
  Repeat under Ethernet/USB load, flash/cache/NVS work and reset/brownout conditions.
- [ ] **F04 — Six drive alarms.** Add GPIO1/2/15/16/18/40 mapping, measured polarity,
  startup qualification, filtering, fault latching and per-axis telemetry. Verify
  each actual drive/cable's alarm behavior and the effects of missing drive/12 V
  power. Alarm faults must participate in the same hardware-enable/arming contract.
- [ ] **F05 — External stop/drive-enable/brake contract.** The DB25 SRV-ON contact
  (pin 6) remains unconnected in r13; its latched STEP/DIR output inhibit does not
  independently command torque removal. Verify the released table against the actual
  drive/cable and provide the external drive stop/inhibit wiring.
  Define gravity/holding/brake behavior before any platform test; removal of torque
  is not automatically a safe state. A software pause is not a safety-rated stop.

## Transport and application correctness

- [ ] **F06 — Bidirectional native USB.** Fix `controller/sdkconfig.defaults.esp32s3`,
  generated configuration and `controller/main/CobsTransport.cpp`: current console
  input is UART0 while native USB is output-only secondary console. Use a primary
  USB Serial/JTAG or explicit USB receive path with binary-safe COBS framing. Test
  host-to-device and device-to-host, reconnect, framing errors, sustained load and
  manual BOOT/RESET recovery. ROM flashing does not prove application reception.
- [ ] **F07 — One versioned Ethernet protocol.** `EthernetTransport.cpp` currently
  delivers a 12-byte payload to `main.cpp`'s 18-byte / six-uint24 decoder. Update the
  host and all enabled network transports together, with exact length, byte order,
  axis order, range and version tests. Prove received packets change expected targets
  while disarmed observations cannot energize outputs. Wi-Fi/BLE wrappers need the
  same audit if built.
- [ ] **F08 — Unique W5500 MAC.** Derive/assign a valid unique Ethernet MAC from the
  ESP identity before netif attachment/start. Test DHCP, reconnect/link recovery,
  duplicate/invalid packets, and a multi-board network. Current code leaves this unset.
- [ ] **F09 — Known position, arming and command-loss policy.** Power up disarmed;
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
