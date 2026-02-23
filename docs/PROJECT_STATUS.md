# 6DOF Rotary Stewart Motion Simulator — Project Status

Global tracking of milestones across firmware, hardware, desktop app, and integration.

## Milestone Summary

| # | Milestone | Status | Date |
|---|-----------|--------|------|
| M1 | Desktop app with SIL 3D viz + IK | ✅ Complete | 2025 |
| M2 | Serial bridge + ESP32 comms | ✅ Complete | 2025 |
| M3 | SimTools UDP integration | ✅ Complete | 2025 |
| M4 | Desktop app beyond-parity (recording, spectrogram, test gen) | ✅ Complete | 2025 |
| M5 | ESP32-S3 firmware port (MCPWM, GPTimer, multi-transport) | ✅ Complete | 2025 |
| M6 | ESP-to-ESP signal validation (protoboard) | ✅ Complete | 2026-02-22 |
| M7 | PCB signal validation | 🔲 Pending | — |
| M8 | SN75174N differential driver integration | 🔲 Pending | — |
| M9 | Single motor bench test (AASD-15A) | 🔲 Pending | — |
| M10 | Full 6-motor system integration | 🔲 Pending | — |
| M11 | Motion platform end-to-end (SimTools → motors) | 🔲 Pending | — |

---

## M6: ESP-to-ESP Signal Validation ✅

**Date**: 2026-02-22  
**Hardware**: Two ESP32-S3-DevKitC-1-N8R2 on protoboard, 13 jumper wires  
**Boards**: COM3 (controller), COM7 (analyzer)

### Tests Passed

| Test | Method | Result |
|------|--------|--------|
| Pin mapping (12 pins) | `PINTEST` — static HIGH/LOW each pin | 12/12 correct, zero swaps |
| Motor test (6 motors) | `MTEST` — 200 steps fwd + 200 back | 6/6 pass, all return to pos=0 |
| Crosstalk sweep | `FREQTEST` — 1Hz to 250kHz on GPIO5 | **0% crosstalk** all frequencies |
| Max step rate | `FREQTEST:1:250000:1000` | ~153 kHz achieved (single motor) |
| Simultaneous rate | `MTEST` all 6 motors | ~17 kHz per motor |

### Issues Found & Resolved

- **Solder blob** on analyzer board shorted GPIO5↔GPIO6 — removed, 0% crosstalk confirmed
- **GPIO layout** on DevKitC-1: GPIO 8, 9, 14, 17 are non-sequential on the left header
- **RGB LED** on GPIO48 requires solder bridge on junction pad to function

### Firmware Additions for Testing

| Command | Description |
|---------|-------------|
| `MTEST` / `MTEST:N` | Run all or single motor self-test (200 steps fwd/back) |
| `PINTEST` | Static pin-by-pin HIGH/LOW validation (reboot after) |
| `FREQTEST:M:F:D` | Generate exact frequency pulses on motor M (reboot after) |
| `MTEST:RESET` | Clear all timing/step statistics |
| `MSTAT` | Print motor stats (position, step timing, errors) |

### Test Scripts

| Script | Purpose |
|--------|---------|
| `test_harness/test_mtest.py` | Run MTEST, read analyzer STATUS |
| `test_harness/test_pintest.py` | Coordinated PINTEST with analyzer PINS |
| `test_harness/test_freq_sweep.py` | Frequency sweep with CSV output |
| `test_harness/test_pins.py` | Read raw GPIO states from analyzer |
| `test_harness/test_ratetest.py` | MCPWM step rate benchmark (burst + continuous) |

### LED Status Indicator

Added `LedStatus.h/.cpp` — priority-based RGB LED system on GPIO48. See [Controller README](../Controller/README.md#led-status-indicator) for full color/pattern table.

---

## M5: ESP32-S3 Firmware Port ✅

- **MCPWM motor control** replacing RMT for all 6 axes — dual-mode stepping:
  - **One-shot burst**: round-robin, 205 kHz single / 40 kHz × 6 — **exact step counts, zero drift**
  - **ISR-counted continuous**: TEZ ISR per pulse, **237 kHz × 1-2 motors, error=0** — limited to ≤2 motors (ISR overload at 6×250kHz)
- **GPTimer scheduler** at 50µs (20kHz) replacing 1ms FreeRTOS tick
- **Multi-transport**: Serial (USB CDC) + WiFi UDP + W5500 Ethernet + BLE
- **Binary protocol**: 15-byte packets with XOR checksum
- **NVS persistence**: Geometry, axis scales, WiFi credentials, MCA config
- **Motion cueing**: Biquad washout + tilt coordination filters
- **E-stop**: Debounced with GPTimer pause (PCBv1 GPIO22, PCBv2 GPIO20 disabled)

See [`docs/firmware/esp32s3_step_dir_roadmap.md`](firmware/esp32s3_step_dir_roadmap.md) for optimization details.

---

## M4: Desktop App — Beyond Parity ✅

See [`docs/ARCHITECTURE_ROADMAP.md`](ARCHITECTURE_ROADMAP.md) for phases 1–4.

Key features: SIL 3D visualizer, serial bridge, SimTools UDP, test signal generator, motion capture recording/playback, rolling spectrogram, multi-entity test bench.

---

## M7–M11: Upcoming Work

### M7: PCB Signal Validation
- [ ] Rerun `PINTEST` and `test_freq_sweep.py` on actual PCB
- [ ] Compare crosstalk profile with protoboard baseline (expect 0%)
- [ ] Verify ground plane integrity

### M8: SN75174N Differential Driver
- [ ] Wire SN75174N quad RS-422 line driver on controller output
- [ ] Wire SN75175N quad RS-422 receiver on analyzer input (Stage 2 test harness)
- [ ] Validate differential signal amplitude and common-mode rejection
- [ ] Rerun frequency sweep through differential chain

### M9: Single Motor Bench Test
- [ ] Connect one AASD-15A servo driver to differential output
- [ ] Verify step counting, direction, and speed control
- [ ] Test E-stop chain with motor energized
- [ ] Validate max step rate against AASD-15A 200kHz spec

### M10: Full 6-Motor System Integration
- [ ] All 6 AASD-15A drivers wired to differential outputs
- [ ] Simultaneous motion test from SimTools
- [ ] Thermal validation under sustained load
- [ ] Watchdog and fault recovery testing

### M11: End-to-End Motion Platform
- [ ] SimTools → WiFi/Ethernet → ESP32-S3 → RS-422 → AASD-15A → mechanical
- [ ] Latency characterization (host packet to motor step)
- [ ] Motion fidelity validation with recorded trajectories
- [ ] Safety sign-off: E-stop, watchdog, rate limiting

---

## Validation Targets

| Metric | Requirement | Measured | Status |
|--------|-------------|----------|--------|
| Max step rate (single, one-shot) | ≥ 200 kHz | 205 kHz | ✅ |
| Max step rate (single, ISR continuous) | 250 kHz | **237 kHz** | ✅ |
| Max step rate (6 simultaneous, one-shot) | ≥ 10 kHz | 40 kHz | ✅ |
| Step accuracy (one-shot, 6 motors) | 0 error | **0 error** (50k steps × 6) | ✅ |
| Step accuracy (ISR continuous, 1 motor) | 0 error | **0 error** (50k steps) | ✅ |
| Crosstalk (protoboard) | 0% | 0% (1Hz–250kHz) | ✅ |
| Pin mapping accuracy | 12/12 | 12/12 | ✅ |
| Motor self-test | 6/6 pass | 6/6 pass | ✅ |
| Timing jitter | < 1 µs pk-pk | TBD (needs scope) | 🔲 |
| E-stop response | < 5 ms | TBD | 🔲 |
| Host-to-step latency | < 200 µs | TBD | 🔲 |

---

## Related Documents

- [Architecture Roadmap](ARCHITECTURE_ROADMAP.md) — Desktop app design and phases
- [Step/Dir Optimization Roadmap](firmware/esp32s3_step_dir_roadmap.md) — Firmware timing details
- [IK Research](IK_RESEARCH.md) — Inverse kinematics analysis and workspace abstraction
- [Single Motor Test Plan](hardware/single_motor_test_plan.md) — Physical hardware test procedure
- [HIL Test Plan](hardware/hil_test_plan.md) — Hardware-in-loop validation
- [Controller README](../Controller/README.md) — Firmware build, pinout, LED status, commands
- [Test Harness README](../test_harness/README.md) — Analyzer firmware and test scripts
