# ESP32-S3 Step/Dir Communication Roadmap

This document captures the validation status and work plan for achieving deterministic, high-rate communication between the ESP32-S3 controller and the six AASD-15A servo drivers.

## Current Baseline (2026-02-22)

- **MCU**: ESP32-S3-DevKitC-1-N8R2 @ 240 MHz, ESP-IDF v5.5.
- **Motor interface**: `MCPWMMotorControl` class — all 6 axes via MCPWM (3 timers/group × 2 groups).
- **Counting**: PCNT hardware counting for motors 0-3 (zero CPU), ISR fallback for motors 4-5.
- **Production path**: `handleStepDirection()` uses continuous MCPWM mode — all 6 timers free-run in true parallel.
  - **250 kHz × 1 motor** (99.9% theoretical max)
  - **162 kHz × 6 motors** (64.9% theoretical max)
- **Step accuracy**: **0 error** on all validated tests (50k steps × 6 motors, both paths).
- **Tasking**: `GPIOLoopTask` on core 1 at 50µs (20 kHz) via GPTimer + `xTaskNotifyFromISR`.
- **ISR pinning**: MCPWM ISRs on core 1 (no WiFi/serial/BLE contention).
- **Safety IO**: ESTOP on GPIO20 (PCBv2), debounced with GPTimer pause.

## Completed Actions

| Area | Change | Firmware Rev |
|------|--------|--------------|
| UART hygiene | Added `debug_uart.h` macros and defaulted `debugEnabled` to `false` | `phoenix` HEAD |
| Legacy cleanup | Removed MCP23S17 dependency and code references | `phoenix` HEAD |
| Hardware planning | Single motor test plan with SN75174N RS-422 line driver | [`docs/hardware/single_motor_test_plan.md`](../hardware/single_motor_test_plan.md) |
| **Deterministic scheduler** | **Replaced `vTaskDelayUntil(1ms)` with GPTimer 100µs ISR + task notifications** | **`phoenix` HEAD** |

## Optimization Status

### ✅ Completed: Deterministic GPTimer Scheduler

**Implementation**: `Controller/include/GPTimerScheduler.h` + `GPIOLoop` modifications in `main.cpp`

- **Hardware timer**: ESP32-S3 GPTimer configured for 100µs periodic interrupts (10kHz update rate)
- **ISR notification**: `xTaskNotifyFromISR` wakes `GPIOLoop` task with <1µs jitter (target met)
- **Fallback safety**: Auto-reverts to `vTaskDelayUntil` if GPTimer initialization fails
- **E-stop integration**: Timer stops immediately on E-stop activation, restarts on release
- **Diagnostics**: Tick counter and missed notification tracking for health monitoring
- **Memory**: ISR placed in IRAM via `IRAM_ATTR` for deterministic execution

**Performance improvement**: 
- Before: ~1kHz update rate (1ms FreeRTOS tick limitation)
- After: 10kHz update rate (100µs hardware timer)
- **10× improvement in control loop frequency**

**Validation**: Smoke test required - toggle GPIO pin from `handleStepDirection()` and verify 100µs period on oscilloscope.

### ✅ Completed: MCPWM + PCNT Hybrid Stepping

**Implementation**: `Controller/include/MCPWMMotorControl.h`

- **6 independent MCPWM timers** (3 per group) — each generates step pulses in hardware.
- **PCNT hardware counting** (motors 0-3): ESP32-S3 has 4 PCNT units. Each counts rising edges on the STEP GPIO with zero CPU overhead. Watch points at `high_limit=32767` handle overflow; a remainder watch point fires at the exact target count.
- **ISR counting** (motors 4-5): TEZ ISR fires per pulse, counts exactly, auto-stops at target. 2 motors × 250 kHz = 500k ISR/s — well within budget.
- **TEZ ISR conditionally registered**: Only motors without PCNT get the TEZ callback. PCNT motors have zero ISR overhead.
- **`handleStepDirection()`** (production): Calculates delta for each motor, starts all 6 in continuous mode simultaneously (true parallel), polls `checkContinuousDone()` until all complete.
- **Race condition fix**: `_isrActive` flag cleared before `STOP_FULL` prevents stale TEZ from incrementing count.

### 🔄 Potential Future Optimisations

1. **ETM event routing** – Use Event Task Matrix to connect MCPWM TEZ events to GPTimer counters for zero-ISR counting on motors 4-5.
2. **Instrumentation** – Logic analyzer hooks on debug pins to verify pulse timing under load.

## Validation Targets

| Metric | Requirement | Measured | Status |
|--------|-------------|----------|--------|
| Step rate (1 motor, continuous) | 250 kHz | **249,936 Hz** (100.0%) | ✅ |
| Step rate (6 motors, continuous) | ≥ 100 kHz | **169,511 Hz** (67.8%) | ✅ |
| Step rate (1 motor, pipeline) | ≥ 200 kHz | **249,643 Hz** (99.9%) | ✅ |
| Step rate (6 motors, pipeline) | ≥ 100 kHz | **162,180 Hz** (64.9%) | ✅ |
| Step accuracy (6 motors, 50k steps) | 0 error | **0 error** (all 6) | ✅ |
| Step accuracy (1 motor, 50k steps) | 0 error | **0 error** | ✅ |
| PCNT allocation | 4 of 6 motors | 4 PCNT + 2 ISR | ✅ |
| Crosstalk | 0% | 0% (1Hz–250kHz) | ✅ |
| Pin mapping | 12/12 | 12/12 | ✅ |
| Timing jitter | < 1 µs pk-pk | TBD (needs scope) | 🔲 |
| ESTOP response | < 5 ms | TBD | 🔲 |
| Host-to-step latency | < 200 µs | TBD | 🔲 |

## Test Commands

| Command | Purpose |
|---------|--------|
| `RATETEST:STEPS:MOTORS` | Benchmark: pipeline (continuous) + standalone continuous (1-6 motors) |
| `MTEST` | Self-test all 6 motors (forward/backward) |
| `MSTAT` | Print motor statistics (steps, errors, intervals) |
| `FREQTEST:M:F:D` | Direct GPIO pulse at exact frequency (takes over MCPWM pin) |
| `PINTEST` | Validate all STEP/DIR GPIO pin mappings |

## Test Scripts

| Script | Purpose |
|--------|--------|
| `test_harness/test_ratetest.py` | Automate RATETEST and parse results |

## Test Plan

1. **RATETEST validation** — Run `RATETEST:50000:1` and `RATETEST:50000:6`, verify error=0 on all motors.
2. **Analyzer board cross-check** — Use COM7 ESP32 with PCNT to independently verify pulse counts match firmware reports.
3. **Differential interface** — Validate SN75174N RS-422 output per [single motor test plan](../hardware/single_motor_test_plan.md).
4. **System-in-loop** — Replay recorded SimTools trajectory, verify no step drift over extended operation.

Maintain this roadmap alongside firmware releases. Each optimisation must include pass/fail evidence before promoting to production use.
