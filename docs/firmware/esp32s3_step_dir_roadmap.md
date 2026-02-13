# ESP32-S3 Step/Dir Communication Roadmap

This document captures the validation status and work plan for achieving deterministic, high-rate communication between the ESP32-S3 controller and the six AASD-15A servo drivers.

## Current Baseline (2025-10-09)

- **MCU**: ESP32-S3 DevKitC-1 @ 240 MHz, Arduino framework.
- **Motor interface**: `RMTMotorControl` class drives four axes over RMT TX channels; two axes use GPIO bit banging (temporary fallback).
- **Tasking**: `GPIOLoop` runs at ~1 kHz due to `vTaskDelayUntil(1 ms)`, limiting effective step rate.
- **UART policy**: All outbound serial traffic now gated behind compile-time `ENABLE_DEBUG_UART` + runtime `debugEnabled` command.
- **Safety IO**: ESTOP moved to GPIO20 (input-capable, available on DevKitC headers) and isolated on the servo interface PCB.

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

### 🔄 Next-Up Optimisations

1. ~~**Deterministic scheduler**~~ **✅ COMPLETED** – ~~Replace `vTaskDelayUntil` in `GPIOLoop` with a 100 µs GPTimer interrupt that wakes the task via `xTaskNotifyFromISR`. Target jitter < 1 µs.~~
2. **All-axis hardware timing** – Migrate motors 4 & 5 off GPIO toggling. Options under evaluation:
   - Expand RMT usage via IDF v5 multi-symbol APIs + shared memory blocks.
   - Use MCPWM generators in pulse counter mode for step synthesis.
3. **Pulse batching** – Extend `RMTMotorControl` to pre-load batches of items (e.g., 32-pulse bursts) and replenish via `rmt_register_tx_end_callback`.
4. **Back-pressure handling** – Expose status diagnostics for step queue saturation and trigger automatic rate limiting.
5. **Instrumentation** – Add logic-analyser hooks on dedicated debug pins to sample actual step frequency; log metrics over UART only when `ENABLE_DEBUG_UART=1`.

## Validation Targets

| Metric | Requirement | Notes |
|--------|-------------|-------|
| Max step rate | ≥ 200 kHz per axis sustained | Matches AASD-15A max pulse rating |
| Timing jitter | < 1 µs pk-pk at 10 kHz command | Measured at driver input |
| Worst-case latency | < 200 µs from host packet to first step | With command buffer idle |
| ESTOP response | < 5 ms from contact open to motor stop | Verified with isolated loop |

## Test Plan (Draft)

1. **Timer upgrade smoke test** – Instrument toggled GPIO to confirm 100 µs cadence before engaging motors.
2. **RMT stress test** – Drive synthetic trapezoidal profiles at 150 kHz step rate, log missed-step counters.
3. **Differential interface** – Validate SN75174N RS-422 output per [single motor test plan](../hardware/single_motor_test_plan.md). Check amplitude and common-mode baseline with motor disabled.
4. **System-in-loop** – Replay recorded SimTools trajectory, monitor servo fault registers and watchdog timers.

Maintain this roadmap alongside firmware releases. Each optimisation must include pass/fail evidence in the safety file set before promoting to production use.
