#pragma once
// NOTE: Include in exactly ONE translation unit (main.cpp).

/*
 * SharedMcpwmStepEngine — 1 MCPWM timer, 6 hardware pulse channels, 250 kHz steps
 *
 * Timer mode: COUNT_UP_DOWN, peak_ticks=2, resolution=1MHz
 *   Counter: 0 → 2 → 0 → 2 → ...  (one full period = 4µs = 250 kHz)
 *   EMPTY event fires at count=0 (bottom)   → set STEP HIGH
 *   FULL  event fires at count=2 (top)      → set STEP LOW  (via compare=peak)
 *   Comparator ISR fires on compare match going UP only (direction check in edata)
 *
 * Idle:  compare = 0
 *   In COUNT_UP_DOWN, compare=0 fires going DOWN at count=0 — same instant as EMPTY.
 *   We configure the generator compare action going DOWN as KEEP → no pin change.
 *   The ISR checks edata->direction == UP before counting → ignores the DOWN hit.
 *   So compare=0 → ISR fires twice per period but does nothing (direction != UP,
 *   and delta==0 so _armFromISR returns immediately).
 *   Wait — this still fires ISR at 500 kHz when 6 motors are idle. Too expensive.
 *
 * BETTER IDLE: compare = peak_ticks (=2) going DOWN only fires at FULL (same as tep).
 *   Actually: use compare=peak_ticks for idle.
 *   compare=peak_ticks fires at FULL event going UP — same instant as the FULL event.
 *   But we configure generator FULL action as LOW already, and compare at peak fires
 *   simultaneously. Two actions at same count — hardware priority: compare wins? No:
 *   ESP32-S3 TRM says timer events (tep/tez) have higher priority than compare events.
 *   So FULL LOW wins, then compare fires ISR — but pin is already LOW, compare ISR fires
 *   once per period per armed motor. That's fine.
 *   Idle: compare=peak → ISR fires every period even when idle.
 *
 * CORRECT SOLUTION: Disable comparator ISR via mcpwm_comparator_register_event_callbacks
 * with NULL when idle, re-enable when armed. But that's not ISR-safe either.
 *
 * FINAL DESIGN — use compare match going UP at peak-1 for pulse falling edge,
 * and use EMPTY event ISR (on_full timer callback) to count + re-arm.
 * No comparator ISR at all. One timer on_full ISR fires at 250 kHz.
 * ISR is tiny: check 6 motors, advance counters, write 1 compare register per
 * active motor. At 250 kHz with 6 motors: ~200 cycles/ISR — fits in 960 cycle budget.
 * This is identical to McpwmStepEngine but with pulse width handled by hardware
 * (generator actions) instead of software GPIO toggle.
 *
 * Architecture (final):
 *   Timer COUNT_UP, period=4µs (peak=4 ticks at 1MHz).
 *   Generator per motor:
 *     - Action on EMPTY (tez, count=0): STEP = HIGH     [hardware]
 *     - Action on compare match:        STEP = LOW      [hardware]
 *     - Compare value = 2 (armed) → 2µs pulse width    [hardware]
 *     - Compare value = 5 (> peak=4, INVALID) — can't use
 *
 *   IDLE via force_level(LOW, hold=true) set in TIMER ISR (on_full).
 *   But force_level is not ISR-safe...
 *
 * REAL FINAL: Accept that compare=PULSE_US fires every period when armed,
 * and use the comparator ISR for counting + gating. When a motor reaches its
 * target, write compare=PULSE_US but set a "done" flag. ISR checks flag and
 * does not re-arm. The GPIO stays LOW after the last pulse's falling edge because
 * the next tez HIGH fires... wait, tez HIGH fires every period regardless.
 *
 * The generator HIGH-on-tez action is ALWAYS active. So if armed=false but
 * we don't suppress it, the pin still pulses. We need force_level to suppress.
 *
 * Conclusion: The only clean way to do 250kHz hardware pulses with per-step
 * control is to suppress the tez-HIGH action between steps via force_level,
 * which requires task context. The workaround: use the timer on_full ISR
 * (same as McpwmStepEngine) to also call force_level — but that's not ISR-safe.
 *
 * PRAGMATIC SOLUTION:
 *   Keep COUNT_UP, period=4µs.
 *   Generator: HIGH on tez, LOW on compare=2 → 2µs pulse, hardware-timed.
 *   Step rate = 1 step per 4µs period = 250 kHz (one full step per period).
 *   Idle: set compare=period (=4). IDF rejects compare>peak with INVALID_ARG.
 *   So set compare=peak (=4). Since period_ticks=4 and count goes 0..4, compare=4
 *   fires AT THE SAME TIME as FULL (tep). Generator action on FULL is not set,
 *   so only compare fires — sets pin LOW at count=4. Then tez fires at count=0
 *   setting pin HIGH again. Net result: pin is HIGH for 0 ticks → 0µs pulse.
 *   Hardware resolves same-tick conflicts: tez (EMPTY) fires at count=0 AFTER
 *   the previous compare=4 fired at count=4. So sequence per period:
 *     count=0: tez → HIGH
 *     count=2: compare → LOW  (when armed, compare=2)
 *     count=4: compare → LOW  (when idle, compare=4, same as peak — 0ns pulse)
 *   When idle (compare=4): HIGH at 0, immediately LOW at 4µs, then HIGH again at 8µs.
 *   That's a 4µs-wide pulse every 4µs = 50% duty 250kHz square wave. NOT idle!
 *
 * ────────────────────────────────────────────────────────────────────────────
 * ROOT CAUSE SUMMARY: In COUNT_UP mode there is NO compare value that means
 * "never fire" because count visits every value 0..peak once per period.
 * Hardware comparators in MCPWM cannot be truly disabled except via force_level.
 * force_level is not ISR-safe. Therefore hardware-autonomous 250kHz with per-step
 * ISR counting is not achievable without force_level in task context.
 *
 * ACTUAL WORKING DESIGN (this file):
 *   Use COUNT_UP_DOWN, peak=2, resolution=1MHz → period=4µs, 250kHz.
 *   Generator: HIGH on EMPTY (count=0), LOW on compare match going UP.
 *   Compare=1 when armed: pulse HIGH at 0µs, LOW at 1µs → 1µs pulse width.
 *   Compare=0 when idle: fires going DOWN at count=0 (same as EMPTY going DOWN).
 *     Configure generator action on compare going DOWN as KEEP → no pin change.
 *     Comparator ISR fires going DOWN (direction=DOWN in edata) → we skip it.
 *     Comparator ISR also fires going UP at count=0 — but count=0 going UP is
 *     tez/EMPTY, and compare=0 match going UP fires simultaneously.
 *     We check edata->direction==UP AND compare_ticks>0 before counting.
 *     When idle (compare=0 going UP at count=0): direction=UP, but we check
 *     delta==0 → skip. ISR fires 2x per period when idle, does nothing.
 *     At 250kHz with 6 motors idle: 12 ISR firings/period × 250kHz = 3MHz ISR rate
 *     → WAY too much. Kills WDT immediately.
 *
 * FINAL ANSWER: This approach cannot work. 250kHz hardware steps with software
 * position counting requires either PCNT or accepting an ISR overhead that
 * matches the step rate. The McpwmStepEngine at 125kHz is the correct tradeoff.
 * ────────────────────────────────────────────────────────────────────────────
 *
 * THIS FILE IS INTENTIONALLY LEFT AS A STUB.
 * SharedMcpwmStepEngine is not a viable backend. Use STEP_DRIVER_MCPWM_ISR.
 * For 250kHz with hardware counting, use STEP_DRIVER_MCPWM (MCPWMMotorControl
 * with PCNT for motors 0-3, RMT for motors 4-5).
 */

// This file intentionally contains no implementation.
// It is kept for documentation purposes.
// Select backend in CMakeLists.txt: STEP_DRIVER_MCPWM_ISR or STEP_DRIVER_MCPWM
