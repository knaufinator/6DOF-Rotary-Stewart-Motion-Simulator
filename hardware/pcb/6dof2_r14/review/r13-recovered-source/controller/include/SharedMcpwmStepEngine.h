#pragma once
// NOTE: This header defines file-scope IRAM_ATTR ISR handlers.
// Include in exactly ONE translation unit (main.cpp).

/*
 * SharedMcpwmStepEngine — 1 MCPWM timer, 6 hardware pulse channels
 *
 * Architecture:
 *   One MCPWM timer (group 0, timer 0) runs COUNT_UP at 4µs period (250 kHz).
 *   3 operators are connected to that single timer.
 *   Each operator owns 2 comparators + 2 generators → 6 channels total.
 *
 *   Hardware topology:
 *     Timer 0 ──┬── Operator 0 ──┬── CmpA / GenA → Motor 0 STEP pin
 *               │                └── CmpB / GenB → Motor 1 STEP pin
 *               ├── Operator 1 ──┬── CmpA / GenA → Motor 2 STEP pin
 *               │                └── CmpB / GenB → Motor 3 STEP pin
 *               └── Operator 2 ──┬── CmpA / GenA → Motor 4 STEP pin
 *                                └── CmpB / GenB → Motor 5 STEP pin
 *
 *   Pulse generation (fully hardware, gated per period):
 *     - Each generator: HIGH on comparator A (count = SMSE_PULSE_START), LOW on
 *       comparator B (count = SMSE_PULSE_START + SMSE_PULSE_US). The compare values
 *       never change while running.
 *     - Whether a period pulses is decided by the generator's continuous software force
 *       (gen_force, forced LOW = no pulse), written by the ISR every tick and
 *       shadow-latched by the hardware at TEZ (SMSE_FORCE_UPMETHOD = 1). One write =
 *       exactly one pulse or none in the period that the latch starts.
 *     - The ISR is a direct level-3 handler raised by TEP; it books steps and corrects
 *       the count when a write is latched one event early or late (exact latch
 *       accounting, see the ISR).
 *
 *   DIR/STEP synchronisation:
 *     Tick N:   physDir changed and no pulse in flight → set DIR via w1ts/w1tc,
 *               dirPending, no step
 *     Tick N+1: dirPending → clear it, no step
 *     Tick N+2: release the gate; it latches at the next TEZ and the hardware pulses
 *               in the period after that
 *     DIR setup ≥ 8µs (2 quiet ticks plus the latch pipeline) — AASD-15A needs ≥ 2µs.
 *     DIR never moves while a released gate is in flight (armedHist).
 *
 *   Motors at their target simply keep their gate forced LOW. The timer runs freely
 *   at 250 kHz; idle motors cost nothing.
 *
 * Properties:
 *   - Max step rate:   250 kHz/motor (one pulse per 4µs period)
 *   - Step pulse width: SMSE_PULSE_US µs (default 2µs, meets AASD-15A ≥ 1.5µs)
 *   - DIR setup time:   ≥ 8µs — AASD-15A requires ≥ 2µs
 *   - MCPWM resources:  1 timer, 3 operators, 6 comparators, 6 generators
 *   - No PCNT, no RMT, no per-motor timer
 *   - ISR budget: ~340-cycle body + up to (period − SMSE_ISR_TAIL_CYC) of boundary-guard
 *     spin, of the 960-cycle period at 240 MHz; ~72 cycles of entry overhead
 *   - DIR via direct GPIO w1ts/w1tc; both STEP edges by the MCPWM generator
 *
 * Verified on the R13 bench 2026-10-20 (hardware/pcb/6dof2_r13/bringup/README.md):
 * pulse-exact against a PCNT counter on the STEP pad over 58 M pulses at 20–250 kHz,
 * hour-long soaks returning to home. The timing constants below were measured for a
 * 4 µs period at 240 MHz and are valid only there (setTickRate refuses other periods).
 */

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_private/periph_ctrl.h"
#include "soc/mcpwm_periph.h"
#include "soc/mcpwm_struct.h"
#include "soc/gpio_struct.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "sdkconfig.h"
#include "debug_uart.h"

#define SMSE_NUM_MOTORS        6
// The SMSE_* constants below are overridable from the build (idf.py -DSMSE_...=..., passed
// through by main/CMakeLists.txt; CMake caches them per build directory). The defaults are
// the verified bench values. SMSE_TEP_BOUNDARY_CYC is calibrated for the 4 µs period only.
#ifndef SMSE_PERIOD_US_DEFAULT
#define SMSE_PERIOD_US_DEFAULT 4    // 4µs period → 250 kHz tick rate, 250 kHz max step rate
#endif
#define SMSE_PERIOD_US_MIN     4    // hard floor: WDT-safe on ESP32-S3
#define SMSE_PERIOD_US_MAX     100  // 100µs period → 5 kHz max step rate
#ifndef SMSE_PULSE_US
#define SMSE_PULSE_US          2    // 2µs pulse width (comparator B − comparator A, must be < period)
#endif
// Pulse window inside the period: rising edge at count SMSE_PULSE_START, falling edge
// SMSE_PULSE_US later. START >= 1 keeps the edge clear of the TEZ latch of the gate.
#ifndef SMSE_PULSE_START
#define SMSE_PULSE_START       1
#endif
// gen_cntuforce_upmethod for the pulse gate: 1 = latch at TEZ (the wrap), 2 = latch at TEP.
// The ISR is a direct handler raised by TEP; with ~0.3 µs entry latency and a ~1.4 µs body
// the gate write lands ~1.7 µs after TEP. Measured 2026-10-20 with BENCH:ISRDELAY: a TEP
// latch sits only ~0.1 µs from that write, a TEZ latch ~1 µs, hence TEZ is the default.
#ifndef SMSE_FORCE_UPMETHOD
#define SMSE_FORCE_UPMETHOD    1
#endif
// Where the latch event sits in the ISR's "since the raising TEP" frame, in cycles past the
// period multiple: the write is latched one event later than nominal when (since mod period)
// crosses SMSE_TEP_BOUNDARY_CYC. Calibrated on the R13 bench 2026-10-20 with BENCH:ISRDELAY
// (shift the gate write in 10–50-cycle steps, watch the STEP-pad counter): ~715 for the TEP
// latch, ~955 for the TEZ latch used now. NOT derived from timer_status.timer_value: that
// register lags the event (a fix based on it cost +4000 pulses per leg). Re-measure after any
// change to the ISR entry path, the CPU clock or the period.
#ifndef SMSE_TEP_BOUNDARY_CYC
#define SMSE_TEP_BOUNDARY_CYC  955
#endif
// Half-width of the band around the boundary in which a tick spins past rather than writes
// (0 disables). The reference's sync error is ~±50 cycles, so ±100 covers it; a wider band
// needs more spin than the period budget allows once the ISR body is ~500 cycles.
// (Hardware classification by the TEZ raw flag was tried and does not work: the force shadow
// applies ~150-200 cycles after the TEZ flag, so the flag mis-classifies writes in between.)
#ifndef SMSE_TEP_GUARD_CYC
#define SMSE_TEP_GUARD_CYC     100
#endif
// Cycles the ISR needs after the guard (gate writes + accounting + exit): the guard stops
// spinning once the ISR has used period - this, whatever the phase.
#ifndef SMSE_ISR_TAIL_CYC
#define SMSE_ISR_TAIL_CYC      250
#endif
// Entry latency of the direct handler after the raising TEP, in cycles (~0.3 µs at 240 MHz).
// Used once per resync to place the phase reference; the boundary above was measured
// relative to it, so the two must change together.
#define SMSE_ISR_ENTRY_CYC     72
static inline void IRAM_ATTR _smse_gate(int op, bool pulseA, bool pulseB);   // defined below
void IRAM_ATTR _smse_direct_isr(void *);                                      // defined below
#define SMSE_INT_TEP0 (1u << 6)          // MCPWM int bit: timer 0 reached its period (TEP)

// Forward declaration
class SharedMcpwmStepEngine;
static DRAM_ATTR SharedMcpwmStepEngine* _smse_instance = nullptr;

// Forward-declare the ISR body so the friend declaration inside the class is valid
void _smse_period_body(void);

class SharedMcpwmStepEngine {
public:
    struct MotorConfig {
        gpio_num_t stepPin;
        gpio_num_t dirPin;
        bool       invertDir;
        int32_t    softLimitMin;
        int32_t    softLimitMax;
        bool       enableSoftLimits;

        MotorConfig() :
            stepPin(GPIO_NUM_NC), dirPin(GPIO_NUM_NC),
            invertDir(false), softLimitMin(-100000), softLimitMax(100000),
            enableSoftLimits(true) {}
    };

    struct MotorState {
        volatile int32_t  currentPos;
        volatile int32_t  targetPos;
        volatile bool     lastDir;
        volatile bool     dirPending;
        volatile uint8_t  armedHist;   // bit0: gate released last tick, bit1: the tick before
        volatile uint32_t totalSteps;
        volatile uint32_t dirChanges;
    };

    static SharedMcpwmStepEngine& instance() {
        static SharedMcpwmStepEngine eng;
        return eng;
    }

    bool init(const MotorConfig configs[SMSE_NUM_MOTORS]) {
        if (_initialized) return true;

        for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
            _cfg[i] = configs[i];

            gpio_config_t io = {};
            io.pin_bit_mask = (1ULL << configs[i].stepPin);
            io.mode = GPIO_MODE_OUTPUT;
            io.pull_up_en = GPIO_PULLUP_DISABLE;
            io.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&io);
            gpio_set_level(configs[i].stepPin, 0);

            io.pin_bit_mask = (1ULL << configs[i].dirPin);
            gpio_config(&io);
            gpio_set_level(configs[i].dirPin, 0);

            _state[i] = {};
        }

        _initialized = true;
        DEBUG_PRINTF("SharedMcpwmStepEngine: %d motors initialized\n", SMSE_NUM_MOTORS);
        return true;
    }

    bool start() {
        if (!_initialized || _running) return _running;
        _smse_instance = this;

        // Reset MCPWM group 0 peripheral for clean slate after any prior WDT reset
        periph_module_reset(PERIPH_PWM0_MODULE);

        // ── Allocate 1 shared timer ────────────────────────────────────
        mcpwm_timer_config_t t_cfg = {};
        t_cfg.group_id      = 0;
        t_cfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
        t_cfg.resolution_hz = 1000000;          // 1 MHz → 1µs per tick
        t_cfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
        t_cfg.period_ticks  = _tickUs;
        t_cfg.intr_priority = 0;
        t_cfg.flags.update_period_on_empty = true;  // shadow: new period loads at next TEZ

        if (mcpwm_new_timer(&t_cfg, &_timer) != ESP_OK) {
            DEBUG_PRINTLN("SharedMcpwmStepEngine: timer alloc failed");
            return false;
        }

        // ── Allocate 3 operators, connect each to the shared timer ─────
        for (int op = 0; op < 3; op++) {
            mcpwm_operator_config_t o_cfg = {};
            o_cfg.group_id      = 0;
            o_cfg.intr_priority = 0;
            o_cfg.flags.update_gen_action_on_tez = true;

            if (mcpwm_new_operator(&o_cfg, &_oper[op]) != ESP_OK) {
                DEBUG_PRINTF("SharedMcpwmStepEngine: operator %d alloc failed\n", op);
                _cleanup();
                return false;
            }
            if (mcpwm_operator_connect_timer(_oper[op], _timer) != ESP_OK) {
                DEBUG_PRINTF("SharedMcpwmStepEngine: operator %d connect failed\n", op);
                _cleanup();
                return false;
            }
        }

        // ── Allocate 6 comparators (2 per operator), fixed pulse window ──
        // Both comparators of an operator are shared by its two generators: A marks the
        // rising edge at count SMSE_PULSE_START, B the falling edge SMSE_PULSE_US later.
        // The values never change; whether a period pulses is decided per generator by
        // the software force (gen_force), see below.
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
            int op  = i / 2;   // operator index: 0,0,1,1,2,2

            mcpwm_comparator_config_t c_cfg = {};
            c_cfg.intr_priority = 0;
            c_cfg.flags.update_cmp_on_tez = false;

            if (mcpwm_new_comparator(_oper[op], &c_cfg, &_cmpr[i]) != ESP_OK) {
                DEBUG_PRINTF("SharedMcpwmStepEngine: comparator %d alloc failed\n", i);
                _cleanup();
                return false;
            }
            mcpwm_comparator_set_compare_value(_cmpr[i], (i % 2 == 0) ? SMSE_PULSE_START
                                                                      : SMSE_PULSE_START + SMSE_PULSE_US);
        }

        // ── Allocate 6 generators: HIGH on comparator A, LOW on comparator B ──
        // No timer-event action: the rising edge is deliberately NOT at TEZ, because the
        // force gate is latched at TEZ and the two must not coincide.
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
            int op  = i / 2;

            mcpwm_generator_config_t g_cfg = {};
            g_cfg.gen_gpio_num = _cfg[i].stepPin;
#if defined(R13_BENCH_ONE_AXIS)
            g_cfg.flags.io_loop_back = 1;   // bench self-test: the STEP pad level can be read back
#endif

            if (mcpwm_new_generator(_oper[op], &g_cfg, &_gen[i]) != ESP_OK) {
                DEBUG_PRINTF("SharedMcpwmStepEngine: generator %d alloc failed\n", i);
                _cleanup();
                return false;
            }

            mcpwm_gen_compare_event_action_t hi_act = {};
            hi_act.direction  = MCPWM_TIMER_DIRECTION_UP;
            hi_act.comparator = _cmpr[2 * op];
            hi_act.action     = MCPWM_GEN_ACTION_HIGH;
            mcpwm_generator_set_action_on_compare_event(_gen[i], hi_act);

            mcpwm_gen_compare_event_action_t lo_act = {};
            lo_act.direction  = MCPWM_TIMER_DIRECTION_UP;
            lo_act.comparator = _cmpr[2 * op + 1];
            lo_act.action     = MCPWM_GEN_ACTION_LOW;
            mcpwm_generator_set_action_on_compare_event(_gen[i], lo_act);
        }

        // ── Pulse gate: continuous software force, shadow-updated at TEZ ──
        // Each generator starts forced LOW. The ISR releases the force for a motor that
        // steps and re-applies it otherwise; with gen_cntuforce_upmethod = TEZ
        // (SMSE_FORCE_UPMETHOD, 2 = TEP) the write is latched into the hardware at the next
        // TEZ, whenever inside the period the ISR happened to run. One write = exactly one
        // pulse (or none) in the period that latch starts, rising at count SMSE_PULSE_START,
        // so the latch and the edge never coincide. The previous scheme moved the compare
        // value between 0 and SMSE_PULSE_US from the ISR with an immediate (non-shadow)
        // write; the ISR runs ~1 µs after its event, so the write landed before or after the
        // rising edge at random and pulses were lost or doubled (R13 bench 2026-10-20, PCNT
        // on STEP0).
        for (int op = 0; op < 3; op++) _smse_gate(op, false, false);   // both forced LOW

        // ── Per-period ISR on TEP ────────────────────────────────────────
        // Timing (4 µs period = counts 0..3, 240 MHz, direct handler): TEP k fires at count
        // 3, the handler starts ~0.3 µs later and writes the gate ~1.7 µs after TEP k. The
        // write is latched at the next TEZ and decides the pulse of the period that TEZ
        // starts (counts SMSE_PULSE_START..+SMSE_PULSE_US). A write near a latch boundary
        // is handled by the guard (SMSE_TEP_GUARD_CYC) and the latch accounting in the ISR.
        // (With the old IDF callback path the write landed within 0.5 µs of the wrap and
        // 250 kHz trains gained a pulse at ~20 % of their boundaries.)
        //
        // Direct interrupt instead of the driver's callback: the IDF path (shared-interrupt
        // dispatcher → mcpwm_timer_default_isr → status read/clear → callback) costs a
        // measured 513 cycles per period on top of the body, i.e. 2.1 µs of a 4 µs period.
        // A non-shared level-3 handler on the group's IRQ clears the TEP flag itself and
        // runs the body; level 3 also keeps the FreeRTOS tick (level 1) from delaying it.
        // No driver callbacks are registered, so the driver never allocates this source.
        if (esp_intr_alloc(mcpwm_periph_signals.groups[0].irq_id,
                           ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                           ::_smse_direct_isr, nullptr, &_intr) != ESP_OK) {
            DEBUG_PRINTLN("SharedMcpwmStepEngine: esp_intr_alloc failed");
            _cleanup();
            return false;
        }
        MCPWM0.int_clr.val = SMSE_INT_TEP0;
        MCPWM0.int_ena.val |= SMSE_INT_TEP0;

        if (mcpwm_timer_enable(_timer) != ESP_OK) {
            DEBUG_PRINTLN("SharedMcpwmStepEngine: timer enable failed");
            _cleanup();
            return false;
        }

#if defined(R13_BENCH_ONE_AXIS)
        // Bench: leave the period timer stopped. The engine ISR takes ~90 % of core 1 at a
        // 4 µs period (2026-10-20 measurements) and, while core 0 is still booting (flash
        // traffic on the shared bus), more than that — the boot task on core 1 then never
        // runs and the task WDT reboots the board. BENCH:ARM resumes the timer, so
        // isRunning() is false after a successful start() in bench builds.
        _running = false;
        return true;
#endif
        // Free-running timer. (A one-shot timer restarted by the ISR was tried on
        // 2026-10-20 to make missed periods impossible: the period stretched to ~5 µs idle
        // and ~12 µs inside a train, so the engine could not hold 250 kHz. Instead the ISR
        // measures how late it is and books the pulses the hardware emitted meanwhile.)
        _tepRefValid = false;
        if (mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
            DEBUG_PRINTLN("SharedMcpwmStepEngine: timer start failed");
            mcpwm_timer_disable(_timer);
            _cleanup();
            return false;
        }

        _running = true;
        DEBUG_PRINTF("SharedMcpwmStepEngine: started — %luµs period, %lu Hz tick, %lu Hz max step/motor\n",
            (unsigned long)_tickUs,
            (unsigned long)(1000000UL / _tickUs),
            (unsigned long)(1000000UL / _tickUs));
        return true;
    }

    // Stop pulse generation. The gate-off write below can be overwritten by the ISR during
    // the 3-tick wait (it rewrites every gate from the current targets each tick), so a motor
    // that still has a target keeps stepping until the timer is stopped; those steps are
    // accounted. Callers that want an immediate halt clear the targets first
    // (emergencyStop). armedHist is reset, so a reversal right after resume() is not
    // protected by the in-flight guard for the first tick — a known open item
    // (hardware/pcb/6dof2_r13/bringup/README.md §7), not changed without a bench test.
    void stop() {
        if (!_running) return;
        for (int op = 0; op < 3; op++) _smse_gate(op, false, false);   // gate every output off (latched at the next TEZ)
        esp_rom_delay_us(3 * _tickUs);                                   // let the ISR account the last pulses
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_FULL);
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) _state[i].armedHist = 0;
        _running = false;
        // (The STEP pads are routed to the MCPWM generators; gpio_set_level would not reach
        // them. The forced-LOW gate keeps them low while the timer is stopped.)
        DEBUG_PRINTLN("SharedMcpwmStepEngine: stopped");
    }

    void resume() {
        if (!_initialized || _running) return;
        _tepRefValid = false;
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
        _running = true;
        DEBUG_PRINTLN("SharedMcpwmStepEngine: resumed");
    }

    bool setTarget(int i, int32_t pos) {
        if (i < 0 || i >= SMSE_NUM_MOTORS) return false;
        if (_cfg[i].enableSoftLimits) {
            if (pos < _cfg[i].softLimitMin || pos > _cfg[i].softLimitMax) return false;
        }
        _state[i].targetPos = pos;
        return true;
    }

    // Re-apply soft limits at runtime (the drive gear changed: limits are kept in degrees
    // by the caller and converted with the new steps_per_degree). Targets are checked
    // against them on the next setTarget(); the current target is not re-validated.
    bool setSoftLimits(int i, int32_t minPos, int32_t maxPos) {
        if (i < 0 || i >= SMSE_NUM_MOTORS || minPos > maxPos) return false;
        _cfg[i].softLimitMin = minPos;
        _cfg[i].softLimitMax = maxPos;
        return true;
    }

    int32_t getPosition(int i)   const { return (i >= 0 && i < SMSE_NUM_MOTORS) ? _state[i].currentPos : 0; }
    int32_t getTarget(int i)     const { return (i >= 0 && i < SMSE_NUM_MOTORS) ? _state[i].targetPos  : 0; }

    void resetPosition(int i) {
        if (i < 0 || i >= SMSE_NUM_MOTORS) return;
        _state[i].currentPos = 0;
        _state[i].targetPos  = 0;
    }

    void emergencyStop() {
        for (int i = 0; i < SMSE_NUM_MOTORS; i++)
            _state[i].targetPos = _state[i].currentPos;
    }

    bool     isInitialized() const { return _initialized; }
    bool     isRunning()     const { return _running; }
    uint32_t getTotalSteps(int i) const { return (i >= 0 && i < SMSE_NUM_MOTORS) ? _state[i].totalSteps : 0; }
    uint32_t getDirChanges(int i) const { return (i >= 0 && i < SMSE_NUM_MOTORS) ? _state[i].dirChanges : 0; }

    void resetStats(int i) {
        if (i < 0 || i >= SMSE_NUM_MOTORS) return;
        _state[i].totalSteps = 0;
        _state[i].dirChanges = 0;
    }

    uint32_t getTickUs()    const { return _tickUs; }
    uint32_t getMaxStepHz() const { return 1000000UL / _tickUs; }  // one pulse per period
    uint32_t getTickHz()    const { return 1000000UL / _tickUs; }
    // Latch-accounting statistics (kept in production; cheap, and the only window into
    // whether the ISR ever had to correct the count).
    void     latchStats(uint32_t *dneg, uint32_t *dpos, uint32_t *dbig, uint32_t *mpos, uint32_t *smax) const {
        *dneg = _dNeg; *dpos = _dPos; *dbig = _dBig; *mpos = _mPos; *smax = _sinceMax;
    }
    uint32_t isrMissed()    const { return _isrMissed; }      // |d| summed: pulses booked by the accounting
    uint32_t resyncs()      const { return _resyncs; }
    uint32_t skipMax()      const { return _skipMax; }
    uint32_t periodCycles() const { return _periodCycles; }
    void     setLatchComp(bool on) { _applyLatchComp = on; }
    bool     latchComp() const { return _applyLatchComp; }
#if defined(R13_BENCH_ONE_AXIS)
    // ISR cost instrumentation (bench): invocations, summed CPU cycles, worst single run,
    // gap from body exit to the next entry.
    uint32_t isrCount()     const { return _isrCount; }
    uint32_t isrCycles()    const { return _isrCycles; }
    uint32_t isrMaxCycles() const { return _isrMax; }
    uint32_t isrGapMin()    const { return _isrGapMin; }
    uint32_t isrGapSum()    const { return _isrGapSum; }
    void     resetIsrStats() { _isrCount = 0; _isrCycles = 0; _isrMax = 0; _isrGapMin = 0xffffffff; _isrGapSum = 0; }
    void     setIsrDelay(uint32_t cyc) { _isrDelayCycles = cyc; }   // BENCH:ISRDELAY boundary calibration
#endif

    // Change the period. Only SMSE_PERIOD_US_DEFAULT is accepted: the latch boundary
    // (SMSE_TEP_BOUNDARY_CYC) and the guard were calibrated at that period and the
    // accounting would be wrong at any other. (A "tick_us" saved in NVS by older firmware
    // is rejected here too.) Kept so TICKRATE: reports a clear error.
    bool setTickRate(uint32_t new_tick_us) {
        if (new_tick_us < SMSE_PERIOD_US_MIN || new_tick_us > SMSE_PERIOD_US_MAX) {
            DEBUG_PRINTF("SharedMcpwmStepEngine: setTickRate %lu µs out of range [%d, %d]\n",
                (unsigned long)new_tick_us, SMSE_PERIOD_US_MIN, SMSE_PERIOD_US_MAX);
            return false;
        }
        if (new_tick_us <= SMSE_PULSE_START + SMSE_PULSE_US) {
            DEBUG_PRINTF("SharedMcpwmStepEngine: setTickRate %lu µs must be > SMSE_PULSE_START + SMSE_PULSE_US (%d)\n",
                (unsigned long)new_tick_us, SMSE_PULSE_START + SMSE_PULSE_US);
            return false;
        }
        if (new_tick_us != SMSE_PERIOD_US_DEFAULT) {
            DEBUG_PRINTF("SharedMcpwmStepEngine: setTickRate %lu µs refused; the latch accounting is calibrated for %d µs only\n",
                (unsigned long)new_tick_us, SMSE_PERIOD_US_DEFAULT);
            return false;
        }
        _tickUs = new_tick_us;
        _periodCycles = _tickUs * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
        if (!_running) return true;  // takes effect on next start()
        esp_err_t e = mcpwm_timer_set_period(_timer, _tickUs);
        if (e != ESP_OK) {
            DEBUG_PRINTF("SharedMcpwmStepEngine: set_period failed: %d\n", e);
            return false;
        }
        _tepRefValid = false;        // the hardware period changes at the next TEZ: resync the phase reference
        // Comparators stay at SMSE_PULSE_START / +SMSE_PULSE_US; idle motors are gated by gen_force.
        DEBUG_PRINTF("SharedMcpwmStepEngine: tick rate → %lu µs (%lu Hz, %lu Hz max step)\n",
            (unsigned long)_tickUs,
            (unsigned long)(1000000UL / _tickUs),
            (unsigned long)(1000000UL / _tickUs));
        return true;
    }

#if defined(R13_BENCH_ONE_AXIS)
    // Latch-event ring (bench diagnostics, BENCH:LATCHDUMP): one entry per tick whose
    // accounting booked d != 0 around a real step of axis 0, plus every resync (d = 99).
    struct LatchEvent { uint32_t tick, since, m, skip; int32_t d; uint8_t hist0; };
    static const int LATCH_RING = 48;
    volatile LatchEvent  _ring[LATCH_RING];
    volatile uint32_t    _ringN = 0;      // total events recorded (ring index = n % LATCH_RING)
#endif

private:
    SharedMcpwmStepEngine() : _initialized(false), _running(false), _timer(nullptr),
                               _tickUs(SMSE_PERIOD_US_DEFAULT) {
        for (int i = 0; i < 3; i++) _oper[i] = nullptr;
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) { _cmpr[i] = nullptr; _gen[i] = nullptr; }
    }

    void _cleanup() {
        if (_intr) { MCPWM0.int_ena.val &= ~SMSE_INT_TEP0; esp_intr_free(_intr); _intr = nullptr; }
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
            if (_gen[i])  { mcpwm_del_generator(_gen[i]);   _gen[i]  = nullptr; }
            if (_cmpr[i]) { mcpwm_del_comparator(_cmpr[i]); _cmpr[i] = nullptr; }
        }
        for (int i = 0; i < 3; i++) {
            if (_oper[i]) { mcpwm_del_operator(_oper[i]); _oper[i] = nullptr; }
        }
        if (_timer) { mcpwm_del_timer(_timer); _timer = nullptr; }
    }

    MotorConfig  _cfg[SMSE_NUM_MOTORS];
    MotorState   _state[SMSE_NUM_MOTORS];
    bool         _initialized;
    bool         _running;

    mcpwm_timer_handle_t _timer;
    intr_handle_t        _intr = nullptr;
    uint32_t             _tickUs;
    uint32_t             _periodCycles = SMSE_PERIOD_US_DEFAULT * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    volatile uint32_t    _tepRef = 0;           // cycle count of the TEP that raised this tick
    volatile uint32_t    _tepSkip = 0;          // TEPs that passed before this tick ran
    volatile uint32_t    _tepIdx = 0;           // running TEP index of the raising TEP (wrap-safe arithmetic)
    volatile uint32_t    _latchPrev = 0;        // TEP index at which the previous gate was latched
    volatile bool        _tepRefValid = false;
    volatile bool        _applyLatchComp = true;    // latch accounting on (validated on the R13 bench 2026-10-20; BENCH:LATCH 0 disables)
    volatile uint32_t    _isrMissed = 0;        // |d| summed (stat)
    volatile uint32_t    _dNeg = 0, _dPos = 0, _dBig = 0, _mPos = 0, _sinceMax = 0;   // stats
    volatile uint32_t    _resyncs = 0, _skipMax = 0;                                   // stats
#if defined(R13_BENCH_ONE_AXIS)
    volatile uint32_t    _isrDelayCycles = 0;                                          // bench calibration knob
    volatile uint32_t    _isrCount = 0, _isrCycles = 0, _isrMax = 0;
    volatile uint32_t    _isrGapMin = 0xffffffff, _isrGapSum = 0, _isrEnd = 0;
#endif
    mcpwm_oper_handle_t  _oper[3];
    mcpwm_cmpr_handle_t  _cmpr[SMSE_NUM_MOTORS];
    mcpwm_gen_handle_t   _gen[SMSE_NUM_MOTORS];

    friend void _smse_period_body(void);
    friend void _smse_direct_isr(void*);
};

// ── Inline GPIO helpers (IRAM, direct register) ───────────────────────────
static inline void IRAM_ATTR _smse_gpio_set(int pin) {
    if (pin < 32) { GPIO.out_w1ts = (1u << pin); }
    else          { GPIO.out1_w1ts.val = (1u << (pin - 32)); }
}
static inline void IRAM_ATTR _smse_gpio_clr(int pin) {
    if (pin < 32) { GPIO.out_w1tc = (1u << pin); }
    else          { GPIO.out1_w1tc.val = (1u << (pin - 32)); }
}
// Pulse gate for both generators of one operator: pulse = release the software force (the
// next period pulses), otherwise force LOW (the next period stays quiet). One pure 32-bit
// write of the whole gen_force register (no read-modify-write on the APB bus); the value
// is latched into the hardware at the next TEZ (gen_cntuforce_upmethod).
static inline void IRAM_ATTR _smse_gate(int op, bool pulseA, bool pulseB) {
    uint32_t v = (uint32_t)SMSE_FORCE_UPMETHOD;
    if (!pulseA) v |= (1u << 6);     // gen_a_cntuforce_mode = 1 (force LOW)
    if (!pulseB) v |= (1u << 8);     // gen_b_cntuforce_mode = 1
    MCPWM0.operators[op].gen_force.val = v;
}
// Latch-boundary shift: the latch event sits SMSE_TEP_BOUNDARY_CYC cycles past the period
// multiple in the "since the raising TEP" frame (scaled to the current period in cycles).
static inline uint32_t IRAM_ATTR _smse_boundary_shift(uint32_t period) {
    return period - (period * SMSE_TEP_BOUNDARY_CYC) / (SMSE_PERIOD_US_DEFAULT * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
}

// ── Period ISR body — called once per TEP by _smse_direct_isr ────────────────
// Per motor per tick: decide whether it steps (DIR change → two quiet ticks, then steps;
// DIR never moves while a released gate is in flight, armedHist). Then: the boundary guard,
// one gen_force write per operator (latched at the next TEZ → pulse in the following
// period), and exact latch accounting that books ±d to motors whose previous gate ran d
// periods more or less than one.
void IRAM_ATTR _smse_period_body(void) {
    SharedMcpwmStepEngine* self = _smse_instance;
    if (!self) return;
    // Phase reference: the cycle count of the TEP that raised this tick, in a running
    // TEP index. CPU (240 MHz) and MCPWM (1 MHz) clocks come from the same PLL, so once
    // synced the reference advances by exactly one period per TEP. A tick that enters
    // more than a period late has absorbed the TEPs in between (the flag is cleared at
    // entry); the next tick is then raised by the first TEP after this entry.
    uint32_t cc0 = esp_cpu_get_cycle_count();
    const uint32_t period = self->_periodCycles;
    if (period == 0) return;                       // never divide by it
    bool firstTick = false;
    if (!self->_tepRefValid) {
        self->_tepRef = cc0 - SMSE_ISR_ENTRY_CYC; self->_tepRefValid = true; self->_tepSkip = 0; self->_tepIdx = 0; firstTick = true;
    } else {
        self->_tepRef += period * (1u + self->_tepSkip);
        self->_tepIdx += 1u + self->_tepSkip;
        const int32_t late = (int32_t)(cc0 - self->_tepRef);
        self->_tepSkip = (late > (int32_t)period) ? (uint32_t)(late / (int32_t)period) : 0u;
    }
#if defined(R13_BENCH_ONE_AXIS)
    { uint32_t gap = cc0 - self->_isrEnd; self->_isrGapSum += gap; if (gap < self->_isrGapMin) self->_isrGapMin = gap; }
#endif

    bool stepNow[SMSE_NUM_MOTORS];
    for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
        SharedMcpwmStepEngine::MotorState&      s = self->_state[i];
        const SharedMcpwmStepEngine::MotorConfig& c = self->_cfg[i];

        // A gate released in either of the last two ticks means a pulse is in flight or
        // ended less than a period ago (write → latched at the next TEZ → pulse in the
        // period after that). DIR must not move until it is clear.
        const bool inFlight = (s.armedHist != 0);
        bool step = false;

        const int32_t cur = s.currentPos;
        const int32_t delta = s.targetPos - cur;
        if (delta != 0) {
            const bool forward = (delta > 0);
            const bool physDir = c.invertDir ? !forward : forward;

            if (physDir != s.lastDir) {
                // Direction change guard. Never move DIR while a pulse is in flight or
                // just finished: wait, then flip DIR and keep two quiet ticks before the
                // first step (same 8 µs DIR setup as McpwmStepEngine).
                if (!inFlight) {
                    if (physDir) _smse_gpio_set(c.dirPin);
                    else         _smse_gpio_clr(c.dirPin);
                    s.lastDir    = physDir;
                    s.dirPending = true;
                    s.dirChanges++;
                }
            } else if (s.dirPending) {
                s.dirPending = false;              // second guard tick, still no step
            } else {
                // Step: the gate release below is latched at the next TEZ; in the period
                // that follows the hardware drives HIGH at SMSE_PULSE_START and LOW
                // SMSE_PULSE_US later. Accounted now.
                step = true;
                s.currentPos = cur + (forward ? 1 : -1);
                s.totalSteps++;
            }
        }

        stepNow[i] = step;
        s.armedHist = (uint8_t)(((s.armedHist << 1) | (step ? 1u : 0u)) & 3u);
    }

#if defined(R13_BENCH_ONE_AXIS)
    if (self->_isrDelayCycles) {               // BENCH:ISRDELAY: shift the gate write for calibration
        uint32_t t = esp_cpu_get_cycle_count();
        while (esp_cpu_get_cycle_count() - t < self->_isrDelayCycles) { }
    }
#endif
    if (SMSE_TEP_GUARD_CYC > 0) {
        // Boundary guard: if the gate write would land within SMSE_TEP_GUARD_CYC of a latch
        // boundary, where the clock classification below is uncertain by the reference's
        // sync error, spin past the band. Hard cap: the ISR never spends more than
        // period - SMSE_ISR_TAIL_CYC cycles in total, so a late tick cannot push the next
        // one later and lock the ISR into back-to-back execution (interrupt WDT 2026-10-20).
        const uint32_t bsh = _smse_boundary_shift(period);
        const uint32_t ccs = esp_cpu_get_cycle_count();
        uint32_t ph0 = (ccs - self->_tepRef + bsh) % period;
        const uint32_t budget = period - SMSE_ISR_TAIL_CYC;
        for (;;) {
            const uint32_t now = esp_cpu_get_cycle_count();
            uint32_t ph = ph0 + (now - ccs);
            while (ph >= period) ph -= period;
            if (ph > (uint32_t)SMSE_TEP_GUARD_CYC && ph < period - (uint32_t)SMSE_TEP_GUARD_CYC) break;
            if (now - cc0 >= budget) break;
        }
    }
    // One gate write per operator per tick (motors 2k and 2k+1 share a register): pulse in
    // the next period, or not. Written every tick, not only on change, so a stale state can
    // never free-run if the ISR is late.
    for (int op = 0; op < 3; op++) _smse_gate(op, stepNow[2 * op], stepNow[2 * op + 1]);

    // Exact accounting. The hardware pulses a motor once per period while its latched
    // gate is released; the software counted one step per tick that released the gate.
    // The two agree only if every gate write is latched exactly one event after the
    // previous one. Here the latch event of this tick's write is determined (the writes
    // normally land ~1.7 µs after the raising TEP, m = 0; a tick held off by a critical
    // section lands m events later; near-boundary writes were moved out of the uncertain
    // band by the guard above). The PREVIOUS tick's gate was in force for
    // (latch_now - latch_prev) periods: 0 if this write overrides it before it was ever
    // latched, 2+ if this write was late. Book the difference to every motor whose previous
    // gate was released.
    {
        const uint32_t since = esp_cpu_get_cycle_count() - self->_tepRef;   // cycles since the raising TEP
        // m = latch events that passed between the raising TEP and this write: the write is
        // latched one event later for every boundary-aligned period it is past.
        const uint32_t bshift = _smse_boundary_shift(period);
        uint32_t m = (since + bshift) / period;
        if (self->_tepSkip > self->_skipMax) self->_skipMax = self->_tepSkip;
        if (m > 8 || self->_tepSkip > 8) {
            // Blocked for more than 8 periods: resync the reference at the next tick and
            // skip the accounting for this one. Counted, and recorded in the event ring,
            // because every such tick can cost a pulse. (The clocks share the PLL, so the
            // reference itself cannot drift; only a 32-bit wrap of >17 s could.)
            self->_resyncs++;
#if defined(R13_BENCH_ONE_AXIS)
            {
                volatile SharedMcpwmStepEngine::LatchEvent &e = self->_ring[self->_ringN % SharedMcpwmStepEngine::LATCH_RING];
                e.tick = self->_tepIdx; e.since = since; e.m = m; e.skip = self->_tepSkip; e.d = 99;
                e.hist0 = self->_state[0].armedHist; self->_ringN++;
            }
#endif
            self->_tepRefValid = false;
            m = 0; firstTick = true;
        }
        if (m > 0) self->_mPos++;
        if (since > self->_sinceMax) self->_sinceMax = since;
        const uint32_t latchNow = self->_tepIdx + m + 1;             // event index of this write's latch
        if (!firstTick) {
            const int32_t d = (int32_t)(latchNow - self->_latchPrev) - 1;   // periods the previous gate ran, minus the one counted
            if (d == -1) self->_dNeg++; else if (d == 1) self->_dPos++; else if (d != 0) self->_dBig++;
#if defined(R13_BENCH_ONE_AXIS)
            if (d != 0 && self->_state[0].armedHist != 0) {   // around real steps of axis 0 only
                volatile SharedMcpwmStepEngine::LatchEvent &e = self->_ring[self->_ringN % SharedMcpwmStepEngine::LATCH_RING];
                e.tick = self->_tepIdx; e.since = since; e.m = m; e.skip = self->_tepSkip; e.d = d;
                e.hist0 = self->_state[0].armedHist;
                self->_ringN++;
            }
#endif
            if (d != 0 && self->_applyLatchComp && d >= -8 && d <= 8) {
                self->_isrMissed += (uint32_t)(d < 0 ? -d : d);
                for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
                    SharedMcpwmStepEngine::MotorState& s = self->_state[i];
                    if (!(s.armedHist & 2u)) continue;              // previous tick's gate (already shifted) was off
                    const SharedMcpwmStepEngine::MotorConfig& c = self->_cfg[i];
                    const bool fwd = c.invertDir ? !s.lastDir : s.lastDir;   // DIR cannot have moved since (in-flight guard)
                    s.currentPos += fwd ? d : -d;
                    s.totalSteps  = (uint32_t)((int32_t)s.totalSteps + d);
                }
            }
        }
        self->_latchPrev = latchNow;
    }

#if defined(R13_BENCH_ONE_AXIS)
    uint32_t dc = esp_cpu_get_cycle_count() - cc0;
    self->_isrCount++; self->_isrCycles += dc; if (dc > self->_isrMax) self->_isrMax = dc;
    self->_isrEnd = esp_cpu_get_cycle_count();
#endif
}

// Direct level-3 handler on the MCPWM0 group interrupt (only TEP of timer 0 is enabled).
void IRAM_ATTR _smse_direct_isr(void *) {
    MCPWM0.int_clr.val = SMSE_INT_TEP0;
    _smse_period_body();
}
