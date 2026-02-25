#pragma once
// NOTE: This header defines file-scope IRAM_ATTR ISR callbacks.
// Include in exactly ONE translation unit (main.cpp).

/*
 * SharedMcpwmStepEngine — 1 MCPWM timer, 6 hardware-assisted pulse channels
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
 *   Pulse generation (hybrid ISR + hardware):
 *     - Rising edge (STEP HIGH): ISR writes GPIO.out_w1ts — software, IRAM-safe.
 *     - Falling edge (STEP LOW):  hardware generator LOW-on-compare fires at
 *       count = SMSE_PULSE_US, driving pin LOW without CPU involvement.
 *
 *   Why not HIGH-on-TEZ (hardware rising edge)?
 *     The TEZ action fires every period unconditionally. There is no compare value
 *     that means "never fire" in COUNT_UP mode (IDF rejects compare > peak).
 *     force_level() is the only suppression mechanism but it is not ISR-safe.
 *     Solution: skip the TEZ HIGH action entirely. ISR controls rising edge;
 *     hardware controls falling edge. Result: pin only goes HIGH when the ISR
 *     explicitly sets it — idle motors stay LOW with zero hardware activity.
 *
 *   DIR/STEP synchronisation (identical to McpwmStepEngine / SimpleStepEngine):
 *     Tick N:   physDir changed → set DIR pin via w1ts/w1tc, set dirPending, skip STEP
 *     Tick N+1: dirPending → clear dirPending, skip STEP  (8µs total gap)
 *     Tick N+2: fire STEP HIGH via w1ts; hardware comparator fires LOW at +SMSE_PULSE_US
 *     DIR setup time = 8µs (2 × 4µs ticks) — meets AASD-15A ≥ 2µs spec (4× margin).
 *
 *   Motors that are at target position or in dir-change hold simply skip their tick.
 *   All other motors are completely unaffected — the timer runs freely at 250 kHz.
 *
 * Properties:
 *   - Max step rate:   250 kHz/motor (one step per 4µs period, hardware falling edge)
 *   - Step pulse width: SMSE_PULSE_US µs (default 2µs, meets AASD-15A ≥ 1.5µs)
 *   - DIR setup time:   8µs (2 ISR ticks) — AASD-15A requires ≥ 2µs
 *   - MCPWM resources:  1 timer, 3 operators, 6 comparators, 6 generators
 *   - No PCNT, no RMT, no per-motor timer
 *   - ISR budget:       ~200 cycles @ 240 MHz for 6 motors — within 960-cycle budget
 *   - Direct GPIO register writes (w1ts/w1tc) for rising edge — IRAM-safe
 *   - Hardware generator for falling edge — zero CPU cost per pulse end
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
#include "debug_uart.h"

#define SMSE_NUM_MOTORS        6
#define SMSE_PERIOD_US_DEFAULT 4    // 4µs period → 250 kHz tick rate, 250 kHz max step rate
#define SMSE_PERIOD_US_MIN     4    // hard floor: WDT-safe on ESP32-S3
#define SMSE_PERIOD_US_MAX     100  // 100µs period → 5 kHz max step rate
#define SMSE_PULSE_US          2    // 2µs pulse width (hardware LOW-on-compare, must be < period)
// Keep old name as alias so StepDriver.h report() still compiles
#define SMSE_PERIOD_US         SMSE_PERIOD_US_DEFAULT

// Forward declaration
class SharedMcpwmStepEngine;
static DRAM_ATTR SharedMcpwmStepEngine* _smse_instance = nullptr;

// Forward-declare ISR so friend decl inside class is valid
bool _smse_tez_isr(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void*);

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
        volatile uint32_t totalSteps;
        volatile uint32_t dirChanges;
        uint8_t           opIdx;    // MCPWM operator index (0-2)
        uint8_t           cmprIdx;  // comparator index within operator (0 or 1)
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
            _state[i].opIdx   = (uint8_t)(i / 2);  // operator 0,0,1,1,2,2
            _state[i].cmprIdx = (uint8_t)(i % 2);  // comparator A or B
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

        // ── Allocate 6 comparators + 6 generators (2 per operator) ────
        // Generator action: LOW when compare fires (hardware falling edge).
        // NO HIGH-on-TEZ action — rising edge is driven by ISR w1ts write.
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
            int op  = i / 2;   // operator index: 0,0,1,1,2,2

            mcpwm_comparator_config_t c_cfg = {};
            c_cfg.intr_priority = 0;
            c_cfg.flags.update_cmp_on_tez = false;  // immediate update: value takes effect this period

            if (mcpwm_new_comparator(_oper[op], &c_cfg, &_cmpr[i]) != ESP_OK) {
                DEBUG_PRINTF("SharedMcpwmStepEngine: comparator %d alloc failed\n", i);
                _cleanup();
                return false;
            }
            // Idle compare value = 0: LOW-on-compare fires at count=0 (same as TEZ).
            // Compare actions have higher priority than timer events on ESP32-S3 MCPWM,
            // so the LOW wins over HIGH-on-TEZ — pin stays LOW when idle.
            // When stepping, ISR sets compare = SMSE_PULSE_US so LOW fires at 2µs.
            mcpwm_comparator_set_compare_value(_cmpr[i], 0);

            mcpwm_generator_config_t g_cfg = {};
            g_cfg.gen_gpio_num = _cfg[i].stepPin;

            if (mcpwm_new_generator(_oper[op], &g_cfg, &_gen[i]) != ESP_OK) {
                DEBUG_PRINTF("SharedMcpwmStepEngine: generator %d alloc failed\n", i);
                _cleanup();
                return false;
            }

            // Fully hardware-driven pulse:
            //   HIGH on TEZ  (timer empty = count wraps to 0) — rising edge
            //   LOW  on compare match (count == SMSE_PULSE_US) — falling edge
            // No ISR GPIO write needed — hardware handles both edges.
            mcpwm_gen_timer_event_action_t tez_act = {};
            tez_act.direction = MCPWM_TIMER_DIRECTION_UP;
            tez_act.event     = MCPWM_TIMER_EVENT_EMPTY;
            tez_act.action    = MCPWM_GEN_ACTION_HIGH;
            mcpwm_generator_set_action_on_timer_event(_gen[i], tez_act);

            mcpwm_gen_compare_event_action_t cmp_act = {};
            cmp_act.direction  = MCPWM_TIMER_DIRECTION_UP;
            cmp_act.comparator = _cmpr[i];
            cmp_act.action     = MCPWM_GEN_ACTION_LOW;
            mcpwm_generator_set_action_on_compare_event(_gen[i], cmp_act);
        }

        // ── Register TEZ (on_full in COUNT_UP = timer wraps to 0) ISR ──
        // on_full fires once per period (every 4µs) — same as McpwmStepEngine.
        mcpwm_timer_event_callbacks_t cbs = {};
        cbs.on_full = ::_smse_tez_isr;
        if (mcpwm_timer_register_event_callbacks(_timer, &cbs, nullptr) != ESP_OK) {
            DEBUG_PRINTLN("SharedMcpwmStepEngine: register_cb failed");
            _cleanup();
            return false;
        }

        if (mcpwm_timer_enable(_timer) != ESP_OK) {
            DEBUG_PRINTLN("SharedMcpwmStepEngine: timer enable failed");
            _cleanup();
            return false;
        }

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

    void stop() {
        if (!_running) return;
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_FULL);
        _running = false;
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
            if (_cfg[i].stepPin != GPIO_NUM_NC) gpio_set_level(_cfg[i].stepPin, 0);
        }
        DEBUG_PRINTLN("SharedMcpwmStepEngine: stopped");
    }

    void resume() {
        if (!_initialized || _running) return;
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
    uint32_t getMaxStepHz() const { return 1000000UL / _tickUs; }  // one step per period (hardware falling edge)
    uint32_t getTickHz()    const { return 1000000UL / _tickUs; }

    // Hot-reconfigure period without stopping the timer.
    // new_tick_us: SMSE_PERIOD_US_MIN–SMSE_PERIOD_US_MAX.
    // Pulse width (SMSE_PULSE_US) must remain < new period — validated here.
    // mcpwm_timer_set_period() loads the new value at the next TEZ.
    // Idle comparator values are also updated so they track the new period.
    bool setTickRate(uint32_t new_tick_us) {
        if (new_tick_us < SMSE_PERIOD_US_MIN || new_tick_us > SMSE_PERIOD_US_MAX) {
            DEBUG_PRINTF("SharedMcpwmStepEngine: setTickRate %lu µs out of range [%d, %d]\n",
                (unsigned long)new_tick_us, SMSE_PERIOD_US_MIN, SMSE_PERIOD_US_MAX);
            return false;
        }
        if (new_tick_us <= SMSE_PULSE_US) {
            DEBUG_PRINTF("SharedMcpwmStepEngine: setTickRate %lu µs must be > SMSE_PULSE_US (%d)\n",
                (unsigned long)new_tick_us, SMSE_PULSE_US);
            return false;
        }
        _tickUs = new_tick_us;
        if (!_running) return true;  // takes effect on next start()
        esp_err_t e = mcpwm_timer_set_period(_timer, _tickUs);
        if (e != ESP_OK) {
            DEBUG_PRINTF("SharedMcpwmStepEngine: set_period failed: %d\n", e);
            return false;
        }
        // Update idle comparator values for all motors currently at target
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
            if (_state[i].targetPos == _state[i].currentPos)
                mcpwm_comparator_set_compare_value(_cmpr[i], 0);
        }
        DEBUG_PRINTF("SharedMcpwmStepEngine: tick rate → %lu µs (%lu Hz, %lu Hz max step)\n",
            (unsigned long)_tickUs,
            (unsigned long)(1000000UL / _tickUs),
            (unsigned long)(1000000UL / _tickUs));
        return true;
    }

private:
    SharedMcpwmStepEngine() : _initialized(false), _running(false), _timer(nullptr),
                               _tickUs(SMSE_PERIOD_US_DEFAULT) {
        for (int i = 0; i < 3; i++) _oper[i] = nullptr;
        for (int i = 0; i < SMSE_NUM_MOTORS; i++) { _cmpr[i] = nullptr; _gen[i] = nullptr; }
    }

    void _cleanup() {
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
    uint32_t             _tickUs;
    mcpwm_oper_handle_t  _oper[3];
    mcpwm_cmpr_handle_t  _cmpr[SMSE_NUM_MOTORS];
    mcpwm_gen_handle_t   _gen[SMSE_NUM_MOTORS];

    friend bool _smse_tez_isr(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void*);
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

// ── TEZ ISR — fires every 4µs (on_full in COUNT_UP mode) ─────────────────
//
// Responsibilities per motor per tick:
//   1. Decide if this motor steps this tick (same state machine as McpwmStepEngine)
//   2. If stepping: write w1ts to raise STEP pin HIGH.
//                   Update compare register to SMSE_PULSE_US so the hardware
//                   generator drives STEP LOW exactly SMSE_PULSE_US later.
//   3. If not stepping: ensure compare register stays at SMSE_PERIOD_US (idle).
//                       Pin is already LOW; the compare-LOW action at count=period
//                       is a harmless no-op.
//
// DIR/STEP timing (identical to McpwmStepEngine):
//   Tick N:   DIR changes → set dir pin, dirPending=true, skip STEP → 4µs gap
//   Tick N+1: dirPending → clear flag, skip STEP → 8µs total gap before first step
//   Tick N+2: STEP HIGH via w1ts; hardware LOW at count=SMSE_PULSE_US
//
// Note on immediate comparator update (update_cmp_on_tez=false):
//   mcpwm_comparator_set_compare_value() with no shadow register takes effect
//   immediately — the comparator hardware sees the new threshold within the
//   same timer period. Since the ISR fires at TEZ (count=0) and the timer is
//   already counting upward, writing compare=SMSE_PULSE_US here causes the
//   hardware generator to drive the pin LOW when count reaches SMSE_PULSE_US
//   during THIS SAME 4µs window. This is exactly what we want.
//
//   Per-motor per-tick logic:
//   - Stepping motor:  ISR writes HIGH (w1ts), then compare=SMSE_PULSE_US
//                      → hardware drives LOW at count=SMSE_PULSE_US (2µs later)
//   - Idle motor:      compare stays at SMSE_PERIOD_US; fires LOW at count=4
//                      which is a no-op on an already-LOW pin. Zero ISR cost.
bool IRAM_ATTR _smse_tez_isr(mcpwm_timer_handle_t /*timer*/,
                               const mcpwm_timer_event_data_t* /*edata*/,
                               void* /*user_ctx*/) {
    SharedMcpwmStepEngine* self = _smse_instance;
    if (!self) return false;

    for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
        SharedMcpwmStepEngine::MotorState&      s = self->_state[i];
        const SharedMcpwmStepEngine::MotorConfig& c = self->_cfg[i];

        int32_t delta = s.targetPos - s.currentPos;
        if (delta == 0) continue;

        bool forward = (delta > 0);
        bool physDir = c.invertDir ? !forward : forward;

        // ── Direction change guard ────────────────────────────────────
        if (physDir != s.lastDir) {
            if (physDir) _smse_gpio_set(c.dirPin);
            else         _smse_gpio_clr(c.dirPin);
            s.lastDir    = physDir;
            s.dirPending = true;
            s.dirChanges++;
            continue;   // skip STEP this tick — DIR just changed
        }

        if (s.dirPending) {
            s.dirPending = false;
            continue;   // skip STEP this tick — second guard tick
        }

        // ── Fire step pulse ───────────────────────────────────────────
        // Hardware generates rising edge at TEZ automatically (HIGH-on-TEZ action).
        // Arm comparator to fire falling edge at SMSE_PULSE_US via direct register
        // write (IRAM-safe — avoids driver flash access at 250 kHz ISR rate).
        MCPWM0.operators[s.opIdx].timestamp[s.cmprIdx].val = SMSE_PULSE_US;

        s.currentPos += forward ? 1 : -1;
        s.totalSteps++;
    }

    // Reset comparators for motors that just reached their target this tick
    // (delta became 0 after the position increment above).
    // Setting compare=0: LOW-on-compare fires at count=0 (higher priority than TEZ HIGH)
    // — pin stays LOW. Zero-width pulse is invisible on the GPIO.
    for (int i = 0; i < SMSE_NUM_MOTORS; i++) {
        SharedMcpwmStepEngine::MotorState& s = self->_state[i];
        if (s.targetPos == s.currentPos) {
            MCPWM0.operators[s.opIdx].timestamp[s.cmprIdx].val = 0;
        }
    }

    return false;  // no high-priority task woken
}
