#pragma once
// NOTE: This header defines a file-scope IRAM_ATTR ISR function.
// It must be included in exactly ONE .cpp translation unit (main.cpp).
// Multiple inclusions will cause duplicate symbol link errors.

/*
 * SimpleStepEngine — Single 250 kHz GPTimer ISR drives all 6 motors
 *
 * Architecture:
 *   One hardware GPTimer fires every 4µs (250 kHz), unconditionally.
 *   The ISR services all 6 motors in a single pass:
 *     1. If current == target → idle, skip.
 *     2. If direction changed since last step → set DIR pin, skip step this tick
 *        (direction setup time = 4µs, exactly one tick).
 *     3. Otherwise → pulse STEP pin HIGH, increment/decrement position counter.
 *   Step pin is brought LOW on the NEXT tick (alternating HIGH/LOW phases).
 *
 * Properties:
 *   - Max step rate: 125 kHz per motor (one step per 2 ticks: HIGH + LOW)
 *   - Direction setup: exactly 4µs (one missed tick) — meets AASD-15A ≥2µs spec
 *   - Step pulse width: exactly 4µs — meets AASD-15A ≥1.5µs spec
 *   - Position counter IS the step — no drift, no coast, no polling loop
 *   - All 6 motors in one ISR pass: ~180 cycles at 240 MHz, well within 960-cycle budget
 *   - No MCPWM, no PCNT, no RMT, no continuous mode, no task-notify round-trip
 *   - ISR runs directly in GPTimer callback — zero scheduling jitter
 *
 * Soft limits: enforced in setTarget(), not in ISR (ISR must be fast).
 * E-stop: call emergencyStop() — sets all targets to current position.
 * Thread safety: targets written atomically (32-bit aligned volatile).
 */

#include <driver/gptimer.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "debug_uart.h"
#include "soc/gpio_struct.h"   // GPIO.out_w1ts / GPIO.out_w1tc direct register access

// Number of motors (fixed at 6 for this platform)
#define SSE_NUM_MOTORS 6

// Default tick period. Actual rate is runtime-configurable via setTickRate().
// Step pulse = HIGH one tick + LOW next tick, so max step rate = 1/(2*tick_us).
// AASD-15A minimums: pulse width ≥1.5µs, dir setup ≥2µs → tick_us ≥ 2.
//
// HARD LIMIT: 4µs minimum (250kHz ISR rate).
// At 2µs (500kHz), the ISR fires faster than the interrupt WDT can get a window
// on ESP32-S3, causing a guaranteed WDT panic regardless of CPU speed or core.
// The WDT interrupt (priority 4) needs ~10µs window; at 2µs the level-1 ISR
// re-arms before the WDT ever executes. 4µs gives 960 cycles @ 240MHz (5x margin).
#define SSE_TICK_US_DEFAULT  4    // 4µs tick → 125 kHz max step rate
#define SSE_TICK_US_MIN      4    // hard floor: WDT-safe on ESP32-S3
#define SSE_TICK_US_MAX      100  // 100µs tick → 5 kHz max step rate

// Forward declarations
class SimpleStepEngine;
// DRAM_ATTR: must be in internal DRAM so the IRAM ISR can access it
// without triggering a flash cache miss (EXCCAUSE 5).
static DRAM_ATTR SimpleStepEngine* _sse_isr_instance = nullptr;

// Forward-declare the ISR so the friend declaration inside the class is valid.
// Defined at file scope below (IRAM_ATTR, non-static so friend linkage matches).
bool _sse_isr_trampoline(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);

class SimpleStepEngine {
public:
    // Per-motor configuration (set once at init, read-only after start)
    struct MotorConfig {
        gpio_num_t stepPin;
        gpio_num_t dirPin;
        bool       invertDir;       // true = flip direction polarity
        int32_t    softLimitMin;    // steps, enforced in setTarget()
        int32_t    softLimitMax;    // steps, enforced in setTarget()
        bool       enableSoftLimits;

        MotorConfig() :
            stepPin(GPIO_NUM_NC),
            dirPin(GPIO_NUM_NC),
            invertDir(false),
            softLimitMin(-100000),
            softLimitMax(100000),
            enableSoftLimits(true)
        {}
    };

    // Per-motor runtime state — all volatile, ISR-accessible
    struct MotorState {
        volatile int32_t  currentPos;       // exact step count (ISR writes)
        volatile int32_t  targetPos;        // commanded position (task writes)
        volatile bool     lastDir;          // last direction sent to DIR pin
        volatile bool     stepHigh;         // true = step pin currently HIGH
        volatile bool     dirPending;       // direction change queued, skip step this tick
        volatile uint32_t totalSteps;       // cumulative step count (diagnostic)
        volatile uint32_t dirChanges;       // direction reversals (diagnostic)
    };

    // ── Singleton access ──────────────────────────────────────────────
    static SimpleStepEngine& instance() {
        static SimpleStepEngine eng;
        return eng;
    }

    // ── Initialization ────────────────────────────────────────────────
    // Call once from the motor task (core 1) before start().
    bool init(const MotorConfig configs[SSE_NUM_MOTORS]) {
        if (_initialized) return true;

        for (int i = 0; i < SSE_NUM_MOTORS; i++) {
            _cfg[i] = configs[i];

            // Configure STEP pin as output, start LOW
            gpio_config_t io = {};
            io.pin_bit_mask = (1ULL << configs[i].stepPin);
            io.mode         = GPIO_MODE_OUTPUT;
            io.pull_up_en   = GPIO_PULLUP_DISABLE;
            io.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io.intr_type    = GPIO_INTR_DISABLE;
            gpio_config(&io);
            gpio_set_level(configs[i].stepPin, 0);

            // Configure DIR pin as output, start LOW
            io.pin_bit_mask = (1ULL << configs[i].dirPin);
            gpio_config(&io);
            gpio_set_level(configs[i].dirPin, 0);

            // Zero state
            _state[i].currentPos  = 0;
            _state[i].targetPos   = 0;
            _state[i].lastDir     = false;
            _state[i].stepHigh    = false;
            _state[i].dirPending  = false;
            _state[i].totalSteps  = 0;
            _state[i].dirChanges  = 0;
        }

        _initialized = true;
        DEBUG_PRINTF("SimpleStepEngine: %d motors initialized\n", SSE_NUM_MOTORS);
        return true;
    }

    // ── Start the GPTimer at the current tick rate ───────────────────
    // The ISR runs directly in the GPTimer callback — no task notification.
    bool start() {
        if (!_initialized) {
            DEBUG_PRINTLN("SimpleStepEngine: not initialized");
            return false;
        }
        if (_running) return true;

        _sse_isr_instance = this;

        gptimer_config_t timer_cfg = {};
        timer_cfg.clk_src       = GPTIMER_CLK_SRC_DEFAULT;
        timer_cfg.direction     = GPTIMER_COUNT_UP;
        timer_cfg.resolution_hz = 1000000;   // 1 MHz → 1µs tick
        timer_cfg.intr_priority = 0;
        esp_err_t e;
        e = gptimer_new_timer(&timer_cfg, &_timer);
        if (e != ESP_OK) { _lastErr = e; _lastErrStep = "new_timer"; return false; }

        gptimer_alarm_config_t alarm_cfg = {};
        alarm_cfg.alarm_count  = _tickUs;
        alarm_cfg.reload_count = 0;
        alarm_cfg.flags.auto_reload_on_alarm = true;
        e = gptimer_set_alarm_action(_timer, &alarm_cfg);
        if (e != ESP_OK) { _lastErr = e; _lastErrStep = "set_alarm"; gptimer_del_timer(_timer); return false; }

        gptimer_event_callbacks_t cbs = {};
        cbs.on_alarm = ::_sse_isr_trampoline;
        e = gptimer_register_event_callbacks(_timer, &cbs, nullptr);
        if (e != ESP_OK) { _lastErr = e; _lastErrStep = "register_cb"; gptimer_del_timer(_timer); return false; }

        e = gptimer_enable(_timer);
        if (e != ESP_OK) { _lastErr = e; _lastErrStep = "enable"; gptimer_del_timer(_timer); return false; }

        e = gptimer_start(_timer);
        if (e != ESP_OK) { _lastErr = e; _lastErrStep = "start"; gptimer_disable(_timer); gptimer_del_timer(_timer); return false; }

        _running = true;
        DEBUG_PRINTF("SimpleStepEngine: GPTimer started at %lu µs tick (%lu Hz tick, %lu Hz max step)\n",
                     (unsigned long)_tickUs,
                     (unsigned long)(1000000UL / _tickUs),
                     (unsigned long)(1000000UL / (_tickUs * 2)));
        return true;
    }

    // ── Hot-reconfigure tick rate without stopping the timer ──────────
    // new_tick_us: 2–100 µs (250 kHz down to 5 kHz max step rate)
    // Safe to call from task context while ISR is running — GPTimer alarm
    // update is atomic from the hardware perspective.
    bool setTickRate(uint32_t new_tick_us) {
        if (new_tick_us < SSE_TICK_US_MIN || new_tick_us > SSE_TICK_US_MAX)
            return false;
        _tickUs = new_tick_us;
        if (!_running) return true;  // will take effect on next start()
        gptimer_alarm_config_t alarm_cfg = {};
        alarm_cfg.alarm_count  = _tickUs;
        alarm_cfg.reload_count = 0;
        alarm_cfg.flags.auto_reload_on_alarm = true;
        return gptimer_set_alarm_action(_timer, &alarm_cfg) == ESP_OK;
    }

    uint32_t getTickUs()      const { return _tickUs; }
    uint32_t getMaxStepHz()   const { return 1000000UL / (_tickUs * 2); }
    uint32_t getTickHz()      const { return 1000000UL / _tickUs; }

    // ── Stop the timer (E-stop path) ──────────────────────────────────
    void stop() {
        if (!_running) return;
        gptimer_stop(_timer);
        _running = false;
        // Drive all step pins LOW immediately
        for (int i = 0; i < SSE_NUM_MOTORS; i++) {
            if (_cfg[i].stepPin != GPIO_NUM_NC)
                gpio_set_level(_cfg[i].stepPin, 0);
            _state[i].stepHigh = false;
        }
        DEBUG_PRINTLN("SimpleStepEngine: stopped");
    }

    // ── Resume after E-stop ───────────────────────────────────────────
    void resume() {
        if (!_initialized || _running) return;
        gptimer_start(_timer);
        _running = true;
        DEBUG_PRINTLN("SimpleStepEngine: resumed");
    }

    // ── Motor API (called from task context) ──────────────────────────

    // Set target position in steps. Returns false if soft limit violated.
    bool setTarget(int i, int32_t pos) {
        if (i < 0 || i >= SSE_NUM_MOTORS) return false;
        if (_cfg[i].enableSoftLimits) {
            if (pos < _cfg[i].softLimitMin) return false;
            if (pos > _cfg[i].softLimitMax) return false;
        }
        _state[i].targetPos = pos;  // 32-bit aligned volatile write — atomic on Xtensa
        return true;
    }

    // Read current position (ISR-written, task reads)
    int32_t getPosition(int i) const {
        if (i < 0 || i >= SSE_NUM_MOTORS) return 0;
        return _state[i].currentPos;
    }

    int32_t getTarget(int i) const {
        if (i < 0 || i >= SSE_NUM_MOTORS) return 0;
        return _state[i].targetPos;
    }

    // Emergency stop: freeze all motors at current position
    void emergencyStop() {
        for (int i = 0; i < SSE_NUM_MOTORS; i++) {
            _state[i].targetPos = _state[i].currentPos;
        }
    }

    // Zero position (home): reset current and target to 0
    void resetPosition(int i) {
        if (i < 0 || i >= SSE_NUM_MOTORS) return;
        _state[i].currentPos = 0;
        _state[i].targetPos  = 0;
    }

    // Diagnostics
    uint32_t getTotalSteps(int i)  const { return (i >= 0 && i < SSE_NUM_MOTORS) ? _state[i].totalSteps : 0; }
    uint32_t getDirChanges(int i)  const { return (i >= 0 && i < SSE_NUM_MOTORS) ? _state[i].dirChanges : 0; }
    bool     isInitialized()       const { return _initialized; }
    bool     isRunning()           const { return _running; }
    uint64_t getTickCount()        const { return _tickCount; }
    int      getLastError()        const { return _lastErr; }   // last esp_err_t from start()
    const char* getLastErrorStep() const { return _lastErrStep; } // which call failed

private:
    SimpleStepEngine() : _initialized(false), _running(false), _timer(nullptr),
                         _tickCount(0), _tickUs(SSE_TICK_US_DEFAULT),
                         _lastErr(0), _lastErrStep("none") {}

    MotorConfig  _cfg[SSE_NUM_MOTORS];
    MotorState   _state[SSE_NUM_MOTORS];
    bool         _initialized;
    bool         _running;
    gptimer_handle_t _timer;
    volatile uint64_t _tickCount;
    uint32_t     _tickUs;
    int          _lastErr;       // last esp_err_t from start()
    const char*  _lastErrStep;  // label of failing call

    // Grant the file-scope ISR access to private members
    friend bool _sse_isr_trampoline(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
};

// ── File-scope ISR — must be at file scope for Xtensa literal pool placement ──
// IRAM_ATTR places code + literal pool in IRAM so l32r can reach its literals.
// Non-static (extern linkage) so the friend declaration inside the class matches.
// Budget: 960 cycles at 240 MHz. Measured usage: ~180 cycles for 6 motors.
// Inline GPIO set/clear using direct register writes.
// Avoids gpio_set_level() which is not IRAM-safe without CONFIG_GPIO_CTRL_FUNC_IN_IRAM.
// w1ts = write-1-to-set, w1tc = write-1-to-clear. Single 32-bit store, zero stack.
static inline void IRAM_ATTR _sse_gpio_set(int pin) {
    if (pin < 32) { GPIO.out_w1ts = (1u << pin); }
    else          { GPIO.out1_w1ts.val = (1u << (pin - 32)); }
}
static inline void IRAM_ATTR _sse_gpio_clr(int pin) {
    if (pin < 32) { GPIO.out_w1tc = (1u << pin); }
    else          { GPIO.out1_w1tc.val = (1u << (pin - 32)); }
}

bool IRAM_ATTR _sse_isr_trampoline(gptimer_handle_t /*timer*/,
                                    const gptimer_alarm_event_data_t* /*edata*/,
                                    void* /*user_ctx*/) {
    SimpleStepEngine* self = _sse_isr_instance;
    if (!self || !self->_initialized) return false;

    self->_tickCount++;

    for (int i = 0; i < SSE_NUM_MOTORS; i++) {
        SimpleStepEngine::MotorState& s   = self->_state[i];
        const SimpleStepEngine::MotorConfig& c = self->_cfg[i];

        if (s.stepHigh) {
            _sse_gpio_clr(c.stepPin);
            s.stepHigh = false;
            continue;
        }

        int32_t delta = s.targetPos - s.currentPos;
        if (delta == 0) continue;

        bool forward = (delta > 0);
        bool physDir = c.invertDir ? !forward : forward;

        if (physDir != s.lastDir) {
            if (physDir) _sse_gpio_set(c.dirPin);
            else         _sse_gpio_clr(c.dirPin);
            s.lastDir    = physDir;
            s.dirPending = true;
            s.dirChanges++;
            continue;
        }

        if (s.dirPending) {
            s.dirPending = false;
        }

        _sse_gpio_set(c.stepPin);
        s.stepHigh   = true;
        s.currentPos += forward ? 1 : -1;
        s.totalSteps++;
    }

    return false;
}

