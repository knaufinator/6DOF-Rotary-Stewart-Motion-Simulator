#pragma once
// NOTE: This header defines file-scope IRAM_ATTR ISR callbacks.
// Include in exactly ONE translation unit (main.cpp).

/*
 * McpwmStepEngine — Single MCPWM TEZ ISR drives all 6 motors
 *
 * Architecture:
 *   One MCPWM timer (group 0, timer 0) runs at 4µs period (250 kHz tick rate).
 *   The TEZ (timer-empty) callback fires every 4µs in IRAM — identical algorithm
 *   to SimpleStepEngine, but using MCPWM hardware instead of GPTimer.
 *
 *   Per tick, for each motor:
 *     1. If step pin is HIGH → bring it LOW, continue.
 *     2. If current == target → idle.
 *     3. If direction changed → set DIR pin, mark dirPending, skip step this tick
 *        (4µs dir setup time, exactly one tick gap).
 *     4. Otherwise → pulse STEP pin HIGH, advance position counter.
 *
 * Advantage over SimpleStepEngine (GPTimer):
 *   MCPWM TEZ ISR handlers are placed in IRAM automatically by ESP-IDF's MCPWM
 *   driver (mcpwm_timer.c uses ESP_INTR_FLAG_IRAM). No extra sdkconfig flags
 *   needed. GPTimer requires CONFIG_GPTIMER_ISR_IRAM_SAFE=y or flash cache
 *   misses stall the ISR at 250 kHz, breaking UART/serial output.
 *
 * Properties:
 *   - Max step rate: 125 kHz/motor (HIGH tick + LOW tick = 2 ticks per step)
 *   - Step pulse width: exactly 4µs (meets AASD-15A ≥1.5µs)
 *   - Direction setup: exactly 4µs gap (meets AASD-15A ≥2µs)
 *   - Uses 1 MCPWM timer of 6 available. No PCNT, no RMT, no per-motor MCPWM.
 *   - Direct GPIO register writes (w1ts/w1tc) — zero stack, IRAM-safe.
 *   - ~180 cycles @ 240 MHz per tick for 6 motors — well within 960-cycle budget.
 */

#include "driver/mcpwm_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_private/periph_ctrl.h"
#include "soc/mcpwm_periph.h"
#include "debug_uart.h"
#include "soc/gpio_struct.h"

#define MSE_NUM_MOTORS  6
#define MSE_TICK_US     4    // 4µs tick → 125 kHz max step rate (2µs=500kHz ISR causes WDT panic on ESP32-S3)

// Forward declaration
class McpwmStepEngine;
static DRAM_ATTR McpwmStepEngine* _mse_instance = nullptr;

// Forward-declare ISR so friend decl inside class is valid
bool _mse_tez_isr(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void*);

class McpwmStepEngine {
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
        volatile bool     stepHigh;
        volatile bool     dirPending;
        volatile uint32_t totalSteps;
        volatile uint32_t dirChanges;
    };

    static McpwmStepEngine& instance() {
        static McpwmStepEngine eng;
        return eng;
    }

    bool init(const MotorConfig configs[MSE_NUM_MOTORS]) {
        if (_initialized) return true;
        for (int i = 0; i < MSE_NUM_MOTORS; i++) {
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

            _state[i] = {};  // zero all volatile fields
        }
        _initialized = true;
        DEBUG_PRINTF("McpwmStepEngine: %d motors initialized\n", MSE_NUM_MOTORS);
        return true;
    }

    bool start() {
        if (!_initialized || _running) return _running;
        _mse_instance = this;

        // Force-reset MCPWM group 0 peripheral before allocating timer.
        // After a WDT-induced software reset the MCPWM driver's static timer
        // registry is cleared (RAM reset) but the peripheral registers are NOT,
        // so mcpwm_new_timer() may see the hardware as already configured.
        // A peripheral reset guarantees a clean slate.
        periph_module_reset(PERIPH_PWM0_MODULE);

        // Allocate MCPWM timer: group 0, timer 0, 1 MHz resolution → 1µs ticks
        mcpwm_timer_config_t timer_cfg = {};
        timer_cfg.group_id      = 0;
        timer_cfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timer_cfg.resolution_hz = 1000000;          // 1 MHz → 1µs per count
        timer_cfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
        timer_cfg.period_ticks  = MSE_TICK_US;      // period = 4 ticks = 4µs
        timer_cfg.intr_priority = 0;
        // flags.update_period_on_empty not needed — fixed period

        esp_err_t e = mcpwm_new_timer(&timer_cfg, &_timer);
        if (e != ESP_OK) {
            DEBUG_PRINTF("McpwmStepEngine: timer alloc failed: %d\n", e);
            return false;
        }

        // Register on_full callback — fires every time counter reaches period value.
        // In MCPWM_TIMER_COUNT_MODE_UP: counter goes 0→period, on_full fires, then wraps.
        // on_empty only fires in UP_DOWN mode (count descends back to 0), not needed here.
        mcpwm_timer_event_callbacks_t cbs = {};
        cbs.on_full = ::_mse_tez_isr;    // fires every 4µs (one full period)
        e = mcpwm_timer_register_event_callbacks(_timer, &cbs, nullptr);
        if (e != ESP_OK) {
            DEBUG_PRINTF("McpwmStepEngine: register_cb failed: %d\n", e);
            mcpwm_del_timer(_timer); return false;
        }

        e = mcpwm_timer_enable(_timer);
        if (e != ESP_OK) {
            DEBUG_PRINTF("McpwmStepEngine: enable failed: %d\n", e);
            mcpwm_del_timer(_timer); return false;
        }

        e = mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
        if (e != ESP_OK) {
            DEBUG_PRINTF("McpwmStepEngine: start failed: %d\n", e);
            mcpwm_timer_disable(_timer); mcpwm_del_timer(_timer); return false;
        }

        _running = true;
        DEBUG_PRINTF("McpwmStepEngine: started — %d µs tick, %lu Hz max step/motor\n",
            MSE_TICK_US, (unsigned long)(1000000UL / (MSE_TICK_US * 2)));
        return true;
    }

    void stop() {
        if (!_running) return;
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_FULL);
        _running = false;
        for (int i = 0; i < MSE_NUM_MOTORS; i++) {
            if (_cfg[i].stepPin != GPIO_NUM_NC) gpio_set_level(_cfg[i].stepPin, 0);
            _state[i].stepHigh = false;
        }
        DEBUG_PRINTLN("McpwmStepEngine: stopped");
    }

    void resume() {
        if (!_initialized || _running) return;
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
        _running = true;
        DEBUG_PRINTLN("McpwmStepEngine: resumed");
    }

    bool    setTarget(int i, int32_t pos) {
        if (i < 0 || i >= MSE_NUM_MOTORS) return false;
        if (_cfg[i].enableSoftLimits) {
            if (pos < _cfg[i].softLimitMin || pos > _cfg[i].softLimitMax) return false;
        }
        _state[i].targetPos = pos;
        return true;
    }
    int32_t getPosition(int i)  const { return (i >= 0 && i < MSE_NUM_MOTORS) ? _state[i].currentPos : 0; }
    int32_t getTarget(int i)    const { return (i >= 0 && i < MSE_NUM_MOTORS) ? _state[i].targetPos  : 0; }
    void    resetPosition(int i) {
        if (i < 0 || i >= MSE_NUM_MOTORS) return;
        _state[i].currentPos = 0; _state[i].targetPos = 0;
    }
    void    emergencyStop() {
        for (int i = 0; i < MSE_NUM_MOTORS; i++) _state[i].targetPos = _state[i].currentPos;
    }

    bool     isInitialized() const { return _initialized; }
    bool     isRunning()     const { return _running; }
    uint32_t getTotalSteps(int i) const { return (i >= 0 && i < MSE_NUM_MOTORS) ? _state[i].totalSteps : 0; }
    uint32_t getDirChanges(int i) const { return (i >= 0 && i < MSE_NUM_MOTORS) ? _state[i].dirChanges : 0; }

private:
    McpwmStepEngine() : _initialized(false), _running(false), _timer(nullptr) {}

    MotorConfig       _cfg[MSE_NUM_MOTORS];
    MotorState        _state[MSE_NUM_MOTORS];
    bool              _initialized;
    bool              _running;
    mcpwm_timer_handle_t _timer;

    friend bool _mse_tez_isr(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void*);
};

// ── Inline GPIO helpers (IRAM, direct register) ───────────────────────
static inline void IRAM_ATTR _mse_gpio_set(int pin) {
    if (pin < 32) { GPIO.out_w1ts = (1u << pin); }
    else          { GPIO.out1_w1ts.val = (1u << (pin - 32)); }
}
static inline void IRAM_ATTR _mse_gpio_clr(int pin) {
    if (pin < 32) { GPIO.out_w1tc = (1u << pin); }
    else          { GPIO.out1_w1tc.val = (1u << (pin - 32)); }
}

// ── TEZ ISR — fires every 4µs ─────────────────────────────────────────
bool IRAM_ATTR _mse_tez_isr(mcpwm_timer_handle_t /*timer*/,
                              const mcpwm_timer_event_data_t* /*edata*/,
                              void* /*user_ctx*/) {
    McpwmStepEngine* self = _mse_instance;
    if (!self) return false;

    for (int i = 0; i < MSE_NUM_MOTORS; i++) {
        McpwmStepEngine::MotorState& s   = self->_state[i];
        const McpwmStepEngine::MotorConfig& c = self->_cfg[i];

        if (s.stepHigh) {
            _mse_gpio_clr(c.stepPin);
            s.stepHigh = false;
            continue;
        }

        int32_t delta = s.targetPos - s.currentPos;
        if (delta == 0) continue;

        bool forward = (delta > 0);
        bool physDir = c.invertDir ? !forward : forward;

        if (physDir != s.lastDir) {
            if (physDir) _mse_gpio_set(c.dirPin);
            else         _mse_gpio_clr(c.dirPin);
            s.lastDir    = physDir;
            s.dirPending = true;
            s.dirChanges++;
            continue;
        }

        if (s.dirPending) {
            s.dirPending = false;
        }

        _mse_gpio_set(c.stepPin);
        s.stepHigh   = true;
        s.currentPos += forward ? 1 : -1;
        s.totalSteps++;
    }

    return false;  // no high-priority task woken
}
