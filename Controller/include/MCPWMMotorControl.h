#pragma once

// All-MCPWM motor control for ESP32-S3
// ESP32-S3 has 2 MCPWM groups × 3 timers each = 6 hardware-timed channels.
// Each motor gets its own timer → operator → comparator → generator chain.
// One-shot mode: START_STOP_FULL fires one pulse then the timer stops.

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "debug_uart.h"

class MCPWMMotorControl {
public:
    struct Config {
        uint32_t stepPulseWidth_us;
        uint32_t dirSetupTime_us;
        uint32_t minStepInterval_us;
        uint32_t maxStepRate;
        int32_t  maxAcceleration;
        bool     invertDirection;
        bool     enableSoftLimits;
        int32_t  softLimitMin;
        int32_t  softLimitMax;

        Config() :
            stepPulseWidth_us(2),
            dirSetupTime_us(5),
            minStepInterval_us(3),
            maxStepRate(200000),
            maxAcceleration(50000),
            invertDirection(false),
            enableSoftLimits(true),
            softLimitMin(-1000000),
            softLimitMax(1000000)
        {}
    };

    enum Error {
        ERROR_NONE = 0,
        ERROR_SOFT_LIMIT_MIN,
        ERROR_SOFT_LIMIT_MAX,
        ERROR_STEP_RATE_EXCEEDED,
        ERROR_NOT_INITIALIZED,
        ERROR_INVALID_CONFIG,
        ERROR_GPIO_CONFIG,
        ERROR_MCPWM_TIMER,
        ERROR_MCPWM_OPER,
        ERROR_MCPWM_CMPR,
        ERROR_MCPWM_GEN,
        ERROR_NO_HW_CHANNEL
    };

    // Timing statistics for debug/test mode
    struct Stats {
        uint32_t totalSteps;
        uint32_t stepErrors;
        uint32_t minInterval_us;   // shortest observed step interval
        uint32_t maxInterval_us;   // longest observed step interval
        uint64_t sumInterval_us;   // for computing average
        uint32_t intervalSamples;

        void reset() {
            totalSteps = 0;
            stepErrors = 0;
            minInterval_us = UINT32_MAX;
            maxInterval_us = 0;
            sumInterval_us = 0;
            intervalSamples = 0;
        }
    };

    // Constructor — step/dir pins only; MCPWM resources auto-allocated
    MCPWMMotorControl(gpio_num_t stepPin, gpio_num_t dirPin) :
        _stepPin(stepPin),
        _dirPin(dirPin),
        _timer(nullptr),
        _oper(nullptr),
        _cmpr(nullptr),
        _gen(nullptr),
        _currentPos(0),
        _targetPos(0),
        _lastStepTime(0),
        _currentVelocity(0),
        _error(ERROR_NONE),
        _initialized(false),
        _lastDirection(false) {
        _stats.reset();
    }

    // Initialize motor using MCPWM one-shot pulse generation.
    // Tries group 0 first, then group 1.  Returns false if no resources left.
    bool begin(const Config& config = Config()) {
        _config = config;

        // Validate
        if (_config.stepPulseWidth_us < 1) {
            DEBUG_PRINTLN("Invalid step pulse width (must be >= 1us)");
            _error = ERROR_INVALID_CONFIG;
            return false;
        }
        if (_config.maxStepRate > 250000) {
            DEBUG_PRINTLN("Invalid max step rate (must be <= 250kHz)");
            _error = ERROR_INVALID_CONFIG;
            return false;
        }
        if (_config.minStepInterval_us < 2) {
            DEBUG_PRINTLN("Invalid min step interval (must be >= 2us)");
            _error = ERROR_INVALID_CONFIG;
            return false;
        }

        // Configure direction pin
        gpio_config_t dir_cfg = {
            .pin_bit_mask = (1ULL << _dirPin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        if (gpio_config(&dir_cfg) != ESP_OK) {
            DEBUG_PRINTF("Failed to configure direction pin %d\n", _dirPin);
            _error = ERROR_GPIO_CONFIG;
            return false;
        }
        gpio_set_level(_dirPin, 0);

        // Try both MCPWM groups (0 then 1) until we get resources
        bool allocated = false;
        for (int group = 0; group < 2 && !allocated; group++) {
            if (!_allocMcpwm(group)) continue;
            allocated = true;
        }

        if (!allocated) {
            DEBUG_PRINTF("ERROR: No MCPWM channel available for step=%d\n", _stepPin);
            _error = ERROR_NO_HW_CHANNEL;
            return false;
        }

        _initialized = true;
        DEBUG_PRINTF("Motor step=%d dir=%d: MCPWM group OK\n", _stepPin, _dirPin);
        return true;
    }

    Error   getLastError()      const { return _error; }
    int32_t getCurrentPosition() const { return _currentPos; }
    int32_t getTargetPosition()  const { return _targetPos; }
    float   getCurrentVelocity() const { return _currentVelocity; }
    gpio_num_t getStepPin()      const { return _stepPin; }
    gpio_num_t getDirPin()       const { return _dirPin; }
    bool    isInitialized()      const { return _initialized; }
    const Stats& getStats()      const { return _stats; }
    void    resetStats()               { _stats.reset(); }

    // Zero both current and target position counters.
    // Call after servo drives complete homing to synchronize ESP32 with physical zero.
    void resetPosition() {
        _currentPos = 0;
        _targetPos  = 0;
        _lastStepTime = 0;
        _currentVelocity = 0;
        _error = ERROR_NONE;
    }

    bool setTargetPosition(int32_t position) {
        if (!_initialized) { _error = ERROR_NOT_INITIALIZED; return false; }
        if (_config.enableSoftLimits) {
            if (position < _config.softLimitMin) { _error = ERROR_SOFT_LIMIT_MIN; return false; }
            if (position > _config.softLimitMax) { _error = ERROR_SOFT_LIMIT_MAX; return false; }
        }
        _targetPos = position;
        return true;
    }

    // Generate one step pulse via MCPWM one-shot (fire-and-forget, non-blocking)
    bool update() {
        if (!_initialized) { _error = ERROR_NOT_INITIALIZED; return false; }

        int32_t delta = _targetPos - _currentPos;
        if (delta == 0) return true;

        bool direction = delta > 0;
        uint64_t now = esp_timer_get_time();
        uint64_t elapsed = now - _lastStepTime;

        if (elapsed < _config.minStepInterval_us) return true; // rate-limit

        // Direction change with setup time
        if (direction != _lastDirection) {
            gpio_set_level(_dirPin, _config.invertDirection ? !direction : direction);
            _lastDirection = direction;
            if (elapsed < _config.dirSetupTime_us) {
                esp_rom_delay_us(_config.dirSetupTime_us - elapsed);
            }
        }

        // Fire one-shot MCPWM pulse.
        // Timer was left stopped at its period value (after init or previous pulse).
        // START_STOP_FULL: counter wraps 0 → TEZ → HIGH → compare → LOW → period → STOP.
        if (mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_STOP_FULL) == ESP_OK) {
            _currentPos += direction ? 1 : -1;
            _stats.totalSteps++;
            // Track step interval stats
            if (_lastStepTime > 0) {
                uint32_t interval = (uint32_t)elapsed;
                if (interval < _stats.minInterval_us) _stats.minInterval_us = interval;
                if (interval > _stats.maxInterval_us) _stats.maxInterval_us = interval;
                _stats.sumInterval_us += interval;
                _stats.intervalSamples++;
            }
            _lastStepTime = now;
            return true;
        }
        _stats.stepErrors++;
        return false;
    }

    void emergencyStop() {
        _targetPos = _currentPos;
        _currentVelocity = 0;
        // Force generator output LOW
        if (_gen) {
            mcpwm_generator_set_force_level(_gen, 0, true);
            // Remove force so normal operation resumes on next update()
            mcpwm_generator_set_force_level(_gen, -1, true);
        }
    }

private:
    // Allocate MCPWM timer + operator + comparator + generator in the given group.
    // Returns true on success; on failure cleans up and returns false.
    bool _allocMcpwm(int group) {
        esp_err_t ret;

        // --- Timer: 1 MHz resolution, period = 2 × pulse width ---
        mcpwm_timer_config_t t_cfg = {};
        t_cfg.group_id       = group;
        t_cfg.clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT;
        t_cfg.resolution_hz  = 1000000;  // 1 MHz → 1 µs ticks
        t_cfg.count_mode     = MCPWM_TIMER_COUNT_MODE_UP;
        t_cfg.period_ticks   = _config.stepPulseWidth_us * 2;
        t_cfg.intr_priority  = 0;

        ret = mcpwm_new_timer(&t_cfg, &_timer);
        if (ret != ESP_OK) { _timer = nullptr; return false; }

        // --- Operator ---
        mcpwm_operator_config_t o_cfg = {};
        o_cfg.group_id      = group;
        o_cfg.intr_priority = 0;
        o_cfg.flags.update_gen_action_on_tez = true;

        ret = mcpwm_new_operator(&o_cfg, &_oper);
        if (ret != ESP_OK) {
            mcpwm_del_timer(_timer); _timer = nullptr;
            _oper = nullptr; return false;
        }

        ret = mcpwm_operator_connect_timer(_oper, _timer);
        if (ret != ESP_OK) {
            mcpwm_del_operator(_oper);  _oper  = nullptr;
            mcpwm_del_timer(_timer);    _timer = nullptr;
            return false;
        }

        // --- Comparator: fires at pulse_width ticks ---
        mcpwm_comparator_config_t c_cfg = {};
        c_cfg.intr_priority = 0;
        c_cfg.flags.update_cmp_on_tez = true;

        ret = mcpwm_new_comparator(_oper, &c_cfg, &_cmpr);
        if (ret != ESP_OK) {
            mcpwm_del_operator(_oper);  _oper  = nullptr;
            mcpwm_del_timer(_timer);    _timer = nullptr;
            _cmpr = nullptr; return false;
        }
        mcpwm_comparator_set_compare_value(_cmpr, _config.stepPulseWidth_us);

        // --- Generator: drives the step pin ---
        mcpwm_generator_config_t g_cfg = {};
        g_cfg.gen_gpio_num = _stepPin;

        ret = mcpwm_new_generator(_oper, &g_cfg, &_gen);
        if (ret != ESP_OK) {
            mcpwm_del_comparator(_cmpr); _cmpr = nullptr;
            mcpwm_del_operator(_oper);   _oper = nullptr;
            mcpwm_del_timer(_timer);     _timer = nullptr;
            _gen = nullptr; return false;
        }

        // Actions: HIGH when counter reaches zero, LOW when counter reaches compare
        mcpwm_gen_timer_event_action_t tez_act = {};
        tez_act.direction = MCPWM_TIMER_DIRECTION_UP;
        tez_act.event     = MCPWM_TIMER_EVENT_EMPTY;
        tez_act.action    = MCPWM_GEN_ACTION_HIGH;
        mcpwm_generator_set_action_on_timer_event(_gen, tez_act);

        mcpwm_gen_compare_event_action_t cmp_act = {};
        cmp_act.direction  = MCPWM_TIMER_DIRECTION_UP;
        cmp_act.comparator = _cmpr;
        cmp_act.action     = MCPWM_GEN_ACTION_LOW;
        mcpwm_generator_set_action_on_compare_event(_gen, cmp_act);

        // Enable timer
        ret = mcpwm_timer_enable(_timer);
        if (ret != ESP_OK) {
            _cleanupMcpwm();
            return false;
        }

        // Dummy one-shot to advance counter from 0 to period.
        // Without this the first real pulse would not trigger TEZ (counter is
        // already at 0, no wrap occurs).  The dummy cycle outputs nothing
        // visible because the generator is already LOW and TEZ doesn't fire
        // until the counter wraps from period→0.
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_STOP_FULL);
        esp_rom_delay_us(_config.stepPulseWidth_us * 2 + 2); // wait for dummy

        return true;
    }

    void _cleanupMcpwm() {
        if (_gen)   { mcpwm_del_generator(_gen);   _gen   = nullptr; }
        if (_cmpr)  { mcpwm_del_comparator(_cmpr); _cmpr  = nullptr; }
        if (_oper)  { mcpwm_del_operator(_oper);   _oper  = nullptr; }
        if (_timer) { mcpwm_del_timer(_timer);     _timer = nullptr; }
    }

    gpio_num_t _stepPin;
    gpio_num_t _dirPin;

    mcpwm_timer_handle_t _timer;
    mcpwm_oper_handle_t  _oper;
    mcpwm_cmpr_handle_t  _cmpr;
    mcpwm_gen_handle_t   _gen;

    Config _config;
    volatile int32_t  _currentPos;
    volatile int32_t  _targetPos;
    volatile uint64_t _lastStepTime;
    volatile float    _currentVelocity;
    Error  _error;
    bool   _initialized;
    bool   _lastDirection;
    Stats  _stats;
};
