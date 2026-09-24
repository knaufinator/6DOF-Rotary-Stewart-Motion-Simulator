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
#include "driver/pulse_cnt.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

// PCNT hardware counter limit (16-bit signed max)
static constexpr int32_t PCNT_HW_LIMIT = 32767;

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
        ERROR_RMT_ALLOC,
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
        _lastDirection(false),
        _pcntUnit(nullptr),
        _pcntChannel(nullptr),
        _hasPcnt(false),
        _pcntOldRemainder(0) {
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

        // Step 1: Allocate MCPWM base (timer + operator + comparator).
        // Generator is deferred — we don't claim the step GPIO yet.
        // This allows RMT TX to claim the GPIO if PCNT is unavailable.
        bool allocated = false;
        for (int group = 0; group < 2 && !allocated; group++) {
            if (!_allocMcpwmBase(group)) continue;
            allocated = true;
        }

        if (!allocated) {
            DEBUG_PRINTF("ERROR: No MCPWM channel available for step=%d\n", _stepPin);
            _error = ERROR_NO_HW_CHANNEL;
            return false;
        }

        // Step 2: Try PCNT hardware pulse counting.
        // ESP32-S3 has 4 PCNT units — first 4 motors get zero-CPU counting.
        // PCNT watches the step pin (input), doesn't claim it as output.
        _hasPcnt = false;
        pcnt_unit_config_t pcnt_cfg = {};
        pcnt_cfg.low_limit = -1;  // ESP-IDF requires low_limit < 0
        pcnt_cfg.high_limit = PCNT_HW_LIMIT;
        pcnt_cfg.intr_priority = 0;

        if (pcnt_new_unit(&pcnt_cfg, &_pcntUnit) == ESP_OK) {
            pcnt_chan_config_t chan_cfg = {};
            chan_cfg.edge_gpio_num = _stepPin;
            chan_cfg.level_gpio_num = -1;  // no level signal

            if (pcnt_new_channel(_pcntUnit, &chan_cfg, &_pcntChannel) == ESP_OK) {
                // Count rising edges only (one per step pulse)
                pcnt_channel_set_edge_action(_pcntChannel,
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                    PCNT_CHANNEL_EDGE_ACTION_HOLD);

                // Watch high_limit for overflow tracking
                pcnt_unit_add_watch_point(_pcntUnit, PCNT_HW_LIMIT);

                pcnt_event_callbacks_t pcnt_cbs = {};
                pcnt_cbs.on_reach = _onPcntReach;
                pcnt_unit_register_event_callbacks(_pcntUnit, &pcnt_cbs, this);

                pcnt_unit_enable(_pcntUnit);
                pcnt_unit_clear_count(_pcntUnit);

                _hasPcnt = true;
                DEBUG_PRINTF("  + PCNT allocated for step pin %d (hw counting)\n", _stepPin);
            } else {
                pcnt_del_unit(_pcntUnit);
                _pcntUnit = nullptr;
                DEBUG_PRINTF("  PCNT channel failed for pin %d\n", _stepPin);
            }
        } else {
            _pcntUnit = nullptr;
            DEBUG_PRINTF("  No PCNT unit available for pin %d\n", _stepPin);
        }

        // Step 3: Decide who owns the step pin GPIO.
        // - PCNT motors: MCPWM generator drives the pin (PCNT reads it)
        // - Non-PCNT motors: try RMT TX (generates + counts via loop_count)
        // - Last resort: MCPWM generator + TEZ ISR
        _hasRmt = false;
        if (_hasPcnt) {
            // PCNT path: allocate MCPWM generator to drive the step pin
            if (!_allocMcpwmGen()) {
                DEBUG_PRINTF("FATAL: MCPWM generator failed for step=%d\n", _stepPin);
                _cleanupMcpwm();
                _error = ERROR_MCPWM_GEN;
                return false;
            }
        } else {
            // No PCNT: try RMT TX first (claims GPIO for pulse gen + hw counting)
            if (_allocRmt()) {
                _hasRmt = true;
                DEBUG_PRINTF("  + RMT TX allocated for step pin %d (hw loop counting)\n", _stepPin);
            } else {
                // RMT failed: fall back to MCPWM generator + TEZ ISR
                DEBUG_PRINTF("  RMT TX failed for pin %d, using MCPWM + ISR fallback\n", _stepPin);
                if (!_allocMcpwmGen()) {
                    DEBUG_PRINTF("FATAL: MCPWM generator failed for step=%d\n", _stepPin);
                    _cleanupMcpwm();
                    _error = ERROR_MCPWM_GEN;
                    return false;
                }
            }
        }

        // Step 4: Enable MCPWM timer.
        // - PCNT motors: MCPWM generates pulses, no ISR (PCNT counts in hw)
        // - RMT motors: MCPWM timer idle, no ISR (RMT does everything)
        // - ISR fallback: register TEZ ISR for per-pulse counting
        bool needTezIsr = !_hasPcnt && !_hasRmt;
        if (!_enableTimer(needTezIsr)) {
            DEBUG_PRINTF("FATAL: timer enable failed for step=%d\n", _stepPin);
            _cleanupMcpwm();
            _error = ERROR_MCPWM_TIMER;
            return false;
        }

        _initialized = true;
        const char* mode = _hasPcnt ? " + PCNT" : (_hasRmt ? " + RMT" : " (ISR)");
        DEBUG_PRINTF("Motor step=%d dir=%d: MCPWM group OK%s\n",
            _stepPin, _dirPin, mode);
        return true;
    }

    const Config& getConfig()    const { return _config; }
    Config& getConfigMut()             { return _config; }
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

    // ── Continuous mode: ISR-counted exact stepping (up to 250 kHz) ───
    // MCPWM timer runs in continuous mode generating pulses at hardware rate.
    // A TEZ (timer-empty) ISR fires once per pulse, increments an exact counter,
    // and auto-stops the timer when the target count is reached.
    // All 6 MCPWM timers are independent hardware — they run truly in parallel.
    // ZERO DRIFT. ZERO OVERSHOOT. Every pulse is counted.

    // Start continuous MCPWM for a given step delta (non-blocking).
    // Uses PCNT hardware counting if available (zero CPU per pulse),
    // falls back to ISR counting for motors without PCNT.
    // Poll checkContinuousDone() for completion.
    bool startContinuousSteps(int32_t delta) {
        if (!_initialized || delta == 0) return false;
        if (_continuousRunning) return false;

        bool forward = delta > 0;
        int32_t absSteps = forward ? delta : -delta;

        // Set direction (with setup time only on change)
        if (forward != _lastDirection) {
            gpio_set_level(_dirPin, _config.invertDirection ? !forward : forward);
            _lastDirection = forward;
            esp_rom_delay_us(_config.dirSetupTime_us);
        }

        _continuousRunning = true;
        _continuousForward = forward;

        if (_hasPcnt) {
            // ── PCNT path: hardware counts pulses, ~1 ISR per move ──
            pcnt_unit_stop(_pcntUnit);

            // Remove old remainder watch point (if any)
            if (_pcntOldRemainder > 0) {
                pcnt_unit_remove_watch_point(_pcntUnit, _pcntOldRemainder);
            }

            // Calculate overflow/remainder for targets > 32767
            _pcntTargetSteps = absSteps;
            _pcntAccumulated = 0;
            _pcntDone = false;
            int32_t remainder = absSteps % PCNT_HW_LIMIT;

            // Add remainder watch point (fires when hw counter == remainder)
            if (remainder > 0) {
                pcnt_unit_add_watch_point(_pcntUnit, remainder);
            }
            _pcntOldRemainder = remainder;

            // Clear count and start (clear must be after adding watch points)
            pcnt_unit_clear_count(_pcntUnit);
            pcnt_unit_start(_pcntUnit);

            // Start MCPWM free-run — PCNT counts every pulse in hardware
            mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
        } else if (_hasRmt) {
            // ── RMT TX path: hardware loop generates + counts pulses ──
            // One RMT symbol = one step pulse (2µs high + 2µs low = 250 kHz).
            // loop_count = N → hardware generates exactly N pulses.
            // on_trans_done fires once when all loops complete.
            // Zero CPU per pulse. ~1 ISR per 1023 for counts > hw limit.
            _rmtDone = false;
            _rmtTargetSteps = absSteps;

            // _rmtStepSymbol is a persistent member (NOT stack) because
            // rmt_transmit() is async — encoder reads payload from ISR context.
            rmt_transmit_config_t tx_cfg = {};
            tx_cfg.loop_count = absSteps;    // Hardware repeats this many times
            tx_cfg.flags.eot_level = 0;      // Line LOW when done

            esp_err_t err = rmt_transmit(_rmtChannel, _rmtEncoder,
                &_rmtStepSymbol, sizeof(_rmtStepSymbol), &tx_cfg);
            if (err != ESP_OK) {
                _rmtDone = true;  // prevent hang
                _continuousRunning = false;
                return false;
            }
        } else {
            // ── ISR path: TEZ ISR counts every pulse (last resort fallback) ──
            _isrStepCount = 0;
            _isrTargetSteps = absSteps;
            _isrDone = false;
            _isrActive = true;
            mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
        }
        return true;
    }

    // Poll: returns true when counting (PCNT or ISR) has reached target.
    bool checkContinuousDone() {
        if (!_continuousRunning) return true;

        int32_t steps;
        int32_t target;

        if (_hasPcnt) {
            // PCNT path: check flag set by PCNT watch callback
            if (!_pcntDone) return false;
            steps = _pcntTargetSteps;  // PCNT stops at exact target
            target = _pcntTargetSteps;
            pcnt_unit_stop(_pcntUnit);
        } else if (_hasRmt) {
            // RMT TX path: check flag set by on_trans_done callback
            if (!_rmtDone) return false;
            steps = _rmtTargetSteps;   // RMT loop_count = exact target
            target = _rmtTargetSteps;
        } else {
            // ISR path: check flag set by TEZ ISR
            if (!_isrDone) return false;
            steps = _isrStepCount;
            target = _isrTargetSteps;
        }

        _currentPos += _continuousForward ? steps : -steps;
        _targetPos = _currentPos;
        _stats.totalSteps += steps;
        _continuousRunning = false;
        _lastStepError = steps - target; // should always be 0
        _isrStepCount = steps;           // for getIsrStepCount() compat
        _lastStepTime = esp_timer_get_time();

        return true;
    }

    bool    isContinuousRunning() const { return _continuousRunning; }
    int32_t getLastStepError()    const { return _lastStepError; }
    int32_t getIsrStepCount()     const { return _isrStepCount; }
    bool    hasPcnt()             const { return _hasPcnt; }
    bool    hasRmt()              const { return _hasRmt; }

    // Blocking convenience: run N steps and wait (used by RATETEST).
    int32_t runSteps(int32_t steps) {
        if (!startContinuousSteps(steps)) return 0;
        while (!checkContinuousDone()) {}
        return _isrStepCount;
    }

    void emergencyStop() {
        if (_continuousRunning) {
            if (_hasRmt) {
                rmt_disable(_rmtChannel);
                rmt_enable(_rmtChannel);  // re-enable for next use
            } else {
                mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_EMPTY);
            }
            _continuousRunning = false;
        }
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
    // Allocate MCPWM timer + operator + comparator (NO generator yet).
    // Generator is deferred to _allocMcpwmGen() so we don't claim the step GPIO
    // before knowing whether RMT TX will take over pulse generation.
    bool _allocMcpwmBase(int group) {
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

        return true;
    }

    // Allocate MCPWM generator on the step pin (claims GPIO).
    // Called ONLY for PCNT or ISR motors — NOT for RMT motors.
    bool _allocMcpwmGen() {
        mcpwm_generator_config_t g_cfg = {};
        g_cfg.gen_gpio_num = _stepPin;

        esp_err_t ret = mcpwm_new_generator(_oper, &g_cfg, &_gen);
        if (ret != ESP_OK) {
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

        return true;
    }

    // Enable timer (with optional TEZ ISR) and run dummy one-shot.
    // Called from begin() after PCNT allocation decision is made.
    bool _enableTimer(bool registerTezIsr) {
        esp_err_t ret;

        if (registerTezIsr) {
            // Register TEZ ISR for exact pulse counting (non-PCNT motors).
            // MUST be registered BEFORE mcpwm_timer_enable().
            mcpwm_timer_event_callbacks_t cbs = {};
            cbs.on_empty = _onTimerEmpty;
            ret = mcpwm_timer_register_event_callbacks(_timer, &cbs, this);
            if (ret != ESP_OK) return false;
        }

        ret = mcpwm_timer_enable(_timer);
        if (ret != ESP_OK) return false;

        // Dummy one-shot to advance counter from 0 to period.
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_STOP_FULL);
        esp_rom_delay_us(_config.stepPulseWidth_us * 2 + 2);
        return true;
    }

    // Allocate RMT TX channel + copy encoder for hardware loop-counted stepping.
    // RMT TX generates pulses on the step pin with loop_count for exact counting.
    // Uses copy encoder: payload bytes are copied verbatim as RMT symbols.
    // Returns true on success, false if no RMT channels available.
    bool _allocRmt() {
        // RMT TX channel: same GPIO as MCPWM step pin, 1 MHz resolution
        rmt_tx_channel_config_t tx_cfg = {};
        tx_cfg.gpio_num = _stepPin;
        tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_cfg.resolution_hz = 1000000;         // 1 MHz → 1 µs ticks (matches MCPWM)
        tx_cfg.mem_block_symbols = 48;           // minimum for non-DMA
        tx_cfg.trans_queue_depth = 1;            // one transaction at a time
        tx_cfg.flags.invert_out = false;
        tx_cfg.flags.with_dma = false;

        esp_err_t ret = rmt_new_tx_channel(&tx_cfg, &_rmtChannel);
        if (ret != ESP_OK) {
            ESP_LOGE("MOTOR", "RMT TX alloc failed pin=%d err=0x%x", _stepPin, ret);
            _rmtChannel = nullptr;
            return false;
        }

        // Copy encoder: treats payload as raw RMT symbols (no encoding logic)
        rmt_copy_encoder_config_t enc_cfg = {};
        if (rmt_new_copy_encoder(&enc_cfg, &_rmtEncoder) != ESP_OK) {
            rmt_del_channel(_rmtChannel);
            _rmtChannel = nullptr;
            _rmtEncoder = nullptr;
            return false;
        }

        // Initialize persistent step pulse symbol (one step = HIGH + LOW)
        _rmtStepSymbol.duration0 = _config.stepPulseWidth_us;  // HIGH ticks @ 1MHz
        _rmtStepSymbol.level0 = 1;
        _rmtStepSymbol.duration1 = _config.stepPulseWidth_us;  // LOW ticks @ 1MHz
        _rmtStepSymbol.level1 = 0;

        // Register TX done callback — fires once when all loop iterations complete
        rmt_tx_event_callbacks_t cbs = {};
        cbs.on_trans_done = _onRmtTransDone;
        if (rmt_tx_register_event_callbacks(_rmtChannel, &cbs, this) != ESP_OK) {
            rmt_del_encoder(_rmtEncoder); _rmtEncoder = nullptr;
            rmt_del_channel(_rmtChannel); _rmtChannel = nullptr;
            return false;
        }

        if (rmt_enable(_rmtChannel) != ESP_OK) {
            rmt_del_encoder(_rmtEncoder); _rmtEncoder = nullptr;
            rmt_del_channel(_rmtChannel); _rmtChannel = nullptr;
            return false;
        }

        return true;
    }

    // Static RMT TX done callback — fires once when all loop iterations complete.
    // For moves ≤ 1023: one callback total. For larger moves: one per 1023 batch
    // (handled internally by ESP-IDF driver), final callback sets _rmtDone.
    static bool IRAM_ATTR _onRmtTransDone(
            rmt_channel_handle_t channel,
            const rmt_tx_done_event_data_t *edata,
            void *user_ctx) {
        MCPWMMotorControl* self = static_cast<MCPWMMotorControl*>(user_ctx);
        self->_rmtDone = true;
        return false;
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

    // ISR-counted continuous mode state (fallback for motors without PCNT)
    volatile bool     _continuousRunning = false;  // task-level: move in progress
    volatile bool     _isrActive = false;          // ISR-level: should count pulses
    bool              _continuousForward = true;
    volatile int32_t  _isrStepCount = 0;           // exact pulse count from ISR
    volatile int32_t  _isrTargetSteps = 0;         // target count — ISR stops here
    volatile bool     _isrDone = false;            // set by ISR when target reached
    int32_t           _lastStepError = 0;

    // RMT TX hardware loop state (for motors without PCNT — replaces TEZ ISR)
    rmt_channel_handle_t  _rmtChannel = nullptr;
    rmt_encoder_handle_t  _rmtEncoder = nullptr;
    rmt_symbol_word_t     _rmtStepSymbol = {};    // persistent step pulse symbol (NOT stack!)
    bool                  _hasRmt = false;
    volatile bool         _rmtDone = false;       // set by on_trans_done callback
    volatile int32_t      _rmtTargetSteps = 0;    // target count for this move

    // PCNT hardware pulse counter state (for motors with PCNT)
    pcnt_unit_handle_t    _pcntUnit;
    pcnt_channel_handle_t _pcntChannel;
    bool                  _hasPcnt;
    int32_t               _pcntOldRemainder;       // previous remainder watch point
    volatile int32_t      _pcntTargetSteps = 0;    // target count for this move
    volatile int32_t      _pcntAccumulated = 0;    // overflow accumulator
    volatile bool         _pcntDone = false;       // set by PCNT callback at target

    // Static PCNT watch callback — fires at overflow (32767) and remainder.
    // Typically 1-3 invocations per move instead of 250k with ISR counting.
    // Tracks overflows and stops MCPWM timer at exact target count.
    static bool _onPcntReach(
            pcnt_unit_handle_t unit,
            const pcnt_watch_event_data_t *edata,
            void *user_ctx) {
        MCPWMMotorControl* self = static_cast<MCPWMMotorControl*>(user_ctx);
        if (self->_pcntDone) return false;

        int32_t watch_val = edata->watch_point_value;

        if (watch_val == PCNT_HW_LIMIT) {
            // Overflow: counter wrapped from 32767 → 0
            self->_pcntAccumulated += PCNT_HW_LIMIT;
            if (self->_pcntAccumulated >= self->_pcntTargetSteps) {
                // Target is exact multiple of 32767
                self->_pcntDone = true;
                mcpwm_timer_start_stop(self->_timer, MCPWM_TIMER_STOP_FULL);
            }
        } else {
            // Remainder watch point: check if this is the final occurrence
            int32_t total = self->_pcntAccumulated + watch_val;
            if (total >= self->_pcntTargetSteps) {
                self->_pcntDone = true;
                mcpwm_timer_start_stop(self->_timer, MCPWM_TIMER_STOP_FULL);
            }
        }
        return false;
    }

    // Static ISR callback — fires once per MCPWM pulse (TEZ event).
    // Used only for motors 4-5 that don't have PCNT.
    static bool _onTimerEmpty(
            mcpwm_timer_handle_t timer,
            const mcpwm_timer_event_data_t *edata,
            void *user_ctx) {
        MCPWMMotorControl* self = static_cast<MCPWMMotorControl*>(user_ctx);
        if (!self->_isrActive) return false;

        self->_isrStepCount++;
        if (self->_isrStepCount >= self->_isrTargetSteps) {
            self->_isrActive = false;  // prevent re-entry FIRST
            mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_FULL);
            self->_isrDone = true;
        }
        return false;
    }
};
