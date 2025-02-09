#pragma once

#include <driver/rmt.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

class RMTMotorControl {
public:
    struct Config {
        uint32_t stepPulseWidth_us;     // Step pulse width in microseconds
        uint32_t dirSetupTime_us;       // Setup time before step after direction change
        uint32_t minStepInterval_us;    // Minimum time between steps
        uint32_t maxStepRate;      // Maximum steps per second (400kHz default)
        int32_t maxAcceleration;    // Maximum acceleration in steps/sec^2
        bool invertDirection;        // Invert direction signal
        bool enableSoftLimits;       // Enable software position limits
        int32_t softLimitMin;    // Minimum position in steps
        int32_t softLimitMax;     // Maximum position in steps

        Config() :
            stepPulseWidth_us(1),
            dirSetupTime_us(1),
            minStepInterval_us(2),
            maxStepRate(400000),
            maxAcceleration(50000),
            invertDirection(false),
            enableSoftLimits(true),
            softLimitMin(-1000000),
            softLimitMax(1000000)
        {}
    };

    RMTMotorControl(gpio_num_t stepPin, gpio_num_t stepPinComplement, gpio_num_t dirPin, gpio_num_t dirPinComplement, rmt_channel_t channel) 
        : _stepPin(stepPin), _stepPinComplement(stepPinComplement), _dirPin(dirPin), _dirPinComplement(dirPinComplement), _channel(channel) {
        _currentPos = 0;
        _targetPos = 0;
        _lastStepTime = 0;
        _currentVelocity = 0;
        _error = ERROR_NONE;
    }

    enum Error {
        ERROR_NONE = 0,
        ERROR_SOFT_LIMIT_MIN,
        ERROR_SOFT_LIMIT_MAX,
        ERROR_STEP_RATE_EXCEEDED,
        ERROR_NOT_INITIALIZED,
        ERROR_INVALID_CONFIG
    };

    bool begin(const Config& config = Config()) {
        _config = config;
        
        // Validate configuration
        if (_config.stepPulseWidth_us < 1 || _config.stepPulseWidth_us > 100 ||
            _config.maxStepRate > 500000 || _config.maxStepRate < 1000) {
            _error = ERROR_INVALID_CONFIG;
            return false;
        }

        // Configure direction pin as normal GPIO
        gpio_config_t dir_pin_config;
        dir_pin_config.pin_bit_mask = (1ULL << _dirPin) | (1ULL << _dirPinComplement);
        dir_pin_config.mode = GPIO_MODE_OUTPUT;
        dir_pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
        dir_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        dir_pin_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&dir_pin_config);

        // Configure step pin as normal GPIO
        gpio_config_t step_pin_config;
        step_pin_config.pin_bit_mask = (1ULL << _stepPin) | (1ULL << _stepPinComplement);
        step_pin_config.mode = GPIO_MODE_OUTPUT;
        step_pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
        step_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        step_pin_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&step_pin_config);

        // Calculate optimal RMT clock divider
        uint8_t clk_div = 80; // 80MHz / 80 = 1MHz resolution
        
        // Configure RMT
        rmt_config_t rmt_cfg;
        rmt_cfg.rmt_mode = RMT_MODE_TX;
        rmt_cfg.channel = _channel;
        rmt_cfg.gpio_num = _stepPin;
        rmt_cfg.clk_div = clk_div;
        rmt_cfg.mem_block_num = 1;
        rmt_cfg.tx_config.loop_en = false;
        rmt_cfg.tx_config.carrier_en = false;
        rmt_cfg.tx_config.idle_output_en = true;
        rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        
        if (rmt_config(&rmt_cfg) != ESP_OK) {
            _error = ERROR_NOT_INITIALIZED;
            return false;
        }
        
        if (rmt_driver_install(_channel, 0, 0) != ESP_OK) {
            _error = ERROR_NOT_INITIALIZED;
            return false;
        }
        
        // Prepare step pulse item
        _stepItem[0].duration0 = _config.stepPulseWidth_us;
        _stepItem[0].level0 = 1;
        _stepItem[0].duration1 = _config.stepPulseWidth_us;
        _stepItem[0].level1 = 0;

        _initialized = true;
        return true;
    }

    void setConfig(const Config& config) {
        if (!_initialized) return;
        
        _config = config;
        
        // Update RMT timing
        _stepItem[0].duration0 = _config.stepPulseWidth_us;
        _stepItem[0].duration1 = _config.stepPulseWidth_us;
    }

    bool setTargetPosition(long target) {
        if (!_initialized) {
            _error = ERROR_NOT_INITIALIZED;
            return false;
        }

        // Check software limits
        if (_config.enableSoftLimits) {
            if (target < _config.softLimitMin) {
                _error = ERROR_SOFT_LIMIT_MIN;
                return false;
            }
            if (target > _config.softLimitMax) {
                _error = ERROR_SOFT_LIMIT_MAX;
                return false;
            }
        }

        _targetPos = target;
        return true;
    }

    bool update() {
        if (!_initialized) {
            _error = ERROR_NOT_INITIALIZED;
            return false;
        }

        if (_currentPos == _targetPos) {
            _currentVelocity = 0;
            return true;
        }

        int64_t now = esp_timer_get_time();
        int64_t timeSinceLastStep = now - _lastStepTime;

        // Check if we're trying to step too fast
        if (timeSinceLastStep < _config.minStepInterval_us) {
            return true; // Not an error, just waiting
        }

        // Calculate direction and update velocity
        bool direction = _currentPos < _targetPos;
        int32_t distanceToGo = abs(_targetPos - _currentPos);
        
        // Calculate maximum allowed velocity based on acceleration limit
        int32_t maxVel = sqrt(2.0 * _config.maxAcceleration * distanceToGo);
        if (maxVel > _config.maxStepRate) maxVel = _config.maxStepRate;
        
        // Update current velocity with acceleration limit
        if (_currentVelocity < maxVel) {
            _currentVelocity += (_config.maxAcceleration * timeSinceLastStep / 1000000.0);
            if (_currentVelocity > maxVel) _currentVelocity = maxVel;
        } else if (_currentVelocity > maxVel) {
            _currentVelocity -= (_config.maxAcceleration * timeSinceLastStep / 1000000.0);
            if (_currentVelocity < maxVel) _currentVelocity = maxVel;
        }

        // Set direction pin
        gpio_set_level(_dirPin, _config.invertDirection ? !direction : direction);
        gpio_set_level(_dirPinComplement, _config.invertDirection ? direction : !direction);
        
        // Wait for direction setup time
        if (_lastDirection != direction) {
            ets_delay_us(_config.dirSetupTime_us);
            _lastDirection = direction;
        }
        
        // Send step pulse
        if (rmt_write_items(_channel, _stepItem, 1, false) != ESP_OK) {
            return false;
        }
        
        // Update position
        if (direction) _currentPos++;
        else _currentPos--;
        
        _lastStepTime = now;
        _error = ERROR_NONE;
        return true;
    }

    Error getLastError() const { return _error; }
    long getCurrentPosition() const { return _currentPos; }
    long getTargetPosition() const { return _targetPos; }
    float getCurrentVelocity() const { return _currentVelocity; }
    bool isAtTarget() const { return _currentPos == _targetPos; }

private:
    gpio_num_t _stepPin;
    gpio_num_t _stepPinComplement;
    gpio_num_t _dirPin;
    gpio_num_t _dirPinComplement;
    rmt_channel_t _channel;
    volatile long _currentPos;
    volatile long _targetPos;
    rmt_item32_t _stepItem[1];
    Config _config;
    Error _error;
    bool _initialized = false;
    bool _lastDirection = false;
    int64_t _lastStepTime = 0;
    float _currentVelocity = 0;
};
