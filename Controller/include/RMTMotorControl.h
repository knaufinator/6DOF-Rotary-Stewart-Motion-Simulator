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
        uint32_t maxStepRate;           // Maximum steps per second
        int32_t maxAcceleration;        // Maximum acceleration in steps/sec^2
        bool invertDirection;           // Invert direction signal
        bool enableSoftLimits;         // Enable software position limits
        int32_t softLimitMin;          // Minimum position in steps
        int32_t softLimitMax;          // Maximum position in steps

        Config() :
            stepPulseWidth_us(2),       // Increased to 2µs for better reliability
            dirSetupTime_us(5),         // Increased to 5µs for better reliability
            minStepInterval_us(3),      // Increased to 3µs minimum interval
            maxStepRate(200000),        // Reduced to 200kHz max for better reliability
            maxAcceleration(50000),
            invertDirection(false),
            enableSoftLimits(true),
            softLimitMin(-1000000),
            softLimitMax(1000000)
        {}
    };

    RMTMotorControl(gpio_num_t stepPin, gpio_num_t dirPin, rmt_channel_t channel) 
        : _stepPin(stepPin), _dirPin(dirPin), _channel(channel) {
        _currentPos = 0;
        _targetPos = 0;
        _lastStepTime = 0;
        _currentVelocity = 0;
        _error = ERROR_NONE;
        _initialized = false;
        _lastDirection = false;
    }

    enum Error {
        ERROR_NONE = 0,
        ERROR_SOFT_LIMIT_MIN,
        ERROR_SOFT_LIMIT_MAX,
        ERROR_STEP_RATE_EXCEEDED,
        ERROR_NOT_INITIALIZED,
        ERROR_INVALID_CONFIG,
        ERROR_GPIO_CONFIG,
        ERROR_RMT_CONFIG,
        ERROR_RMT_INSTALL
    };

    bool begin(const Config& config = Config()) {
        _config = config;
        
        // Validate configuration with detailed error reporting
        if (_config.stepPulseWidth_us < 1) {
            Serial.println("Invalid step pulse width (must be >= 1µs)");
            _error = ERROR_INVALID_CONFIG;
            return false;
        }
        if (_config.maxStepRate > 250000) {  // Reduced max rate
            Serial.println("Invalid max step rate (must be <= 250kHz)");
            _error = ERROR_INVALID_CONFIG;
            return false;
        }
        if (_config.minStepInterval_us < 2) {
            Serial.println("Invalid min step interval (must be >= 2µs)");
            _error = ERROR_INVALID_CONFIG;
            return false;
        }

        // Configure direction pin
        gpio_config_t dir_pin_config = {
            .pin_bit_mask = (1ULL << _dirPin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        if (gpio_config(&dir_pin_config) != ESP_OK) {
            Serial.printf("Failed to configure direction pin %d\n", _dirPin);
            _error = ERROR_GPIO_CONFIG;
            return false;
        }

        // Configure step pin
        gpio_config_t step_pin_config = {
            .pin_bit_mask = (1ULL << _stepPin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        if (gpio_config(&step_pin_config) != ESP_OK) {
            Serial.printf("Failed to configure step pin %d\n", _stepPin);
            _error = ERROR_GPIO_CONFIG;
            return false;
        }

        // Set initial pin states
        gpio_set_level(_dirPin, 0);
        gpio_set_level(_stepPin, 0);

        // Configure RMT
        rmt_config_t rmt_cfg;
        rmt_cfg.rmt_mode = RMT_MODE_TX;
        rmt_cfg.channel = _channel;
        rmt_cfg.gpio_num = _stepPin;
        rmt_cfg.clk_div = 80;  // 80MHz / 80 = 1MHz resolution
        rmt_cfg.mem_block_num = 1;
        rmt_cfg.tx_config.loop_en = false;
        rmt_cfg.tx_config.carrier_en = false;
        rmt_cfg.tx_config.idle_output_en = true;
        rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        
        if (rmt_config(&rmt_cfg) != ESP_OK) {
            Serial.printf("Failed to configure RMT for channel %d\n", _channel);
            _error = ERROR_RMT_CONFIG;
            return false;
        }
        
        if (rmt_driver_install(_channel, 0, 0) != ESP_OK) {
            Serial.printf("Failed to install RMT driver for channel %d\n", _channel);
            _error = ERROR_RMT_INSTALL;
            return false;
        }

        Serial.printf("Successfully initialized motor on step pin %d, dir pin %d, channel %d\n", 
                     _stepPin, _dirPin, _channel);
        _initialized = true;
        return true;
    }

    Error getLastError() const { return _error; }
    int32_t getCurrentPosition() const { return _currentPos; }
    int32_t getTargetPosition() const { return _targetPos; }
    float getCurrentVelocity() const { return _currentVelocity; }

    bool setTargetPosition(int32_t position) {
        if (!_initialized) {
            _error = ERROR_NOT_INITIALIZED;
            return false;
        }

        if (_config.enableSoftLimits) {
            if (position < _config.softLimitMin) {
                _error = ERROR_SOFT_LIMIT_MIN;
                return false;
            }
            if (position > _config.softLimitMax) {
                _error = ERROR_SOFT_LIMIT_MAX;
                return false;
            }
        }

        _targetPos = position;
        return true;
    }

    bool update() {
        if (!_initialized) {
            _error = ERROR_NOT_INITIALIZED;
            return false;
        }

        int32_t delta = _targetPos - _currentPos;
        if (delta == 0) return true;

        bool direction = delta > 0;
        uint64_t now = esp_timer_get_time();
        uint64_t timeSinceLastStep = now - _lastStepTime;

        // Check if we're trying to step too fast
        if (timeSinceLastStep < _config.minStepInterval_us) {
            return true; // Not an error, just waiting
        }

        // Set direction and wait for setup time if direction changed
        if (direction != _lastDirection) {
            gpio_set_level(_dirPin, _config.invertDirection ? !direction : direction);
            _lastDirection = direction;
            if (timeSinceLastStep < _config.dirSetupTime_us) {
                ets_delay_us(_config.dirSetupTime_us - timeSinceLastStep);
            }
        }

        // Generate step pulse using RMT
        rmt_item32_t items[1];
        items[0].duration0 = _config.stepPulseWidth_us;
        items[0].level0 = 1;
        items[0].duration1 = _config.stepPulseWidth_us;
        items[0].level1 = 0;
        
        if (rmt_write_items(_channel, items, 1, false) == ESP_OK) {
            _currentPos += direction ? 1 : -1;
            _lastStepTime = now;
            return true;
        }
        
        return false;
    }

    void emergencyStop() {
        // Immediately stop all motion
        _targetPos = _currentPos;  // Set target to current to stop motion
        _currentVelocity = 0;      // Zero velocity
        gpio_set_level(_stepPin, 0); // Ensure step pin is low
    }

private:
    gpio_num_t _stepPin;
    gpio_num_t _dirPin;
    rmt_channel_t _channel;
    Config _config;
    volatile int32_t _currentPos;
    volatile int32_t _targetPos;
    volatile uint64_t _lastStepTime;
    volatile float _currentVelocity;
    Error _error;
    bool _initialized;
    bool _lastDirection;
};
