#pragma once

#include <driver/gptimer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "debug_uart.h"

/**
 * GPTimerScheduler - Deterministic 100µs hardware timer for motion control
 * 
 * Replaces FreeRTOS vTaskDelayUntil (1ms resolution, ~200µs jitter) with
 * hardware GPTimer interrupt (100µs resolution, <1µs jitter target).
 * 
 * The timer ISR wakes the GPIOLoop task via xTaskNotifyFromISR, providing
 * deterministic scheduling for step/direction signal generation.
 * 
 * Safety: Timer automatically stops on E-stop or task deletion.
 */
class GPTimerScheduler {
public:
    /**
     * Initialize the GPTimer with specified interval
     * @param interval_us Timer interval in microseconds (default: 100µs)
     * @return true if initialization successful
     */
    static bool begin(uint32_t interval_us = 100) {
        if (_initialized) {
            DEBUG_PRINTLN("GPTimer already initialized");
            return true;
        }

        _interval_us = interval_us;

        // Configure GPTimer
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,  // 1MHz resolution (1µs tick)
            .intr_priority = 0,        // Auto-select interrupt priority
            .flags = {0},
        };

        esp_err_t ret = gptimer_new_timer(&timer_config, &_timer_handle);
        if (ret != ESP_OK) {
            DEBUG_PRINTF("Failed to create GPTimer: %s\n", esp_err_to_name(ret));
            return false;
        }

        // Configure alarm (interval-based)
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = interval_us,  // Alarm every N microseconds
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = true  // Auto-reload for periodic interrupts
            }
        };

        ret = gptimer_set_alarm_action(_timer_handle, &alarm_config);
        if (ret != ESP_OK) {
            DEBUG_PRINTF("Failed to set GPTimer alarm: %s\n", esp_err_to_name(ret));
            gptimer_del_timer(_timer_handle);
            return false;
        }

        // Register event callback
        gptimer_event_callbacks_t cbs = {
            .on_alarm = _timerCallback,
        };

        ret = gptimer_register_event_callbacks(_timer_handle, &cbs, NULL);
        if (ret != ESP_OK) {
            DEBUG_PRINTF("Failed to register GPTimer callback: %s\n", esp_err_to_name(ret));
            gptimer_del_timer(_timer_handle);
            return false;
        }

        // Enable timer
        ret = gptimer_enable(_timer_handle);
        if (ret != ESP_OK) {
            DEBUG_PRINTF("Failed to enable GPTimer: %s\n", esp_err_to_name(ret));
            gptimer_del_timer(_timer_handle);
            return false;
        }

        _initialized = true;
        DEBUG_PRINTF("GPTimer initialized: %luµs interval\n", interval_us);
        return true;
    }

    /**
     * Start the timer with periodic interrupts
     * @param taskToNotify Task handle to receive notifications from ISR
     * @return true if started successfully
     */
    static bool start(TaskHandle_t taskToNotify) {
        if (!_initialized) {
            DEBUG_PRINTLN("GPTimer not initialized");
            return false;
        }

        if (_running) {
            DEBUG_PRINTLN("GPTimer already running");
            return true;
        }

        _taskToNotify = taskToNotify;

        esp_err_t ret = gptimer_start(_timer_handle);
        if (ret != ESP_OK) {
            DEBUG_PRINTF("Failed to start GPTimer: %s\n", esp_err_to_name(ret));
            return false;
        }

        _running = true;
        DEBUG_PRINTF("GPTimer started, notifying task 0x%p\n", taskToNotify);
        return true;
    }

    /**
     * Stop the timer (e.g., during E-stop)
     * @return true if stopped successfully
     */
    static bool stop() {
        if (!_initialized || !_running) {
            return true;
        }

        esp_err_t ret = gptimer_stop(_timer_handle);
        if (ret != ESP_OK) {
            DEBUG_PRINTF("Failed to stop GPTimer: %s\n", esp_err_to_name(ret));
            return false;
        }

        _running = false;
        DEBUG_PRINTLN("GPTimer stopped");
        return true;
    }

    /**
     * Clean up timer resources
     */
    static void end() {
        if (!_initialized) {
            return;
        }

        if (_running) {
            stop();
        }

        gptimer_disable(_timer_handle);
        gptimer_del_timer(_timer_handle);
        _initialized = false;
        _timer_handle = nullptr;
        _taskToNotify = nullptr;
        DEBUG_PRINTLN("GPTimer deinitialized");
    }

    /**
     * Get current timer status
     */
    static bool isRunning() { return _running; }
    static uint32_t getInterval() { return _interval_us; }

    /**
     * Get statistics (for diagnostics)
     */
    static uint64_t getTickCount() { return _tick_count; }
    static uint32_t getMissedNotifications() { return _missed_notifications; }

private:
    // Timer ISR callback (IRAM-safe requirement disabled in sdkconfig)
    static bool _timerCallback(gptimer_handle_t timer, 
                              const gptimer_alarm_event_data_t *edata, 
                              void *user_ctx) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        _tick_count++;
        
        // Notify the GPIO task to wake up
        if (_taskToNotify != nullptr) {
            BaseType_t result = xTaskNotifyFromISR(_taskToNotify, 
                                                   0,
                                                   eNoAction,
                                                   &xHigherPriorityTaskWoken);
            if (result != pdPASS) {
                _missed_notifications++;
            }
        }
        
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    // Static members
    static gptimer_handle_t _timer_handle;
    static TaskHandle_t _taskToNotify;
    static uint32_t _interval_us;
    static bool _initialized;
    static bool _running;
    static uint64_t _tick_count;
    static uint32_t _missed_notifications;
};

// Static member initialization
gptimer_handle_t GPTimerScheduler::_timer_handle = nullptr;
TaskHandle_t GPTimerScheduler::_taskToNotify = nullptr;
uint32_t GPTimerScheduler::_interval_us = 100;
bool GPTimerScheduler::_initialized = false;
bool GPTimerScheduler::_running = false;
uint64_t GPTimerScheduler::_tick_count = 0;
uint32_t GPTimerScheduler::_missed_notifications = 0;
