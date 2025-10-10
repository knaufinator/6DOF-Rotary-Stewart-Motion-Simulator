/*
 * Stewart Platform Motion Controller - ESP-IDF Native Implementation
 * 
 * Pure ESP-IDF implementation (no Arduino framework dependency)
 * Optimized for ESP32-S3 with deterministic real-time control
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

// Project headers
#include "helpers.h"
#include "debug_uart.h"
#include "RMTMotorControl.h"
#include "GPTimerScheduler.h"

static const char *TAG = "stewart_main";

// Motor control variables
RMTMotorControl* motors[6];
volatile float arr[6] = {0, 0, 0, 0, 0, 0};
static long servo_pos[6] = {0, 0, 0, 0, 0, 0};

// GPIO pins for motors (using board-specific definitions)
const gpio_num_t stepPins[6] = {
    (gpio_num_t)STEP_PIN_1, (gpio_num_t)STEP_PIN_2, (gpio_num_t)STEP_PIN_3,
    (gpio_num_t)STEP_PIN_4, (gpio_num_t)STEP_PIN_5, (gpio_num_t)STEP_PIN_6
};
const gpio_num_t dirPins[6] = {
    (gpio_num_t)DIR_PIN_1, (gpio_num_t)DIR_PIN_2, (gpio_num_t)DIR_PIN_3,
    (gpio_num_t)DIR_PIN_4, (gpio_num_t)DIR_PIN_5, (gpio_num_t)DIR_PIN_6
};

// ESP32-S3 has 4 TX channels (0-3) available for RMT
// We'll use these for motors 0-3, and use direct GPIO for motors 4-5
const rmt_channel_t channels[6] = {
    RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3,
    RMT_CHANNEL_0, RMT_CHANNEL_1  // Channel values for motors 4-5 not actually used
};

// Timing variables
int64_t currentMicros = 0;
int64_t previousMicros = 0;
int64_t lastDebugOutput = 0;
int microInterval = MICRO_INTERVAL_FAST;
const int64_t DEBUG_OUTPUT_INTERVAL = 100000; // 100ms = 10Hz debug output

// E-stop debouncing
typedef struct {
    gpio_num_t pin;
    bool current_state;
    bool debounced_state;
    int64_t last_change_time;
    int debounce_time_us;
} debounce_button_t;

static debounce_button_t estop_button;

// State variables
bool isPausedEStop = false;
bool isRateLimiting = false;
bool debugEnabled = false;  // Debug output state, default disabled for safety
SemaphoreHandle_t xMutex = NULL;
TaskHandle_t xGPIOLoopHandle = NULL;  // Task handle for GPTimer notification

// UART configuration
#define UART_NUM UART_NUM_0
#define UART_TX_PIN (1)
#define UART_RX_PIN (3)
#define UART_BUF_SIZE (1024)

// Function declarations
void setupMotorPins();
void handleStepDirection();
void process_data(char * data);
void processIncomingByte(const uint8_t inByte);
void InterfaceMonitorTask(void * pvParameters);
void GPIOLoopTask(void * pvParameters);
void EStopMonitorTask(void * pvParameters);
void outputDebugData();
void initDebounceButton(debounce_button_t *btn, gpio_num_t pin, int debounce_ms);
bool updateDebounceButton(debounce_button_t *btn);

/* ESP-IDF replacement for Arduino Serial.print/println */
void serial_printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    uart_write_bytes(UART_NUM, buffer, strlen(buffer));
}

void serial_println(const char *str) {
    uart_write_bytes(UART_NUM, str, strlen(str));
    uart_write_bytes(UART_NUM, "\r\n", 2);
}

/* ESP-IDF replacements for Arduino timing functions */
static inline int64_t micros() {
    return esp_timer_get_time();
}

static inline int64_t millis() {
    return esp_timer_get_time() / 1000;
}

// Initialize debounce button
void initDebounceButton(debounce_button_t *btn, gpio_num_t pin, int debounce_ms) {
    btn->pin = pin;
    btn->current_state = gpio_get_level(pin);
    btn->debounced_state = btn->current_state;
    btn->last_change_time = esp_timer_get_time();
    btn->debounce_time_us = debounce_ms * 1000;
    
    // Configure GPIO as input with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// Update debounce button state, returns true if state changed
bool updateDebounceButton(debounce_button_t *btn) {
    bool current = gpio_get_level(btn->pin);
    int64_t now = esp_timer_get_time();
    
    if (current != btn->current_state) {
        btn->current_state = current;
        btn->last_change_time = now;
    }
    
    if ((now - btn->last_change_time) > btn->debounce_time_us) {
        if (current != btn->debounced_state) {
            btn->debounced_state = current;
            return true;  // State changed
        }
    }
    
    return false;  // No change
}

void setPos() {  
    //Platform and Base Coords
    for(int i = 0; i < 6; i++) {    
        long x = 0;
        float alpha = getAlpha(i, arr);
        
        //convert to steps
        x = alpha * STEPS_PER_DEGREE;
        
        //set motor target position
        xSemaphoreTake(xMutex, portMAX_DELAY);
        if (!motors[i]->setTargetPosition(x)) {
            DEBUG_PRINTF("Motor %d position error: %d\n", i, motors[i]->getLastError());
        }
        xSemaphoreGive(xMutex);
    }
}

void setupMotorPins() {
    // Configure motor settings
    RMTMotorControl::Config motorConfig;
    motorConfig.stepPulseWidth_us = 2;      // 2µs pulse width for better reliability
    motorConfig.dirSetupTime_us = 5;        // 5µs direction setup time for better reliability
    motorConfig.minStepInterval_us = 5;     // 5µs minimum between steps (200kHz max)
    motorConfig.maxStepRate = 200000;       // 200kHz max step rate for better reliability
    motorConfig.maxAcceleration = 50000;    // 50k steps/sec^2 acceleration
    motorConfig.enableSoftLimits = true;
    motorConfig.softLimitMin = -100000;     // Adjust these limits based on your setup
    motorConfig.softLimitMax = 100000;

    // Initialize first 4 motors with RMT (unique channels 0-3)
    for(int i = 0; i < 4; i++) {
        motors[i] = new RMTMotorControl(stepPins[i], dirPins[i], channels[i]);
        if (!motors[i]->begin(motorConfig)) {
            DEBUG_PRINTF("Failed to initialize motor %d with RMT, error: %d\n", i, motors[i]->getLastError());
        } else {
            DEBUG_PRINTF("Successfully initialized motor %d with RMT channel %d\n", i, channels[i]);
        }
    }
    
    // For motors 4 & 5, configure GPIO pins directly for step/direction
    for(int i = 4; i < 6; i++) {
        // Configure GPIO pins for direct control
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << stepPins[i]) | (1ULL << dirPins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        gpio_set_level(stepPins[i], 0);
        gpio_set_level(dirPins[i], 0);
        
        // Create motor objects but mark them as special GPIO-only motors
        motors[i] = new RMTMotorControl(stepPins[i], dirPins[i], RMT_CHANNEL_0); // Channel won't be used
        motors[i]->beginGPIOOnly(); // Custom initialization for GPIO-only operation

        DEBUG_PRINTF("Initialized motor %d with direct GPIO control\n", i);
    }
}

void handleStepDirection() {
    //lock access to motor array
    xSemaphoreTake(xMutex, portMAX_DELAY);
    
    // Update each motor
    for (int i = 0; i < 6; i++) {
        if (motors[i] && !motors[i]->update()) {
            DEBUG_PRINTF("Motor %d update error: %d\n", i, motors[i]->getLastError());
        }
    }
    
    //give up lock
    xSemaphoreGive(xMutex);
}

void InterfaceMonitorTask(void * pvParameters) {
    uint8_t data[128];
    
    for(;;) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                processIncomingByte(data[i]);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void GPIOLoopTask(void * pvParameters) {
    // Subscribe this task to the watchdog
    esp_task_wdt_add(NULL);
    
    // Store task handle for GPTimer notifications
    xGPIOLoopHandle = xTaskGetCurrentTaskHandle();
    
    DEBUG_PRINTLN("GPIOLoop: Initializing GPTimer scheduler (100µs)");
    
    // Initialize and start GPTimer for deterministic 100µs scheduling
    if (!GPTimerScheduler::begin(100)) {  // 100µs interval = 10kHz update rate
        DEBUG_PRINTLN("ERROR: Failed to initialize GPTimer! Falling back to vTaskDelayUntil");
        // Fallback to old timing method if GPTimer fails
        TickType_t xLastWakeTime = xTaskGetTickCount();
        for(;;) {
            esp_task_wdt_reset();
            currentMicros = micros();
            
            if (currentMicros - previousMicros >= microInterval) {
                previousMicros = currentMicros;
                handleStepDirection();
                
                if (currentMicros - lastDebugOutput >= DEBUG_OUTPUT_INTERVAL) {
                    outputDebugData();
                    lastDebugOutput = currentMicros;
                }
            }
            
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        }
        return;
    }
    
    if (!GPTimerScheduler::start(xGPIOLoopHandle)) {
        DEBUG_PRINTLN("ERROR: Failed to start GPTimer!");
        return;
    }
    
    DEBUG_PRINTLN("GPIOLoop: GPTimer started, entering main loop");
    
    for(;;) {
        // Wait for notification from GPTimer ISR (blocking, no busy-wait)
        // This provides deterministic 100µs wake-up with <1µs jitter
        ulTaskNotifyTake(pdTRUE,           // Clear notification count on exit
                        portMAX_DELAY);   // Wait indefinitely for notification
        
        // Reset watchdog
        esp_task_wdt_reset();
        
        currentMicros = micros();
        
        // Execute motor step/direction logic every iteration (100µs rate)
        if (currentMicros - previousMicros >= microInterval) {
            previousMicros = currentMicros;
            handleStepDirection();
            
            // Only output debug data every 100ms (1000 iterations at 100µs)
            if (currentMicros - lastDebugOutput >= DEBUG_OUTPUT_INTERVAL) {
                outputDebugData();
                lastDebugOutput = currentMicros;
            }
        }
    }
}

void EStopMonitorTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool prev_state = estop_button.debounced_state;
    
    for(;;) {
        // Update watchdog to indicate E-stop monitoring is alive
        esp_task_wdt_reset();
        
        // Update button status through debounce filter
        if (updateDebounceButton(&estop_button)) {
            bool current_state = estop_button.debounced_state;
            
            // Check for falling edge (button pressed - E-stop activated)
            if (prev_state == 1 && current_state == ESTOP_ACTIVE_STATE) {
                // SAFETY: Stop GPTimer to prevent motion during E-stop
                GPTimerScheduler::stop();
                
                // Immediately disable all motor outputs
                for(int i = 0; i < 6; i++) {
                    if (motors[i]) {
                        motors[i]->emergencyStop();
                    }
                }
                isPausedEStop = true;
                
                // Log E-stop activation
                DEBUG_PRINTLN("E-STOP ACTIVATED - GPTimer stopped");
            }
            
            // Check for rising edge (button released)
            if (prev_state == ESTOP_ACTIVE_STATE && current_state == 1) {
                // Don't automatically resume - require explicit reset
                isRateLimiting = true;
                DEBUG_PRINTLN("E-STOP RELEASED - Reset required");
                
                // Restart GPTimer when E-stop is cleared and system is ready
                if (xGPIOLoopHandle != NULL) {
                    GPTimerScheduler::start(xGPIOLoopHandle);
                    DEBUG_PRINTLN("GPTimer restarted after E-stop release");
                }
            }
            
            prev_state = current_state;
        }
        
        // Check E-stop state more frequently than debounce time
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ESTOP_CHECK_INTERVAL_MS));
    }
}

void process_data(char * data) {
    // Check for debug control commands
    if (strcmp(data, DEBUG_ENABLE_CMD) == 0) {
        debugEnabled = true;
        DEBUG_PRINTLN("Debug output enabled");
        return;
    } else if (strcmp(data, DEBUG_DISABLE_CMD) == 0) {
        debugEnabled = false;
        DEBUG_PRINTLN("Debug output disabled");
        return;
    }

    char * tok;
    float temp;
    int i = 0;
    float arrRaw[6] = {0,0,0, 0,0,0};
    float arrRateLimited[6] = {0,0,0, 0,0,0};
    
    // Debug output - show received data
    DEBUG_PRINTF("Received data: %s\n", data);
    
    tok = strtok(data, ",");
    
    while (tok != NULL && i < 6) {
        float value = atof(tok);
        
        if(i == 2)
            temp = mapfloat(value, 0, 4094, -7, 7);//hieve 
        else if(i > 2)//rotations, pitch,roll,yaw
            temp = mapfloat(value, 0, 4094, -30, 30) * (M_PI/180.0);
        else//sway,surge
            temp = mapfloat(value, 0, 4094, -8, 8); 
        
        arrRaw[i] = temp;
        
        // Debug output - show parsed values
        DEBUG_PRINTF("Axis %d: Raw=%0.3f Mapped=%0.3f\n", i, value, temp);
        
        i++;
        tok = strtok(NULL, ",");
    }   
    
    //if we are not in an estop pause, allow setting of the current position
    if(!isPausedEStop) {
        //if we are just after resetting estop, we will be in a ratelimited mode until the ratelimited position is within close proximity of the actual last stored location.
        if(isRateLimiting) {    
            bool isNotWithinLimit = false;
            for(int i=0; i<6; i++) {
                arrRateLimited[i] = rateLimit(arrRaw[i], arr[i]);
                
                //check if we are within limits
                if(fabs(arrRaw[i] - arr[i]) > 0.01) {
                    isNotWithinLimit = true;
                }
            }
            
            //if we are within limits, stop rate limiting.
            if(!isNotWithinLimit) {
                isRateLimiting = false;
                DEBUG_PRINTLN("Rate limiting disabled - within limits");
            }
            
            //set position
            memcpy((void*)arr, arrRateLimited, sizeof(arr));
            DEBUG_PRINTLN("Using rate-limited values");
        } else {
            //set position directly
            memcpy((void*)arr, arrRaw, sizeof(arr));
            DEBUG_PRINTLN("Using direct values");
        }
        
        //calculate and set new position
        setPos();
    } else {
        DEBUG_PRINTLN("Motion paused - E-Stop active");
    }
}

void processIncomingByte(const uint8_t inByte) {
    static char input_line[MAX_SERIAL_INPUT];
    static unsigned int input_pos = 0;
    static int64_t lastMessageTime = 0;
    int64_t currentTime;
    int64_t timeSinceLastMessage;
    
    switch (inByte) {
        case 'X':   // end of text
            input_line[input_pos] = 0;  // terminating null byte
            
            currentTime = millis();
            timeSinceLastMessage = currentTime - lastMessageTime;
            DEBUG_PRINTF("Message interval: %lldms\n", timeSinceLastMessage);
            lastMessageTime = currentTime;
            
            process_data(input_line);          
            input_pos = 0;  
            break;
            
        default:
            if (input_pos < (MAX_SERIAL_INPUT - 1))
                input_line[input_pos++] = inByte;
            break;
    }
}

void outputDebugData() {
    if (!debugEnabled) return;  // Skip if debug output is disabled
    
    int64_t now = micros();
    
    // Only output if enough time has passed (10Hz)
    if (now - lastDebugOutput >= DEBUG_OUTPUT_INTERVAL) {
        lastDebugOutput = now;
        
        // Output format: DEBUG,timestamp,angles[6],targetX,targetY,targetZ,rotX,rotY,rotZ
        DEBUG_PRINTF("DEBUG,%lld,", now);
        
        // Output current angles
        for(int i = 0; i < 6; i++) {
            DEBUG_PRINTF("%.2f,", arr[i]);
        }
        
        // Output target position and rotation (placeholder values for now)
        DEBUG_PRINTF("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                 0.0f, 0.0f, 0.0f,  // X, Y, Z
                 0.0f, 0.0f, 0.0f); // rotX, rotY, rotZ
    }
}

// ESP-IDF entry point (replaces Arduino setup/loop)
extern "C" void app_main(void)
{
    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configure UART for serial communication
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "Stewart Platform Controller Starting...");
    DEBUG_PRINTLN("Starting up...");
  
    // Initialize watchdog - ESP-IDF 5.2.0 API
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,        // Don't watch idle tasks
        .trigger_panic = true       // Panic on timeout
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_config));
  
    // Configure E-Stop button with debouncing
    initDebounceButton(&estop_button, (gpio_num_t)ESTOP_PIN, ESTOPDEBOUNCETIME);
    
    // Initialize motor control pins
    setupMotorPins();
    
    // Create mutex for thread safety
    xMutex = xSemaphoreCreateMutex();
    
    // Create high-priority E-stop monitoring task
    xTaskCreatePinnedToCore(
        EStopMonitorTask,        /* Task function. */
        "EStopMonitor",          /* name of task. */
        10000,                   /* Stack size of task */
        NULL,                    /* parameter of the task */
        configMAX_PRIORITIES-1,  /* Highest priority */
        NULL,                    /* Task handle */
        0);                      /* pin task to core 0 */
    
    // Create tasks for interface monitoring and GPIO control with proper stack sizes
    xTaskCreatePinnedToCore(
        InterfaceMonitorTask,    /* Task function. */
        "InterfaceMonitor",      /* name of task. */
        8192,                    /* Stack size of task */
        NULL,                    /* parameter of the task */
        2,                       /* priority of the task */
        NULL,                    /* Task handle */
        0);                      /* pin task to core 0 */
    
    xTaskCreatePinnedToCore(
        GPIOLoopTask,            /* Task function. */
        "GPIOLoop",              /* name of task. */
        8192,                    /* Stack size of task */
        NULL,                    /* parameter of the task */
        3,                       /* priority of the task */
        NULL,                    /* Task handle */
        1);                      /* pin task to core 1 */
    
    ESP_LOGI(TAG, "Setup Complete!");
    DEBUG_PRINTLN("Setup Complete!");
    
    // app_main returns, tasks continue running
}
