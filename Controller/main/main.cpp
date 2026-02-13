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
#include <fcntl.h>
#include <unistd.h>
#include "nvs_flash.h"
#include "esp_task_wdt.h"

// Project headers
#include "helpers.h"
#include "debug_uart.h"
#include "InverseKinematics.h"
#include "AxisScaling.h"
#include "MotionCueing.h"
#include "MCPWMMotorControl.h"
#include "GPTimerScheduler.h"
#include "version.h"
#ifdef ENABLE_ETHERNET
#include "EthernetTransport.h"
#endif
#ifdef ENABLE_WIFI
#include "WifiTransport.h"
#endif
#ifdef ENABLE_BLE
#include "BleTransport.h"
#endif

static const char *TAG = "stewart_main";

// Platform configuration (single source of truth for geometry)
static StewartConfig stewartConfig;

// Per-axis scaling (derived from platform geometry at boot, read-only via SCALE?)
static AxisScaleConfig axisScales;

// Input bit depth — determines max_raw for mapRawToPosition
// Configurable at runtime via BITS:N serial command, default 12-bit
static uint8_t  inputBitRange = 12;
static float    maxRawInput   = 4094.0f;  // (1 << 12) - 2 = 4094

// Motion cueing (classical washout + tilt coordination, NVS-persistent)
static MotionCueingConfig mcaConfig;

// Motor control variables
MCPWMMotorControl* motors[6];
volatile float arr[6] = {0, 0, 0, 0, 0, 0};

// GPIO pins for motors (using board-specific definitions)
const gpio_num_t stepPins[6] = {
    (gpio_num_t)STEP_PIN_1, (gpio_num_t)STEP_PIN_2, (gpio_num_t)STEP_PIN_3,
    (gpio_num_t)STEP_PIN_4, (gpio_num_t)STEP_PIN_5, (gpio_num_t)STEP_PIN_6
};
const gpio_num_t dirPins[6] = {
    (gpio_num_t)DIR_PIN_1, (gpio_num_t)DIR_PIN_2, (gpio_num_t)DIR_PIN_3,
    (gpio_num_t)DIR_PIN_4, (gpio_num_t)DIR_PIN_5, (gpio_num_t)DIR_PIN_6
};

// All 6 motors use MCPWM hardware-timed one-shot pulses (2 groups × 3 timers)

// Loop timing instrumentation (updated every GPTimer tick)
static volatile uint32_t loopCount = 0;
static volatile uint32_t loopMin_us = UINT32_MAX;
static volatile uint32_t loopMax_us = 0;
static volatile uint64_t loopSum_us = 0;
static volatile bool     motorTestRunning = false;

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
volatile float lastServoAngles[6] = {0};  // Latest IK output (radians) for telemetry


// Binary protocol constants
// Input:  [0xAA] [0x55] [uint16_t × 6 little-endian] [XOR checksum] = 15 bytes
#define BIN_SYNC_0       0xAA
#define BIN_SYNC_1       0x55
#define BIN_PAYLOAD_SIZE 12   // 6 × uint16_t
#define BIN_PACKET_SIZE  15   // 2 sync + 12 payload + 1 checksum

// Telemetry response: [0xBB] [0xCC] [float32 × 6 little-endian] [XOR checksum] = 27 bytes
// Sends computed servo angles (radians) back to dashboard after each motion packet
#define TEL_SYNC_0       0xBB
#define TEL_SYNC_1       0xCC
#define TEL_PAYLOAD_SIZE 24   // 6 × float32
#define TEL_PACKET_SIZE  27   // 2 sync + 24 payload + 1 checksum

// Function declarations
void setupMotorPins();
void handleStepDirection();
void applyMotionValues(float values[6]);
void process_data(char * data);
void process_binary_packet(const uint8_t *payload);
void processIncomingByte(const uint8_t inByte);
void InterfaceMonitorTask(void * pvParameters);
void GPIOLoopTask(void * pvParameters);
void EStopMonitorTask(void * pvParameters);
void outputDebugData();
void initDebounceButton(debounce_button_t *btn, gpio_num_t pin, int debounce_ms);
bool updateDebounceButton(debounce_button_t *btn);

/* ESP-IDF replacement for Arduino Serial.print/println */
void serial_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
}

void serial_println(const char *str) {
    printf("%s\r\n", str);
    fflush(stdout);
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
    btn->debounce_time_us = debounce_ms * 1000;
    
    // Configure GPIO as input with pull-up BEFORE reading initial state,
    // otherwise floating pin may read LOW and trigger false E-stop.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Now read initial state with pull-up active
    btn->current_state = gpio_get_level(pin);
    btn->debounced_state = btn->current_state;
    btn->last_change_time = esp_timer_get_time();
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
    float position[6];
    // Copy volatile arr under implicit atomicity (single writer)
    for (int i = 0; i < 6; i++) {
        position[i] = arr[i];
    }

    float servoAngles[6];
    calculateAllServoAngles(position, &stewartConfig, servoAngles);
    memcpy((void*)lastServoAngles, servoAngles, sizeof(lastServoAngles));

    xSemaphoreTake(xMutex, portMAX_DELAY);
    for (int i = 0; i < 6; i++) {
        long x = (long)(servoAngles[i] * IK_RAD_TO_DEG * stewartConfig.steps_per_degree);
        if (!motors[i]->setTargetPosition(x)) {
            DEBUG_PRINTF("Motor %d position error: %d\n", i, motors[i]->getLastError());
        }
    }
    xSemaphoreGive(xMutex);
}

void setupMotorPins() {
    MCPWMMotorControl::Config motorConfig;
    motorConfig.stepPulseWidth_us = 2;      // 2µs pulse width (most drivers spec >= 1.5µs)
    motorConfig.dirSetupTime_us = 5;        // 5µs direction setup (driver-safe minimum)
    motorConfig.minStepInterval_us = 4;     // 4µs minimum between steps (250kHz max)
    motorConfig.maxStepRate = 250000;       // 250kHz max step rate
    motorConfig.maxAcceleration = 100000;   // 100k steps/sec² acceleration
    motorConfig.enableSoftLimits = true;
    motorConfig.softLimitMin = -100000;
    motorConfig.softLimitMax = 100000;

    // Initialize all 6 motors via MCPWM (2 groups × 3 timers = 6 hw channels)
    int ok_count = 0;
    for (int i = 0; i < 6; i++) {
        motors[i] = new MCPWMMotorControl(stepPins[i], dirPins[i]);
        if (!motors[i]->begin(motorConfig)) {
            DEBUG_PRINTF("FATAL: motor %d init failed, error: %d\n", i, motors[i]->getLastError());
        } else {
            ok_count++;
        }
    }
    DEBUG_PRINTF("Motors initialized: %d/6 MCPWM hardware-timed\n", ok_count);
}

void handleStepDirection() {
    // Burst stepping: fire multiple steps per motor per 50µs tick.
    // Round-robin cycles through all 6 motors, firing one step each per round.
    // Each round naturally spaces calls to the same motor by ~6× function overhead,
    // satisfying minStepInterval_us (4µs) between consecutive steps on the same motor.
    // MCPWM one-shot pulses are fire-and-forget (hardware-timed), so overlapping
    // pulses on different motors are fine.
    //
    // Effective rate: ~80k–200k steps/sec per motor (vs 20k with single-step),
    // depending on how many motors are simultaneously active.

    xSemaphoreTake(xMutex, portMAX_DELAY);

    const uint64_t BURST_BUDGET_US = 45;  // 45µs of the 50µs tick (5µs headroom)
    uint64_t tickStart = esp_timer_get_time();
    bool anyActive = true;

    while (anyActive) {
        if ((esp_timer_get_time() - tickStart) >= BURST_BUDGET_US) break;
        anyActive = false;
        for (int i = 0; i < 6; i++) {
            if (!motors[i]) continue;
            if (motors[i]->getTargetPosition() == motors[i]->getCurrentPosition()) continue;
            anyActive = true;
            motors[i]->update();  // fires one step if minStepInterval elapsed, else no-op
        }
    }

    xSemaphoreGive(xMutex);
}

static uint32_t binPktCount = 0;

void InterfaceMonitorTask(void * pvParameters) {
    uint8_t data[128];
    
    // Read from stdin (USB Serial JTAG via secondary console VFS)
    int fd = fileno(stdin);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    
    int64_t lastDbg = 0;
    for(;;) {
        int len = read(fd, data, sizeof(data) - 1);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                processIncomingByte(data[i]);
            }
        }

        // Debug output runs here (not in GPIOLoopTask) so fflush can't stall motors
        int64_t now = micros();
        if (now - lastDbg >= DEBUG_OUTPUT_INTERVAL) {
            lastDbg = now;
            outputDebugData();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void GPIOLoopTask(void * pvParameters) {
    // Subscribe this task to the watchdog
    esp_task_wdt_add(NULL);
    
    // Store task handle for GPTimer notifications
    xGPIOLoopHandle = xTaskGetCurrentTaskHandle();
    
    DEBUG_PRINTLN("GPIOLoop: Initializing GPTimer scheduler (50µs)");
    
    // Initialize and start GPTimer for deterministic 50µs scheduling
    if (!GPTimerScheduler::begin(MICRO_INTERVAL_FAST)) {  // 50µs interval = 20kHz update rate
        DEBUG_PRINTLN("ERROR: Failed to initialize GPTimer! Falling back to vTaskDelayUntil");
        // Fallback to old timing method if GPTimer fails
        TickType_t xLastWakeTime = xTaskGetTickCount();
        for(;;) {
            esp_task_wdt_reset();
            currentMicros = micros();
            
            if (currentMicros - previousMicros >= microInterval) {
                previousMicros = currentMicros;
                handleStepDirection();
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
        // This provides deterministic 50µs wake-up with <1µs jitter
        ulTaskNotifyTake(pdTRUE,           // Clear notification count on exit
                        portMAX_DELAY);   // Wait indefinitely for notification
        
        // Reset watchdog
        esp_task_wdt_reset();
        
        currentMicros = micros();
        
        // Execute motor step/direction logic every iteration (50µs rate)
        if (currentMicros - previousMicros >= microInterval) {
            int64_t loopStart = currentMicros;
            previousMicros = currentMicros;
            handleStepDirection();
            
            // Track loop execution time
            uint32_t dur = (uint32_t)(micros() - loopStart);
            loopCount++;
            loopSum_us += dur;
            if (dur < loopMin_us) loopMin_us = dur;
            if (dur > loopMax_us) loopMax_us = dur;
            
        }
    }
}

void EStopMonitorTask(void * pvParameters) {
    // Subscribe this task to the watchdog
    esp_task_wdt_add(NULL);
    
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

// Shared motion-value handler: applies rate limiting, updates arr[], calls setPos()
void applyMotionValues(float values[6]) {
    if (isPausedEStop) {
        DEBUG_PRINTLN("Motion paused - E-Stop active");
        return;
    }

    if (isRateLimiting) {
        bool withinLimit = true;
        float limited[6];
        for (int i = 0; i < 6; i++) {
            limited[i] = rateLimit(values[i], arr[i]);
            if (fabs(values[i] - arr[i]) > 0.01f)
                withinLimit = false;
        }
        if (withinLimit) {
            isRateLimiting = false;
            DEBUG_PRINTLN("Rate limiting disabled - within limits");
        }
        memcpy((void*)arr, limited, sizeof(arr));
    } else {
        memcpy((void*)arr, values, sizeof(arr));
    }
    setPos();
}

// ── Binary motion packet (0xAA 0x55 + 12-byte payload + XOR checksum) ──
// Tests:  High-throughput motion data path from SimTools/dashboard
// Proves: Binary protocol framing (sync + checksum) validated by
//         processIncomingByte state machine, uint16 LE decode works,
//         mapRawToPosition() axis scaling, processMotionCueing() biquad
//         filter chain, applyMotionValues() rate limiter → setPos() pipeline,
//         full data path from USB RX → IK → MCPWM at ~30-60 Hz packet rate
void send_telemetry(const float angles[6]) {
    // ASCII telemetry — binary 0xBB/0xCC sync desynchronizes when float
    // bytes collide with sync pattern. Text is robust over USB Serial JTAG.
    // Format: TEL,a1..a6,p1..p6  (angles in radians, then input positions)
    // Positions allow the dashboard to verify ESP32 IK input→output correctness.
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "TEL,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        (double)angles[0], (double)angles[1], (double)angles[2],
        (double)angles[3], (double)angles[4], (double)angles[5],
        (double)arr[0], (double)arr[1], (double)arr[2],
        (double)arr[3], (double)arr[4], (double)arr[5]);
    if (n > 0)
        fwrite(buf, 1, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1, stdout);
    // No fflush here — the periodic outputDebugData() flush at 10Hz
    // will push telemetry out. Avoids blocking on USB FIFO.
}

void process_binary_packet(const uint8_t *payload) {
    binPktCount++;
    float raw[6], mapped[6];
    for (int i = 0; i < 6; i++) {
        raw[i] = (float)((uint16_t)payload[i * 2] | ((uint16_t)payload[i * 2 + 1] << 8));
    }
    mapRawToPosition(raw, &axisScales, maxRawInput, mapped);
    // No MCA here — the SIL server already applies motion cueing before
    // encoding raw values. Applying it again causes double-filtering and
    // filter instability at low packet rates.
    applyMotionValues(mapped);
}

// Legacy CSV parser (kept for debug commands and backward compatibility)
void process_data(char * data) {
    // ── DBG:1 / DBG:0 — Toggle verbose debug output ──────────────────
    // Tests:  Serial RX command parsing, bidirectional USB Serial JTAG link
    // Proves: VFS stdin→processIncomingByte pipeline, printf→USB TX path,
    //         debug_uart.h macro gating (DEBUG_PRINTLN only emits when enabled)
    if (strcmp(data, DEBUG_ENABLE_CMD) == 0) {
        debugEnabled = true;
        DEBUG_PRINTLN("Debug output enabled");
        return;
    } else if (strcmp(data, DEBUG_DISABLE_CMD) == 0) {
        debugEnabled = false;
        DEBUG_PRINTLN("Debug output disabled");
        return;
    }

    // ── Motor timing diagnostics ──────────────────────────────────────

    // ── MSTAT — Report loop timing + per-motor step statistics ────────
    // Tests:  GPIOLoopTask scheduling, MCPWM motor driver state
    // Proves: FreeRTOS task is running (loopCount > 0),
    //         GPTimer ISR fires at correct interval (loop_us ~ microInterval),
    //         all 6 MCPWMMotorControl instances initialized (init=1),
    //         step timing jitter within bounds (min/avg/max step_us)
    if (strcmp(data, "MSTAT") == 0) {
        // Loop timing
        uint32_t cnt = loopCount;
        float avg = cnt > 0 ? (float)loopSum_us / cnt : 0;
        serial_printf("MSTAT:loop_count=%lu,loop_us(min/avg/max)=%lu/%.1f/%lu,interval=%d\r\n",
            cnt, (unsigned long)loopMin_us, avg, (unsigned long)loopMax_us, microInterval);
        // Per-motor stats
        for (int i = 0; i < 6; i++) {
            if (!motors[i]) continue;
            const MCPWMMotorControl::Stats& s = motors[i]->getStats();
            float savg = s.intervalSamples > 0 ? (float)s.sumInterval_us / s.intervalSamples : 0;
            uint32_t smin = s.minInterval_us == UINT32_MAX ? 0 : s.minInterval_us;
            serial_printf("  M%d: pos=%ld tgt=%ld steps=%lu errs=%lu step_us(min/avg/max)=%lu/%.1f/%lu init=%d\r\n",
                i, (long)motors[i]->getCurrentPosition(), (long)motors[i]->getTargetPosition(),
                (unsigned long)s.totalSteps, (unsigned long)s.stepErrors,
                (unsigned long)smin, savg, (unsigned long)s.maxInterval_us,
                motors[i]->isInitialized() ? 1 : 0);
        }
        return;
    }

    // ── MTEST:RESET — Clear all timing/step statistics ────────────────
    // Tests:  Stats reset path for loop counters and per-motor accumulators
    // Proves: resetStats() zeroes all MCPWMMotorControl::Stats fields,
    //         loop timing counters reset cleanly (no stale data in MSTAT)
    if (strcmp(data, "MTEST:RESET") == 0) {
        loopCount = 0; loopMin_us = UINT32_MAX; loopMax_us = 0; loopSum_us = 0;
        for (int i = 0; i < 6; i++) {
            if (motors[i]) motors[i]->resetStats();
        }
        serial_printf("MTEST:RESET=OK\r\n");
        return;
    }

    // ── MTEST — Motor self-test: 200 steps forward, 200 steps back ───
    // Tests:  Full MCPWM stepper drive pipeline, end-to-end
    // Proves: (1) MCPWM timer→operator→comparator→generator chain fires pulses
    //         (2) setTargetPosition() / getCurrentPosition() track correctly
    //         (3) All 6 motors return to exact start position (no lost steps)
    //         (4) Step rate throughput (steps/s) meets real-time requirements
    //         (5) Mutex arbitration between InterfaceMonitorTask and GPIOLoopTask
    //         (6) Loop timing during load (MTEST:LOOP min/avg/max)
    //         (7) PASS/FAIL verdict: position error = 0 for all motors
    if (strcmp(data, "MTEST") == 0) {
        if (motorTestRunning) {
            serial_printf("MTEST:BUSY\r\n");
            return;
        }
        motorTestRunning = true;
        const int TEST_STEPS = 200;
        serial_printf("MTEST:START steps=%d\r\n", TEST_STEPS);

        // Save current positions
        int32_t savedPos[6];
        for (int i = 0; i < 6; i++) {
            savedPos[i] = motors[i] ? motors[i]->getCurrentPosition() : 0;
        }

        // Set forward targets, THEN reset stats so setup overhead is excluded
        xSemaphoreTake(xMutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            if (motors[i]) motors[i]->setTargetPosition(savedPos[i] + TEST_STEPS);
        }
        loopCount = 0; loopMin_us = UINT32_MAX; loopMax_us = 0; loopSum_us = 0;
        for (int i = 0; i < 6; i++) {
            if (motors[i]) motors[i]->resetStats();
        }
        xSemaphoreGive(xMutex);

        // Wait for forward motion to complete (timeout 2s)
        int64_t t0 = micros();
        bool done = false;
        while (!done && (micros() - t0) < 2000000) {
            done = true;
            for (int i = 0; i < 6; i++) {
                if (motors[i] && motors[i]->getCurrentPosition() != motors[i]->getTargetPosition())
                    done = false;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        int64_t fwd_us = micros() - t0;

        // Capture forward stats before resetting for back phase
        uint32_t fwd_loops = loopCount;
        float fwd_avg = fwd_loops > 0 ? (float)loopSum_us / fwd_loops : 0;
        uint32_t fwd_loop_min = loopMin_us, fwd_loop_max = loopMax_us;

        // Set back targets, then reset stats for back phase
        xSemaphoreTake(xMutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            if (motors[i]) motors[i]->setTargetPosition(savedPos[i]);
        }
        loopCount = 0; loopMin_us = UINT32_MAX; loopMax_us = 0; loopSum_us = 0;
        for (int i = 0; i < 6; i++) {
            if (motors[i]) motors[i]->resetStats();
        }
        xSemaphoreGive(xMutex);

        t0 = micros();
        done = false;
        while (!done && (micros() - t0) < 2000000) {
            done = true;
            for (int i = 0; i < 6; i++) {
                if (motors[i] && motors[i]->getCurrentPosition() != motors[i]->getTargetPosition())
                    done = false;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        int64_t back_us = micros() - t0;

        // Report timing
        serial_printf("MTEST:FWD %d steps in %lld us (%.1f steps/s)\r\n",
            TEST_STEPS, fwd_us, TEST_STEPS * 1000000.0f / fwd_us);
        serial_printf("MTEST:FWD_LOOP loops=%lu dur_us(min/avg/max)=%lu/%.1f/%lu\r\n",
            (unsigned long)fwd_loops, (unsigned long)fwd_loop_min, fwd_avg, (unsigned long)fwd_loop_max);
        serial_printf("MTEST:BACK %d steps in %lld us (%.1f steps/s)\r\n",
            TEST_STEPS, back_us, TEST_STEPS * 1000000.0f / back_us);
        uint32_t back_loops = loopCount;
        float back_avg = back_loops > 0 ? (float)loopSum_us / back_loops : 0;
        serial_printf("MTEST:BACK_LOOP loops=%lu dur_us(min/avg/max)=%lu/%.1f/%lu\r\n",
            (unsigned long)back_loops, (unsigned long)loopMin_us, back_avg, (unsigned long)loopMax_us);

        // Check all motors returned to original position
        bool allOk = true;
        for (int i = 0; i < 6; i++) {
            if (!motors[i]) continue;
            int32_t diff = motors[i]->getCurrentPosition() - savedPos[i];
            if (diff != 0) {
                serial_printf("MTEST:M%d POSITION ERROR off=%ld\r\n", i, (long)diff);
                allOk = false;
            }
        }

        // Per-motor step stats (back phase only — most recent clean window)
        for (int i = 0; i < 6; i++) {
            if (!motors[i]) continue;
            const MCPWMMotorControl::Stats& s = motors[i]->getStats();
            float savg = s.intervalSamples > 0 ? (float)s.sumInterval_us / s.intervalSamples : 0;
            uint32_t smin = s.minInterval_us == UINT32_MAX ? 0 : s.minInterval_us;
            serial_printf("  M%d: steps=%lu errs=%lu step_us(min/avg/max)=%lu/%.1f/%lu\r\n",
                i, (unsigned long)s.totalSteps, (unsigned long)s.stepErrors,
                (unsigned long)smin, savg, (unsigned long)s.maxInterval_us);
        }

        serial_printf("MTEST:%s\r\n", allOk ? "PASS" : "FAIL");
        motorTestRunning = false;
        return;
    }

    // ── E-Stop serial commands ─────────────────────────────────────────

    // ── ESTOP:SOFT — Graceful stop: return all motors to home position ──
    // Motors smoothly travel to center. GPTimer keeps running.
    // Use when you want to safely park the platform without a hard cut.
    if (strcmp(data, "ESTOP:SOFT") == 0) {
        // Set all motor targets to home (position 0 = center)
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (int i = 0; i < 6; i++) {
                arr[i] = 0;
                if (motors[i]) {
                    motors[i]->setTargetPosition(0);
                }
            }
            xSemaphoreGive(xMutex);
        }
        serial_printf("ESTOP:SOFT — Motors homing to center\r\n");
        return;
    }

    // ── ESTOP:FULL — Hard emergency stop: kill all motion immediately ───
    // Stops GPTimer, emergency-stops all motors, blocks applyMotionValues().
    // Identical to pressing the physical E-Stop button.
    // Requires ESTOP:RESET to resume operation.
    if (strcmp(data, "ESTOP:FULL") == 0) {
        GPTimerScheduler::stop();
        for (int i = 0; i < 6; i++) {
            if (motors[i]) {
                motors[i]->emergencyStop();
            }
        }
        isPausedEStop = true;
        serial_printf("ESTOP:FULL — GPTimer stopped, all motors killed. Send ESTOP:RESET to recover.\r\n");
        return;
    }

    // ── ESTOP:RESET — Recover from FULL E-Stop ─────────────────────────
    // Clears E-Stop flag, enables rate limiting for safe ramp-up,
    // restarts GPTimer. Does nothing if not in E-Stop state.
    if (strcmp(data, "ESTOP:RESET") == 0) {
        if (!isPausedEStop) {
            serial_printf("ESTOP:RESET — Not in E-Stop state, nothing to do\r\n");
            return;
        }
        isPausedEStop = false;
        isRateLimiting = true;  // safe ramp-up after recovery
        if (xGPIOLoopHandle != NULL) {
            GPTimerScheduler::start(xGPIOLoopHandle);
        }
        serial_printf("ESTOP:RESET — E-Stop cleared, GPTimer restarted (rate-limited)\r\n");
        return;
    }

    // ── ESTOP? — Query current E-Stop state ─────────────────────────────
    if (strcmp(data, "ESTOP?") == 0) {
        serial_printf("ESTOP:state=%s\r\n", isPausedEStop ? "ACTIVE" : "OK");
        return;
    }

    // ── ZERO — Reset all motor step counters to 0 (post-homing sync) ──
    // Call after AASD-15A servo drives complete their homing sequence.
    // This synchronizes the ESP32's internal position tracking with the
    // physical zero established by the drives.
    // Tests:  Position counter reset, mutex safety
    // Proves: All 6 motors report pos=0/tgt=0 after reset
    if (strcmp(data, "ZERO") == 0) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < 6; i++) {
                if (motors[i]) {
                    motors[i]->resetPosition();
                    motors[i]->resetStats();
                }
                arr[i] = 0;
            }
            xSemaphoreGive(xMutex);
            serial_printf("ZERO:OK — All motor positions reset to 0\r\n");
        } else {
            serial_printf("ZERO:ERR — Mutex timeout\r\n");
        }
        return;
    }

    // ── ZERO? — Query sync state (all motor current/target positions) ──
    if (strcmp(data, "ZERO?") == 0) {
        bool allZero = true;
        for (int i = 0; i < 6; i++) {
            if (!motors[i]) continue;
            int32_t pos = motors[i]->getCurrentPosition();
            int32_t tgt = motors[i]->getTargetPosition();
            if (pos != 0 || tgt != 0) allZero = false;
            serial_printf("  M%d: pos=%ld tgt=%ld\r\n", i, (long)pos, (long)tgt);
        }
        serial_printf("ZERO:sync=%s\r\n", allZero ? "OK" : "DRIFT");
        return;
    }

    // ── SCALE? — Query current axis scaling factors (geometry-derived) ─
    // Scales are computed from platform dimensions, not manually set.
    if (strcmp(data, "SCALE?") == 0) {
        serial_printf("SCALE:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            axisScales.scale[0], axisScales.scale[1], axisScales.scale[2],
            axisScales.scale[3], axisScales.scale[4], axisScales.scale[5]);
        return;
    }

    // ── BITS:N — Set input bit depth (determines max_raw for axis scaling) ─
    if (strncmp(data, "BITS:", 5) == 0) {
        int bits = atoi(data + 5);
        if (bits >= 8 && bits <= 20) {
            inputBitRange = (uint8_t)bits;
            maxRawInput = (float)((1 << bits) - 1);
            // Binary protocol uses uint16 — cap at 65535
            if (maxRawInput > 65535.0f) maxRawInput = 65535.0f;
            resetMotionCueing(&mcaConfig);
            serial_printf("BITS:%d,max_raw=%.0f\r\n", inputBitRange, maxRawInput);
        } else {
            serial_printf("ERR:BITS range 8-20\r\n");
        }
        return;
    }

    // ── BITS? — Query current input bit depth ─────────────────────────
    if (strcmp(data, "BITS?") == 0) {
        serial_printf("BITS:%d,max_raw=%.0f\r\n", inputBitRange, maxRawInput);
        return;
    }

    // ── VERSION? — Report firmware version ────────────────────────────
    if (strcmp(data, "VERSION?") == 0) {
        serial_printf("VERSION:%s,date=%s,time=%s\r\n",
            FW_VERSION_STRING, FW_BUILD_DATE, FW_BUILD_TIME);
        return;
    }

    // ── Platform configuration serial commands ─────────────────────────

    // ── CONFIG? — Query full platform geometry + drive train ────────────
    if (strcmp(data, "CONFIG?") == 0) {
        serial_printf("CONFIG:RD=%.2f,PD=%.2f,L1=%.2f,L2=%.2f,height=%.2f,theta_r=%.2f,theta_p=%.2f\r\n",
            stewartConfig.RD, stewartConfig.PD,
            stewartConfig.ServoArmLengthL1, stewartConfig.ConnectingArmLengthL2,
            stewartConfig.platformHeight, stewartConfig.theta_r, stewartConfig.theta_p);
        serial_printf("DRIVE:encoder=%d,gear=%.4f,planetary=%.2f,steps_deg=%.4f\r\n",
            stewartConfig.encoder_ppr, stewartConfig.virtual_gear,
            stewartConfig.planetary_ratio, stewartConfig.steps_per_degree);
        return;
    }

    // ── CONFIG:key=value — Set platform geometry parameter ──────────────
    // Recomputes axis scales from geometry after change.
    // Keys: RD, PD, L1, L2, height, theta_r, theta_p
    if (strncmp(data, "CONFIG:", 7) == 0) {
        char *param = data + 7;
        char *eq = strchr(param, '=');
        if (eq) {
            *eq = '\0';
            float val = atof(eq + 1);
            bool changed = true;
            if (strcmp(param, "RD") == 0) stewartConfig.RD = val;
            else if (strcmp(param, "PD") == 0) stewartConfig.PD = val;
            else if (strcmp(param, "L1") == 0) stewartConfig.ServoArmLengthL1 = val;
            else if (strcmp(param, "L2") == 0) stewartConfig.ConnectingArmLengthL2 = val;
            else if (strcmp(param, "height") == 0) stewartConfig.platformHeight = val;
            else if (strcmp(param, "theta_r") == 0) stewartConfig.theta_r = val;
            else if (strcmp(param, "theta_p") == 0) stewartConfig.theta_p = val;
            else { changed = false; serial_printf("CONFIG:ERR unknown key '%s'\r\n", param); }
            if (changed) {
                computeAxisScalesFromGeometry(&axisScales, &stewartConfig, 0.90f);
                serial_printf("CONFIG:OK %s=%.4f (scales recomputed)\r\n", param, val);
            }
        }
        return;
    }

    // ── DRIVE:key=value — Set drive train parameter ────────────────────
    // Recomputes steps_per_degree after change.
    // Keys: encoder, gear, planetary
    if (strncmp(data, "DRIVE:", 6) == 0) {
        char *param = data + 6;
        char *eq = strchr(param, '=');
        if (eq) {
            *eq = '\0';
            float val = atof(eq + 1);
            bool changed = true;
            if (strcmp(param, "encoder") == 0) stewartConfig.encoder_ppr = (int)val;
            else if (strcmp(param, "gear") == 0) stewartConfig.virtual_gear = val;
            else if (strcmp(param, "planetary") == 0) stewartConfig.planetary_ratio = val;
            else { changed = false; serial_printf("DRIVE:ERR unknown key '%s'\r\n", param); }
            if (changed) {
                computeStepsPerDegree(&stewartConfig);
                serial_printf("DRIVE:OK %s=%.4f → steps_deg=%.4f\r\n", param, val, stewartConfig.steps_per_degree);
            }
        }
        return;
    }

    // ── Motion Cueing serial commands ────────────────────────────────

    // ── MCA? — Query full Motion Cueing Algorithm config ─────────────
    // Tests:  MCA configuration readback, multi-line serial output
    // Proves: mcaConfig struct is coherent, preset name resolves correctly,
    //         all 6 axis channel filters + tilt coordination params serialize,
    //         serial_printf handles large multi-line output without truncation
    if (strcmp(data, "MCA?") == 0) {
        serial_printf("MCA:preset=%s,enabled=%d,sr=%.0f\r\n",
            mcaPresetName(mcaConfig.preset), mcaConfig.enabled, mcaConfig.sample_rate);
        const char* ax_names[] = {"surge","sway","heave","roll","pitch","yaw"};
        for (int j = 0; j < 6; j++) {
            AxisChannelFilter* ch = &mcaConfig.channels[j];
            serial_printf("  %s: hp=%d(fc=%.2f,Q=%.3f) lp=%d(fc=%.1f,Q=%.3f) gain=%.2f rl=%.2f\r\n",
                ax_names[j], ch->hp_enabled, ch->hp.fc, ch->hp.Q,
                ch->lp_enabled, ch->lp.fc, ch->lp.Q, ch->gain, ch->rate_limit);
        }
        serial_printf("  tilt: en=%d surge_g=%.4f sway_g=%.4f fc=%.2f Q=%.3f\r\n",
            mcaConfig.tilt.enabled, mcaConfig.tilt.surge_gain,
            mcaConfig.tilt.sway_gain, mcaConfig.tilt.fc, mcaConfig.tilt.Q);
        return;
    }

    // ── MCA:preset:<name> — Apply MCA preset ────────────────────────
    // Tests:  Preset loading and filter coefficient initialization
    // Proves: setMotionCueingPreset() populates all 6 channel biquad configs,
    //         resetMotionCueing() clears filter history before applying new preset,
    //         valid presets: off, gentle, moderate, aggressive, race_pro
    if (strncmp(data, "MCA:preset:", 11) == 0) {
        const char* name = data + 11;
        int preset = -1;
        if (strcmp(name, "off") == 0) preset = MCA_OFF;
        else if (strcmp(name, "gentle") == 0) preset = MCA_GENTLE;
        else if (strcmp(name, "moderate") == 0) preset = MCA_MODERATE;
        else if (strcmp(name, "aggressive") == 0) preset = MCA_AGGRESSIVE;
        else if (strcmp(name, "race_pro") == 0) preset = MCA_RACE_PRO;
        if (preset >= 0) {
            resetMotionCueing(&mcaConfig);
            setMotionCueingPreset(&mcaConfig, preset);
            serial_printf("MCA:preset=%s\r\n", mcaPresetName(mcaConfig.preset));
        } else {
            serial_printf("MCA:ERR unknown preset '%s'\r\n", name);
        }
        return;
    }

    // ── MCA:enabled:<0|1> — Master enable/disable for motion cueing ──
    // Tests:  MCA bypass path
    // Proves: When disabled (0), processMotionCueing() passes input through
    //         unfiltered; when enabled (1), biquad washout filters are active
    if (strncmp(data, "MCA:enabled:", 12) == 0) {
        mcaConfig.enabled = atoi(data + 12) ? 1 : 0;
        serial_printf("MCA:enabled=%d\r\n", mcaConfig.enabled);
        return;
    }

    // ── MCA:ch:<axis>:<param>:<value> — Per-channel filter parameter ─
    // Tests:  Individual axis filter tuning over serial
    // Proves: Runtime biquad coefficient recalculation (hp_fc, hp_Q, lp_fc,
    //         lp_Q), per-axis gain and rate limiter update, command parser
    //         handles nested colon-delimited format correctly
    //         e.g. MCA:ch:0:hp_fc:1.5  MCA:ch:3:gain:0.8
    if (strncmp(data, "MCA:ch:", 7) == 0) {
        int axis = -1;
        char param[16] = {0};
        float val = 0;
        char* p = data + 7;
        axis = atoi(p);
        p = strchr(p, ':');
        if (p) {
            p++;
            char* colon2 = strchr(p, ':');
            if (colon2) {
                int plen = colon2 - p;
                if (plen > 0 && plen < 16) {
                    strncpy(param, p, plen);
                    param[plen] = 0;
                }
                val = atof(colon2 + 1);
            }
        }
        if (axis >= 0 && axis < 6 && param[0]) {
            if (strcmp(param, "hp_fc") == 0) mcaSetChannelHpFc(&mcaConfig, axis, val);
            else if (strcmp(param, "hp_Q") == 0) mcaSetChannelHpQ(&mcaConfig, axis, val);
            else if (strcmp(param, "lp_fc") == 0) mcaSetChannelLpFc(&mcaConfig, axis, val);
            else if (strcmp(param, "lp_Q") == 0) mcaSetChannelLpQ(&mcaConfig, axis, val);
            else if (strcmp(param, "gain") == 0) mcaSetChannelGain(&mcaConfig, axis, val);
            else if (strcmp(param, "rate_limit") == 0) mcaSetChannelRateLimit(&mcaConfig, axis, val);
            else if (strcmp(param, "hp_en") == 0) mcaSetChannelHpEnabled(&mcaConfig, axis, (int)val);
            else if (strcmp(param, "lp_en") == 0) mcaSetChannelLpEnabled(&mcaConfig, axis, (int)val);
            else { serial_printf("MCA:ERR unknown param '%s'\r\n", param); return; }
            serial_printf("MCA:ch:%d:%s=%.4f\r\n", axis, param, val);
        } else {
            serial_printf("MCA:ERR bad ch command\r\n");
        }
        return;
    }

    // ── MCA:tilt:<param>:<value> — Tilt coordination parameters ──────
    // Tests:  Tilt coordination subsystem configuration
    // Proves: Sustained-acceleration tilt comp can be tuned live (enabled,
    //         surge_gain, sway_gain, fc, Q), filter state updates immediately
    if (strncmp(data, "MCA:tilt:", 9) == 0) {
        char* p = data + 9;
        char* colon = strchr(p, ':');
        if (colon) {
            char param[16] = {0};
            int plen = colon - p;
            if (plen > 0 && plen < 16) strncpy(param, p, plen);
            float val = atof(colon + 1);
            if (strcmp(param, "enabled") == 0) mcaSetTiltEnabled(&mcaConfig, (int)val);
            else if (strcmp(param, "surge_gain") == 0) mcaSetTiltSurgeGain(&mcaConfig, val);
            else if (strcmp(param, "sway_gain") == 0) mcaSetTiltSwayGain(&mcaConfig, val);
            else if (strcmp(param, "fc") == 0) mcaSetTiltFc(&mcaConfig, val);
            else if (strcmp(param, "Q") == 0) mcaSetTiltQ(&mcaConfig, val);
            else { serial_printf("MCA:ERR unknown tilt param '%s'\r\n", param); return; }
            serial_printf("MCA:tilt:%s=%.4f\r\n", param, val);
        }
        return;
    }

    // ── MCA:save — Persist MCA config to Non-Volatile Storage ────────
    // Tests:  NVS write path, config serialization
    // Proves: mcaSaveToNVS() writes full config blob to flash,
    //         survives power cycle, NVS partition is functional
    if (strcmp(data, "MCA:save") == 0) {
        int ret = mcaSaveToNVS(&mcaConfig);
        serial_printf("MCA:save=%s\r\n", ret == 0 ? "OK" : "FAIL");
        return;
    }

    // ── MCA:load — Load MCA config from NVS ─────────────────────────
    // Tests:  NVS read path, schema version check, fallback behavior
    // Proves: mcaLoadFromNVS() deserializes stored config correctly,
    //         falls back to initMotionCueing() defaults on version mismatch
    if (strcmp(data, "MCA:load") == 0) {
        int ret = mcaLoadFromNVS(&mcaConfig);
        if (ret != 0) {
            initMotionCueing(&mcaConfig, mcaConfig.sample_rate);
            serial_printf("MCA:load=FAIL (using defaults)\r\n");
        } else {
            serial_printf("MCA:load=OK preset=%s\r\n", mcaPresetName(mcaConfig.preset));
        }
        return;
    }

    // ── MCA:reset — Reset filter state (clear history, keep params) ──
    // Tests:  Biquad filter state reset without losing configuration
    // Proves: resetMotionCueing() zeroes all delay-line history (x[n-1],
    //         x[n-2], y[n-1], y[n-2]) so filters restart from silence
    if (strcmp(data, "MCA:reset") == 0) {
        resetMotionCueing(&mcaConfig);
        serial_printf("MCA:reset=OK\r\n");
        return;
    }

    // ── MCA:defaults — Reinitialize to factory defaults ──────────────
    // Tests:  Full MCA config reset path
    // Proves: initMotionCueing() restores all channels, tilt, preset to
    //         compile-time defaults; useful after bad manual tuning
    if (strcmp(data, "MCA:defaults") == 0) {
        initMotionCueing(&mcaConfig, mcaConfig.sample_rate);
        serial_printf("MCA:defaults=OK\r\n");
        return;
    }

    // ── WiFi serial commands ─────────────────────────────────────────
#ifdef ENABLE_WIFI
    // ── WIFI? — Query WiFi state (connection, IP, RSSI, SSID) ────────
    if (strcmp(data, "WIFI?") == 0) {
        serial_printf("WIFI:state=%s,ip=%s,rssi=%d,ssid=%s,udp_port=%d\r\n",
            wifi_transport_state_str(),
            wifi_transport_get_ip(),
            wifi_transport_get_rssi(),
            wifi_transport_get_ssid(),
            ETH_UDP_PORT);
        return;
    }

    // ── WIFI:SSID:<ssid> — Set WiFi SSID ─────────────────────────────
    if (strncmp(data, "WIFI:SSID:", 10) == 0) {
        const char *ssid = data + 10;
        wifi_transport_set_credentials(ssid, NULL);  // preserve existing password
        serial_printf("WIFI:SSID=%s\r\n", ssid);
        return;
    }

    // ── WIFI:PASS:<password> — Set WiFi password ─────────────────────
    if (strncmp(data, "WIFI:PASS:", 10) == 0) {
        const char *pass = data + 10;
        // Set password while keeping current SSID
        char current_ssid[33];
        strncpy(current_ssid, wifi_transport_get_ssid(), sizeof(current_ssid) - 1);
        current_ssid[sizeof(current_ssid) - 1] = '\0';
        wifi_transport_set_credentials(current_ssid, pass);
        serial_printf("WIFI:PASS=SET (%d chars)\r\n", (int)strlen(pass));
        return;
    }

    // ── WIFI:CRED:<ssid>:<password> — Set both SSID and password ─────
    if (strncmp(data, "WIFI:CRED:", 10) == 0) {
        char buf[98];
        strncpy(buf, data + 10, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char *sep = strchr(buf, ':');
        if (sep) {
            *sep = '\0';
            wifi_transport_set_credentials(buf, sep + 1);
            serial_printf("WIFI:CRED ssid=%s pass=SET\r\n", buf);
        } else {
            wifi_transport_set_credentials(buf, "");
            serial_printf("WIFI:CRED ssid=%s (open network)\r\n", buf);
        }
        return;
    }

    // ── WIFI:CONNECT — Connect with current credentials ──────────────
    if (strcmp(data, "WIFI:CONNECT") == 0) {
        wifi_transport_connect();
        serial_printf("WIFI:CONNECT initiated ssid=%s\r\n", wifi_transport_get_ssid());
        return;
    }

    // ── WIFI:DISCONNECT — Disconnect from WiFi ──────────────────────
    if (strcmp(data, "WIFI:DISCONNECT") == 0) {
        wifi_transport_disconnect();
        serial_printf("WIFI:DISCONNECT=OK\r\n");
        return;
    }

    // ── WIFI:SAVE — Save credentials to NVS for auto-connect on boot ─
    if (strcmp(data, "WIFI:SAVE") == 0) {
        int ret = wifi_transport_save_credentials();
        serial_printf("WIFI:SAVE=%s\r\n", ret == 0 ? "OK" : "FAIL");
        return;
    }
#endif // ENABLE_WIFI

    // ── BLE serial commands ──────────────────────────────────────────
#ifdef ENABLE_BLE
    // ── BLE? — Query BLE state ───────────────────────────────────────
    if (strcmp(data, "BLE?") == 0) {
        serial_printf("BLE:state=%s\r\n", ble_transport_state_str());
        return;
    }
#endif // ENABLE_BLE

    // ── CSV motion data fallback (legacy SimTools format) ────────────
    // Tests:  Legacy ASCII comma-separated motion path
    // Proves: Backward compatibility with older SimTools serial output,
    //         same mapRawToPosition → MCA → applyMotionValues pipeline
    //         as binary path, just slower parsing (atof per axis)
    float raw[6];
    char *tok = strtok(data, ",");
    int i = 0;
    while (tok != NULL && i < 6) {
        raw[i++] = atof(tok);
        tok = strtok(NULL, ",");
    }
    float mapped[6];
    mapRawToPosition(raw, &axisScales, maxRawInput, mapped);
    // Motion cueing filter (washout + tilt coordination)
    float filtered[6];
    processMotionCueing(&mcaConfig, mapped, filtered);
    applyMotionValues(filtered);
}

// State machine that auto-detects binary (0xAA 0x55 header) vs legacy CSV ('X' terminated)
void processIncomingByte(const uint8_t inByte) {
    // Binary packet state
    static uint8_t bin_buf[BIN_PAYLOAD_SIZE];
    static uint8_t bin_pos = 0;
    static uint8_t bin_state = 0;  // 0=idle, 1=got sync0, 2=collecting payload, 3=checksum

    // Legacy ASCII state
    static char input_line[MAX_SERIAL_INPUT];
    static unsigned int input_pos = 0;

    // Binary protocol state machine
    switch (bin_state) {
        case 0: // Waiting for first byte
            if (inByte == BIN_SYNC_0) {
                bin_state = 1;
                return;
            }
            break; // fall through to ASCII handler

        case 1: // Got 0xAA, expecting 0x55
            if (inByte == BIN_SYNC_1) {
                bin_state = 2;
                bin_pos = 0;
                return;
            }
            // Not a binary packet — push both bytes into ASCII buffer
            bin_state = 0;
            if (input_pos < (MAX_SERIAL_INPUT - 1))
                input_line[input_pos++] = BIN_SYNC_0;
            break; // fall through with current byte

        case 2: // Collecting payload bytes
            bin_buf[bin_pos++] = inByte;
            if (bin_pos >= BIN_PAYLOAD_SIZE) {
                bin_state = 3;
            }
            return;

        case 3: { // Checksum byte
            uint8_t xor_check = 0;
            for (int i = 0; i < BIN_PAYLOAD_SIZE; i++)
                xor_check ^= bin_buf[i];

            bin_state = 0;
            if (xor_check == inByte) {
                process_binary_packet(bin_buf);
            } else {
                DEBUG_PRINTF("BIN checksum fail: expected 0x%02X got 0x%02X\n", xor_check, inByte);
            }
            return;
        }
    }

    // Legacy ASCII path: accumulate until 'X' terminator
    if (inByte == 'X') {
        input_line[input_pos] = 0;
        process_data(input_line);
        input_pos = 0;
    } else {
        if (input_pos < (MAX_SERIAL_INPUT - 1))
            input_line[input_pos++] = inByte;
    }
}

void outputDebugData() {
    // Caller (InterfaceMonitorTask) already throttles at DEBUG_OUTPUT_INTERVAL (10Hz).
    // Telemetry is always sent (dashboard needs ESP32's IK angles for arm viz).
    // Debug text is only sent when debugEnabled.
    // Single fflush at the end pushes everything in one go.

    // Binary telemetry: 27-byte packet with servo angles (always)
    send_telemetry((const float*)lastServoAngles);

    // Text debug line (only when enabled)
    if (debugEnabled) {
        char buf[256];
        int n = snprintf(buf, sizeof(buf), "DEBUG,%lld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            (long long)micros(),
            (double)arr[0], (double)arr[1], (double)arr[2],
            (double)arr[3], (double)arr[4], (double)arr[5]);
        if (n > 0)
            fwrite(buf, 1, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1, stdout);
    }

    fflush(stdout);  // single flush pushes telemetry + debug text
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

    ESP_LOGI(TAG, "Stewart Platform Controller v%s (%s %s)",
        FW_VERSION_STRING, FW_BUILD_DATE, FW_BUILD_TIME);
  
    // Reconfigure watchdog (already auto-initialized by ESP-IDF 5.2 startup)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,        // Don't watch idle tasks
        .trigger_panic = true       // Panic on timeout
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));
  
    // Initialize Stewart platform configuration
    initDefaultStewartConfig(&stewartConfig);
    // Derive axis scales from actual IK workspace (no magic numbers)
    computeAxisScalesFromGeometry(&axisScales, &stewartConfig, 0.90f);
    ESP_LOGI(TAG, "Axis scales (from geometry, 90%% margin): %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
        axisScales.scale[0], axisScales.scale[1], axisScales.scale[2],
        axisScales.scale[3], axisScales.scale[4], axisScales.scale[5]);
    ESP_LOGI(TAG, "Drive train: encoder=%d, gear=%.1f, planetary=%.1f → %.2f steps/deg",
        stewartConfig.encoder_ppr, stewartConfig.virtual_gear,
        stewartConfig.planetary_ratio, stewartConfig.steps_per_degree);

    // Initialize motion cueing — try NVS first, fall back to defaults
    initMotionCueing(&mcaConfig, 60.0f);  // default 60Hz sample rate
    if (mcaLoadFromNVS(&mcaConfig) == 0) {
        DEBUG_PRINTF("MCA: Loaded saved config (preset=%s)\n", mcaPresetName(mcaConfig.preset));
    } else {
        DEBUG_PRINTLN("MCA: Using factory defaults");
    }

    // NOTE: E-Stop disabled — GPIO 20 (ESTOP_PIN) is USB D+ on ESP32-S3,
    // conflicts with USB Serial JTAG causing false triggers. Move ESTOP_PIN
    // to an unused GPIO before re-enabling.
    // initDebounceButton(&estop_button, (gpio_num_t)ESTOP_PIN, ESTOPDEBOUNCETIME);
    
    // Initialize motor control pins
    setupMotorPins();
    
    // Create mutex for thread safety
    xMutex = xSemaphoreCreateMutex();
    
    // E-stop task disabled — see ESTOP_PIN note above.
    // xTaskCreatePinnedToCore(
    //     EStopMonitorTask, "EStopMonitor", 10000, NULL,
    //     configMAX_PRIORITIES-1, NULL, 0);
    
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
    
#ifdef ENABLE_ETHERNET
    // Initialize W5500 SPI Ethernet + UDP listener
    // UDP packets feed into the same binary packet handler as serial
    if (!ethernet_transport_init(process_binary_packet)) {
        ESP_LOGW(TAG, "Ethernet init failed — serial-only mode");
    }
#endif

#ifdef ENABLE_WIFI
    // Initialize WiFi STA + UDP listener
    // WiFi credentials loaded from NVS (saved via WIFI:SAVE command)
    // UDP packets feed into the same binary packet handler as serial
    if (!wifi_transport_init(process_binary_packet)) {
        ESP_LOGW(TAG, "WiFi init failed — serial-only mode");
    }
#endif

#ifdef ENABLE_BLE
    // Initialize BLE GATT server
    // Advertises as "StewartPlatform", accepts binary motion packets via BLE write
    if (!ble_transport_init(process_binary_packet)) {
        ESP_LOGW(TAG, "BLE init failed — serial/WiFi only");
    }
#endif

    ESP_LOGI(TAG, "Setup Complete!");
    serial_println("READY");
    
    // app_main returns, tasks continue running
}
