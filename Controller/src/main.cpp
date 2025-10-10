#include <Arduino.h>
#include <Bounce2.h>
#include <Preferences.h>
#include "helpers.h"
#include "debug_uart.h"
#include <RMTMotorControl.h>
#include <GPTimerScheduler.h>
#include <esp_task_wdt.h>

using namespace std;

// Global variables
Bounce2::Button debouncedEStop = Bounce2::Button();
Preferences preferences;

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
unsigned long currentMicros = 0;
unsigned long previousMicros = 0;
unsigned long lastDebugOutput = 0;  // For tracking debug output timing
int microInterval = MICRO_INTERVAL_FAST;
const unsigned long DEBUG_OUTPUT_INTERVAL = 100000; // 100ms = 10Hz debug output

// State variables
bool isPausedEStop = false;
bool isRateLimiting = false;
bool debugEnabled = false;  // Debug output state, default disabled for safety
SemaphoreHandle_t xMutex = NULL;
TaskHandle_t xGPIOLoopHandle = NULL;  // Task handle for GPTimer notification

// Function declarations
void setupPWMpins();
void handleStepDirection();
void checkEStop();
void process_data(char * data);
void processIncomingByte(const byte inByte);
void InterfaceMonitorCode(void * pvParameters);
void GPIOLoop(void * pvParameters);
void EStopMonitorCode(void * pvParameters);
void outputDebugData();  // New debug output function

void setPos(){  
    //Platform and Base Coords
    for(int i = 0; i < 6; i++) {    
        long x = 0;
        float alpha = getAlpha(i,arr);
        
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

void setupPWMpins() {
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
        pinMode(stepPins[i], OUTPUT);
        pinMode(dirPins[i], OUTPUT);
        digitalWrite(stepPins[i], LOW);
        digitalWrite(dirPins[i], LOW);
        
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

void loop() {
    checkEStop();
}

void InterfaceMonitorCode(void * pvParameters) {
    for(;;) {
        if (Serial.available() > 0)
            processIncomingByte(Serial.read());
        
        checkEStop();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void GPIOLoop(void * pvParameters) {
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

void EStopMonitorCode(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        // Update watchdog to indicate E-stop monitoring is alive
        esp_task_wdt_reset();
        
        // Update button status through debounce filter
        debouncedEStop.update();
        
        if (debouncedEStop.fell()) {  // Button pressed (transition to active state)
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
        
        if (debouncedEStop.rose()) {  // Button released
            // Don't automatically resume - require explicit reset
            isRateLimiting = true;
            DEBUG_PRINTLN("E-STOP RELEASED - Reset required");
            
            // Restart GPTimer when E-stop is cleared and system is ready
            if (xGPIOLoopHandle != NULL) {
                GPTimerScheduler::start(xGPIOLoopHandle);
                DEBUG_PRINTLN("GPTimer restarted after E-stop release");
            }
        }
        
        // Check E-stop state more frequently than debounce time
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ESTOP_CHECK_INTERVAL_MS));
    }
}

void checkEStop() {
    //Update button status through debounce filter
    debouncedEStop.update();
    
    if (debouncedEStop.fell()) {  // Call code if button transitions from HIGH to LOW
        isPausedEStop = true;
    }
    
    if (debouncedEStop.rose()) {  // Call code if button transitions from HIGH to LOW
        isPausedEStop = false;
        isRateLimiting = true;
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
            temp = mapfloat(value, 0, 4094, -30, 30) * (PI/180.0);
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
                if(abs(arrRaw[i] - arr[i]) > 0.01) {
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

void processIncomingByte(const byte inByte) {
    static char input_line[MAX_SERIAL_INPUT];
    static unsigned int input_pos = 0;
    static unsigned long lastMessageTime = 0;
    unsigned long currentTime;
    unsigned long timeSinceLastMessage;
    
    switch (inByte) {
        case 'X':   // end of text
            input_line[input_pos] = 0;  // terminating null byte
            
            currentTime = millis();
            timeSinceLastMessage = currentTime - lastMessageTime;
            DEBUG_PRINTF("Message interval: %lums\n", timeSinceLastMessage);
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

void setup() {
 
    Serial.begin(115200);
    DEBUG_PRINTLN("Starting up...");
  
  // Initialize watchdog first
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000.0, true); // 3 second timeout, panic on timeout
  

  // Configure E-Stop button with debouncing
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  debouncedEStop.attach(ESTOP_PIN);
  debouncedEStop.interval(ESTOPDEBOUNCETIME);
  debouncedEStop.setPressedState(ESTOP_ACTIVE_STATE);
  
  // Initialize motor control pins
  setupPWMpins();
  
  // Create mutex for thread safety
  xMutex = xSemaphoreCreateMutex();
  
  // Create high-priority E-stop monitoring task
  xTaskCreatePinnedToCore(
        EStopMonitorCode,    /* Task function. */
        "EStopMonitor",      /* name of task. */
        10000,               /* Stack size of task */
        NULL,                /* parameter of the task */
        configMAX_PRIORITIES-1, /* Highest priority */
        NULL,                /* Task handle */
        0);                  /* pin task to core 0 */
  
  // Create tasks for interface monitoring and GPIO control with proper stack sizes
  xTaskCreatePinnedToCore(
                    InterfaceMonitorCode,   /* Task function. */
                    "InterfaceMonitor",     /* name of task. */
                    8192,                   /* Stack size of task */
                    NULL,                   /* parameter of the task */
                    2,                      /* priority of the task */
                    NULL,                   /* Task handle */
                    0);                     /* pin task to core 0 */
  
  xTaskCreatePinnedToCore(
                    GPIOLoop,              /* Task function. */
                    "GPIOLoop",            /* name of task. */
                    8192,                  /* Stack size of task */
                    NULL,                  /* parameter of the task */
                    3,                     /* priority of the task */
                    NULL,                  /* Task handle */
                    1);                    /* pin task to core 1 */
  
    DEBUG_PRINTLN("Setup Complete!");
}

void outputDebugData() {
    if (!debugEnabled) return;  // Skip if debug output is disabled
    
    unsigned long now = micros();
    
    // Only output if enough time has passed (10Hz)
    if (now - lastDebugOutput >= DEBUG_OUTPUT_INTERVAL) {
        lastDebugOutput = now;
        
        // Output format: DEBUG,timestamp,angles[6],targetX,targetY,targetZ,rotX,rotY,rotZ
        DEBUG_PRINTF("DEBUG,%lu,", now);
        
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
