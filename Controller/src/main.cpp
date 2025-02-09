#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MCP23S17.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <Preferences.h>
#include "helpers.h"
#include "RMTMotorControl.h"

using namespace std;

// Global variables
MCP23S17 outputBank(MCP_CS_PIN);  // Using hardware SPI with default address 0
Bounce2::Button debouncedEStop = Bounce2::Button();
Preferences preferences;

// Motor control variables
RMTMotorControl* motors[6];
volatile float arr[6] = {0, 0, 0, 0, 0, 0};
static long servo_pos[6] = {0, 0, 0, 0, 0, 0};

// GPIO pins for motors (using direct ESP32 GPIO numbers)
const gpio_num_t stepPins[6] = {GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25};
const gpio_num_t stepPinsComplement[6] = {GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_28, GPIO_NUM_29, GPIO_NUM_30};
const gpio_num_t dirPins[6] = {GPIO_NUM_23, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_17};
const gpio_num_t dirPinsComplement[6] = {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_36, GPIO_NUM_39};
const rmt_channel_t channels[6] = {RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, 
                                  RMT_CHANNEL_3, RMT_CHANNEL_4, RMT_CHANNEL_5};

// Timing variables
unsigned long currentMicros = 0;
unsigned long previousMicros = 0;
int microInterval = MICRO_INTERVAL_FAST;

// State variables
bool isPausedEStop = false;
bool isRateLimiting = false;
SemaphoreHandle_t xMutex = NULL;

// Function declarations
void setupPWMpins();
void handleStepDirection();
void checkEStop();
void process_data(char * data);
void processIncomingByte(const byte inByte);
void InterfaceMonitorCode(void * pvParameters);
void GPIOLoop(void * pvParameters);

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
            Serial.printf("Motor %d position error: %d\n", i, motors[i]->getLastError());
        }
        xSemaphoreGive(xMutex);
    }
}

void setupPWMpins() {
    // Configure motor settings
    RMTMotorControl::Config motorConfig;
    motorConfig.stepPulseWidth_us = 1;      // 1µs pulse width
    motorConfig.dirSetupTime_us = 1;        // 1µs direction setup time
    motorConfig.minStepInterval_us = 2;     // 2µs minimum between steps (500kHz max)
    motorConfig.maxStepRate = 400000;       // 400kHz max step rate
    motorConfig.maxAcceleration = 50000;    // 50k steps/sec^2 acceleration
    motorConfig.enableSoftLimits = true;
    motorConfig.softLimitMin = -100000;     // Adjust these limits based on your setup
    motorConfig.softLimitMax = 100000;

    // Initialize RMT motor controls
    for(int i = 0; i < 6; i++) {
        motors[i] = new RMTMotorControl(stepPins[i], stepPinsComplement[i], dirPins[i], dirPinsComplement[i], channels[i]);
        if (!motors[i]->begin(motorConfig)) {
            Serial.printf("Failed to initialize motor %d, error: %d\n", i, motors[i]->getLastError());
        }
    }
}

void handleStepDirection() {
    //lock access to motor array
    xSemaphoreTake(xMutex, portMAX_DELAY);
    
    // Update each motor
    for (int i = 0; i < 6; i++) {
        if (!motors[i]->update()) {
            Serial.printf("Motor %d update error: %d\n", i, motors[i]->getLastError());
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
    }
}

void GPIOLoop(void * pvParameters) {
    for(;;) {
        handleStepDirection();
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
    char * tok;
    float temp;
    int i = 0;
    float arrRaw[6] = {0,0,0, 0,0,0};
    float arrRateLimited[6] = {0,0,0, 0,0,0};
    
    tok = strtok(data, ",");
    
    while (tok != NULL && i < 6) {
        float value = atof(tok);
        
        if(i == 2)
            temp = mapfloat(value, 0, 4094, -7, 7);//hieve 
        else if(i > 2)//rotations, pitch,roll,yaw
            temp = mapfloat(value, 0, 4094, -30, 30) * (PI/180.0);
        else//sway,surge
            temp = mapfloat(value, 0, 4094, -8, 8); 
        
        arrRaw[i++] = temp;
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
            }
            
            //set position
            memcpy((void*)arr, arrRateLimited, sizeof(arr));
        } else {
            //set position directly
            memcpy((void*)arr, arrRaw, sizeof(arr));
        }
        
        //calculate and set new position
        setPos();
    }
}

void processIncomingByte(const byte inByte) {
    static char input_line[MAX_SERIAL_INPUT];
    static unsigned int input_pos = 0;
    
    switch (inByte) {
        case 'X':   // end of text
            input_line[input_pos] = 0;  // terminating null byte
            process_data(input_line);          
            input_pos = 0;  
            break;
            
        case '\r':   // discard carriage return
            break;
            
        default:
            //buffer data
            if (input_pos < (MAX_SERIAL_INPUT - 1))
                input_line[input_pos++] = inByte;
            break;
    }
}

void setup() {
 
  Serial.begin(115200); 
  Wire.begin();
  
  // Initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  // Initialize SPI
  SPI.begin();

  // Initialize MCP23S17 chips
  if (!outputBank.begin()) {
    Serial.println("Error initializing MCP23S17.");
    while (1);
  }
  
  // Configure all pins as outputs
  outputBank.pinMode16(0x0000);  // Set all pins to OUTPUT mode
  
  // Configure E-Stop button with debouncing
  pinMode(ESTOPPIN, INPUT_PULLUP);
  debouncedEStop.attach(ESTOPPIN);
  debouncedEStop.interval(ESTOPDEBOUNCETIME);
  debouncedEStop.setPressedState(LOW);
  
  // Initialize motor control pins
  setupPWMpins();
  
  // Create mutex for thread safety
  xMutex = xSemaphoreCreateMutex();
  
  // Create tasks for interface monitoring and GPIO control
  xTaskCreatePinnedToCore(
                    InterfaceMonitorCode,   /* Task function. */
                    "InterfaceMonitor", /* name of task. */
                    10000,                   /* Stack size of task */
                    NULL,                   /* parameter of the task */
                    1,                      /* priority of the task */
                    NULL,  /* Task handle to keep track of created task */
                    0);                     /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    GPIOLoop,              /* Task function. */
                    "GPIOLoop",           /* name of task. */
                    10000,                 /* Stack size of task */
                    NULL,                 /* parameter of the task */
                    1,                    /* priority of the task */
                    NULL,        /* Task handle to keep track of created task */
                    1);                   /* pin task to core 1 */

  Serial.println("Setup Complete!");
}
