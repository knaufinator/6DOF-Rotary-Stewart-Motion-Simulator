#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP23X17.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <Preferences.h>
#include "helpers.h"

using namespace std;

// Constants
#ifndef MICRO_INTERVAL_FAST
#define MICRO_INTERVAL_FAST 100
#endif

#ifndef MICRO_INTERVAL_SLOW
#define MICRO_INTERVAL_SLOW 10000
#endif

#ifndef TIMER_INTERVAL
#define TIMER_INTERVAL 1000
#endif

#ifndef ESTOPPIN
#define ESTOPPIN 4
#endif

#ifndef MAX_SERIAL_INPUT
#define MAX_SERIAL_INPUT 60
#endif

#ifndef EEPROM_SIZE
#define EEPROM_SIZE 512
#endif

#ifndef ESTOPDEBOUNCETIME
#define ESTOPDEBOUNCETIME 5
#endif

#ifndef INV1
#define INV1 0
#endif

#ifndef INV2
#define INV2 1
#endif

#ifndef INV3
#define INV3 2
#endif

#ifndef servo_min
#define servo_min -60
#endif

#ifndef servo_max
#define servo_max 60
#endif

#ifndef servoPulseMultiplierPerRadian
#define servoPulseMultiplierPerRadian 1
#endif

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679
#endif

// Global variables
Adafruit_MCP23X17 outputBank;
Bounce2::Button debouncedEStop = Bounce2::Button();
Preferences preferences;

// Motor control variables
acServo motors[6];
volatile float arr[6] = {0, 0, 0, 0, 0, 0};
static long servo_pos[6] = {0, 0, 0, 0, 0, 0};
int stepPins[6] = {8, 9, 10, 11, 12, 13};
int dirPins[6] = {0, 1, 2, 3, 4, 5};
uint16_t motorStepDirValue = 0;
uint16_t motorStepDirValue2 = 0;

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
    for(int i = 0; i < 6; i++)
    {    
        long x = 0;
        float alpha = getAlpha(i,arr);

        if(alpha >= servo_min && alpha <= servo_max)
        {
            //this takes the Radian angle, and scales that value to pulse position.
            //This is calibrated to the real world. with a 50:1 gear, and instructed to move +-60 degrees and finding a servoPulseMultiplierPerRadian that makes that happen.
            if(i==INV1||i==INV2||i==INV3){
                x = -(alpha)*servoPulseMultiplierPerRadian;
            }
            else{
                x = (alpha)*servoPulseMultiplierPerRadian;
            }
    
            servo_pos[i] = x;            
        }     
    }

    //lock access to motor array
    xSemaphoreTake( xMutex, portMAX_DELAY );
    
    for(int i = 0; i < 6; i++)
    {
        motors[i].targetpos = servo_pos[i];
    }   
    
    //give up lock
    xSemaphoreGive( xMutex );     
}

void setupPWMpins() {
    // Set up the pins for step and direction control
    for (int i = 0; i < 6; i++) {
        outputBank.pinMode(stepPins[i], OUTPUT);
        outputBank.pinMode(dirPins[i], OUTPUT);
    }
}

void handleStepDirection() {
    currentMicros = micros();
    int dif = currentMicros - previousMicros;
    
    if (dif >= microInterval) {
        //lock access to motor array
        xSemaphoreTake(xMutex, portMAX_DELAY);
        
        //clear output values
        motorStepDirValue = 0;
        motorStepDirValue2 = 0;
        
        //build output values for each motor
        for (int i = 0; i < 6; i++) {
            //check if we need to move motor
            if (motors[i].currentpos != motors[i].targetpos) {
                //set direction pin based on if we need to move up or down
                if (motors[i].currentpos < motors[i].targetpos) {
                    motorStepDirValue |= (1 << dirPins[i]);
                    motors[i].currentpos++;
                } else {
                    motors[i].currentpos--;
                }
                //set step pin
                motorStepDirValue2 |= (1 << stepPins[i]);
            }
        }
        
        //give up lock
        xSemaphoreGive(xMutex);
        
        //output direction pins first
        outputBank.writeGPIOAB(motorStepDirValue);
        
        //wait a bit
        delayMicroseconds(2);
        
        //output step pins
        outputBank.writeGPIOAB(motorStepDirValue2);
        
        previousMicros = currentMicros;
    }
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
  if (!outputBank.begin_I2C()) {
    Serial.println("Error initializing MCP23017.");
    while (1);
  }
  
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
