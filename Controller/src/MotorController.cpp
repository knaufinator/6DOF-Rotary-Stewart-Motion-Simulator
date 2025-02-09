#include "MotorController.h"

MotorController::MotorController() 
    : outputBank(MCP_CS_PIN)
    , servo_pos{0, 0, 0, 0, 0, 0}
    , stepPins{8, 9, 10, 11, 12, 13}
    , dirPins{0, 1, 2, 3, 4, 5}
    , motorStepDirValue(0)
    , motorStepDirValue2(0)
    , currentMicros(0)
    , previousMicros(0)
    , microInterval(MICRO_INTERVAL_FAST)
    , isPausedEStop(false)
    , isRateLimiting(false)
    , xMutex(NULL)
{
    xMutex = xSemaphoreCreateMutex();
}

void MotorController::begin() {
    initializePins();
}

void MotorController::setupPins() {
    // Set all pins to OUTPUT mode at once
    outputBank.pinMode16(0x0000);  // Set all pins to OUTPUT mode
    
    // Initialize all pins to LOW
    for (uint8_t i = 0; i < 16; i++) {
        outputBank.write1(i, 0);  // Write 0 to each pin
    }
}

void MotorController::handleStepDirection() {
    currentMicros = micros();
    int dif = currentMicros - previousMicros;
    
    if (dif >= microInterval) {
        previousMicros = currentMicros;
        
        if (!isPausedEStop) {
            xSemaphoreTake(xMutex, portMAX_DELAY);
            
            for (int i = 0; i < 6; i++) {
                long error = motors[i].targetpos - motors[i].currentpos;
                
                if (error != 0) {
                    // Set direction based on error
                    if (error > 0) {
                        BIT_SET(motorStepDirValue, dirPins[i]);
                    } else {
                        BIT_CLEAR(motorStepDirValue, dirPins[i]);
                    }
                    
                    // Set step pin
                    BIT_SET(motorStepDirValue, stepPins[i]);
                    
                    // Update current position
                    if (error > 0) {
                        motors[i].currentpos++;
                    } else {
                        motors[i].currentpos--;
                    }
                }
            }
            
            xSemaphoreGive(xMutex);
            
            // Write all values to output bank
            for (uint8_t pin = 0; pin < 16; pin++) {
                bool pinState = (motorStepDirValue & (1 << pin)) != 0;
                outputBank.write1(pin, pinState);
            }
            
            // Clear step pins for next cycle
            for (int i = 0; i < 6; i++) {
                BIT_CLEAR(motorStepDirValue, stepPins[i]);
            }
        }
    }
}

void MotorController::setTargetPositions(volatile float positions[6]) {
    for(int i = 0; i < 6; i++) {    
        long x = 0;
        float alpha = getAlpha(i, positions);

        if(alpha >= servo_min_rad && alpha <= servo_max_rad) {
            //this takes the Radian angle, and scales that value to pulse position.
            if(i==INV1 || i==INV2 || i==INV3) {
                x = -(alpha)*servoPulseMultiplierPerRadian;
            } else {
                x = (alpha)*servoPulseMultiplierPerRadian;
            }
            servo_pos[i] = x;            
        }     
    }

    xSemaphoreTake(xMutex, portMAX_DELAY);
    for(int i = 0; i < 6; i++) {
        motors[i].targetpos = servo_pos[i];
    }   
    xSemaphoreGive(xMutex);     
}

void MotorController::initializePins() {
    // Initialize the MCP23S17 and set up pins
    setupPins();
    
    // Initialize motor structures
    for(int i = 0; i < 6; i++) {
        motors[i].stepPin = stepPins[i];
        motors[i].dirPin = dirPins[i];
        motors[i].currentpos = 0;
        motors[i].targetpos = 0;
    }
}
