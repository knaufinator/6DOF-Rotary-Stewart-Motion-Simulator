#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <MCP23S17.h>
#include "helpers.h"

class MotorController {
public:
    MotorController();
    void begin();
    void setupPins();
    void handleStepDirection();
    void setTargetPositions(volatile float positions[6]);
    bool isPaused() const { return isPausedEStop; }
    void setPaused(bool paused) { isPausedEStop = paused; }

private:
    MCP23S17 outputBank;
    acServo motors[6];
    long servo_pos[6];
    int stepPins[6];
    int dirPins[6];
    uint16_t motorStepDirValue;
    uint16_t motorStepDirValue2;
    unsigned long currentMicros;
    unsigned long previousMicros;
    int microInterval;
    bool isPausedEStop;
    bool isRateLimiting;
    SemaphoreHandle_t xMutex;

    void initializePins();
    void updateMotorPositions();
};

#endif // MOTOR_CONTROLLER_H
