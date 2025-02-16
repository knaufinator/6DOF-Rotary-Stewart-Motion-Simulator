#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <Arduino.h>
#include "MotorController.h"
#include "helpers.h"

class SerialInterface {
public:
    SerialInterface(MotorController& motorCtrl);
    void begin();
    void processIncomingByte(byte inByte);
    void monitorTask();

private:
    MotorController& motorController;
    char inputBuffer[MAX_SERIAL_INPUT];  // Using MAX_SERIAL_INPUT from helpers.h
    byte inputPos;
    
    void processData(char* data);
    void clearBuffer();
};

#endif // SERIAL_INTERFACE_H
