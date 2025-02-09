#include "SerialInterface.h"
#include <string.h>

SerialInterface::SerialInterface(MotorController& motorCtrl)
    : motorController(motorCtrl)
    , inputPos(0)
{
    memset(inputBuffer, 0, MAX_INPUT);
}

void SerialInterface::begin() {
    Serial.begin(115200);
}

void SerialInterface::processIncomingByte(byte inByte) {
    switch (inByte) {
        case '\n':                   // end of text
            inputBuffer[inputPos] = 0;  // terminating null byte
            processData(inputBuffer);    // terminating null byte
            clearBuffer();
            break;

        case '\r':                   // discard carriage return
            break;

        default:
            if (inputPos < (MAX_INPUT - 1)) {
                inputBuffer[inputPos++] = inByte;
            }
            break;
    }
}

void SerialInterface::processData(char* data) {
    volatile float positions[6] = {0, 0, 0, 0, 0, 0};
    char* token = strtok(data, ",");
    int index = 0;
    
    Serial.println("Received data:");  // Debug print
    
    while (token != NULL && index < 6) {
        positions[index] = atof(token);
        Serial.print(positions[index]); // Debug print
        Serial.print(" ");  // Debug print
        index++;
        token = strtok(NULL, ",");
    }
    Serial.println();  // Debug print
    
    if (index == 6) {
        Serial.println("Setting target positions"); // Debug print
        motorController.setTargetPositions(positions);
    }
}

void SerialInterface::clearBuffer() {
    memset(inputBuffer, 0, MAX_INPUT);
    inputPos = 0;
}

void SerialInterface::monitorTask() {
    while (Serial.available() > 0) {
        processIncomingByte(Serial.read());
    }
}
