#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#include <Arduino.h>
#include <Bounce2.h>
#include "MotorController.h"

class SafetyController {
public:
    SafetyController(MotorController& motorCtrl);
    void begin();
    void check();
    bool isEStopActive() const { return eStopActive; }

private:
    MotorController& motorController;
    Bounce2::Button debouncedEStop;
    bool eStopActive;

    void setupEStop();
};

#endif // SAFETY_CONTROLLER_H
