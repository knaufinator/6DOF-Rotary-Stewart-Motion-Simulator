#include "SafetyController.h"

SafetyController::SafetyController(MotorController& motorCtrl)
    : motorController(motorCtrl)
    , eStopActive(false)
{
}

void SafetyController::begin() {
    setupEStop();
}

void SafetyController::setupEStop() {
    // Initialize debounced E-Stop button
    debouncedEStop.attach(ESTOP_PIN, INPUT_PULLUP);
    debouncedEStop.interval(ESTOPDEBOUNCETIME);
    debouncedEStop.setPressedState(LOW);
}

void SafetyController::check() {
    debouncedEStop.update();
    
    if (debouncedEStop.pressed()) {
        eStopActive = true;
        motorController.setPaused(true);
    } else if (debouncedEStop.released()) {
        eStopActive = false;
        motorController.setPaused(false);
    }
}
