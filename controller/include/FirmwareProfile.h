#pragma once

// Host integration defaults to the locked full-controller protocol.
#ifndef FIRMWARE_PROFILE_MINI
#define FIRMWARE_PROFILE_MINI 0
#endif
#ifndef FIRMWARE_PROFILE_NAME
#define FIRMWARE_PROFILE_NAME "r14"
#endif

namespace controller {
inline const char* profile_name() { return FIRMWARE_PROFILE_NAME; }
inline const char* actuator_name() {
    return FIRMWARE_PROFILE_MINI ? "pwm-servo" : "step-dir";
}
}
