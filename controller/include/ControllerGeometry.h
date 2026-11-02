#pragma once
#include <cstring>

namespace controller {
// Storage layout, persistence and scale recomputation remain with each backend.
template<class Config>
bool set_geometry_parameter(Config& config, const char* key, float value) {
    if (std::strcmp(key, "RD") == 0) config.RD = value;
    else if (std::strcmp(key, "PD") == 0) config.PD = value;
    else if (std::strcmp(key, "L1") == 0) config.ServoArmLengthL1 = value;
    else if (std::strcmp(key, "L2") == 0) config.ConnectingArmLengthL2 = value;
    else if (std::strcmp(key, "height") == 0) config.platformHeight = value;
    else if (std::strcmp(key, "theta_r") == 0) config.theta_r = value;
    else if (std::strcmp(key, "theta_p") == 0) config.theta_p = value;
    else return false;
    return true;
}
}
