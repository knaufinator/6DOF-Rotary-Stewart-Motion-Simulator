#include "AxisScaling.h"

void computeAxisScalesFromGeometry(AxisScaleConfig* config, const StewartConfig* stewart, float margin) {
    if (!config || !stewart) return;

    // is_angle flags: translation axes 0-2, rotation axes 3-5
    config->is_angle[0] = 0;  // surge (mm)
    config->is_angle[1] = 0;  // sway  (mm)
    config->is_angle[2] = 0;  // heave (mm)
    config->is_angle[3] = 1;  // roll  (deg → rad)
    config->is_angle[4] = 1;  // pitch (deg → rad)
    config->is_angle[5] = 1;  // yaw   (deg → rad)

    // Upper bounds for binary search:
    //   Translation: no platform moves more than L1 + L2
    //   Rotation: search in degrees, convert to radians for IK probe
    float upper[6];
    float max_linear = stewart->ServoArmLengthL1 + stewart->ConnectingArmLengthL2;
    upper[0] = max_linear;  // surge mm
    upper[1] = max_linear;  // sway  mm
    upper[2] = max_linear;  // heave mm
    upper[3] = 90.0f;       // roll  deg
    upper[4] = 90.0f;       // pitch deg
    upper[5] = 90.0f;       // yaw   deg

    // Binary search each axis independently from home [0,0,0,0,0,0]
    float limits[AXIS_COUNT];
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        float lo = 0.0f;
        float hi = upper[axis];

        for (int iter = 0; iter < 50; iter++) {
            float mid = (lo + hi) * 0.5f;
            float pos[6] = {0, 0, 0, 0, 0, 0};

            // Set probe value: rotation axes need deg→rad for IK input
            if (config->is_angle[axis]) {
                pos[axis] = mid * (float)(M_PI / 180.0);
            } else {
                pos[axis] = mid;
            }

            if (validatePosition(pos, stewart) == 0) {
                lo = mid;  // still reachable, push higher
            } else {
                hi = mid;  // out of range, pull back
            }
        }
        limits[axis] = lo;
    }

    // Symmetrical platform → uniform scaling per group.
    // Use the tightest (minimum) limit so no axis ever exceeds workspace.
    float min_linear = limits[0];
    for (int i = 1; i < 3; i++) {
        if (limits[i] < min_linear) min_linear = limits[i];
    }
    float min_angular = limits[3];
    for (int i = 4; i < 6; i++) {
        if (limits[i] < min_angular) min_angular = limits[i];
    }

    // Apply safety margin, round to whole number
    float linear_scale = floorf(min_linear * margin);
    float angular_scale = floorf(min_angular * margin);

    for (int i = 0; i < 3; i++) config->scale[i] = linear_scale;
    for (int i = 3; i < 6; i++) config->scale[i] = angular_scale;
}

void mapRawToPosition(const float raw[6], const AxisScaleConfig* config, float max_raw, float position[6]) {
    float home = (float)((int)max_raw / 2);  // 2047 for 4094 or 4095, 127 for 255
    for (int i = 0; i < AXIS_COUNT; i++) {
        float s = config->scale[i];
        // Center on integer home value: raw==home → 0, raw==0 → -s, raw==max_raw → ~+s
        position[i] = (raw[i] - home) * (s / home);
        if (config->is_angle[i]) {
            position[i] *= (float)(M_PI / 180.0);
        }
    }
}
