#include "helpers.h"

// Helper function to map float values
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Rate limiting function to smooth transitions
float rateLimit(float target, float current) {
    float diff = target - current;
    float maxChange = 0.01; // Maximum change allowed per update
    
    if(diff > maxChange)
        return current + maxChange;
    else if(diff < -maxChange)
        return current - maxChange;
    else
        return target;
}
