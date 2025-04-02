#include "InverseKinematics.h"

// Helper macro to convert degrees to radians
#define radians(deg) ((deg)*IK_DEG_TO_RAD)

/**
 * Calculate servo angle for a specific servo based on platform position and orientation
 */
float calculateServoAngle(int servoIndex, const float position[6], const StewartConfig* config) {
    // Arrays for platform & base coordinates calculation
    float platformPDx, platformPDy, platformAngle;
    float platformCoordsx, platformCoordsy;
    float basePDx, basePDy, baseAngle;
    float baseCoordsx, baseCoordsy;
    float DxMultiplier[6] = {1, 1, 1, -1, -1, -1};
    float AngleMultiplier[6] = {1, -1, 1, 1, -1, 1};
    float OffsetAngle[6] = {IK_PI/6, IK_PI/6, -IK_PI/2, -IK_PI/2, IK_PI/6, IK_PI/6};
    
    // Platform pivots
    float platformPivotx, platformPivoty, platformPivotz;
    
    float deltaLx, deltaLy, deltaLz, deltaL2Virtual;
    float l, m, n;
    
    // Bounds checking
    if (servoIndex < 0 || servoIndex > 5 || config == NULL) {
        return 0.0f;
    }
    
    // Calculate platform coordinates
    platformPDx = DxMultiplier[servoIndex] * config->RD;
    platformPDy = config->RD;
    platformAngle = OffsetAngle[servoIndex] + AngleMultiplier[servoIndex] * radians(config->theta_r);
    platformCoordsx = platformPDx * cos(platformAngle);
    platformCoordsy = platformPDy * sin(platformAngle);
    
    // Calculate base coordinates
    basePDx = DxMultiplier[servoIndex] * config->PD;
    basePDy = config->PD;
    baseAngle = OffsetAngle[servoIndex] + AngleMultiplier[servoIndex] * radians(config->theta_p);
    baseCoordsx = basePDx * cos(baseAngle);
    baseCoordsy = basePDy * sin(baseAngle);
    
    // Platform pivot positions based on position and orientation
    platformPivotx = platformCoordsx * cos(position[3]) * cos(position[5]) + 
                    platformCoordsy * (sin(position[4]) * sin(position[3]) * cos(position[3]) - cos(position[4]) * sin(position[5])) + 
                    position[0];
                    
    platformPivoty = platformCoordsx * cos(position[4]) * sin(position[5]) + 
                    platformCoordsy * (cos(position[3]) * cos(position[5]) + sin(position[3]) * sin(position[4]) * sin(position[5])) + 
                    position[1];
                    
    platformPivotz = -platformCoordsx * sin(position[3]) + 
                    platformCoordsy * sin(position[4]) * cos(position[3]) + 
                    config->platformHeight + position[2];
    
    // Calculate deltas between base and platform
    deltaLx = baseCoordsx - platformPivotx;
    deltaLy = baseCoordsy - platformPivoty;
    deltaLz = -platformPivotz;
    
    deltaL2Virtual = sqrt(pow(deltaLx, 2.0) + pow(deltaLy, 2.0) + pow(deltaLz, 2.0));

    // Calculate servo angle using inverse kinematics
    l = pow(deltaL2Virtual, 2.0) - (pow(config->ConnectingArmLengthL2, 2.0) - pow(config->ServoArmLengthL1, 2.0));
    m = 2 * config->ServoArmLengthL1 * (platformPivotz);
    n = 2 * config->ServoArmLengthL1 * (cos(config->theta_s[servoIndex] * IK_PI/180) * (platformPivotx - baseCoordsx) + 
                                     sin(config->theta_s[servoIndex] * IK_PI/180) * (platformPivoty - baseCoordsy));

    return asin(l / (sqrt(pow(m, 2.0) + pow(n, 2.0)))) - atan(n / m);
}

/**
 * Calculate all servo angles based on platform position and orientation
 */
void calculateAllServoAngles(const float position[6], const StewartConfig* config, float servoAngles[6]) {
    // Bounds checking
    if (config == NULL || servoAngles == NULL) {
        return;
    }
    
    // Calculate angle for each servo
    for (int i = 0; i < 6; i++) {
        servoAngles[i] = calculateServoAngle(i, position, config);
    }
}

/**
 * Initialize a StewartConfig structure with default values
 */
void initDefaultStewartConfig(StewartConfig* config) {
    if (config == NULL) {
        return;
    }
    
    // Default values from original implementation
    config->theta_r = 10.0f;
    config->theta_s[0] = 150.0f;
    config->theta_s[1] = -90.0f;
    config->theta_s[2] = 30.0f;
    config->theta_s[3] = 150.0f;
    config->theta_s[4] = -90.0f;
    config->theta_s[5] = 30.0f;
    config->theta_p = 30.0f;
    config->RD = 15.75f;
    config->PD = 16.0f;
    config->ServoArmLengthL1 = 7.25f;
    config->ConnectingArmLengthL2 = 28.5f;
    config->platformHeight = 25.5170749f;
}
