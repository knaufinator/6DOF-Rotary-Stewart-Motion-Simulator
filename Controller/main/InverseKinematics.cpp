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
    float OffsetAngle[6] = {(float)(IK_PI/6), (float)(IK_PI/6), (float)(-IK_PI/2), (float)(-IK_PI/2), (float)(IK_PI/6), (float)(IK_PI/6)};
    
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
    platformAngle = OffsetAngle[servoIndex] + AngleMultiplier[servoIndex] * (float)radians(config->theta_r);
    platformCoordsx = platformPDx * cosf(platformAngle);
    platformCoordsy = platformPDy * sinf(platformAngle);
    
    // Calculate base coordinates
    basePDx = DxMultiplier[servoIndex] * config->PD;
    basePDy = config->PD;
    baseAngle = OffsetAngle[servoIndex] + AngleMultiplier[servoIndex] * (float)radians(config->theta_p);
    baseCoordsx = basePDx * cosf(baseAngle);
    baseCoordsy = basePDy * sinf(baseAngle);
    
    // Platform pivot positions based on position and orientation
    platformPivotx = platformCoordsx * cosf(position[3]) * cosf(position[5]) + 
                    platformCoordsy * (sinf(position[4]) * sinf(position[3]) * cosf(position[3]) - cosf(position[4]) * sinf(position[5])) + 
                    position[0];
                    
    platformPivoty = platformCoordsx * cosf(position[4]) * sinf(position[5]) + 
                    platformCoordsy * (cosf(position[3]) * cosf(position[5]) + sinf(position[3]) * sinf(position[4]) * sinf(position[5])) + 
                    position[1];
                    
    platformPivotz = -platformCoordsx * sinf(position[3]) + 
                    platformCoordsy * sinf(position[4]) * cosf(position[3]) + 
                    config->platformHeight + position[2];
    
    // Calculate deltas between base and platform
    deltaLx = baseCoordsx - platformPivotx;
    deltaLy = baseCoordsy - platformPivoty;
    deltaLz = -platformPivotz;
    
    deltaL2Virtual = sqrtf(deltaLx * deltaLx + deltaLy * deltaLy + deltaLz * deltaLz);

    // Calculate servo angle using inverse kinematics
    l = deltaL2Virtual * deltaL2Virtual - (config->ConnectingArmLengthL2 * config->ConnectingArmLengthL2 - config->ServoArmLengthL1 * config->ServoArmLengthL1);
    m = 2.0f * config->ServoArmLengthL1 * platformPivotz;
    n = 2.0f * config->ServoArmLengthL1 * (cosf(config->theta_s[servoIndex] * (float)(IK_PI / 180.0)) * (platformPivotx - baseCoordsx) + 
                                           sinf(config->theta_s[servoIndex] * (float)(IK_PI / 180.0)) * (platformPivoty - baseCoordsy));

    // Clamp asin argument to [-1, 1] to prevent NaN when outside workspace
    float asinArg = l / sqrtf(m * m + n * n);
    if (asinArg > 1.0f) asinArg = 1.0f;
    if (asinArg < -1.0f) asinArg = -1.0f;

    float angle = asinf(asinArg) - atanf(n / m);

    // Clamp to physical servo limits
    if (angle > IK_SERVO_MAX_RAD) angle = IK_SERVO_MAX_RAD;
    if (angle < IK_SERVO_MIN_RAD) angle = IK_SERVO_MIN_RAD;

    return angle;
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
int validatePosition(const float position[6], const StewartConfig* config) {
    if (config == NULL) {
        return 0x3F; // all bits set = all invalid
    }

    int result = 0;
    float angles[6];
    calculateAllServoAngles(position, config, angles);

    for (int i = 0; i < 6; i++) {
        if (angles[i] <= IK_SERVO_MIN_RAD || angles[i] >= IK_SERVO_MAX_RAD) {
            result |= (1 << i);
        }
    }
    return result;
}

void initDefaultStewartConfig(StewartConfig* config) {
    if (config == NULL) {
        return;
    }
    
    // Default values — all lengths in mm (converted from inches × 25.4)
    config->theta_r = 10.0f;            // degrees
    config->theta_s[0] = 150.0f;        // degrees
    config->theta_s[1] = -90.0f;
    config->theta_s[2] = 30.0f;
    config->theta_s[3] = 150.0f;
    config->theta_s[4] = -90.0f;
    config->theta_s[5] = 30.0f;
    config->theta_p = 30.0f;            // degrees
    config->RD = 400.05f;               // 15.75 in → mm
    config->PD = 406.4f;                // 16.0  in → mm
    config->ServoArmLengthL1 = 184.15f; // 7.25  in → mm
    config->ConnectingArmLengthL2 = 723.9f; // 28.5 in → mm
    config->platformHeight = 648.134f;  // 25.517 in → mm

    // Drive train defaults
    config->virtual_gear = 80.0f;       // AASD-15A electronic gear numerator (pn098/pn099)
    config->planetary_ratio = 50.0f;    // 50:1 planetary gearbox
    config->encoder_ppr = 2500;         // AASD-15A encoder lines per revolution
    computeStepsPerDegree(config);      // derive steps_per_degree from above
}

void computeStepsPerDegree(StewartConfig* config) {
    if (config == NULL || config->virtual_gear <= 0.0f) return;
    // encoder_ppr × 4 (quadrature) / virtual_gear = pulses per motor revolution
    // × planetary_ratio = pulses per output shaft revolution
    // / 360 = steps per degree of output shaft rotation
    float encoder_counts = config->encoder_ppr * 4.0f;
    float pulses_per_motor_rev = encoder_counts / config->virtual_gear;
    float pulses_per_output_rev = pulses_per_motor_rev * config->planetary_ratio;
    config->steps_per_degree = pulses_per_output_rev / 360.0f;
}

// ── Generalized Per-Actuator IK ────────────────────────────────────

void buildPlatformFromConfig(const StewartConfig* config, PlatformDef* platform) {
    if (!config || !platform) return;

    // Topology arrays for the standard 3-pair symmetric arrangement
    float DxMul[6]    = { 1,  1,  1, -1, -1, -1};
    float AngleMul[6] = { 1, -1,  1,  1, -1,  1};
    float OffAng[6]   = {(float)(IK_PI/6), (float)(IK_PI/6), (float)(-IK_PI/2),
                         (float)(-IK_PI/2), (float)(IK_PI/6), (float)(IK_PI/6)};

    for (int k = 0; k < 6; k++) {
        ActuatorDef* a = &platform->actuators[k];

        // Platform joint P_k (in platform frame, z=0)
        float pAngle = OffAng[k] + AngleMul[k] * (float)radians(config->theta_r);
        a->plat_pos[0] = DxMul[k] * config->RD * cosf(pAngle);
        a->plat_pos[1] = config->RD * sinf(pAngle);
        a->plat_pos[2] = 0.0f;

        // Base joint B_k (servo shaft position, z=0)
        float bAngle = OffAng[k] + AngleMul[k] * (float)radians(config->theta_p);
        a->base_pos[0] = DxMul[k] * config->PD * cosf(bAngle);
        a->base_pos[1] = config->PD * sinf(bAngle);
        a->base_pos[2] = 0.0f;

        // Servo axis orientation β_k (theta_s in radians)
        a->beta = config->theta_s[k] * (float)(IK_PI / 180.0);

        // Arm lengths (shared for symmetric platform)
        a->L1 = config->ServoArmLengthL1;
        a->L2 = config->ConnectingArmLengthL2;
    }

    platform->home_height  = config->platformHeight;
    platform->servo_min_rad = IK_SERVO_MIN_RAD;
    platform->servo_max_rad = IK_SERVO_MAX_RAD;

    // Copy drive train
    platform->steps_per_degree = config->steps_per_degree;
    platform->virtual_gear     = config->virtual_gear;
    platform->planetary_ratio  = config->planetary_ratio;
    platform->encoder_ppr      = config->encoder_ppr;
}

float calcActuatorAngle(const float position[6], const ActuatorDef* act,
                        float home_h, float servo_min, float servo_max) {
    if (!act) return 0.0f;

    float roll  = position[3];
    float pitch = position[4];
    float yaw   = position[5];

    // Rotation matrix R (ZYX Euler: yaw × pitch × roll)
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);

    // Rotated platform joint: R * P_k + T
    float px = act->plat_pos[0];
    float py = act->plat_pos[1];
    float pz = act->plat_pos[2];

    float rpx = (cy*cp)*px + (cy*sp*sr - sy*cr)*py + (cy*sp*cr + sy*sr)*pz + position[0];
    float rpy = (sy*cp)*px + (sy*sp*sr + cy*cr)*py + (sy*sp*cr - cy*sr)*pz + position[1];
    float rpz = (-sp)*px   + (cp*sr)*py             + (cp*cr)*pz            + home_h + position[2];

    // Leg vector: l = rotated_platform_joint - base_joint
    float lx = rpx - act->base_pos[0];
    float ly = rpy - act->base_pos[1];
    float lz = rpz - act->base_pos[2];

    // e, f, g coefficients (Eisele formulation)
    float L1 = act->L1;
    float L2 = act->L2;
    float lsq = lx*lx + ly*ly + lz*lz;

    float e = 2.0f * L1 * lz;
    float f = 2.0f * L1 * (cosf(act->beta) * lx + sinf(act->beta) * ly);
    float g = lsq - (L2*L2 - L1*L1);

    // α_k = asin(g / sqrt(e² + f²)) - atan2(f, e)
    float ef_mag = sqrtf(e*e + f*f);
    float asin_arg = (ef_mag > 0.0f) ? (g / ef_mag) : 0.0f;
    if (asin_arg >  1.0f) asin_arg =  1.0f;
    if (asin_arg < -1.0f) asin_arg = -1.0f;

    float angle = asinf(asin_arg) - atan2f(f, e);

    // Clamp to physical limits
    if (angle > servo_max) angle = servo_max;
    if (angle < servo_min) angle = servo_min;

    return angle;
}

void calcAllActuatorAngles(const float position[6], const PlatformDef* platform, float angles[6]) {
    if (!platform || !angles) return;
    for (int k = 0; k < 6; k++) {
        angles[k] = calcActuatorAngle(position, &platform->actuators[k],
                                      platform->home_height,
                                      platform->servo_min_rad, platform->servo_max_rad);
    }
}

int validatePositionV2(const float position[6], const PlatformDef* platform) {
    if (!platform) return 0x3F;
    int result = 0;
    float angles[6];
    calcAllActuatorAngles(position, platform, angles);
    for (int k = 0; k < 6; k++) {
        if (angles[k] <= platform->servo_min_rad || angles[k] >= platform->servo_max_rad) {
            result |= (1 << k);
        }
    }
    return result;
}
