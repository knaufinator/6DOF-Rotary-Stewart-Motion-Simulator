#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constants for angle conversions
#define IK_DEG_TO_RAD 0.017453292519943295769236907684886
#define IK_RAD_TO_DEG 57.295779513082320876798154814105
#define IK_PI 3.14159265359

/**
 * Platform configuration structure
 */
typedef struct {
    float theta_r;                  // Base rotation angle in degrees
    float theta_s[6];               // Servo angles array
    float theta_p;                  // Platform rotation angle in degrees
    float RD;                       // Radius of the base
    float PD;                       // Radius of the platform
    float ServoArmLengthL1;         // Length of servo arm
    float ConnectingArmLengthL2;    // Length of connecting arm
    float platformHeight;           // Neutral height of platform
} StewartConfig;

/**
 * Calculate servo angle for a specific servo based on platform position and orientation
 * 
 * @param servoIndex Index of the servo (0-5)
 * @param position Array of 6 values: [x, y, z, roll, pitch, yaw] in mm and radians
 * @param config Platform configuration parameters
 * @return Servo angle in radians
 */
float calculateServoAngle(int servoIndex, const float position[6], const StewartConfig* config);

/**
 * Calculate all servo angles based on platform position and orientation
 * 
 * @param position Array of 6 values: [x, y, z, roll, pitch, yaw] in mm and radians
 * @param config Platform configuration parameters
 * @param servoAngles Output array to store the calculated angles in radians
 */
void calculateAllServoAngles(const float position[6], const StewartConfig* config, float servoAngles[6]);

/**
 * Initialize a StewartConfig structure with default values
 * 
 * @param config Pointer to the configuration structure to initialize
 */
void initDefaultStewartConfig(StewartConfig* config);

#ifdef __cplusplus
}
#endif

#endif // INVERSE_KINEMATICS_H
