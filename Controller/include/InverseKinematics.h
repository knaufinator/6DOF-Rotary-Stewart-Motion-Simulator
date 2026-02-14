#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <math.h>

// Export macro for shared-library test builds (no-op on ESP-IDF)
#if defined(IK_BUILD_SHARED) && defined(_WIN32)
    #define IK_API __declspec(dllexport)
#else
    #define IK_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Constants for angle conversions
#define IK_DEG_TO_RAD 0.017453292519943295769236907684886
#define IK_RAD_TO_DEG 57.295779513082320876798154814105
#define IK_PI 3.14159265359

// Servo angle limits (radians) — physical range of the rotary actuators
#define IK_SERVO_MIN_RAD -1.0471975512f  /* -60 degrees */
#define IK_SERVO_MAX_RAD  1.0471975512f  /*  60 degrees */

/**
 * Platform configuration structure (compact form — 3-pair symmetric topology)
 */
typedef struct {
    // Platform geometry
    float theta_r;                  // Base rotation angle in degrees
    float theta_s[6];               // Servo angles array
    float theta_p;                  // Platform rotation angle in degrees
    float RD;                       // Radius of the base
    float PD;                       // Radius of the platform
    float ServoArmLengthL1;         // Length of servo arm
    float ConnectingArmLengthL2;    // Length of connecting arm
    float platformHeight;           // Neutral height of platform

    // Drive train parameters (configurable at runtime)
    float steps_per_degree;         // Motor steps per degree of servo rotation
    float virtual_gear;             // AASD-15A electronic gear numerator (pn098)
    float planetary_ratio;          // Planetary gearbox ratio (e.g. 50:1)
    int   encoder_ppr;              // Encoder lines per revolution
} StewartConfig;

/**
 * Per-actuator definition — generalized form supporting arbitrary motor placement.
 * Each actuator is fully described by its base/platform joint positions,
 * servo axis orientation, and arm lengths.
 */
typedef struct {
    float base_pos[3];      // B_k: servo shaft position on base [x,y,z] (mm)
    float plat_pos[3];      // P_k: ball joint on platform in platform frame [x,y,z] (mm)
    float beta;             // Servo axis orientation angle in base x-y plane (radians)
    float L1;               // Servo arm length (mm)
    float L2;               // Connecting rod length (mm)
} ActuatorDef;

/**
 * Full platform definition using per-actuator parameterization.
 */
typedef struct {
    ActuatorDef actuators[6];
    float       home_height;        // Neutral platform height (mm)
    float       servo_min_rad;      // Physical servo lower limit (radians)
    float       servo_max_rad;      // Physical servo upper limit (radians)

    // Drive train (shared across actuators for now)
    float       steps_per_degree;
    float       virtual_gear;
    float       planetary_ratio;
    int         encoder_ppr;
} PlatformDef;

/**
 * Calculate servo angle for a specific servo based on platform position and orientation
 * 
 * @param servoIndex Index of the servo (0-5)
 * @param position Array of 6 values: [x, y, z, roll, pitch, yaw] in mm and radians
 * @param config Platform configuration parameters
 * @return Servo angle in radians
 */
IK_API float calculateServoAngle(int servoIndex, const float position[6], const StewartConfig* config);

/**
 * Calculate all servo angles based on platform position and orientation
 * 
 * @param position Array of 6 values: [x, y, z, roll, pitch, yaw] in mm and radians
 * @param config Platform configuration parameters
 * @param servoAngles Output array to store the calculated angles in radians
 */
IK_API void calculateAllServoAngles(const float position[6], const StewartConfig* config, float servoAngles[6]);

/**
 * Initialize a StewartConfig structure with default values
 * 
 * @param config Pointer to the configuration structure to initialize
 */
IK_API void initDefaultStewartConfig(StewartConfig* config);

/**
 * Compute steps_per_degree from drive train parameters.
 * Formula: (encoder_ppr × 4 / virtual_gear) × planetary_ratio / 360
 *
 * @param config Platform configuration (reads encoder_ppr, virtual_gear, planetary_ratio;
 *               writes steps_per_degree)
 */
IK_API void computeStepsPerDegree(StewartConfig* config);

/**
 * Validate whether a platform position is reachable by all servos.
 * Returns 0 if all servos are within workspace.
 * Otherwise returns a bitmask: bit i is set if servo i is out of range.
 *
 * @param position Array of 6 values: [x, y, z, roll, pitch, yaw]
 * @param config Platform configuration parameters
 * @return 0 = valid, nonzero = bitmask of out-of-range servos
 */
IK_API int validatePosition(const float position[6], const StewartConfig* config);

/**
 * Build per-actuator definitions from a compact StewartConfig.
 * Generates the 6 ActuatorDef entries for the standard 3-pair symmetric topology.
 */
IK_API void buildPlatformFromConfig(const StewartConfig* config, PlatformDef* platform);

/**
 * Compute the correct home height for a PlatformDef so that all servo
 * angles are 0 when the platform is level (all inputs zero).
 * Returns the averaged z_home across all 6 actuators.
 * If the geometry is truly symmetric the values will be identical.
 */
IK_API float computeHomeHeight(const PlatformDef* platform);

/**
 * Generalized IK: calculate servo angle for one actuator.
 * Uses per-actuator beta_k parameterization — supports arbitrary motor placement.
 *
 * @param position  [x, y, z, roll, pitch, yaw] in mm and radians
 * @param act       Single actuator definition
 * @param home_h    Platform home height (mm)
 * @param servo_min Physical lower limit (radians)
 * @param servo_max Physical upper limit (radians)
 * @return Servo angle in radians
 */
IK_API float calcActuatorAngle(const float position[6], const ActuatorDef* act,
                               float home_h, float servo_min, float servo_max);

/**
 * Generalized IK: calculate all 6 servo angles using PlatformDef.
 */
IK_API void calcAllActuatorAngles(const float position[6], const PlatformDef* platform,
                                  float angles[6]);

/**
 * Generalized validate: returns bitmask of out-of-range actuators.
 */
IK_API int validatePositionV2(const float position[6], const PlatformDef* platform);

#ifdef __cplusplus
}
#endif

#endif // INVERSE_KINEMATICS_H
