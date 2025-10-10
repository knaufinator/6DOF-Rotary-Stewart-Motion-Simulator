#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>

//calculation helpers
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define pi  3.14159265359
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))

// Board-specific GPIO pin definitions for ESP32-S3
#define STEP_PIN_1 4
#define STEP_PIN_2 5
#define STEP_PIN_3 6
#define STEP_PIN_4 7
#define STEP_PIN_5 8
#define STEP_PIN_6 9
#define DIR_PIN_1 10
#define DIR_PIN_2 11
#define DIR_PIN_3 12
#define DIR_PIN_4 13
#define DIR_PIN_5 14
#define DIR_PIN_6 17
#define ESTOP_PIN 20

// Timing constants
#define MICRO_INTERVAL_FAST 100    // 100 microseconds = 10kHz update rate
#define MICRO_INTERVAL_SLOW 1000   // 1ms = 1kHz update rate
#define ESTOPDEBOUNCETIME 50       // 50ms debounce time
#define ESTOP_CHECK_INTERVAL_MS 10 // Check E-stop every 10ms
#define WDT_TIMEOUT_MS 3000        // 3 second watchdog timeout

// Debug control commands
#define DEBUG_ENABLE_CMD "DBG:1"   // Command to enable debug output
#define DEBUG_DISABLE_CMD "DBG:0"  // Command to disable debug output

// Serial communication
#define MAX_SERIAL_INPUT 60        // Maximum length of serial input buffer

// E-stop configuration
#define ESTOP_ACTIVE_STATE 0       // E-stop is active when pin is LOW (normally closed)

// Motor Constants
#define INV1 0  // Counter-clockwise motors
#define INV2 2
#define INV3 4

// Platform Configuration
static float theta_r = 10;
static float theta_s[6]={150,-90,30, 150,-90,30};
static float theta_p = 30;
static float RD = 15.75;
static float PD = 16;
static float ServoArmLengthL1 = 7.25;
static float ConnectingArmLengthL2 = 28.5;
static float platformHeight = 25.5170749;

// Servo Configuration
static const float servo_min_rad = radians(-60);  // -60 degrees
static const float servo_max_rad = radians(60);   // +60 degrees
static float servoPulseMultiplierPerRadian = 800/(pi/4);  // Calibrated pulses per radian

// Motor step configuration
#define STEPS_PER_DEGREE 100.0f    // Number of motor steps per degree of rotation

// Helper function declarations
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
float rateLimit(float target, float current);
float getAlpha(int i, volatile float arr[]);

//used to hold current status of a motor
struct acServo {
    int stepPin;
    int dirPin;
    bool pinState;
    long currentpos;
    long targetpos;  
};

#endif // HELPERS_H
