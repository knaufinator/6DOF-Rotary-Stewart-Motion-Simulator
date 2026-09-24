#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>

// Calculation helpers
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define pi  3.14159265359
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))

// ── Mini-6DOF Board GPIO Pin Definitions ─────────────────────────────
// PWM servo pins (directly drive hobby servos via LEDC)
#define SERVO_PIN_0  15
#define SERVO_PIN_1  14
#define SERVO_PIN_2   4
#define SERVO_PIN_3  32
#define SERVO_PIN_4  33
#define SERVO_PIN_5   5

// Servo enable pin (controls power relay/MOSFET for servo rail)
#define SERVO_ENABLE_PIN  27

// E-stop pin
#define ESTOP_PIN  22

// Servo PWM parameters
#define SERVO_FREQ_HZ      50       // Standard hobby servo frequency
#define SERVO_MIN_US       800      // Minimum pulse width (microseconds)
#define SERVO_MAX_US       2200     // Maximum pulse width (microseconds)
#define SERVO_CENTER_US    1500     // Center/home pulse width (microseconds)

// Inverted servos (mounted mirrored — rotate opposite direction)
#define INV1 0
#define INV2 2
#define INV3 4

// Timing constants
#define ESTOP_DEBOUNCE_MS     50
#define ESTOP_CHECK_INTERVAL_MS 10
#define WDT_TIMEOUT_MS        3000

// Debug control commands
#define DEBUG_ENABLE_CMD  "DBG:1"
#define DEBUG_DISABLE_CMD "DBG:0"

// Serial communication
#define MAX_SERIAL_INPUT 256

// Helper function declarations
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

#endif // HELPERS_H
