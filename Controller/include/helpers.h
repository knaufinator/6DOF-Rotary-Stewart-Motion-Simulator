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

//variables for platform positions
static float theta_r = 10;
static float theta_s[6]={150,-90,30, 150,-90,30};
static float theta_p = 30;
static float RD = 15.75;
static float PD = 16;
static float ServoArmLengthL1 = 7.25;
static float ConnectingArmLengthL2 = 28.5;
static float platformHeight = 25.5170749;

//how many pulses per radian of arm movement this value is calibrated to my setup
static float servoPulseMultiplierPerRadian = 800/(pi/4);

// Servo angle limits in radians
static const float servo_min = radians(-60);  // -60 degrees
static const float servo_max = radians(60);   // +60 degrees

//special pins and timing
#define MICRO_INTERVAL_FAST 100
#define MICRO_INTERVAL_SLOW 10000
#define ESTOPPIN 4
#define ESTOPDEBOUNCETIME 5

//Define the 3 motors that are running counter clockwise
#define INV1 0
#define INV2 2
#define INV3 4

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
