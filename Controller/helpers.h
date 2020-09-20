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
static float servoPulseMultiplierPerRadian =  800/(pi/4);


//Ble 
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PAUSECHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define FILTERCHARACTERISTIC_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a9"
#define POSITIONCHARACTERISTIC_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a4"

#define NAMESPACE "6dofPrefv1"
#define AXIS1_KEY "Axis1"
#define AXIS2_KEY "Axis2"
#define AXIS3_KEY "Axis3"
#define AXIS4_KEY "Axis4"
#define AXIS5_KEY "Axis5"
#define AXIS6_KEY "Axis6"

//special pins
#define ESTOPPIN 22
#define ESTOPDEBOUNCETIME 10
//Deine the 3 motors that are running counter clockwise
#define INV1 0
#define INV2 2
#define INV3 4

//pulse width minimum length
#define MICRO_INTERVAL_FAST  10
#define MICRO_INTERVAL_SLOW  20000
#define MICRO_INTERVAL_BLE_SEND  100000
#define ESTOP_RATE_LIMIT_TIME 3000

// how much serial data we expect before a newline
#define MAX_INPUT 60
   
//used to hold current status of a motor
struct acServo {
  int stepPin;
  int dirPin;
  bool pinState;
  long currentpos;
  long targetpos;  
};

float rateLimit(float currentSample,float lastSample);
float mapfloat(double x, double in_min, double in_max, double out_min, double out_max);
float getAlpha(int i,volatile float arr[]);
