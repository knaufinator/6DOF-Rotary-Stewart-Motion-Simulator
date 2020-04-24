#include <Chrono.h>
#include <Wire.h>
#include <SPI.h>
#include <MCP23S17.h> 

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

using namespace std;

SemaphoreHandle_t xMutex;

TaskHandle_t Task1;
TaskHandle_t Task2;

//multiplexers for communicating with all 6 servo controllers. 
SPIClass hspi( HSPI );
MCP23S17 outputBank( &hspi, 15, 0 );
MCP23S17 inputBank( &hspi, 15, 1 );

#define INV1 0
#define INV2 2
#define INV3 4
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))

//pin number for step and direction pins on the MCP23S17
const int stepPins[] = {0,1,2,3,4,5};   
const int dirPins[] = {6,7,8,9,10,11};

//reserved  inputs from servos to detect if they are ready, or are alarming.
const int highPins[] = {15,1,2,3,4,5};   
const int lowPins[] = {6,7,8,9,10,11};

//max angle to allow the platform arms travel in degrees ie +-60 degrees. 
const float servo_min=radians(-60),servo_max=radians(60);

const int eStopPin = 12;

TimerHandle_t tmr;

int64_t currentMicros = esp_timer_get_time();
int64_t  previousMicros = 0;
int  microInterval = 10;

// how much serial data we expect before a newline
const unsigned int MAX_INPUT = 60;

struct acServo {
  int stepPin;
  int dirPin;
  bool pinState;
  long currentpos;
  long targetpos;  
};

volatile struct acServo motors[6];

//helpers
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define pi  3.14159265359
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)

//variables for platform positions... need better..definitions....
static float theta_r = 15;
static float theta_s[6]={150,-90,30, 150,-90,30};
static float theta_p = 30;
static float RD = 15.75;
static float PD = 20.5;
static float ServoArmLengthL1 = 7.4;
static float ConnectingArmLengthL2 = 28.5;
static float platformHeight = 25.546;

//how many pulses per radians of arm movement, tuned untill actual degrees matched expected output
static float servoPulseMultiplierPerRadian =  800/(pi/4);

//current target from pc, modified from 2 seperate tasks/cores
static volatile float arr[6]={0,0,0, 0,0,0};

// for Platform Coord algorithm
float platformPDx[6]={0,0,0, 0,0,0};
float platformPDy[6]={0,0,0, 0,0,0};
float platformAngle[6]={0,0,0,0,0,0};
float platformCoordsx[6]={0,0,0, 0,0,0};
float platformCoordsy[6]={0,0,0, 0,0,0};
float basePDx[6]={0,0,0, 0,0,0};
float basePDy[6]={0,0,0, 0,0,0};
float baseAngle[6]={0,0,0,0,0,0};
float baseCoordsx[6]={0,0,0, 0,0,0};
float baseCoordsy[6]={0,0,0, 0,0,0};
float DxMultiplier[6]={1,1,1, -1,-1,-1};
float AngleMultiplier[6]={1,-1,1,1,-1,1};
float OffsetAngle[6]={pi/6,pi/6,-pi/2, -pi/2,pi/6,pi/6};

//base rotation values
float platformPivotx[6]={0,0,0, 0,0,0};
float platformPivoty[6]={0,0,0, 0,0,0};
float platformPivotz[6]={0,0,0, 0,0,0};

float deltaLx[6] = {0,0,0, 0,0,0};
float deltaLy[6] = {0,0,0, 0,0,0};
float deltaLz[6] = {0,0,0, 0,0,0};
float deltaL2Virtual[6] = {0,0,0, 0,0,0};

float l[6] = {0,0,0, 0,0,0};
float m[6] = {0,0,0, 0,0,0};
float n[6] = {0,0,0, 0,0,0};
float alpha[6] = {0,0,0, 0,0,0};

static long servo_pos[6];

float mapfloat(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

//function calculating needed servo rotation value
float getAlpha(int i){
        platformPDx[i] = DxMultiplier[i] * RD;
        platformPDy[i] = RD;
        platformAngle[i] = OffsetAngle[i] + AngleMultiplier[i]* radians(theta_r);
        platformCoordsx[i] = platformPDx[i] * cos(platformAngle[i]);
        platformCoordsy[i] = platformPDy[i] * sin(platformAngle[i]);
        
        basePDx[i] = DxMultiplier[i] * PD;
        basePDy[i] = PD;
        baseAngle[i] = OffsetAngle[i] + AngleMultiplier[i]* radians(theta_p);
        baseCoordsx[i] = basePDx[i] * cos(baseAngle[i]);
        baseCoordsy[i] = basePDy[i] * sin(baseAngle[i]);
        
        //Platform pivots
        platformPivotx[i] = platformCoordsx[i]*cos(arr[3])*cos(arr[5])+platformCoordsy[i]*(sin(arr[4])*sin(arr[3])*cos(arr[3])-cos(arr[4])*sin(arr[5]))+arr[0]; 
        platformPivoty[i] = platformCoordsx[i]*cos(arr[4])*sin(arr[5])+platformCoordsy[i]*(cos(arr[3])*cos(arr[5])+sin(arr[3])*sin(arr[4])*sin(arr[5]))+arr[1];
        platformPivotz[i] = -platformCoordsx[i]*sin(arr[3])+platformCoordsy[i]*sin(arr[4])*cos(arr[3])+platformHeight+arr[2];
        
        deltaLx[i] = baseCoordsx[i] - platformPivotx[i];
        deltaLy[i] = baseCoordsy[i] - platformPivoty[i];
        deltaLz[i] = -platformPivotz[i];
        
        deltaL2Virtual[i] = sqrt(pow(deltaLx[i], 2.0) + pow(deltaLy[i], 2.0) +pow(deltaLz[i], 2.0));
    
        l[i] = pow(deltaL2Virtual[i], 2.0)-(pow(ConnectingArmLengthL2, 2.0)-pow(ServoArmLengthL1, 2.0));
        m[i] = 2*ServoArmLengthL1*(platformPivotz[i]);
        n[i] = 2*ServoArmLengthL1*(cos(theta_s[i]*pi/180)*(platformPivotx[i] -  baseCoordsx[i])+sin(theta_s[i]*pi/180)*(platformPivoty[i]- baseCoordsy[i]));

        return asin(l[i]/(sqrt(pow(m[i], 2.0)+pow(n[i], 2.0))))-atan(n[i]/m[i]);
}

void setPos(){  
  
    //Platform and Base Coords
    for(int i = 0; i < 6; i++)
    {    
        long x = 0;
        float alpha = getAlpha(i);

          if(alpha >= servo_min && alpha <= servo_max)
          {
              if(i==INV1||i==INV2||i==INV3){
                  x = -(alpha)*servoPulseMultiplierPerRadian;
              }
              else{
                  x = (alpha)*servoPulseMultiplierPerRadian;
              }
      
              servo_pos[i] = x;            
          }     
    }

    //lock access to motor array
    xSemaphoreTake( xMutex, portMAX_DELAY );
    
    for(int i = 0; i < 6; i++)
    {
          motors[i].targetpos = servo_pos[i];
    }   
    
    //give up lock
    xSemaphoreGive( xMutex );

     
}

//parses the data packet from the pc => x,y,z,Ry,Rx,RZ
void process_data ( char * data)
{ 
    int i = 0; 
    char *tok = strtok(data, ",");
    
    while (tok != NULL) {
        double value = (float)atof(tok);
        float temp = 0.0;
        //these are tuned to my specific platform,.. to ensure a value does not get to high and break something...  
        if(i == 2)
          temp =mapfloat(value, 0, 4094, -7, 7);//hieve 
        else if(i > 2)//rotations, pitch,roll,yaw
          temp = mapfloat(value, 0, 4094, -30, 30) *(pi/180.0);
        else//sway,surge
          temp = mapfloat(value, 0, 4094, -8, 8); 
          
          arr[i++] = temp;
    
          tok = strtok(NULL, ",");
    }   
    
    setPos();
} 


//show the target of the all the motors every so often
void ping( TimerHandle_t xTimer )
{ 
    //Resets the watchdog timer in the ESP32/rtos
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;

      //lock access to motor array
    //  xSemaphoreTake( xMutex, portMAX_DELAY );

    //disable when not debugging so it no waste time
      for(int i=0;i<6;i++)    
      {
        //Serial.print(motors[i].currentpos);
        //Serial.print(",");
      }
        
      //Serial.println("");

  //    xSemaphoreGive( xMutex );
      
    xTimerStart(tmr, 0);
}

void setup(){
   Serial.begin(115200);  
   
   xMutex = xSemaphoreCreateMutex();
   
   xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

   xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

   
   inputBank.begin();
   outputBank.begin();

   //setup the outputs for the AC servo controllers
   setupPWMpins();

   //use during debugging, this will output every one second the position the motors are trying to achive
   tmr = xTimerCreate("tmr", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(ping));// pos serial output every 1 second
   xTimerStart(tmr, 0);
}

void setupPWMpins() {
    //pins to use for pin state on reset
    for(int i=0;i<6;i++)
    {
        motors[i].stepPin = stepPins[i];
        motors[i].dirPin = dirPins[i];
        motors[i].pinState = false;
        motors[i].currentpos = 0;
        motors[i].targetpos = 0;

        outputBank.pinMode( motors[i].stepPin, OUTPUT );
        outputBank.pinMode( motors[i].dirPin, OUTPUT );

        inputBank.pinMode( highPins[i], INPUT_PULLUP );
     }
  
       inputBank.pinMode( eStopPin, INPUT_PULLUP );
}

boolean pinState = false;
     
uint16_t motorStepDirValue = 0;   
uint16_t motorStepDirValue2 = 0;   
  
//pulse train for step and direction, builds output to multiplexer
void handlePWM() {

  currentMicros = esp_timer_get_time();
  int dif = currentMicros - previousMicros;
  
  // check to see if we need to increment our PWM counters yet
  if (dif >= microInterval) {
        if (pinState) {
          
          //we need to bring the step pins down from what they where.
          //keep the direction the same, 
          pinState = false;

          //lock access to motor array
          xSemaphoreTake( xMutex, portMAX_DELAY );

          for(int i =0;i<6;i++)
          {
            motorStepDirValue = BIT_CLEAR(motorStepDirValue,stepPins[i]);
            motorStepDirValue2 = BIT_CLEAR(motorStepDirValue2,stepPins[i]);//bad things happen if you leave this out....
          }
          
           //give access back up
           xSemaphoreGive( xMutex );
        
          outputBank.digitalWrite(motorStepDirValue);      
        
        } else { 
            pinState = true;

            //lock access to motor array
            xSemaphoreTake( xMutex, portMAX_DELAY );

            //set direction pins first
            for(int i =0;i<6;i++)
            {
              if(motors[i].currentpos > motors[i].targetpos)
              {
                 motorStepDirValue = BIT_CLEAR(motorStepDirValue,dirPins[i]);
                 motorStepDirValue2 = BIT_CLEAR(motorStepDirValue2,dirPins[i]);
              }
              else if(motors[i].currentpos < motors[i].targetpos)
              { 
                motorStepDirValue = BIT_SET(motorStepDirValue,dirPins[i]);
                motorStepDirValue2 = BIT_SET(motorStepDirValue2,dirPins[i]);
              }
            }
            
            //Next, set the step pins
            for(int i =0;i<6;i++)
            {
              if(motors[i].currentpos > motors[i].targetpos)
              {
                  motors[i].currentpos--;
                  motorStepDirValue2 = BIT_SET(motorStepDirValue2,stepPins[i]);
              }
              else if(motors[i].currentpos < motors[i].targetpos)
              { 
                  motors[i].currentpos++;
                  motorStepDirValue2 = BIT_SET(motorStepDirValue2,stepPins[i]);
              }
            }

            //give access back up
            xSemaphoreGive( xMutex );

            //two outputs, one to set direction, one to then( reafirm direction + step)
            outputBank.digitalWrite(motorStepDirValue);     
            outputBank.digitalWrite(motorStepDirValue2);      
        }
        
        previousMicros = currentMicros;         
  }
}

void loop()
{      
}

float average (int * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

//reads the serial line,  x,y,z,RX,RY,RZX 
void processIncomingByte (const byte inByte)
{
    static char input_line [MAX_INPUT];
    static unsigned int input_pos = 0;
  
    switch (inByte)
    {
        case 'X':   // end of text
          input_line [input_pos] = 0;  // terminating null byte
          process_data (input_line);
          input_pos = 0;  
          break;
        case '\r':   // discard carriage return
          break;
        
        default:
          //buffer data
          if (input_pos < (MAX_INPUT - 1))
            input_line [input_pos++] = inByte;
          break;
    } 
} 


void Task1code( void * pvParameters ){
  
  for(;;){

   while (Serial.available () > 0)
        processIncomingByte (Serial.read ());
  } 
}

void Task2code( void * pvParameters ){
  
  for(;;){
      handlePWM();   
  }
}
