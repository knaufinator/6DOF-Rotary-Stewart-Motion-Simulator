#include <sstream>
#include <Preferences.h>
#include <Bounce2.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>  
#include <Wire.h>
#include <SPI.h>
#include "MCP23S17.h"
#include <EEPROM.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <BLE2902.h>
#include <cstring>
#include "gbj_filter_exponential.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

using namespace std;

//uncomment to see de bug datas, this mode will also slow down motor rate so you can visually 
//see the incrementing of the motor positions in real time. 
//#define DEBUGMOTORS 1

//for saving of filter parameters
Preferences preferences;
#define NAMESPACE "6dofPrefv1"
#define AXIS1_KEY "Axis1"
#define AXIS2_KEY "Axis2"
#define AXIS3_KEY "Axis3"
#define AXIS4_KEY "Axis4"
#define AXIS5_KEY "Axis5"
#define AXIS6_KEY "Axis6"

//DOF Filters - Dynamic with Bluetooth App/save to EEProm
gbj_filter_exponential FilterAxisList[6];

//special pins
#define ESTOPPIN 22
#define ESTOPDEBOUNCETIME 10

//Ble 
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PAUSECHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define FILTERCHARACTERISTIC_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a9"
#define POSITIONCHARACTERISTIC_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a4"

//calculation helpers
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define pi  3.14159265359
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))

//Deine the 3 motors that are running counter clockwise
#define INV1 0
#define INV2 2
#define INV3 4

BLEServer* pServer = NULL;
BLECharacteristic* pPostionCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

//Debouncer for Estop/Pause button
Bounce debouncedEStop = Bounce(); 

//pin number for step and direction pins on the MCP23S17
const int stepPins[] = {0,1,2,3,4,5};   
const int dirPins[] = {6,7,8,9,10,11};

//Lock to protect the Motor position array
SemaphoreHandle_t xMutex;

//CPU tasks, all GPIO for motors is on 2nd CPU, this frees main CPU to process position data and talk to PC and BLE client
TaskHandle_t InterfaceMonitorTask;
TaskHandle_t GPIOLoopTask;

TimerHandle_t wtmr;

//multiplexers for communicating with all 6 servo controllers. 
SPIClass hspi( HSPI );
MCP23S17 outputBank( &hspi, 15, 0 );
MCP23S17 inputBank( &hspi, 15, 1 );

//soft estop, you should have a power kill near by as well, this will prevent changes of position from pc to be applied.
//todo: add abiliy to softly move back to home when paused. perhapse smoothing filter with overridden high filter value.
volatile bool isPausedEStop = false;
volatile bool isPausedBle = false;

//max angle to allow the platform arms travel in degrees ie +-60 degrees. 
const float servo_min=radians(-60),servo_max=radians(60);

//time helper for creating individual pulses for motors
int64_t currentMicros = esp_timer_get_time();
int64_t  previousMicros = 0;

//time helpers for creating Ble outputs
int64_t currentMicrosBle = esp_timer_get_time();
int64_t  previousMicrosBle = 0;

//pulse width minimum length
int  microInterval = 10;
int microIntervalBle  = 100000;

// how much serial data we expect before a newline
const unsigned int MAX_INPUT = 60;

//used to hold current status of a motor
struct acServo {
  int stepPin;
  int dirPin;
  bool pinState;
  long currentpos;
  long targetpos;  
};

//Access to each of the 6 ac motor current status.
volatile struct acServo motors[6];

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

//helper for pulses sent to GPIO, when false, means that the next pulse will be logical 0. 
boolean pinState = false;

//buffers for GPIO state, each motor pulse state dir / position, is loaded into these. 
//two are used to stagger output, for you need to set direction slightly prior to setting position change.
uint16_t motorStepDirValue = 0;   
uint16_t motorStepDirValue2 = 0;   

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

//this should be refactored out?
static long servo_pos[6];

//map a float value of known range to a value of another range of values
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
              //this takes the Radian angle, and scales that value to pulse position.
              //This is calibrated to the real world. with a 50:1 gear, and instructed to move +-60 degrees and finding a servoPulseMultiplierPerRadian that makes that happen.
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
        
        //these are tuned to my specific platform,.. to ensure a value does not get to high and break something  
        //*****modify to a switch,and just cover all independantly, this is so we can apply each multiplier we will be getting from ble service.
        if(i == 2)
          temp =mapfloat(value, 0, 4094, -7, 7);//hieve 
        else if(i > 2)//rotations, pitch,roll,yaw
          temp = mapfloat(value, 0, 4094, -30, 30) *(pi/180.0);
        else//sway,surge
          temp = mapfloat(value, 0, 4094, -8, 8); 

          //place filters in array?
          float filteredTemp = FilterAxisList[i].getValue(temp);  
          arr[i++] = filteredTemp;
          tok = strtok(NULL, ",");
    }   

    //if we are not paused, allow setting position from PC.
    if(!isPausedBle && !isPausedEStop)
      setPos();
} 

//reset timer
void ping( TimerHandle_t xTimer )
{ 
    //Resets the watchdog timer in the ESP32/rtos
    //this is needed in order not have the thing reset every few seconds.
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;

    //Restart timer for this method
    xTimerStart(wtmr, 0);
}

//to pause platform, send "1", to start send "0" string
class BlePauseCallback: public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *pCharacteristic) {
   
      string result = pCharacteristic->getValue().c_str();
      
      Serial.println("BlePauseCallback");
      Serial.println(pCharacteristic->getValue().c_str());
    
      int i = atoi(pCharacteristic->getValue().c_str());
    
      if(i == 0)
      {
        Serial.println("Stop");
        isPausedBle = true;   
      }
      else if(i == 1)
      {
        Serial.println("Start");
        isPausedBle = false;    
      }
    }    
};

//Filter saving from Ble, expecting array of Doubles, 0-100 for each axis comma delimeted.
//i.e. 14.2,16.3,55.6,54.3,34.9
class BleFilterCallback: public BLECharacteristicCallbacks {

    //read filter parameters into Android App
    void onRead(BLECharacteristic *pCharacteristic) {
            
      std::ostringstream os;
      os << getAxis1Filter() << "," << getAxis2Filter() << "," << getAxis3Filter() << "," << getAxis4Filter() << "," << getAxis5Filter() << "," << getAxis6Filter();
  
      pCharacteristic->setValue(os.str());
    }

    //Android app sent new parameters, lets save them to eeprom and reload current filter
    void onWrite(BLECharacteristic *pCharacteristic) {
      
      std::string value = pCharacteristic->getValue();
      Serial.println("Filter settings BLE");
      Serial.println(value.c_str());
      if (value.length() > 0) {
          int arr[6]={0,0,0, 0,0,0};
          int i = 0; 
          char *tok;
          char *rawTokens = new char[value.size()+1];
          strcpy(rawTokens, value.c_str());
       
          tok = strtok(rawTokens, ",");
      
          while (tok != NULL) {
              int value = (int)atoi(tok);
                arr[i++] = value;
                tok = strtok(NULL, ",");
          }   
          
          Serial.println("Save");
          Serial.print("Axis 1: ");
          Serial.println(arr[0]);
          Serial.print("Axis 2: ");
          Serial.println(arr[1]);
          Serial.print("Axis 3: ");
          Serial.println(arr[2]);
          Serial.print("Axis 4: ");
          Serial.println(arr[3]);
          Serial.print("Axis 5: ");
          Serial.println(arr[4]);
          Serial.print("Axis 6: ");
          Serial.println(arr[5]);

          setAxis1(arr[0]);
          setAxis2(arr[1]);
          setAxis3(arr[2]);
          setAxis4(arr[3]);
          setAxis5(arr[4]);
          setAxis6(arr[5]);

          //After saving to EEPROM, load to current running filter
          loadFilterAxis();
      }
    }
};

//loads the EEPROM values for each axis
void loadFilterAxis(){


  Serial.println("Load");
  Serial.print("Axis 1: ");
  Serial.println(getAxis1Filter());
  Serial.print("Axis 2: ");
  Serial.println(getAxis2Filter());
  Serial.print("Axis 3: ");
  Serial.println(getAxis3Filter());
  Serial.print("Axis 4: ");
  Serial.println(getAxis4Filter());
  Serial.print("Axis 5: ");
  Serial.println(getAxis5Filter());
  Serial.print("Axis 6: ");
  Serial.println(getAxis6Filter());
  
  FilterAxisList[0] = gbj_filter_exponential(getAxis1Filter());
  FilterAxisList[1] = gbj_filter_exponential(getAxis2Filter());
  FilterAxisList[2] = gbj_filter_exponential(getAxis3Filter());
  FilterAxisList[3] = gbj_filter_exponential(getAxis4Filter());
  FilterAxisList[4] = gbj_filter_exponential(getAxis5Filter());
  FilterAxisList[5] = gbj_filter_exponential(getAxis6Filter());
}

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

//setup BLE access notify service and configuration characteristics 
void setupBle(){
  
  Serial.println("Starting BLE init!");

  BLEDevice::init("Open 6DOF");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  BLECharacteristic *pPauseCharacteristic = pService->createCharacteristic(
                                         PAUSECHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  BLECharacteristic *pFilterCharacteristic = pService->createCharacteristic(
                                         FILTERCHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pPostionCharacteristic  = pService->createCharacteristic(
                      POSITIONCHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
                                       
  pPostionCharacteristic->addDescriptor(new BLE2902());

   
  pPauseCharacteristic->setCallbacks(new BlePauseCallback());
  pFilterCharacteristic->setCallbacks(new BleFilterCallback());  
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising(); 
}

//start here
void setup(){
 
  Serial.begin(250000); 
  initConfigStorage();
  loadFilterAxis();
  setupBle();

  //setup estop/pause button 
  pinMode(ESTOPPIN, INPUT_PULLUP);
  debouncedEStop.attach(ESTOPPIN);
  debouncedEStop.interval(5); 

  //check if we already do have the estop depressed, so we do not take in data when we should not.
  isPausedEStop = !debouncedEStop.read();
  Serial.print("Estop:");
  Serial.println(isPausedEStop);
         
  //if debugging make the pulse train very slow, so we can visually see the incrementing values in output
  #ifdef DEBUGMOTORS
    microInterval = 10000;
    Serial.println("Motor debug mode active!");
  #endif
  
  xMutex = xSemaphoreCreateMutex();
 
  xTaskCreatePinnedToCore(
                    InterfaceMonitorCode,   /* Task function. */
                    "InterfaceMonitor",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &InterfaceMonitorTask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

  xTaskCreatePinnedToCore(
                    GPIOLoop,   /* Task function. */
                    "GPIOLoopTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &GPIOLoopTask,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
   
  inputBank.begin();
  outputBank.begin();

   //setup the outputs for the AC servo controllers
  setupPWMpins();

  //Timer for watchdog reset
  wtmr = xTimerCreate("wtmr", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(ping));
  xTimerStart(wtmr, 0); 
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
     }
}
  
//pulse train for step and direction, builds output to multiplexer
void handleStepDirection() {

  currentMicros = esp_timer_get_time();
  int dif = currentMicros - previousMicros;

  //creates the pulses in realtime, if enough time has passed.
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
            motorStepDirValue2 = BIT_CLEAR(motorStepDirValue2,stepPins[i]);
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

            //print motor positions, and GPIO request bit array 
            #ifdef DEBUGMOTORS
   
              for(int i=0;i<6;i++)   
              {
                Serial.print(motors[i].currentpos);
                Serial.print(",");
              }
                  
              Serial.println("");
         
            #endif
     
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

//reads the serial line,  x,y,z,RX,RY,RZX, Where X is used to indicate end of line. 
//else data will buffer and parsed once a full line is available.
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

//check estop button often, will flip estop variable when button changes
void checkEStop(){

  //Update button status through debounce filter
  debouncedEStop.update();

  // Get the updated value :
  int value = debouncedEStop.read();

  if ( debouncedEStop.fell() ) {  // Call code if button transitions from HIGH to LOW
     isPausedEStop = true;
     Serial.print("Estop:");
     Serial.println(isPausedEStop);
   }

   if ( debouncedEStop.rose() ) {  // Call code if button transitions from HIGH to LOW
     isPausedEStop = false;
     Serial.print("Estop:");
     Serial.println(isPausedEStop);
   } 
}

//notify method for Ble service, if a device is connected will send periodic motor position data,
void checkBleNotify(){

  currentMicrosBle  = esp_timer_get_time();
  int dif = currentMicrosBle  - previousMicrosBle ;

  //creates the pulses in realtime, if enough time has passed.
  if (dif >= microIntervalBle ) {
    
    if (deviceConnected) {
        
        //lock access to motor array
        xSemaphoreTake( xMutex, portMAX_DELAY );
            
        String output;
        for(int i=0;i<6;i++)   
        {
          output +=  String(motors[i].currentpos);
        
          if(i<5)
            output +=",";
        }
               
        //give access back up
        xSemaphoreGive( xMutex );
            
        pPostionCharacteristic->setValue(output.c_str());
        pPostionCharacteristic->notify();
    }

    previousMicrosBle  = currentMicrosBle;         
  }
}

//Thread that handles reading from PC uart
void InterfaceMonitorCode( void * pvParameters ){
  for(;;){
    while (Serial.available () > 0)
      processIncomingByte (Serial.read ());
    checkEStop();
    checkBleNotify();
  } 
}

//Thread/loop that handles generating pulses for swing arm movement.
void GPIOLoop( void * pvParameters ){
  for(;;){
      handleStepDirection();   
  }
}

bool initConfigStorage() {
  return preferences.begin(NAMESPACE, false);
}

int getAxis1Filter() {
  return preferences.getInt(AXIS1_KEY, 100);
}

void setAxis1(int value) {
  preferences.putInt(AXIS1_KEY, value);
}

int getAxis2Filter() {
  return preferences.getInt(AXIS2_KEY, 100);
}

void setAxis2(int value) {
  preferences.putInt(AXIS2_KEY, value);
}

int getAxis3Filter() {
  return preferences.getInt(AXIS3_KEY, 100);
}

void setAxis3(int value) {
  preferences.putInt(AXIS3_KEY, value);
}

int getAxis4Filter() {
  return preferences.getInt(AXIS4_KEY, 100);
}

void setAxis4(int value) {
  preferences.putInt(AXIS4_KEY, value);
}

int getAxis5Filter() {
  return preferences.getInt(AXIS5_KEY, 100);
}

void setAxis5(int value) {
  preferences.putInt(AXIS5_KEY, value);
}

int getAxis6Filter() {
  return preferences.getInt(AXIS6_KEY, 100);
}

void setAxis6(int value) {
  preferences.putInt(AXIS6_KEY, value);
}
