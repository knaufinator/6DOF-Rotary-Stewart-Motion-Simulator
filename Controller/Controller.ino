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
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "DigitalFilters.h"
#include "helpers.h"

using namespace std;

//Filter test
constexpr float dtUsed = 0.001;//time between incoming PC packet used in filter calc
std::vector<LowPassFilter> lpfVec;

//uncomment this to slow down motor rate so you can visually see the incrementing of the motor positions in real time. 
//#define DEBUG_MOTORS 1

//when bench testing the esp32, and have no estop connected to PCB, uncomment to allow system to activate motors anyway.
//#define DEBUG_NO_ESTOP 1

//for saving of filter parameters
Preferences preferences;
int microInterval = MICRO_INTERVAL_FAST;

BLEServer* pServer = NULL;
BLECharacteristic* pPostionCharacteristic = NULL;
BLECharacteristic* pPauseCharacteristic = NULL;

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

//soft estop,  this will prevent changes of position from pc to be applied.
//you should have a power kill near by as well,
volatile bool isPausedEStop = false;
volatile bool isRateLimiting = false;

//max angle to allow the platform arms travel in degrees ie +-60 degrees. 
const float servo_min=radians(-60),servo_max=radians(60);

//time helper for creating individual pulses for motors
int64_t currentMicros = esp_timer_get_time();
int64_t  previousMicros = 0;

//time helpers for creating Ble outputs
int64_t currentMicrosBle = esp_timer_get_time();
int64_t  previousMicrosBle = 0;

//this should be refactored out?
static long servo_pos[6];

//Access to each of the 6 ac motor current status.
volatile struct acServo motors[6];

//helper for pulses sent to GPIO, when false, means that the next pulse will be logical 0. 
boolean pinState = false;

//buffers for GPIO state, each motor pulse state dir / position, is loaded into these. 
//two are used to stagger output, for you need to set direction slightly prior to setting position change.
uint16_t motorStepDirValue = 0;   
uint16_t motorStepDirValue2 = 0;   

//current target from pc, modified from 2 seperate tasks/cores
static volatile float arr[6]={0,0,0, 0,0,0};

void setPos(){  
  
    //Platform and Base Coords
    for(int i = 0; i < 6; i++)
    {    
        long x = 0;
        float alpha = getAlpha(i,arr);

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
    
    float arrRaw[]={0,0,0,0,0,0};
    float arrRateLimited[]={0,0,0,0,0,0};
    
    while (tok != NULL) {
      double value = (float)atof(tok);
      float temp = 0.0;
      
      //these are tuned to my specific platform,.. to ensure a value does not get to high and break something  
      //todo, make this dynamic and locked in from the android app
      if(i == 2)
        temp =mapfloat(value, 0, 4094, -7, 7);//hieve 
      else if(i > 2)//rotations, pitch,roll,yaw
        temp = mapfloat(value, 0, 4094, -30, 30) *(pi/180.0);
      else//sway,surge
        temp = mapfloat(value, 0, 4094, -8, 8); 

        arrRaw[i++] = temp;
      
        tok = strtok(NULL, ",");
    }   
      
  //filter 
  //TODo make this a function call, add ability to stack filters?
  //Apply filter to raw PC Data
  //for(int i=0;i<6;i++)
  //{
  //  arrTemp[i] = lpfVec[i].update(arrTemp[i]);
  //}

  //if we are not in an estop pause, allow setting of the current position
  if(!isPausedEStop)
  {
    //if we are just after resetting estop, we will be in a ratelimited mode until the ratelimited position is within close proximity of the actual last stored location.
    if(isRateLimiting)
    {    
        bool isNotWithinLimit = false;
        for(int i=0;i<6;i++)
        {
          arrRateLimited[i] = rateLimit(arrRaw[i],arr[i]);
        
          //close proximity detector, if we are within tolerance, we will set a flag so we break out of the rate limited mode
          float diff = arr[i] - arrRaw[i];
          if (diff > .1 || diff < -.1) {
            isNotWithinLimit = true;
          }  
        }

        //when all are within tolerance, flip ratelimit... rider Go Fast now...
        if(!isNotWithinLimit)
        {
          isRateLimiting = false;
        }

        for(int i=0;i<6;i++)
        {
          arr[i] = arrRateLimited[i];
        }
    }
    else
    {
        for(int i=0;i<6;i++)
        {
          arr[i] = arrRaw[i];
        }
    }

    //Computes the next position of each of the arms based on the values sent.
    setPos();
  }
    
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
        pauseEStop();
      }
      else if(i == 1)
      {
        resumeEStop();
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
  
  LowPassFilter lpf1(dtUsed, 2 * M_PI * getAxis1Filter());
  LowPassFilter lpf2(dtUsed, 2 * M_PI * getAxis2Filter());
  LowPassFilter lpf3(dtUsed, 2 * M_PI * getAxis3Filter());
  LowPassFilter lpf4(dtUsed, 2 * M_PI * getAxis4Filter());
  LowPassFilter lpf5(dtUsed, 2 * M_PI * getAxis5Filter());
  LowPassFilter lpf6(dtUsed, 2 * M_PI * getAxis6Filter());
  
  lpfVec.clear();
  lpfVec.push_back(lpf1);
  lpfVec.push_back(lpf2);
  lpfVec.push_back(lpf3);
  lpfVec.push_back(lpf4);
  lpfVec.push_back(lpf5);
  lpfVec.push_back(lpf6);
  
  Serial.print("lpfVec vector loaded");
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

  BLEDevice::init("Open 6DOF Services");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pPauseCharacteristic = pService->createCharacteristic(
                                         PAUSECHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
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
  pPauseCharacteristic->addDescriptor(new BLE2902());

   
  pPauseCharacteristic->setCallbacks(new BlePauseCallback());
  pFilterCharacteristic->setCallbacks(new BleFilterCallback());  
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising(); 
}

//start here
void setup(){
 
  Serial.begin(115200); 
  initConfigStorage();
  loadFilterAxis();
  setupBle();

  //setup estop/pause button 
  pinMode(ESTOPPIN, INPUT_PULLUP);
  debouncedEStop.attach(ESTOPPIN);
  debouncedEStop.interval(5); 

  //poll estop now directly to get a base value, after this the debouncer will catch rise/fall events of estop/pause button.
  isPausedEStop = !digitalRead(ESTOPPIN);
 
  //uncomment debugnoestop during bench debugging so you dont need to have a estop on the PCB/... or then .. no PCB... and just an ESP32
  #ifdef DEBUG_NO_ESTOP 
    isPausedEStop = 0;
  #endif

  
  
  Serial.print("init Estop check:");
  Serial.println(isPausedEStop);
         
  //if debugging make the pulse train very slow, so we can visually see the incrementing values in output
  #ifdef DEBUG_MOTORS
    microInterval = MICRO_INTERVAL_SLOW;
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
            #ifdef DEBUG_MOTORS
   
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

//reads the serial line: x,y,z,RX,RY,RZX, Where X is used to indicate end of line. 
//else data will buffer and parsed once a full line is available.
void processIncomingByte (const byte inByte)
{
    static char input_line [MAX_SERIAL_INPUT];
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
     pauseEStop();
  }

   if ( debouncedEStop.rose() ) {  // Call code if button transitions from HIGH to LOW
      resumeEStop();
   } 
}

void resumeEStop(){
    isPausedEStop = false;
    isRateLimiting = true;

    notifyPausedStatus();
     
    Serial.print("Estop:");
    Serial.println(isPausedEStop);
}

void pauseEStop(){ 
     isPausedEStop = true;
     isRateLimiting = false;
     
     notifyPausedStatus();
      
     Serial.print("Estop:");
     Serial.println(isPausedEStop);
}

//notify method for ble service, if a device is connected will send paused status as a string
void notifyPausedStatus()
{
    if(deviceConnected)
    {
        pPauseCharacteristic->setValue(BoolToString(isPausedEStop));
        pPauseCharacteristic->notify(); 
    }
}

//notify method for Ble service, if a device is connected will send periodic motor position data,
void checkBleNotify(){

  currentMicrosBle  = esp_timer_get_time();
  int dif = currentMicrosBle  - previousMicrosBle ;
  
  //If enough time elapsed, send BLE packet
  if (dif >= MICRO_INTERVAL_BLE_SEND ) {
    
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

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
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
