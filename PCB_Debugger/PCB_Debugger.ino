#include <SPI.h>
#include "MCP23S17.h"


SPIClass hspi( HSPI );
MCP23S17 outputBank( &hspi, 15, 0 );


#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))


//mcp1
const int stepPins[] = {0,1,2,3,4,5};   
const int dirPins[] = {6,7,8,9,10,11};


void setup() {  
  Serial.begin(115200);
   outputBank.begin();

    for (int8_t i = 0; i <= 15; i++) {
     outputBank.pinMode( i, OUTPUT );
   }
}

int64_t currentMicros = esp_timer_get_time();
int64_t previousMicros = 0;
int microInterval = 1000000;//2 sec?

int state = 0;
void loop() {

  uint16_t motorStepDirValue = 0;   
  currentMicros = esp_timer_get_time();
  int elapsedTime = currentMicros - previousMicros;

  // check to see if we need to increment our PWM counters yet
  if (elapsedTime >= microInterval) {
    
      if(state == 0)
      {
          for(int i =0;i<6;i++){         
            motorStepDirValue = BIT_SET(motorStepDirValue,stepPins[i]);
            motorStepDirValue = BIT_SET(motorStepDirValue,dirPins[i]);
          }
        
        state = 1;
      }
      else
      {
        state = 0;
      }

        Serial.println("Outputs");
        for (byte i=0; i<16; i++) {
          byte state = bitRead(motorStepDirValue, i);
          Serial.print(state);
        }
        
        Serial.println("");

      outputBank.digitalWrite(motorStepDirValue);

      previousMicros = currentMicros;
  }
}
