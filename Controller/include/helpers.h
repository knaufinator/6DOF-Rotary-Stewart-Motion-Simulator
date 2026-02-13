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

// W5500 SPI Ethernet pins (active when ENABLE_ETHERNET is defined)
#ifdef SPI3_HOST
#define ETH_SPI_HOST   SPI3_HOST
#endif
#ifndef ETH_SPI_HOST
#define ETH_SPI_HOST   2
#endif
#define ETH_SPI_MOSI   35
#define ETH_SPI_MISO   37
#define ETH_SPI_SCLK   36
#define ETH_SPI_CS     38
#define ETH_SPI_INT    39
#define ETH_SPI_CLOCK_MHZ 20
#define ETH_UDP_PORT   4210

// Timing constants
#define MICRO_INTERVAL_FAST 50     // 50 microseconds = 20kHz update rate
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

// Helper function declarations
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
float rateLimit(float target, float current);

#endif // HELPERS_H
