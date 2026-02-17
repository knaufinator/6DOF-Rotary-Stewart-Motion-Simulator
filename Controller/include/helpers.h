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

// ── Board-Specific Pin Definitions ───────────────────────────────────
// PCB_VERSION is set automatically by CMakeLists.txt based on target chip:
//   idf.py set-target esp32    → PCB_VERSION=1 (original DevKit + MCP23S17)
//   idf.py set-target esp32s3  → PCB_VERSION=2 (new board, direct MCPWM GPIO)

#if PCB_VERSION == 1
// ── PCBv1: Original ESP32 DevKit with MCP23S17 SPI GPIO expander ────
// Step/dir signals routed through MCP23S17 (not ESP32 GPIO directly).
// Pin numbers below are MCP23S17 port pins (0-15), not ESP32 GPIO.
#define MCP_STEP_PIN_0  0
#define MCP_STEP_PIN_1  1
#define MCP_STEP_PIN_2  2
#define MCP_STEP_PIN_3  3
#define MCP_STEP_PIN_4  4
#define MCP_STEP_PIN_5  5
#define MCP_DIR_PIN_0   6
#define MCP_DIR_PIN_1   7
#define MCP_DIR_PIN_2   8
#define MCP_DIR_PIN_3   9
#define MCP_DIR_PIN_4   10
#define MCP_DIR_PIN_5   11

// MCP23S17 SPI bus (HSPI on ESP32)
#define MCP_SPI_HOST    SPI2_HOST
#define MCP_SPI_MOSI    13
#define MCP_SPI_MISO    12
#define MCP_SPI_CLK     14
#define MCP_CS_PIN      15

// E-stop on direct ESP32 GPIO (active LOW, normally-closed switch)
#define ESTOP_PIN       22

// UDP port (shared with WiFi transport on V1)
#define ETH_UDP_PORT    4210

// Motors that run counter-clockwise (inverted direction)
#define INV_MOTOR_0     0
#define INV_MOTOR_1     2
#define INV_MOTOR_2     4

#else  // PCB_VERSION == 2
// ── PCBv2: ESP32-S3 with direct GPIO MCPWM step/dir output ─────────
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

#endif // PCB_VERSION

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
