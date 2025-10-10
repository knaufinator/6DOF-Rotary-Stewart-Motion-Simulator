#pragma once

#include <Arduino.h>

// Compile-time switch for UART debug output. Defaults to disabled for production safety.
#ifndef ENABLE_DEBUG_UART
#define ENABLE_DEBUG_UART 0
#endif

extern bool debugEnabled;

#if ENABLE_DEBUG_UART
#define DEBUG_PRINT(msg)        do { if (debugEnabled) Serial.print(msg); } while (0)
#define DEBUG_PRINTLN(msg)      do { if (debugEnabled) Serial.println(msg); } while (0)
#define DEBUG_PRINTF(fmt, ...)  do { if (debugEnabled) Serial.printf((fmt), ##__VA_ARGS__); } while (0)
#else
#define DEBUG_PRINT(msg)        do { (void)(msg); } while (0)
#define DEBUG_PRINTLN(msg)      do { (void)(msg); } while (0)
#define DEBUG_PRINTF(fmt, ...)  do { (void)(fmt); } while (0)
#endif
