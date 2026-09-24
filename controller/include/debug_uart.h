#pragma once

#include <stdio.h>
#include "esp_log.h"

// Compile-time switch for UART debug output. Defaults to disabled for production safety.
#ifndef ENABLE_DEBUG_UART
#define ENABLE_DEBUG_UART 0
#endif

extern bool debugEnabled;

// Forward declare serial_printf and serial_println from main.cpp
#ifdef __cplusplus
extern "C" {
#endif
void serial_printf(const char *format, ...);
void serial_println(const char *str);
#ifdef __cplusplus
}
#endif

#if ENABLE_DEBUG_UART
#define DEBUG_PRINT(msg)        do { if (debugEnabled) printf("%s", msg); } while (0)
#define DEBUG_PRINTLN(msg)      do { if (debugEnabled) serial_println(msg); } while (0)
#define DEBUG_PRINTF(fmt, ...)  do { if (debugEnabled) serial_printf((fmt), ##__VA_ARGS__); } while (0)
#else
#define DEBUG_PRINT(msg)        do { (void)(msg); } while (0)
#define DEBUG_PRINTLN(msg)      do { (void)(msg); } while (0)
#define DEBUG_PRINTF(fmt, ...)  do { (void)(fmt); } while (0)
#endif
