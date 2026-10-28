#pragma once
#include "FreeRTOS.h"
using TaskFunction_t = void (*)(void*);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, unsigned, void*, int);
void vTaskDelay(TickType_t);
