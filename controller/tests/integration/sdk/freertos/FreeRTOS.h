#pragma once
#include <cstdint>
using TickType_t = uint32_t;
using BaseType_t = int;
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
