#pragma once
#include <cstdint>
using BaseType_t = int;
using TickType_t = uint32_t;
static constexpr BaseType_t pdTRUE = 1;
static constexpr BaseType_t pdFALSE = 0;
static constexpr BaseType_t pdPASS = 1;
static constexpr TickType_t portMAX_DELAY = 0xFFFFFFFFu;
#define pdMS_TO_TICKS(x) static_cast<TickType_t>(x)
