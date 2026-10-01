#pragma once
#include "freertos/FreeRTOS.h"
using TaskHandle_t = void*;
using TaskFunction_t = void (*)(void*);
inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, uint32_t, TaskHandle_t *out, int) {
  if (out) *out = reinterpret_cast<void*>(1);
  return pdPASS;
}
inline uint32_t ulTaskNotifyTake(BaseType_t, TickType_t) { return 1; }
inline BaseType_t xTaskNotifyGive(TaskHandle_t) { return pdTRUE; }
inline void taskYIELD() {}
