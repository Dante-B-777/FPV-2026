#pragma once
#include "freertos/FreeRTOS.h"
struct StubSemaphore { bool available=false; };
using SemaphoreHandle_t = StubSemaphore*;
inline SemaphoreHandle_t xSemaphoreCreateBinary() { return new StubSemaphore{}; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t) {
  if (s && s->available) { s->available=false; return pdTRUE; }
  return pdFALSE;
}
inline BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t s, BaseType_t *woken) {
  if (s) s->available=true;
  if (woken) *woken=pdFALSE;
  return pdTRUE;
}
