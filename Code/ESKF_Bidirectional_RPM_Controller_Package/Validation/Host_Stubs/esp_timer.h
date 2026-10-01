#pragma once
#include <cstdint>
#include "esp_err.h"
using esp_timer_handle_t = void*;
enum esp_timer_dispatch_t { ESP_TIMER_TASK = 0, ESP_TIMER_ISR = 1 };
struct esp_timer_create_args_t {
  void (*callback)(void*) = nullptr;
  void *arg = nullptr;
  esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK;
  const char *name = nullptr;
  bool skip_unhandled_events = false;
};
inline esp_err_t esp_timer_create(const esp_timer_create_args_t*, esp_timer_handle_t *out) { if(out) *out=reinterpret_cast<void*>(1); return ESP_OK; }
inline esp_err_t esp_timer_start_periodic(esp_timer_handle_t, uint64_t) { return ESP_OK; }
