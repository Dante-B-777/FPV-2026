#pragma once
#include <cstdint>
#include "esp_err.h"
struct esp_now_recv_info_t{};
using esp_now_recv_cb_t = void(*)(const esp_now_recv_info_t*, const uint8_t*, int);
inline esp_err_t esp_now_init(){return ESP_OK;}
inline void esp_now_register_recv_cb(esp_now_recv_cb_t){}
