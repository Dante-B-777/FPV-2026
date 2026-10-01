#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_err.h"
using rmt_encoder_handle_t = void*;
struct rmt_copy_encoder_config_t {};
inline esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t*, rmt_encoder_handle_t *out) { if(out) *out=reinterpret_cast<void*>(1); return ESP_OK; }
