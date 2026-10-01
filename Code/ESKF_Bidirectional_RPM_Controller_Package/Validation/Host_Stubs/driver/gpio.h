#pragma once
#include "esp_err.h"
#ifndef GPIO_NUM_T_DEFINED
#define GPIO_NUM_T_DEFINED
using gpio_num_t = int;
#endif
#ifndef GPIO_NUM_NC
static constexpr gpio_num_t GPIO_NUM_NC = -1;
#endif
static constexpr int GPIO_MODE_OUTPUT = 1;
inline esp_err_t gpio_reset_pin(gpio_num_t) { return ESP_OK; }
inline esp_err_t gpio_set_direction(gpio_num_t, int) { return ESP_OK; }
inline esp_err_t gpio_set_level(gpio_num_t, int) { return ESP_OK; }
inline esp_err_t gpio_input_enable(gpio_num_t) { return ESP_OK; }
inline esp_err_t gpio_pulldown_dis(gpio_num_t) { return ESP_OK; }
inline esp_err_t gpio_pullup_en(gpio_num_t) { return ESP_OK; }
