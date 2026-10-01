#pragma once
#include <cstdint>
#include "Arduino.h"
enum dshot_mode_t { DSHOT300 };
static constexpr uint16_t DSHOT_CMD_MOTOR_STOP=0;
class DShotRMT { public: DShotRMT(gpio_num_t,dshot_mode_t,bool){} void begin(){} void sendThrottle(uint16_t){} };
