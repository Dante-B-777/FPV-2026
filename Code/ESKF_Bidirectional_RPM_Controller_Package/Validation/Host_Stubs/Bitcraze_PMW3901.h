#pragma once
#include <cstdint>
class Bitcraze_PMW3901 { public: explicit Bitcraze_PMW3901(uint8_t){} bool begin(){return true;} void readMotionCount(int16_t*x,int16_t*y){*x=0;*y=0;} };
