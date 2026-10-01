#pragma once
#include <cstdint>
class VL53L0X { public: void setTimeout(uint16_t){} bool init(){return true;} bool setMeasurementTimingBudget(uint32_t){return true;} void startContinuous(){} uint16_t readRangeContinuousMillimeters(){return 500;} bool timeoutOccurred(){return false;} };
