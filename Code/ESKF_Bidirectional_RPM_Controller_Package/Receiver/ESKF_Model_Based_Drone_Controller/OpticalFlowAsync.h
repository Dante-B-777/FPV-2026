#pragma once

#include <Arduino.h>

// Fixed-size cross-core message produced by the PMW3901 SPI task and consumed
// by the flight-control core. Keeping this type in a header also makes it
// visible to Arduino's auto-generated function prototypes.
struct OpticalFlowRawSample
{
  int16_t rawDeltaX = 0;
  int16_t rawDeltaY = 0;
  uint32_t sampleUs = 0;
  uint32_t sampleMs = 0;
  uint32_t intervalUs = 0;
  uint32_t sequence = 0;
  float rollDeltaRad = 0.0f;
  float pitchDeltaRad = 0.0f;
  float yawDeltaCwRad = 0.0f;
};

struct OpticalFlowTaskStats
{
  uint64_t ioTotalUs = 0;
  uint32_t ioMaxUs = 0;
  uint32_t ioSamples = 0;
  uint64_t intervalTotalUs = 0;
  uint32_t intervalSamples = 0;
};
