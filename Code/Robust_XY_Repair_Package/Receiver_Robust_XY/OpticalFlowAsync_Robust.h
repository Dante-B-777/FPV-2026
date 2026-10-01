#pragma once

#include <Arduino.h>

// Full PMW3901 motion-burst sample passed from the SPI task to the flight core.
// This replaces the old two-count-only message. The quality fields are needed
// to decide whether optical flow is safe to fuse or use for position hold.
struct OpticalFlowRawSample
{
  int16_t rawDeltaX = 0;
  int16_t rawDeltaY = 0;

  uint8_t motion = 0;
  uint8_t observation = 0;
  uint8_t squal = 0;
  uint8_t rawDataSum = 0;
  uint8_t maxRawData = 0;
  uint8_t minRawData = 0;
  uint16_t shutter = 0;
  bool transferPlausible = false;

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

  uint32_t plausibleTransfers = 0;
  uint32_t motionB0Samples = 0;
  uint64_t squalTotal = 0;
  uint32_t maximumShutter = 0;
};
