#include <cassert>
#include <cmath>
#include <cstdio>

#include "MotionModelFF.h"
#include "RotorFeedbackModel.h"

int main()
{
  RotorFeedbackModel bridge;
  RotorFeedbackModel::Observation observation;
  const uint16_t commands[4] = {500, 500, 500, 500};

  const float rpm[4] = {9360.0f, 9440.0f, 9480.0f, 9520.0f};
  for (int i = 0; i < 4; ++i)
  {
    observation.omegaRadPerSec[i] =
      rpm[i] * 2.0f * 3.14159265358979323846f / 60.0f;
    observation.valid[i] = true;
    observation.newMeasurement[i] = true;
    observation.ageUs[i] = 1000;
  }

  RotorFeedbackModel::State state;
  for (int step = 0; step < 80; ++step)
  {
    state = bridge.update(commands, observation, 0.004f);
  }

  assert(state.allMeasured);
  assert(state.measuredMask == 0x0F);
  assert(state.fallbackMask == 0x00);
  assert(std::fabs(state.omegaRadPerSec[0] - observation.omegaRadPerSec[0]) < 0.01f);
  assert(state.forceBodyN[2] > 4.0f);
  assert(state.forceBodyN[2] < 10.0f);
  assert(state.processNoiseScale == 1.0f);

  // The online scale should move away from the original 874.8/505 map toward
  // the observed approximately 980 rad/s at DShot 500.
  const float originalScale =
    MotionModelFeedforward::hoverOmegaRadPerSecond() / 505.0f;
  assert(state.omegaPerDshot[0] > originalScale);

  MotionModelFeedforward model;
  MotionModelFeedforward::RotorFeedbackInput input;
  for (int i = 0; i < 4; ++i)
  {
    input.omegaRadPerSecond[i] = state.omegaRadPerSec[i];
    input.omegaPerDshot[i] = state.omegaPerDshot[i];
    input.measured[i] = state.measured[i];
    input.stale[i] = state.stale[i];
    input.ageUs[i] = state.ageUs[i];
  }

  auto output = model.update(0.0f, 0.0f, 0.0f, 0.0f, 0.004f, 505.0f, &input);
  for (int step = 0; step < 80; ++step)
  {
    output = model.update(6.0f, 0.0f, 0.0f, 0.0f, 0.004f, 505.0f, &input);
  }

  assert(output.valid);
  assert(output.rotorFeedbackUsed);
  assert(output.measuredRotorMask == 0x0F);
  assert(std::fabs(output.rotorOmegaInputRadPerSecond[0] - state.omegaRadPerSec[0]) < 0.01f);
  assert(std::fabs(output.omegaPerDshotUsed[0] - state.omegaPerDshot[0]) < 0.001f);


  // Force/torque sign checks in the right-handed model frame. Controller
  // positive pitch/roll are the negative of RH body X/Y, respectively.
  RotorFeedbackModel pitchBridge;
  RotorFeedbackModel::Observation pitchObservation;
  for (int i = 0; i < 4; ++i)
  {
    const float pitchRpm = i < 2 ? 8500.0f : 10000.0f;
    pitchObservation.omegaRadPerSec[i] =
      pitchRpm * 2.0f * 3.14159265358979323846f / 60.0f;
    pitchObservation.valid[i] = true;
    pitchObservation.newMeasurement[i] = true;
    pitchObservation.ageUs[i] = 1000;
  }
  const auto pitchState = pitchBridge.update(commands, pitchObservation, 0.004f);
  assert(pitchState.torqueBodyNm[0] < 0.0f);

  RotorFeedbackModel rollBridge;
  RotorFeedbackModel::Observation rollObservation;
  for (int i = 0; i < 4; ++i)
  {
    const bool rightMotor = i == 1 || i == 2;
    const float rollRpm = rightMotor ? 10000.0f : 8500.0f;
    rollObservation.omegaRadPerSec[i] =
      rollRpm * 2.0f * 3.14159265358979323846f / 60.0f;
    rollObservation.valid[i] = true;
    rollObservation.newMeasurement[i] = true;
    rollObservation.ageUs[i] = 1000;
  }
  const auto rollState = rollBridge.update(commands, rollObservation, 0.004f);
  assert(rollState.torqueBodyNm[1] < 0.0f);

  // Drop telemetry. The bridge must continue from its corrected motor state,
  // not jump immediately to zero.
  RotorFeedbackModel::Observation missing;
  for (int i = 0; i < 4; ++i)
  {
    missing.valid[i] = false;
    missing.ageUs[i] = 25000;
  }

  const float beforeLoss = state.omegaRadPerSec[0];
  state = bridge.update(commands, missing, 0.004f);
  assert(!state.allMeasured);
  assert(state.measuredMask == 0x00);
  assert(state.fallbackMask == 0x0F);
  assert(state.processNoiseScale == 2.0f);
  assert(state.omegaRadPerSec[0] > 0.0f);
  assert(std::fabs(state.omegaRadPerSec[0] - beforeLoss) < 150.0f);

  std::printf(
    "rotor feedback Fz=%.3f N scale0=%.4f omega0=%.2f fallback0=%.2f\n",
    static_cast<double>(state.forceBodyN[2]),
    static_cast<double>(state.omegaPerDshot[0]),
    static_cast<double>(beforeLoss),
    static_cast<double>(state.omegaRadPerSec[0])
  );
  std::puts("Rotor feedback tests passed");
  return 0;
}
