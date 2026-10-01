#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "MotionModelFF.h"

/*
  Lightweight rotor-state bridge between Bluejay RPM telemetry and the
  inverse-dynamics model.

  Fresh filtered RPM is used directly. Between valid replies, a first-order
  command-to-speed estimate continues the state without a discontinuity. The
  local omega/DShot scale is learned slowly from valid telemetry, so the motion
  model no longer relies only on the fixed hover conversion.

  Rotor speeds remain positive magnitudes. Separate signs are applied only to
  rotor angular momentum and reaction torque.
*/
class RotorFeedbackModel
{
public:
  static constexpr size_t MOTOR_COUNT = 4;

  struct Observation
  {
    float omegaRadPerSec[MOTOR_COUNT] = {};
    bool valid[MOTOR_COUNT] = {};
    bool newMeasurement[MOTOR_COUNT] = {};
    uint32_t ageUs[MOTOR_COUNT] = {};
  };

  struct State
  {
    float omegaRadPerSec[MOTOR_COUNT] = {};
    float fallbackOmegaRadPerSec[MOTOR_COUNT] = {};
    float omegaPerDshot[MOTOR_COUNT] = {};
    float dshotPerOmega[MOTOR_COUNT] = {};
    uint16_t dshotCommand[MOTOR_COUNT] = {};

    bool measured[MOTOR_COUNT] = {};
    bool stale[MOTOR_COUNT] = {};
    uint32_t ageUs[MOTOR_COUNT] = {};
    uint8_t measuredMask = 0;
    uint8_t fallbackMask = 0x0F;
    bool allMeasured = false;

    float rotorThrustN[MOTOR_COUNT] = {};
    float forceBodyN[3] = {};
    float torqueBodyNm[3] = {};
    float rotorAngularMomentumZ = 0.0f;

    float processNoiseScale = 2.0f;
  };

  RotorFeedbackModel()
  {
    reset();
  }

  void reset()
  {
    state_ = State{};

    const float initialOmegaPerDshot =
      MotionModelFeedforward::hoverOmegaRadPerSecond() / DEFAULT_HOVER_DSHOT;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
      rotorEstimate_[i] = 0.0f;
      omegaPerDshot_[i] = initialOmegaPerDshot;
      state_.omegaPerDshot[i] = initialOmegaPerDshot;
      state_.dshotPerOmega[i] = 1.0f / initialOmegaPerDshot;
      state_.stale[i] = true;
      state_.ageUs[i] = UINT32_MAX;
    }

    state_.fallbackMask = 0x0F;
    state_.processNoiseScale = MAX_PROCESS_NOISE_SCALE;
  }

  const State &update(
    const uint16_t dshotCommands[MOTOR_COUNT],
    const Observation &observation,
    float dt
  )
  {
    dt = clamp(dt, 0.001f, 0.020f);

    State next;
    next.measuredMask = 0;
    next.fallbackMask = 0;

    for (size_t motor = 0; motor < MOTOR_COUNT; ++motor)
    {
      const uint16_t command = dshotCommands[motor];
      next.dshotCommand[motor] = command;

      const float commandOmega = command >= 48
        ? omegaPerDshot_[motor] * static_cast<float>(command)
        : 0.0f;

      const float motorAlpha = dt / (MOTOR_TIME_CONSTANT_S + dt);
      rotorEstimate_[motor] +=
        motorAlpha * (commandOmega - rotorEstimate_[motor]);

      const bool measurementUsable =
        observation.valid[motor] &&
        isfinite(observation.omegaRadPerSec[motor]) &&
        observation.omegaRadPerSec[motor] >= 0.0f;

      if (measurementUsable)
      {
        const float measuredOmega = observation.omegaRadPerSec[motor];

        // Correct the fallback state toward the measurement. The motion model
        // itself receives the measured value directly while it is fresh.
        rotorEstimate_[motor] += MEASUREMENT_CORRECTION_ALPHA *
          (measuredOmega - rotorEstimate_[motor]);

        if (observation.newMeasurement[motor] &&
            command >= MIN_SCALE_LEARNING_DSHOT &&
            measuredOmega >= MIN_SCALE_LEARNING_OMEGA)
        {
          const float measuredScale = clamp(
            measuredOmega / static_cast<float>(command),
            MIN_OMEGA_PER_DSHOT,
            MAX_OMEGA_PER_DSHOT
          );

          omegaPerDshot_[motor] += SCALE_LEARNING_ALPHA *
            (measuredScale - omegaPerDshot_[motor]);
        }

        next.omegaRadPerSec[motor] = measuredOmega;
        next.measured[motor] = true;
        next.stale[motor] = false;
        next.ageUs[motor] = observation.ageUs[motor];
        next.measuredMask |= static_cast<uint8_t>(1U << motor);
      }
      else
      {
        next.omegaRadPerSec[motor] =
          fmaxf(0.0f, rotorEstimate_[motor]);
        next.measured[motor] = false;
        next.stale[motor] = true;
        next.ageUs[motor] = observation.ageUs[motor];
        next.fallbackMask |= static_cast<uint8_t>(1U << motor);
      }

      next.fallbackOmegaRadPerSec[motor] =
        fmaxf(0.0f, rotorEstimate_[motor]);
      next.omegaPerDshot[motor] = omegaPerDshot_[motor];
      next.dshotPerOmega[motor] = 1.0f /
        fmaxf(omegaPerDshot_[motor], MIN_OMEGA_PER_DSHOT);
    }

    next.allMeasured = next.measuredMask == 0x0FU;

    const uint8_t measuredCount = popcount4(next.measuredMask);
    next.processNoiseScale = clamp(
      1.0f +
        static_cast<float>(MOTOR_COUNT - measuredCount) *
        PROCESS_NOISE_PER_MISSING_MOTOR,
      1.0f,
      MAX_PROCESS_NOISE_SCALE
    );

    computeForcesAndTorques(next);
    state_ = next;
    return state_;
  }

  const State &state() const
  {
    return state_;
  }

  void setTorqueCoefficientKm(float coefficient)
  {
    if (isfinite(coefficient) && coefficient >= 0.0f)
    {
      torqueCoefficientKm_ = coefficient;
    }
  }

  float torqueCoefficientKm() const
  {
    return torqueCoefficientKm_;
  }

  static constexpr float motorTimeConstantSeconds()
  {
    return MOTOR_TIME_CONSTANT_S;
  }

private:
  static constexpr float DEFAULT_HOVER_DSHOT = 505.0f;
  static constexpr float MOTOR_TIME_CONSTANT_S = 0.080f;
  static constexpr float MEASUREMENT_CORRECTION_ALPHA = 0.65f;
  static constexpr float SCALE_LEARNING_ALPHA = 0.020f;
  static constexpr uint16_t MIN_SCALE_LEARNING_DSHOT = 100;
  static constexpr float MIN_SCALE_LEARNING_OMEGA = 100.0f;
  static constexpr float MIN_OMEGA_PER_DSHOT = 0.50f;
  static constexpr float MAX_OMEGA_PER_DSHOT = 4.00f;
  static constexpr float PROCESS_NOISE_PER_MISSING_MOTOR = 0.25f;
  static constexpr float MAX_PROCESS_NOISE_SCALE = 2.0f;

  // Rotor angular-velocity sign from the supplied Python model:
  // M1/M3 CCW positive, M2/M4 CW negative.
  static constexpr float ROTOR_SPIN_SIGN[MOTOR_COUNT] = {
    +1.0f, -1.0f, +1.0f, -1.0f
  };

  // Reaction torque on the airframe is opposite rotor spin direction.
  static constexpr float BODY_REACTION_TORQUE_SIGN[MOTOR_COUNT] = {
    -1.0f, +1.0f, -1.0f, +1.0f
  };

  float rotorEstimate_[MOTOR_COUNT] = {};
  float omegaPerDshot_[MOTOR_COUNT] = {};
  float torqueCoefficientKm_ = 0.0f;
  State state_;

  static float clamp(float value, float minimum, float maximum)
  {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
  }

  static uint8_t popcount4(uint8_t value)
  {
    value &= 0x0FU;
    uint8_t count = 0;
    while (value != 0)
    {
      value &= static_cast<uint8_t>(value - 1U);
      count++;
    }
    return count;
  }

  void computeForcesAndTorques(State &state) const
  {
    const float halfSideToSide =
      0.5f * MotionModelFeedforward::frameLengthSideSideMeters();
    const float halfFrontToBack =
      0.5f * MotionModelFeedforward::frameWidthFrontBackMeters();

    // Motor positions in the right-handed model frame (+X right, +Y forward):
    // M1 front-left, M2 front-right, M3 rear-right, M4 rear-left.
    const float motorX[MOTOR_COUNT] = {
      -halfSideToSide,
      +halfSideToSide,
      +halfSideToSide,
      -halfSideToSide
    };

    const float motorY[MOTOR_COUNT] = {
      +halfFrontToBack,
      +halfFrontToBack,
      -halfFrontToBack,
      -halfFrontToBack
    };

    float totalThrust = 0.0f;
    float torqueX = 0.0f;
    float torqueY = 0.0f;
    float torqueZ = 0.0f;
    float rotorMomentumZ = 0.0f;

    for (size_t motor = 0; motor < MOTOR_COUNT; ++motor)
    {
      const float omega = fmaxf(0.0f, state.omegaRadPerSec[motor]);
      const float omegaSquared = omega * omega;
      const float thrust =
        MotionModelFeedforward::thrustCoefficientKf() * omegaSquared;

      state.rotorThrustN[motor] = thrust;
      totalThrust += thrust;

      // r x F for F=[0,0,+thrust].
      torqueX += motorY[motor] * thrust;
      torqueY -= motorX[motor] * thrust;

      torqueZ += BODY_REACTION_TORQUE_SIGN[motor] *
        torqueCoefficientKm_ * omegaSquared;

      rotorMomentumZ += ROTOR_SPIN_SIGN[motor] *
        MotionModelFeedforward::rotorInertiaKgM2() * omega;
    }

    state.forceBodyN[0] = 0.0f;
    state.forceBodyN[1] = 0.0f;
    state.forceBodyN[2] = totalThrust;

    state.torqueBodyNm[0] = torqueX;
    state.torqueBodyNm[1] = torqueY;
    state.torqueBodyNm[2] = torqueZ;
    state.rotorAngularMomentumZ = rotorMomentumZ;
  }
};

constexpr float RotorFeedbackModel::ROTOR_SPIN_SIGN[
  RotorFeedbackModel::MOTOR_COUNT
];

constexpr float RotorFeedbackModel::BODY_REACTION_TORQUE_SIGN[
  RotorFeedbackModel::MOTOR_COUNT
];
