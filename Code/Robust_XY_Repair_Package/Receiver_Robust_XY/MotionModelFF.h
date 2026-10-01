#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace motion_model_detail
{
  constexpr float square(float value)
  {
    return value * value;
  }
}

/*
  Real-time C++ port of the supplied Python inverse-dynamics operation loop.

  The original model computes desired body rates from the change in normalized
  pitch/roll commands, computes the total and differential rotor thrusts, then
  solves a scalar lambda root so body/rotor yaw angular momentum is consistent.

  Embedded additions:
    - command smoothing and rate/acceleration limits, because a 250 Hz stick
      step with the original finite differences would otherwise request an
      impossible impulse;
    - bisection instead of scipy.optimize.brentq;
    - explicit current-airframe mixer signs;
    - conversion from predicted rotor rad/s to a DShot feed-forward delta;
    - optional measured rotor-state input from Bluejay bidirectional DShot.

  Without RPM, the DShot/rad-s map falls back to the original local hover
  approximation. With RPM, the caller supplies a slowly learned local scale
  and current rotor angular momentum. The feedback PID remains authoritative
  and the feed-forward blend is intentionally limited.
*/
class MotionModelFeedforward
{
public:
  struct RotorFeedbackInput
  {
    // Positive mechanical rotor-speed magnitudes. Rotation direction remains
    // a separate model sign; clockwise motors do not use negative RPM.
    float omegaRadPerSecond[4] = {};
    float omegaPerDshot[4] = {};
    bool measured[4] = {};
    bool stale[4] = {};
    uint32_t ageUs[4] = {};
  };

  struct Output
  {
    bool valid = false;
    bool saturated = false;
    bool rotorFeedbackUsed = false;
    uint8_t measuredRotorMask = 0;
    float rotorOmegaRadPerSecond[4] = {};
    float rotorOmegaInputRadPerSecond[4] = {};
    float omegaPerDshotUsed[4] = {};
    float motorDeltaDshot[4] = {};
    float collectiveDeltaDshot = 0.0f;
    float desiredPitchRateDps = 0.0f;
    float desiredRollRateDps = 0.0f;
    float inducedYawRateDps = 0.0f;
    float lambdaNewtons = 0.0f;
    float demandedAccelerationZ = 0.0f;
    float rotorAngularMomentumInput = 0.0f;
  };

  MotionModelFeedforward()
  {
    reset();
  }

  void reset()
  {
    memset(signal_, 0, sizeof(signal_));
    memset(previousSignal_, 0, sizeof(previousSignal_));
    memset(previousDesiredBodyRate_, 0, sizeof(previousDesiredBodyRate_));
    initialized_ = false;
    lastOutput_ = Output{};
  }

  Output update(
    float desiredPitchDeg,
    float desiredRollDeg,
    float desiredYawRateClockwiseDps,
    float desiredVerticalVelocityMetersPerSecond,
    float dt,
    float hoverDshot,
    const RotorFeedbackInput *rotorFeedback = nullptr
  )
  {
    Output output;

    if (!isfinite(desiredPitchDeg) || !isfinite(desiredRollDeg) ||
        !isfinite(desiredYawRateClockwiseDps) ||
        !isfinite(desiredVerticalVelocityMetersPerSecond) ||
        !isfinite(dt) || !isfinite(hoverDshot))
    {
      return output;
    }

    dt = clamp(dt, 0.002f, 0.010f);
    hoverDshot = clamp(hoverDshot, 430.0f, 560.0f);

    float currentRotorOmega[4] = {
      HOVER_OMEGA, HOVER_OMEGA, HOVER_OMEGA, HOVER_OMEGA
    };

    const float defaultOmegaPerDshot = HOVER_OMEGA / hoverDshot;
    float omegaPerDshot[4] = {
      defaultOmegaPerDshot,
      defaultOmegaPerDshot,
      defaultOmegaPerDshot,
      defaultOmegaPerDshot
    };

    if (rotorFeedback != nullptr)
    {
      output.rotorFeedbackUsed = true;

      for (int motor = 0; motor < 4; ++motor)
      {
        const float suppliedOmega = rotorFeedback->omegaRadPerSecond[motor];
        if (isfinite(suppliedOmega) && suppliedOmega >= 0.0f)
        {
          currentRotorOmega[motor] = suppliedOmega;
        }

        const float suppliedScale = rotorFeedback->omegaPerDshot[motor];
        if (isfinite(suppliedScale) && suppliedScale >= 0.50f &&
            suppliedScale <= 4.00f)
        {
          omegaPerDshot[motor] = suppliedScale;
        }

        if (rotorFeedback->measured[motor] &&
            !rotorFeedback->stale[motor])
        {
          output.measuredRotorMask |= static_cast<uint8_t>(1U << motor);
        }
      }
    }

    for (int motor = 0; motor < 4; ++motor)
    {
      output.rotorOmegaInputRadPerSecond[motor] = currentRotorOmega[motor];
      output.omegaPerDshotUsed[motor] = omegaPerDshot[motor];
    }

    const float pitchRad = clamp(
      desiredPitchDeg * MODEL_DEG_TO_RAD,
      -MAX_COMMAND_TILT_RAD,
      MAX_COMMAND_TILT_RAD
    );
    const float rollRad = clamp(
      desiredRollDeg * MODEL_DEG_TO_RAD,
      -MAX_COMMAND_TILT_RAD,
      MAX_COMMAND_TILT_RAD
    );

    float targetSignal[4] = {
      clamp(sinf(pitchRad) / SIN_MAX_TILT, -0.95f, 0.95f),
      clamp(sinf(rollRad) / SIN_MAX_TILT, -0.95f, 0.95f),
      clamp(
        -desiredYawRateClockwiseDps * MODEL_DEG_TO_RAD / MAX_YAW_RATE_RAD_PER_SECOND,
        -0.50f,
        0.50f
      ),
      clamp(
        desiredVerticalVelocityMetersPerSecond / MAX_ASCENT_VELOCITY,
        -0.80f,
        0.80f
      )
    };

    if (!initialized_)
    {
      memcpy(signal_, targetSignal, sizeof(signal_));
      memcpy(previousSignal_, signal_, sizeof(previousSignal_));
      memset(previousDesiredBodyRate_, 0, sizeof(previousDesiredBodyRate_));
      initialized_ = true;

      for (int i = 0; i < 4; ++i)
      {
        output.rotorOmegaRadPerSecond[i] = HOVER_OMEGA;
      }
      output.valid = true;
      lastOutput_ = output;
      return output;
    }

    previousSignal_[0] = signal_[0];
    previousSignal_[1] = signal_[1];
    previousSignal_[2] = signal_[2];
    previousSignal_[3] = signal_[3];

    signal_[0] = smoothAndSlew(
      signal_[0], targetSignal[0], dt, 0.12f, 2.8f
    );
    signal_[1] = smoothAndSlew(
      signal_[1], targetSignal[1], dt, 0.12f, 2.8f
    );
    signal_[2] = smoothAndSlew(
      signal_[2], targetSignal[2], dt, 0.18f, 2.0f
    );
    signal_[3] = smoothAndSlew(
      signal_[3], targetSignal[3], dt, 0.20f, 1.5f
    );

    const float sigPitch = signal_[0];
    const float sigRoll = signal_[1];

    const float sSquared = fmaxf(
      0.05f,
      1.0f - square(sigPitch * SIN_MAX_TILT)
    );
    const float s = sqrtf(sSquared);

    const float zSquared = fmaxf(
      0.05f,
      1.0f -
        (square(sigPitch) + square(sigRoll)) * square(SIN_MAX_TILT)
    );
    const float z = sqrtf(zSquared);

    float desiredPitchRate = -(
      sigRoll * sigPitch *
        (signal_[1] - previousSignal_[1]) *
        cube(SIN_MAX_TILT) +
      sSquared *
        (signal_[0] - previousSignal_[0]) * SIN_MAX_TILT
    ) / (z * s * dt);

    float desiredRollRate =
      (signal_[1] - previousSignal_[1]) *
      SIN_MAX_TILT / (s * dt);

    float inducedYawRate =
      sigRoll * SIN_MAX_TILT * desiredPitchRate / s;

    desiredPitchRate = clamp(
      desiredPitchRate,
      -MAX_BODY_RATE_RAD_PER_SECOND,
      MAX_BODY_RATE_RAD_PER_SECOND
    );
    desiredRollRate = clamp(
      desiredRollRate,
      -MAX_BODY_RATE_RAD_PER_SECOND,
      MAX_BODY_RATE_RAD_PER_SECOND
    );
    inducedYawRate = clamp(
      inducedYawRate,
      -MAX_INDUCED_YAW_RATE_RAD_PER_SECOND,
      MAX_INDUCED_YAW_RATE_RAD_PER_SECOND
    );

    float pitchAngularAcceleration = clamp(
      (desiredPitchRate - previousDesiredBodyRate_[0]) / dt,
      -MAX_ANGULAR_ACCELERATION,
      MAX_ANGULAR_ACCELERATION
    );

    float rollAngularAcceleration = clamp(
      (desiredRollRate - previousDesiredBodyRate_[1]) / dt,
      -MAX_ANGULAR_ACCELERATION,
      MAX_ANGULAR_ACCELERATION
    );

    const float verticalAcceleration = clamp(
      (signal_[3] - previousSignal_[3]) /
        dt * MAX_ASCENT_VELOCITY,
      -MAX_VERTICAL_ACCELERATION,
      MAX_VERTICAL_ACCELERATION
    );

    output.demandedAccelerationZ = verticalAcceleration;

    float accelerationMagnitude =
      (verticalAcceleration + GRAVITY) / z;
    accelerationMagnitude = clamp(
      accelerationMagnitude,
      0.25f * GRAVITY,
      1.80f * GRAVITY
    );

    // The supplied Python C matrix is preserved, then mapped into the
    // already-validated receiver motor ordering.  In this airframe's command
    // convention, a positive pitch command raises the rear pair, while a
    // positive roll correction raises the right pair.  The Python pitch-rate
    // expression already has the required opposite mathematical sign, while
    // the roll torque requires a sign reversal for this motor numbering.
    const float totalForce = MASS * accelerationMagnitude;
    const float pitchTorqueEquivalent =
      2.0f * FRAME_WIDTH_FRONT_BACK * MASS *
      pitchAngularAcceleration;
    const float rollTorqueEquivalent =
      -2.0f * FRAME_LENGTH_SIDE_SIDE * MASS *
      rollAngularAcceleration;

    float C[4] = {
      0.25f * (totalForce + pitchTorqueEquivalent + rollTorqueEquivalent),
      0.25f * (totalForce + pitchTorqueEquivalent - rollTorqueEquivalent),
      0.25f * (totalForce - pitchTorqueEquivalent - rollTorqueEquivalent),
      0.25f * (totalForce - pitchTorqueEquivalent + rollTorqueEquivalent)
    };

    const float desiredYawRateRightHand =
      signal_[2] * MAX_YAW_RATE_RAD_PER_SECOND;

    const float bodyYawAngularMomentum =
      BODY_YAW_INERTIA_APPROX *
      (inducedYawRate + desiredYawRateRightHand);

    const float currentRotorMomentum =
      ROTOR_INERTIA_1 * currentRotorOmega[0] -
      ROTOR_INERTIA_2 * currentRotorOmega[1] +
      ROTOR_INERTIA_3 * currentRotorOmega[2] -
      ROTOR_INERTIA_4 * currentRotorOmega[3];

    output.rotorAngularMomentumInput = currentRotorMomentum;
    const float L = bodyYawAngularMomentum + currentRotorMomentum;

    const float lowerBound =
      fmaxf(-C[1], -C[3]) + MIN_THRUST_NEWTONS;
    const float upperBound =
      fminf(C[0], C[2]) - MIN_THRUST_NEWTONS;

    if (!isfinite(lowerBound) || !isfinite(upperBound) ||
        lowerBound >= upperBound)
    {
      lastOutput_ = output;
      return output;
    }

    bool rootSaturated = false;
    float lambda = 0.0f;
    if (!solveLambda(C, L, lowerBound, upperBound, lambda, rootSaturated))
    {
      lastOutput_ = output;
      return output;
    }

    const float thrust[4] = {
      C[0] - lambda,
      C[1] + lambda,
      C[2] - lambda,
      C[3] + lambda
    };

    float dshotPrediction[4] = {};
    float dshotMean = 0.0f;

    for (int motor = 0; motor < 4; ++motor)
    {
      if (!isfinite(thrust[motor]) || thrust[motor] <= 0.0f)
      {
        lastOutput_ = output;
        return output;
      }

      output.rotorOmegaRadPerSecond[motor] =
        sqrtf(thrust[motor] / THRUST_COEFFICIENT);

      // Use the locally learned omega/DShot scale supplied by the RPM bridge.
      // Without telemetry, omegaPerDshot falls back to the original hover map.
      dshotPrediction[motor] =
        output.rotorOmegaRadPerSecond[motor] /
        fmaxf(omegaPerDshot[motor], 0.50f);
      dshotMean += 0.25f * dshotPrediction[motor];
    }

    const float rawCollectiveDeltaDshot = dshotMean - hoverDshot;
    bool outputLimited =
      fabsf(rawCollectiveDeltaDshot) > MAX_COLLECTIVE_DELTA_DSHOT;

    output.collectiveDeltaDshot = clamp(
      rawCollectiveDeltaDshot,
      -MAX_COLLECTIVE_DELTA_DSHOT,
      MAX_COLLECTIVE_DELTA_DSHOT
    );

    for (int motor = 0; motor < 4; ++motor)
    {
      const float rawMotorDeltaDshot =
        dshotPrediction[motor] - dshotMean;

      if (fabsf(rawMotorDeltaDshot) > MAX_MOTOR_DIFFERENTIAL_DSHOT)
      {
        outputLimited = true;
      }

      output.motorDeltaDshot[motor] = clamp(
        rawMotorDeltaDshot,
        -MAX_MOTOR_DIFFERENTIAL_DSHOT,
        MAX_MOTOR_DIFFERENTIAL_DSHOT
      );
    }

    // Convert the Python model's pitch sign into the receiver's established
    // positive-pitch-rate convention.  Roll already matches.
    output.desiredPitchRateDps = -desiredPitchRate * MODEL_RAD_TO_DEG;
    output.desiredRollRateDps = desiredRollRate * MODEL_RAD_TO_DEG;
    output.inducedYawRateDps = inducedYawRate * MODEL_RAD_TO_DEG;
    output.lambdaNewtons = lambda;
    output.saturated = rootSaturated || outputLimited;
    output.valid = true;

    previousDesiredBodyRate_[0] = desiredPitchRate;
    previousDesiredBodyRate_[1] = desiredRollRate;
    previousDesiredBodyRate_[2] = inducedYawRate;

    lastOutput_ = output;
    return output;
  }

  const Output &lastOutput() const
  {
    return lastOutput_;
  }

  // Physical parameters ported directly from the supplied Python model.
  static constexpr float MASS = 0.45359237f; // 1 pound [kg]
  static constexpr float GRAVITY = 9.80665f;
  static constexpr float FRAME_WIDTH_FRONT_BACK = 0.1645f;
  static constexpr float FRAME_LENGTH_SIDE_SIDE = 0.1630f;
  static constexpr float PROP_RADIUS = 0.0635f; // 2.5 inch [m]
  static constexpr float AIR_DENSITY = 1.225f;
  static constexpr float CT = 0.18f;
  static constexpr float HOVER_OMEGA = 961.327f;

  // constexpr accessors avoid C++11 out-of-class definitions when these
  // physical constants are used by the separate rotor-feedback bridge.
  static constexpr float massKg() { return MASS; }
  static constexpr float frameWidthFrontBackMeters()
  {
    return FRAME_WIDTH_FRONT_BACK;
  }
  static constexpr float frameLengthSideSideMeters()
  {
    return FRAME_LENGTH_SIDE_SIDE;
  }
  static constexpr float hoverOmegaRadPerSecond() { return HOVER_OMEGA; }

private:
  static constexpr float PI_F = 3.14159265358979323846f;
  static constexpr float MODEL_DEG_TO_RAD = PI_F / 180.0f;
  static constexpr float MODEL_RAD_TO_DEG = 180.0f / PI_F;

  static constexpr float MAX_TILT_RAD = PI_F / 6.0f;
  static constexpr float SIN_MAX_TILT = 0.5f;
  static constexpr float MAX_COMMAND_TILT_RAD = 12.0f * MODEL_DEG_TO_RAD;
  static constexpr float MAX_ASCENT_VELOCITY = 1.0f;
  static constexpr float MAX_YAW_RATE_RAD_PER_SECOND = 2.0f * PI_F;

  static constexpr float ROTOR_INERTIA_1 =
    0.006f * motion_model_detail::square(1.4e-2f) +
    0.0038f * motion_model_detail::square(PROP_RADIUS);
  static constexpr float ROTOR_INERTIA_2 = ROTOR_INERTIA_1;
  static constexpr float ROTOR_INERTIA_3 = ROTOR_INERTIA_1;
  static constexpr float ROTOR_INERTIA_4 = ROTOR_INERTIA_1;

  static constexpr float BODY_YAW_INERTIA_APPROX =
    MASS * (
      motion_model_detail::square(FRAME_LENGTH_SIDE_SIDE) +
      motion_model_detail::square(FRAME_WIDTH_FRONT_BACK)
    );

  static constexpr float THRUST_COEFFICIENT =
    4.0f / (PI_F * PI_F) * CT * AIR_DENSITY *
    motion_model_detail::square(motion_model_detail::square(PROP_RADIUS));

public:
  static constexpr float thrustCoefficientKf()
  {
    return THRUST_COEFFICIENT;
  }

  static constexpr float rotorInertiaKgM2()
  {
    return ROTOR_INERTIA_1;
  }

private:
  static constexpr float MAX_BODY_RATE_RAD_PER_SECOND = 2.5f;
  static constexpr float MAX_INDUCED_YAW_RATE_RAD_PER_SECOND = 1.5f;
  static constexpr float MAX_ANGULAR_ACCELERATION = 12.0f;
  static constexpr float MAX_VERTICAL_ACCELERATION = 3.5f;
  static constexpr float MIN_THRUST_NEWTONS = 1.0e-5f;
  static constexpr float MAX_COLLECTIVE_DELTA_DSHOT = 30.0f;
  static constexpr float MAX_MOTOR_DIFFERENTIAL_DSHOT = 24.0f;

  float signal_[4] = {};
  float previousSignal_[4] = {};
  float previousDesiredBodyRate_[3] = {};
  bool initialized_ = false;
  Output lastOutput_;

  static float square(float value)
  {
    return value * value;
  }

  static float cube(float value)
  {
    return value * value * value;
  }

  static float clamp(float value, float minimum, float maximum)
  {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
  }

  static float smoothAndSlew(
    float current,
    float target,
    float dt,
    float timeConstant,
    float maximumRatePerSecond
  )
  {
    const float alpha = dt / (timeConstant + dt);
    float requestedStep = alpha * (target - current);
    const float maximumStep = maximumRatePerSecond * dt;
    requestedStep = clamp(requestedStep, -maximumStep, maximumStep);
    return current + requestedStep;
  }

  static float rootFunction(
    float lambda,
    const float C[4],
    float L
  )
  {
    const float t1 = fmaxf(C[0] - lambda, MIN_THRUST_NEWTONS);
    const float t2 = fmaxf(C[1] + lambda, MIN_THRUST_NEWTONS);
    const float t3 = fmaxf(C[2] - lambda, MIN_THRUST_NEWTONS);
    const float t4 = fmaxf(C[3] + lambda, MIN_THRUST_NEWTONS);

    return
      -ROTOR_INERTIA_1 * sqrtf(t1) +
       ROTOR_INERTIA_2 * sqrtf(t2) -
       ROTOR_INERTIA_3 * sqrtf(t3) +
       ROTOR_INERTIA_4 * sqrtf(t4) -
       L * sqrtf(THRUST_COEFFICIENT);
  }

  static bool solveLambda(
    const float C[4],
    float L,
    float lower,
    float upper,
    float &lambda,
    bool &saturated
  )
  {
    float lowerValue = rootFunction(lower, C, L);
    float upperValue = rootFunction(upper, C, L);

    if (!isfinite(lowerValue) || !isfinite(upperValue))
    {
      return false;
    }

    if (fabsf(lowerValue) < 1.0e-9f)
    {
      lambda = lower;
      return true;
    }

    if (fabsf(upperValue) < 1.0e-9f)
    {
      lambda = upper;
      return true;
    }

    if (lowerValue * upperValue > 0.0f)
    {
      // The requested yaw angular momentum lies outside the feasible thrust
      // set. Use the nearest feasible endpoint rather than failing hard.
      lambda = fabsf(lowerValue) < fabsf(upperValue) ? lower : upper;
      saturated = true;
      return true;
    }

    for (int iteration = 0; iteration < 28; ++iteration)
    {
      const float middle = 0.5f * (lower + upper);
      const float middleValue = rootFunction(middle, C, L);

      if (!isfinite(middleValue))
      {
        return false;
      }

      if (fabsf(middleValue) < 1.0e-8f ||
          fabsf(upper - lower) < 1.0e-7f)
      {
        lambda = middle;
        return true;
      }

      if (lowerValue * middleValue <= 0.0f)
      {
        upper = middle;
        upperValue = middleValue;
      }
      else
      {
        lower = middle;
        lowerValue = middleValue;
      }
    }

    lambda = 0.5f * (lower + upper);
    return isfinite(lambda);
  }
};
