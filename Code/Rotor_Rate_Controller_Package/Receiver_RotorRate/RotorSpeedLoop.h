#pragma once

#include "FrameConventions.h"

/*
  ============================================================================
  PER-ROTOR ANGULAR VELOCITY LOOP
  ============================================================================

  Closes the loop on measured mechanical rotor speed from Bluejay bidirectional
  DShot. The allocator hands this class four angular velocity setpoints in
  rad/s; this class produces four DShot commands.

  Why this matters beyond tidiness: with an open-loop DShot mixer, every motor
  and ESC nonlinearity, every battery sag, and every prop difference shows up
  directly as an attitude disturbance the outer PID has to chase. Closing the
  rotor loop turns the four actuators into something that actually matches the
  rigid-body model the allocator inverted, so the outer loops can be tuned
  against physics instead of against motor curves.

  STRUCTURE PER ROTOR

      dshot = feedforward(omegaSetpoint) + PI(omegaSetpoint - omegaMeasured)

  The feedforward uses a slowly learned gain dshotPerOmega. The relation is not
  actually through the origin (ESCs have an idle offset and the curve bends),
  so the integrator is left to absorb the offset rather than trying to fit an
  affine model on an ESP32 in flight.

  TELEMETRY LOSS

  Bidirectional DShot drops packets. When a rotor's RPM goes stale the loop
  holds its integrator, stops learning, and runs feedforward only. It does not
  keep integrating against a frozen measurement, which would wind up and then
  slam the motor when telemetry returned.
*/
class RotorSpeedLoop
{
public:
  static constexpr size_t MOTOR_COUNT = 4;

  static constexpr uint16_t DSHOT_IDLE = 48;
  static constexpr uint16_t DSHOT_FLIGHT_MIN = 48;
  static constexpr uint16_t DSHOT_FLIGHT_MAX = 750;

  struct Measurement
  {
    float omegaRadPerSec[MOTOR_COUNT] = {};
    bool valid[MOTOR_COUNT] = {};
  };

  struct Status
  {
    uint16_t dshotCommand[MOTOR_COUNT] = {};
    float omegaSetpoint[MOTOR_COUNT] = {};
    float omegaMeasured[MOTOR_COUNT] = {};
    float omegaEstimated[MOTOR_COUNT] = {};
    float omegaError[MOTOR_COUNT] = {};
    float feedforwardDshot[MOTOR_COUNT] = {};
    float integralDshot[MOTOR_COUNT] = {};
    float dshotPerOmega[MOTOR_COUNT] = {};
    bool usingMeasurement[MOTOR_COUNT] = {};
    uint8_t measuredMask = 0;
    bool anySaturated = false;
  };

  // Runtime-tunable gains, in DShot counts per (rad/s) and per (rad/s)*s.
  float kp = 0.15f;
  float ki = 2.00f;

  /*
    Setpoint slew limit, rad/s^2.

    A rotor cannot follow a step. Bench testing from standstill to 500 rad/s
    showed the measured speed overshooting to 635 rad/s, 27% high, because the
    integrator kept accumulating during the ~50 ms the motor physically needed
    to get there. With props off that is harmless. At hover it would be a
    momentary 60% thrust excursion, because thrust goes as omega squared.

    4000 rad/s^2 lets a differential command move 80 rad/s in 20 ms, which is
    far more than the attitude loop asks for, while removing the step.
  */
  float maxSlewRadPerSec2 = 4000.0f;

  RotorSpeedLoop()
  {
    reset();
  }

  void reset()
  {
    /*
      Seeded from bench measurement on this airframe, not a guess.

      Sweeping 400/500/700/900 rad/s gave DShot 162/204/291/384, which fits
      dshot = 0.444*omega - 15.6 closely. The loop models gain only and lets
      the integrator take the offset, so the right seed is the slope-through-
      hover value: 0.444*875 - 15.6 = 373 DShot at 875 rad/s.

      The previous 490/875 = 0.560 seed was 33% high and made the loop
      command far too much DShot on the first step before it learned better.
    */
    const float nominalOmega = hoverOmegaRadPerSec();
    const float nominalDshot = 373.0f;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
      dshotPerOmega_[i] = nominalDshot / nominalOmega;
      integral_[i] = 0.0f;
      omegaEstimate_[i] = 0.0f;
      slewedSetpoint_[i] = 0.0f;
      lastCommand_[i] = 0;
    }
  }

  // Clear only the integrators. Used when flight is enabled so the loop does
  // not start with accumulated ground error.
  void resetIntegrators()
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
      integral_[i] = 0.0f;
    }
  }

  Status update(
    const float omegaSetpointRadPerSec[MOTOR_COUNT],
    const Measurement &measurement,
    float dt
  )
  {
    Status status;

    dt = clampFloatFc(dt, 0.0005f, 0.020f);

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
      const float requested = clampFloatFc(
        omegaSetpointRadPerSec[i],
        0.0f,
        2000.0f
      );

      // Slew-limit the setpoint so the loop never demands a step the rotor
      // cannot physically follow.
      const float maximumStep = maxSlewRadPerSec2 * dt;
      const float slewError = requested - slewedSetpoint_[i];

      if (slewError > maximumStep) slewedSetpoint_[i] += maximumStep;
      else if (slewError < -maximumStep) slewedSetpoint_[i] -= maximumStep;
      else slewedSetpoint_[i] = requested;

      const float setpoint = slewedSetpoint_[i];
      status.omegaSetpoint[i] = setpoint;

      // ----------------------------------------------------------
      // Open-loop rotor state estimate, always maintained. It carries the
      // loop through telemetry gaps and seeds the learned gain before any
      // measurement has arrived.
      // ----------------------------------------------------------

      const float commandedOmega =
        (lastCommand_[i] >= DSHOT_IDLE && dshotPerOmega_[i] > 1e-6f)
          ? static_cast<float>(lastCommand_[i]) / dshotPerOmega_[i]
          : 0.0f;

      const float motorAlpha = dt / (MOTOR_TIME_CONSTANT_S + dt);
      omegaEstimate_[i] += motorAlpha * (commandedOmega - omegaEstimate_[i]);

      const bool measurementUsable =
        measurement.valid[i] &&
        isfinite(measurement.omegaRadPerSec[i]) &&
        measurement.omegaRadPerSec[i] >= 0.0f;

      if (measurementUsable)
      {
        status.measuredMask |= static_cast<uint8_t>(1u << i);

        // Pull the open-loop estimate toward truth so a later telemetry gap
        // starts from the right place.
        omegaEstimate_[i] +=
          ESTIMATE_CORRECTION_ALPHA *
          (measurement.omegaRadPerSec[i] - omegaEstimate_[i]);
      }

      const float omegaForControl = measurementUsable
        ? measurement.omegaRadPerSec[i]
        : omegaEstimate_[i];

      status.omegaMeasured[i] =
        measurementUsable ? measurement.omegaRadPerSec[i] : 0.0f;
      status.omegaEstimated[i] = omegaEstimate_[i];
      status.usingMeasurement[i] = measurementUsable;

      // ----------------------------------------------------------
      // Learn the DShot-per-omega gain. Only from fresh measurements, only
      // while actually spinning, and only slowly. A bad gain here degrades
      // to a feedforward error the integrator can absorb.
      // ----------------------------------------------------------

      if (measurementUsable &&
          measurement.omegaRadPerSec[i] > LEARN_MIN_OMEGA &&
          lastCommand_[i] >= LEARN_MIN_DSHOT)
      {
        const float observedRatio =
          static_cast<float>(lastCommand_[i]) /
          measurement.omegaRadPerSec[i];

        if (isfinite(observedRatio) &&
            observedRatio > MIN_DSHOT_PER_OMEGA &&
            observedRatio < MAX_DSHOT_PER_OMEGA)
        {
          dshotPerOmega_[i] +=
            GAIN_LEARN_ALPHA * (observedRatio - dshotPerOmega_[i]);
        }
      }

      status.dshotPerOmega[i] = dshotPerOmega_[i];

      // ----------------------------------------------------------
      // Feedforward + PI.
      // ----------------------------------------------------------

      const float feedforward = dshotPerOmega_[i] * setpoint;
      status.feedforwardDshot[i] = feedforward;

      const float error = setpoint - omegaForControl;
      status.omegaError[i] = error;

      /*
        Integrate only when telemetry is present AND the error is small
        enough to be a genuine steady-state offset rather than a transient.

        A large error means the rotor is still accelerating toward a setpoint
        it will reach on its own. Integrating through that is what produced
        the 27% overshoot on the bench; the integrator's job here is to
        absorb the ESC's idle offset, not to force acceleration.
      */
      /*
        The band is a PERCENTAGE of the setpoint, floored at the old absolute
        value. It used to be the flat 60 rad/s alone, and that flat number
        deadlocked the loop on 2026-08-15.

        At a 1045 rad/s hover-climb setpoint the rotors settled at 964, an
        81 rad/s standing offset. That is outside a 60 rad/s band, so the
        integrator was frozen; the only correction left was kp * error, which
        at kp = 0.15 is about 12 DShot counts against the ~90 needed. So the
        error stayed at 81, which kept the integrator frozen, which kept the
        error at 81. The aircraft sat in Climb commanding +0.30 m/s and never
        rose, with DShot at ~420 and the flight cap at 750 - three quarters of
        the authority unused.

        Scaling with setpoint keeps the original intent intact. Spool-up
        transients carry errors of several hundred rad/s and stay well outside
        the band, so the integrator still cannot force acceleration and the
        bench overshoot that motivated this guard does not return. What it no
        longer does is declare a genuine steady-state offset untouchable just
        because the aircraft happens to be running fast.
      */
      const float integralBand = fmaxf(
        INTEGRAL_ENABLE_BAND, INTEGRAL_ENABLE_FRACTION * fabsf(setpoint)
      );

      if (measurementUsable && fabsf(error) <= integralBand)
      {
        integral_[i] += ki * error * dt;
        integral_[i] = clampFloatFc(
          integral_[i],
          -INTEGRAL_LIMIT_DSHOT,
          INTEGRAL_LIMIT_DSHOT
        );
      }

      float command = feedforward + kp * error + integral_[i];

      // Anti-windup: if the command is pinned and the integrator is pushing
      // further into the stop, unwind it.
      if (command > static_cast<float>(DSHOT_FLIGHT_MAX))
      {
        status.anySaturated = true;
        const float excess = command - static_cast<float>(DSHOT_FLIGHT_MAX);
        if (integral_[i] > 0.0f)
        {
          integral_[i] = fmaxf(0.0f, integral_[i] - excess);
        }
        command = static_cast<float>(DSHOT_FLIGHT_MAX);
      }
      else if (command < static_cast<float>(DSHOT_FLIGHT_MIN))
      {
        status.anySaturated = true;
        const float deficit = static_cast<float>(DSHOT_FLIGHT_MIN) - command;
        if (integral_[i] < 0.0f)
        {
          integral_[i] = fminf(0.0f, integral_[i] + deficit);
        }
        command = static_cast<float>(DSHOT_FLIGHT_MIN);
      }

      status.integralDshot[i] = integral_[i];

      if (!isfinite(command))
      {
        command = static_cast<float>(DSHOT_FLIGHT_MIN);
      }

      lastCommand_[i] = static_cast<uint16_t>(
        clampFloatFc(
          roundf(command),
          static_cast<float>(DSHOT_FLIGHT_MIN),
          static_cast<float>(DSHOT_FLIGHT_MAX)
        )
      );

      status.dshotCommand[i] = lastCommand_[i];
    }

    return status;
  }

  // Called when motors are commanded stopped so the open-loop estimate and
  // the learned state do not drift while the rotors coast down.
  void notifyStopped()
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
      integral_[i] = 0.0f;
      omegaEstimate_[i] = 0.0f;
      slewedSetpoint_[i] = 0.0f;
      lastCommand_[i] = 0;
    }
  }

  float dshotPerOmega(size_t motorIndex) const
  {
    return (motorIndex < MOTOR_COUNT) ? dshotPerOmega_[motorIndex] : 0.0f;
  }

private:
  static constexpr float MOTOR_TIME_CONSTANT_S = 0.050f;
  static constexpr float ESTIMATE_CORRECTION_ALPHA = 0.35f;
  static constexpr float GAIN_LEARN_ALPHA = 0.002f;

  static constexpr float LEARN_MIN_OMEGA = 300.0f;
  static constexpr uint16_t LEARN_MIN_DSHOT = 200;

  static constexpr float MIN_DSHOT_PER_OMEGA = 0.20f;
  static constexpr float MAX_DSHOT_PER_OMEGA = 1.50f;

  static constexpr float INTEGRAL_LIMIT_DSHOT = 120.0f;

  // Integrate only inside this error band, in rad/s. See update().
  static constexpr float INTEGRAL_ENABLE_BAND = 60.0f;

  /*
    Fraction of setpoint. 10% of a 1045 rad/s setpoint is a 105 rad/s band,
    which admits the 81 rad/s steady offset seen in flight while still
    excluding the several-hundred rad/s errors of a spool-up.
  */
  static constexpr float INTEGRAL_ENABLE_FRACTION = 0.10f;

  float dshotPerOmega_[MOTOR_COUNT] = {};
  float integral_[MOTOR_COUNT] = {};
  float omegaEstimate_[MOTOR_COUNT] = {};
  float slewedSetpoint_[MOTOR_COUNT] = {};
  uint16_t lastCommand_[MOTOR_COUNT] = {};
};
