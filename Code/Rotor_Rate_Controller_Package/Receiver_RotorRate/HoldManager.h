#pragma once

#include "FrameConventions.h"

/*
  Shaped pilot stick input for one control cycle.

  This lives in a header rather than in the sketch because the Arduino builder
  injects generated function prototypes above the first function definition,
  which puts them ahead of any type declared partway down a .ino file.
*/
struct StickCommands
{
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float throttle = 0.0f;

  bool rollPitchActive = false;
  bool yawActive = false;
  bool throttleActive = false;
};

/*
  ============================================================================
  HOLD ENGAGEMENT LOGIC
  ============================================================================

  Two small state machines that decide WHEN a hold owns an axis. Keeping this
  separate from the controllers matters because the old build's XY hold armed
  on a single boolean expression evaluated mid-takeoff: it captured a position
  target at 0.27 m while climbing at 0.43 m/s, which is the worst possible
  moment to establish a navigation origin.

  SETTLE-THEN-CAPTURE

  While a stick is held the axis is FREE and the pilot has it with no
  resistance from the hold. On release the axis does not snap to a target
  immediately, because the vehicle still has momentum and would capture a
  point it is moving away from, overshoot, and pull back. Instead it waits for
  the measured speed on that axis to fall below a threshold and stay there for
  a dwell, then captures.

  The cost is a small drift during the settle. The benefit is that every
  captured target is a point the vehicle is actually at and nearly stationary
  over.
*/
class SettleCapture
{
public:
  enum class State : uint8_t
  {
    Free = 0,      // pilot has the axis, or holding is not permitted
    Settling = 1,  // stick released, waiting for the axis to slow down
    Held = 2       // target captured, hold controller active
  };

  // Configuration.
  float settleSpeedThreshold = 0.15f;  // units per second, axis dependent
  uint32_t settleDwellMs = 400;

  void reset()
  {
    state_ = State::Free;
    settleStartMs_ = 0;
    justCaptured_ = false;
    justReleased_ = false;
  }

  /*
    stickActive    the pilot is commanding this axis right now
    measuredSpeed  magnitude of the axis rate, same units as the threshold
    holdPermitted  every precondition for this hold is currently satisfied
  */
  State update(
    bool stickActive,
    float measuredSpeed,
    bool holdPermitted,
    uint32_t nowMs
  )
  {
    justCaptured_ = false;
    justReleased_ = false;

    const State previous = state_;

    if (stickActive || !holdPermitted)
    {
      state_ = State::Free;
      settleStartMs_ = 0;

      if (previous == State::Held)
      {
        justReleased_ = true;
      }

      return state_;
    }

    if (state_ == State::Held)
    {
      return state_;
    }

    // Free or Settling, with holding now permitted and no stick input.
    if (!isfinite(measuredSpeed) ||
        fabsf(measuredSpeed) > settleSpeedThreshold)
    {
      state_ = State::Settling;
      settleStartMs_ = 0;
      return state_;
    }

    state_ = State::Settling;

    if (settleStartMs_ == 0)
    {
      settleStartMs_ = nowMs;
      if (settleStartMs_ == 0) settleStartMs_ = 1;  // 0 is the sentinel
      return state_;
    }

    if (static_cast<uint32_t>(nowMs - settleStartMs_) >= settleDwellMs)
    {
      state_ = State::Held;
      settleStartMs_ = 0;
      justCaptured_ = true;
    }

    return state_;
  }

  State state() const { return state_; }
  bool held() const { return state_ == State::Held; }
  bool justCaptured() const { return justCaptured_; }
  bool justReleased() const { return justReleased_; }

  const char *stateName() const
  {
    switch (state_)
    {
      case State::Free: return "FREE";
      case State::Settling: return "SETTLING";
      case State::Held: return "HELD";
    }
    return "?";
  }

private:
  State state_ = State::Free;
  uint32_t settleStartMs_ = 0;
  bool justCaptured_ = false;
  bool justReleased_ = false;
};

/*
  ============================================================================
  YAW STABILITY PROOF
  ============================================================================

  XY hold is a navigation-frame controller. It rotates body velocity and
  position error through the heading, so it cannot hold a fixed point while
  the heading itself is wrong or oscillating. The previous build's log showed
  the yaw loop reversing between +118 and -120 deg/s while XY hold was active
  and the position error grew past 1.5 m; no XY gain could have fixed that.

  This monitor makes XY hold wait for evidence, within the current flight,
  that the yaw loop is actually behaving. It is not a boot-time flag and not
  an operator promise: it latches only after the yaw rate and heading error
  have both stayed inside a tight band continuously for a required period
  while airborne. Disarming clears it, so every flight re-proves it.
*/
class YawStabilityMonitor
{
public:
  float maxYawRateDps = 10.0f;
  float maxHeadingErrorDeg = 8.0f;
  uint32_t requiredStableMs = 3000;

  void reset()
  {
    stableStartMs_ = 0;
    proven_ = false;
    stableForMs_ = 0;
  }

  void update(
    float yawRateDps,
    float headingErrorDeg,
    bool flightActive,
    bool headingHoldActive,
    uint32_t nowMs
  )
  {
    if (!flightActive)
    {
      reset();
      return;
    }

    const bool withinBand =
      isfinite(yawRateDps) &&
      isfinite(headingErrorDeg) &&
      fabsf(yawRateDps) <= maxYawRateDps &&
      (!headingHoldActive ||
        fabsf(headingErrorDeg) <= maxHeadingErrorDeg);

    if (!withinBand)
    {
      stableStartMs_ = 0;
      stableForMs_ = 0;
      // Deliberately not clearing proven_. Once the yaw loop has demonstrated
      // control in this flight, a single gust should not drop XY hold; the
      // XY controller has its own abort conditions for genuine yaw upsets.
      return;
    }

    if (stableStartMs_ == 0)
    {
      stableStartMs_ = nowMs;
      if (stableStartMs_ == 0) stableStartMs_ = 1;
    }

    stableForMs_ = static_cast<uint32_t>(nowMs - stableStartMs_);

    if (stableForMs_ >= requiredStableMs)
    {
      proven_ = true;
    }
  }

  bool proven() const { return proven_; }
  uint32_t stableForMs() const { return stableForMs_; }

private:
  uint32_t stableStartMs_ = 0;
  uint32_t stableForMs_ = 0;
  bool proven_ = false;
};
