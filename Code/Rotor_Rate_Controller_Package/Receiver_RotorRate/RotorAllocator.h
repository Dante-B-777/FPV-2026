#pragma once

#include "FrameConventions.h"

/*
  ============================================================================
  CONTROL ALLOCATION - body wrench to per-rotor angular velocity
  ============================================================================

  This is the layer the old build never had. It used to go straight from three
  independent PID outputs to DShot counts through a hand-signed mixer, which is
  why the yaw column could sit inverted for the entire life of the project
  without anything catching it. Here the mixer is derived from geometry and
  spin direction instead of being written down, so a sign error would have to
  be a geometry error.

  Rotor i produces thrust along body +Z and a reaction torque about body +Z:

      T_i    = kF * omega_i^2                       (always >= 0, upward)
      tauZ_i = -spinSign_i * kM * omega_i^2

  With u_i = omega_i^2 and the rotor positions from FrameConventions.h,
  summing r x F over the four rotors gives

      T    = kF * (u1 + u2 + u3 + u4)
      tauX = kF * armY * (u1 + u2 - u3 - u4)     nose up
      tauY = kF * armX * (u1 - u2 - u3 + u4)     right wing down
      tauZ = -kM * (u1 - u2 + u3 - u4)           counter-clockwise

  Inverting:

      u1 = c_T + c_X + c_Y - c_Z
      u2 = c_T + c_X - c_Y + c_Z
      u3 = c_T - c_X - c_Y - c_Z
      u4 = c_T - c_X + c_Y + c_Z

  where

      c_T = T    / (4 kF)
      c_X = tauX / (4 kF armY)
      c_Y = tauY / (4 kF armX)
      c_Z = tauZ / (4 kM)

  SATURATION POLICY
  -----------------

  Priority order, highest first:  ROLL/PITCH  ->  THRUST  ->  YAW.

  Yaw is last, and that ordering is not a style choice - it follows from the
  hardware. Roll and pitch are actuated by THRUST differential across an arm;
  yaw is actuated only by rotor DRAG torque. On this airframe:

      roll/pitch authority = 4 * kF * arm * dU  =  0.336 N m
      yaw authority        = 4 * kM      * dU  =  0.041 N m

  Yaw is the weakest axis by a factor of 8. Any policy that lets the yaw
  demand draw on shared rotor range will therefore spend the whole budget on
  the one axis that can least afford to buy it - and it will spend it out of
  roll, pitch and thrust, which are what keep the aircraft upright and in the
  air.

  The 2026-08-15 flight log is exactly that failure. The yaw loop demanded
  tauZ = -0.1622 N m, four times the 0.041 N m the rotors can produce at hover
  collective. Under the OLD policy - slide the collective first, then scale all
  three torque axes by one common factor - that single impossible number:

    * slid the collective to make room, dumping and surging thrust
      (T swinging 4.26 -> 5.90 N with the throttle stick at zero, altitude
      wandering 0.31 - 0.68 m against a 0.41 m hold), and
    * scaled roll and pitch down by the SAME factor, stripping the attitude
      loops of authority at the exact moment the aircraft was tilting.

  It ended in "SAFETY STOP: excessive tilt" at 39.9 deg of roll.

  So the collective is now slid for the roll/pitch differential ONLY, and yaw
  is fitted into whatever rotor range is left over. Yaw yields; thrust and
  attitude do not yield to yaw.

  Roll and pitch are still scaled by ONE common factor when they cannot both
  fit, because independently clipping those two DOES rotate the commanded
  torque vector and is a classic cause of coupled wobble near full deflection.
  That argument never applied to yaw, which is a physically separate actuation
  mechanism rather than another component of the same thrust-difference vector.
*/
class RotorAllocator
{
public:
  struct Output
  {
    bool valid = false;
    float rotorOmegaRadPerSec[4] = {};
    float rotorThrustN[4] = {};

    // Diagnostics.
    bool collectiveSlid = false;
    bool torqueScaled = false;
    float torqueScale = 1.0f;      // applied to roll and pitch together
    float yawScale = 1.0f;         // applied to yaw alone
    bool yawScaled = false;
    float achievedThrustN = 0.0f;
    float achievedTauX = 0.0f;
    float achievedTauY = 0.0f;
    float achievedTauZ = 0.0f;
  };

  // Rotor speed bounds in rad/s. The lower bound is deliberately non-zero:
  // letting a rotor stop in flight costs hundreds of milliseconds to spin
  // back up and is a well known way to lose an axis.
  static constexpr float MIN_ROTOR_OMEGA = 250.0f;
  static constexpr float MAX_ROTOR_OMEGA = 1600.0f;

  static Output allocate(
    float thrustN,
    float tauXNm,
    float tauYNm,
    float tauZNm,
    float dragCoefficientKm
  )
  {
    Output out;

    if (!isfinite(thrustN) || !isfinite(tauXNm) ||
        !isfinite(tauYNm) || !isfinite(tauZNm) ||
        !isfinite(dragCoefficientKm) || dragCoefficientKm <= 0.0f)
    {
      return out;
    }

    const float uMin = MIN_ROTOR_OMEGA * MIN_ROTOR_OMEGA;
    const float uMax = MAX_ROTOR_OMEGA * MAX_ROTOR_OMEGA;

    float collective = thrustN / (4.0f * ROTOR_THRUST_KF);

    float dX = tauXNm / (4.0f * ROTOR_THRUST_KF * ROTOR_ARM_Y_M);
    float dY = tauYNm / (4.0f * ROTOR_THRUST_KF * ROTOR_ARM_X_M);
    float dZ = tauZNm / (4.0f * dragCoefficientKm);

    /*
      Split the per-rotor differential into the ROLL/PITCH part and the YAW
      part, because they are now prioritised separately. Summed they are the
      `differential` of the inversion above.
    */
    float rollPitch[4] = {
      +dX + dY,
      +dX - dY,
      -dX - dY,
      -dX + dY
    };

    // Unit yaw pattern; the signed yaw contribution is yawPattern[i] * dZ.
    static constexpr float yawPattern[4] = { -1.0f, +1.0f, -1.0f, +1.0f };

    // ------------------------------------------------------------
    // Step 1: slide the collective so the ROLL/PITCH differential fits.
    //
    // Yaw is deliberately excluded from this calculation. Including it is
    // what let an impossible yaw demand drag the collective - and therefore
    // the aircraft's altitude - around behind the pilot's back.
    // ------------------------------------------------------------

    float lowest = rollPitch[0];
    float highest = rollPitch[0];
    for (int i = 1; i < 4; ++i)
    {
      if (rollPitch[i] < lowest) lowest = rollPitch[i];
      if (rollPitch[i] > highest) highest = rollPitch[i];
    }

    const float requestedCollective = collective;

    if (collective + lowest < uMin)
    {
      collective = uMin - lowest;
    }
    if (collective + highest > uMax)
    {
      collective = uMax - highest;
    }

    // Keep the collective itself inside the achievable band. If the
    // differential span alone exceeds the whole rotor range this clamp wins
    // and step 2 does the rest.
    collective = clampFloatFc(collective, uMin, uMax);

    out.collectiveSlid =
      fabsf(collective - requestedCollective) > 1.0f;

    // ------------------------------------------------------------
    // Step 2: if roll/pitch still does not fit, scale those two by one
    // common factor. One factor, so the commanded tilt DIRECTION survives.
    // ------------------------------------------------------------

    float scale = 1.0f;
    for (int i = 0; i < 4; ++i)
    {
      const float value = collective + rollPitch[i];

      if (value < uMin)
      {
        const float room = collective - uMin;
        const float need = collective - value;
        if (need > 1e-6f)
        {
          const float axisScale = (room > 0.0f) ? (room / need) : 0.0f;
          if (axisScale < scale) scale = axisScale;
        }
      }
      else if (value > uMax)
      {
        const float room = uMax - collective;
        const float need = value - collective;
        if (need > 1e-6f)
        {
          const float axisScale = (room > 0.0f) ? (room / need) : 0.0f;
          if (axisScale < scale) scale = axisScale;
        }
      }
    }

    scale = clampFloatFc(scale, 0.0f, 1.0f);
    out.torqueScaled = scale < 0.999f;
    out.torqueScale = scale;

    if (out.torqueScaled)
    {
      dX *= scale;
      dY *= scale;

      rollPitch[0] = +dX + dY;
      rollPitch[1] = +dX - dY;
      rollPitch[2] = -dX - dY;
      rollPitch[3] = -dX + dY;
    }

    // ------------------------------------------------------------
    // Step 3: fit yaw into whatever rotor range is left over.
    //
    // Nothing above moves to accommodate this. If the yaw demand does not
    // fit in the remaining room it is scaled down - possibly all the way to
    // zero - and the aircraft simply loses heading authority for that cycle
    // while keeping thrust and attitude intact. Losing yaw is survivable;
    // losing collective at 0.4 m is not.
    // ------------------------------------------------------------

    float yawScale = 1.0f;
    for (int i = 0; i < 4; ++i)
    {
      const float base = collective + rollPitch[i];
      const float contribution = yawPattern[i] * dZ;

      if (contribution > 1e-6f)
      {
        const float room = uMax - base;
        const float axisScale = (room > 0.0f) ? (room / contribution) : 0.0f;
        if (axisScale < yawScale) yawScale = axisScale;
      }
      else if (contribution < -1e-6f)
      {
        const float room = base - uMin;
        const float axisScale = (room > 0.0f) ? (room / -contribution) : 0.0f;
        if (axisScale < yawScale) yawScale = axisScale;
      }
    }

    yawScale = clampFloatFc(yawScale, 0.0f, 1.0f);
    out.yawScaled = yawScale < 0.999f;
    out.yawScale = yawScale;

    dZ *= yawScale;

    float differential[4];
    for (int i = 0; i < 4; ++i)
    {
      differential[i] = rollPitch[i] + yawPattern[i] * dZ;
    }

    // ------------------------------------------------------------
    // Step 3: resolve to rotor speeds.
    // ------------------------------------------------------------

    float u[4];
    for (int i = 0; i < 4; ++i)
    {
      u[i] = clampFloatFc(collective + differential[i], uMin, uMax);

      out.rotorOmegaRadPerSec[i] = sqrtf(u[i]);
      out.rotorThrustN[i] = ROTOR_THRUST_KF * u[i];

      if (!isfinite(out.rotorOmegaRadPerSec[i]))
      {
        return out;
      }
    }

    // ------------------------------------------------------------
    // Step 4: report what was actually achieved, for telemetry and for the
    // yaw torque identifier, which must never learn from a demand that the
    // rotors did not deliver.
    // ------------------------------------------------------------

    out.achievedThrustN = ROTOR_THRUST_KF * (u[0] + u[1] + u[2] + u[3]);

    out.achievedTauX =
      ROTOR_THRUST_KF * ROTOR_ARM_Y_M * (u[0] + u[1] - u[2] - u[3]);

    out.achievedTauY =
      ROTOR_THRUST_KF * ROTOR_ARM_X_M * (u[0] - u[1] - u[2] + u[3]);

    out.achievedTauZ =
      -dragCoefficientKm * (u[0] - u[1] + u[2] - u[3]);

    out.valid = true;
    return out;
  }

  /*
    Forward model, used by the identifier and by validation. Given four rotor
    speeds and the current drag coefficient, what wrench does the airframe see?
  */
  static void forward(
    const float rotorOmegaRadPerSec[4],
    float dragCoefficientKm,
    float &thrustN,
    float &tauXNm,
    float &tauYNm,
    float &tauZNm
  )
  {
    float u[4];
    for (int i = 0; i < 4; ++i)
    {
      u[i] = rotorOmegaRadPerSec[i] * rotorOmegaRadPerSec[i];
    }

    thrustN = ROTOR_THRUST_KF * (u[0] + u[1] + u[2] + u[3]);

    tauXNm =
      ROTOR_THRUST_KF * ROTOR_ARM_Y_M * (u[0] + u[1] - u[2] - u[3]);

    tauYNm =
      ROTOR_THRUST_KF * ROTOR_ARM_X_M * (u[0] - u[1] - u[2] + u[3]);

    tauZNm =
      -dragCoefficientKm * (u[0] - u[1] + u[2] - u[3]);
  }

  /*
    Net rotor angular momentum about body +Z. Used for the gyroscopic term in
    the yaw identifier: the four rotors do not cancel exactly during transients,
    and ignoring it biases the learned drag coefficient.
  */
  static float rotorAngularMomentumZ(const float rotorOmegaRadPerSec[4])
  {
    float total = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
      total +=
        ROTOR_SPIN_SIGN[i] * ROTOR_INERTIA_KGM2 * rotorOmegaRadPerSec[i];
    }
    return total;
  }
};
