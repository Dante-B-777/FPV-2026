#pragma once

#include "FrameConventions.h"

/*
  ============================================================================
  IN-FLIGHT ROTOR DRAG COEFFICIENT IDENTIFICATION
  ============================================================================

  The prop drag-torque coefficient kM is the one physical parameter this
  airframe has never had a value for. Thrust was characterised (kF), the frame
  was measured, the rotor inertia was estimated, but yaw torque was previously
  side-stepped entirely by solving an angular-momentum lambda instead of
  modelling drag. That is a large part of why yaw was never properly closed.

  YAW DYNAMICS

  The motor must supply, to rotor i, both the torque to accelerate it and the
  torque to overcome aerodynamic drag:

      motorTorque_i = spinSign_i * (Irotor * omegaDot_i + kM * omega_i^2)

  The airframe receives the reaction, so summing over four rotors:

      Izz * alphaZ = -dHz/dt - kM * D + disturbance

  where

      Hz = sum( spinSign_i * Irotor * omega_i )        rotor angular momentum
      D  = u1 - u2 + u3 - u4,   u_i = omega_i^2        signed squared-speed sum

  Rearranged into a linear regression with a single unknown:

      y = Izz * alphaZ + dHz/dt  =  -kM * D

  ESTIMATOR

  Normalised least-mean-squares on that one parameter. Normalisation by D^2
  makes the step size independent of how hard yaw is being driven, so a gentle
  heading correction and an aggressive stick input contribute comparably.

  GATING - this is the important part

  An adaptive loop on the axis that was broken is only safe if it refuses to
  learn from anything ambiguous. Updates require ALL of:

    - explicit commissioning flag set by the operator;
    - flight enabled, both ramps complete;
    - fresh RPM telemetry on all four rotors, since D is built from measured
      speeds and one stale rotor corrupts it;
    - enough excitation, |D| above a floor, so the regressor is not noise;
    - the allocator not saturated, so demanded torque was actually delivered;
    - near-level attitude, so roll/pitch coupling is not attributed to yaw;
    - a plausible residual, rejecting outliers from collisions or gusts.

  The estimate is hard-clamped to a physically sensible band at all times, and
  the controller uses the clamped value. A failure of this identifier degrades
  to "yaw authority is somewhat wrong", never to a sign flip.
*/
class YawTorqueIdentifier
{
public:
  struct Status
  {
    float dragCoefficientKm = 0.0f;
    float dragToThrustRatioM = 0.0f;
    float lastRegressor = 0.0f;      // D
    float lastObservation = 0.0f;    // y
    float lastResidual = 0.0f;
    float filteredResidualMagnitude = 0.0f;
    float yawDampingC = 0.0f;        // N m per rad/s, jointly estimated
    float kmSpreadFraction = 1.0f;   // spread of kM about its own mean
    uint32_t updateCount = 0;
    uint32_t rejectCount = 0;
    bool converged = false;
    bool updatedThisCall = false;
  };

  bool commissioningEnabled = false;

  YawTorqueIdentifier()
  {
    reset();
  }

  void reset()
  {
    dragCoefficientKm_ = initialRotorDragKm();
    previousMomentumZ_ = 0.0f;
    havePreviousMomentum_ = false;
    filteredYawAccel_ = 0.0f;
    filteredResidualMagnitude_ = 0.0f;
    yawDampingC_ = 0.0f;
    filteredKmMean_ = dragCoefficientKm_;
    filteredKmSpread_ = dragCoefficientKm_;
    updateCount_ = 0;
    rejectCount_ = 0;
  }

  float dragCoefficientKm() const { return dragCoefficientKm_; }

  // Allow the operator to seed or override the value from the console.
  bool setDragToThrustRatio(float ratioMeters)
  {
    if (!isfinite(ratioMeters) ||
        ratioMeters < DRAG_TO_THRUST_RATIO_MIN_M ||
        ratioMeters > DRAG_TO_THRUST_RATIO_MAX_M)
    {
      return false;
    }

    dragCoefficientKm_ = ROTOR_THRUST_KF * ratioMeters;
    return true;
  }

  /*
    Call once per control cycle.

      rotorOmegaRadPerSec   measured rotor speeds, all four fresh
      allRotorsFresh        every rotor reported valid RPM this cycle
      yawRateCcwRadPerSec   right-handed body yaw rate about +Z
      allocatorSaturated    allocator scaled or slid the demand
      rollRhRad, pitchRhRad current attitude, for the level gate
      flightActive          armed, both ramps complete
      dt                    seconds
  */
  Status update(
    const float rotorOmegaRadPerSec[4],
    bool allRotorsFresh,
    float yawRateCcwRadPerSec,
    bool allocatorSaturated,
    float rollRhRad,
    float pitchRhRad,
    /*
      PHYSICALLY airborne, not merely "the flight state machine is running".

      This used to take flightActive, which is true the moment the aircraft
      arms and spools up. The 2026-08-15 Phase 8 run therefore identified kM
      across an entire flight that never left ground effect: no "Airborne
      confirmed" line anywhere, A=FREE and X=FREE on every line, altitude
      pinned at 0.09-0.14 m against a 0.25 m target. A quad skimming the
      floor is not a free rigid body in yaw, and the fit is meaningless.
    */
    bool physicallyAirborne,
    float dt
  )
  {
    Status status;

    dt = clampFloatFc(dt, 0.0005f, 0.020f);

    // ------------------------------------------------------------
    // Yaw angular acceleration, filtered. Differentiating a gyro signal is
    // noisy; this filter is deliberately heavier than the control path
    // because the identifier is allowed to be slow.
    // ------------------------------------------------------------

    const float rawYawAccel =
      (dt > 0.0f) ? ((yawRateCcwRadPerSec - previousYawRate_) / dt) : 0.0f;

    previousYawRate_ = yawRateCcwRadPerSec;

    if (isfinite(rawYawAccel))
    {
      filteredYawAccel_ +=
        YAW_ACCEL_FILTER_ALPHA * (rawYawAccel - filteredYawAccel_);
    }

    // ------------------------------------------------------------
    // Rotor angular momentum and its derivative.
    // ------------------------------------------------------------

    float momentumZ = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
      momentumZ +=
        ROTOR_SPIN_SIGN[i] * ROTOR_INERTIA_KGM2 * rotorOmegaRadPerSec[i];
    }

    float momentumRate = 0.0f;
    if (havePreviousMomentum_ && dt > 0.0f)
    {
      momentumRate = (momentumZ - previousMomentumZ_) / dt;
    }
    previousMomentumZ_ = momentumZ;
    havePreviousMomentum_ = true;

    if (!isfinite(momentumRate))
    {
      momentumRate = 0.0f;
    }

    filteredMomentumRate_ +=
      MOMENTUM_RATE_FILTER_ALPHA * (momentumRate - filteredMomentumRate_);

    // ------------------------------------------------------------
    // Regressor and observation.
    // ------------------------------------------------------------

    const float regressorD =
      rotorOmegaRadPerSec[0] * rotorOmegaRadPerSec[0] -
      rotorOmegaRadPerSec[1] * rotorOmegaRadPerSec[1] +
      rotorOmegaRadPerSec[2] * rotorOmegaRadPerSec[2] -
      rotorOmegaRadPerSec[3] * rotorOmegaRadPerSec[3];

    const float observationY =
      BODY_YAW_INERTIA_KGM2 * filteredYawAccel_ + filteredMomentumRate_;

    status.lastRegressor = regressorD;
    status.lastObservation = observationY;

    // ------------------------------------------------------------
    // Gates.
    // ------------------------------------------------------------

    const bool levelEnough =
      fabsf(rollRhRad) <= MAX_LEVEL_RAD &&
      fabsf(pitchRhRad) <= MAX_LEVEL_RAD;

    const bool excited = fabsf(regressorD) >= MIN_EXCITATION_D;

    const bool gatesPass =
      commissioningEnabled &&
      physicallyAirborne &&
      allRotorsFresh &&
      !allocatorSaturated &&
      levelEnough &&
      excited &&
      isfinite(observationY) &&
      isfinite(regressorD) &&
      fabsf(yawRateCcwRadPerSec) <= MAX_YAW_RATE_RAD;

    if (!gatesPass)
    {
      status.dragCoefficientKm = dragCoefficientKm_;
      status.dragToThrustRatioM = dragCoefficientKm_ / ROTOR_THRUST_KF;
      status.filteredResidualMagnitude = filteredResidualMagnitude_;
      status.yawDampingC = yawDampingC_;
      status.kmSpreadFraction = kmSpreadFraction();
      status.updateCount = updateCount_;
      status.rejectCount = rejectCount_;
      status.converged = converged();
      return status;
    }

    /*
      ------------------------------------------------------------
      Normalised LMS step, TWO parameters.

        model:  yHat = -kM * D  -  c * wz

      The damping term is the 2026-08-15 fix. The old single-parameter
      model had no aerodynamic yaw damping, and that is what destroyed the
      first Phase 8 run.

      Consider a sustained yaw at constant rate, which is what a held yaw
      button produces. The yaw rate is constant, so alphaZ is ~0. The rotor
      speeds are steady, so dHz/dt is ~0. The observation is therefore ~0
      while D is large and non-zero - and the only way the old model could
      satisfy that was to drive kM toward zero. The 2026-08-15 log shows
      exactly this: R pinned at 0.0020, the DRAG_TO_THRUST_RATIO_MIN_M
      clamp floor, through every held-stick segment, and swinging from
      there up to 0.0197 with no settling across 8813 samples.

      Physically the rotor drag torque in that steady turn is being
      balanced by aerodynamic damping on the airframe and blades. With
      c in the model those segments become INFORMATIVE about damping
      instead of poisoning kM.

      Sanity check from the same log: at a steady -40 deg/s (0.70 rad/s)
      the regressor sat near 4.4e5, so with a plausible kM of 1.45e-8 the
      applied torque was ~0.0064 N m, implying c ~ 0.009 N m per rad/s.
      That is comfortably inside the bound below.

      NUMERICAL NOTE: D is order 1e5 and wz is order 1. Feeding both to one
      NLMS denominator unscaled would let D swamp wz completely and the
      damping term would never move. Each regressor is therefore normalised
      by a fixed scale so both are order 1, and the parameters are carried
      in scaled form.
      ------------------------------------------------------------
    */

    const float scaledD = regressorD / REGRESSOR_D_SCALE;
    const float scaledW = yawRateCcwRadPerSec / REGRESSOR_W_SCALE;

    // Parameters in scaled units.
    float scaledKm = dragCoefficientKm_ * REGRESSOR_D_SCALE;
    float scaledC = yawDampingC_ * REGRESSOR_W_SCALE;

    const float predicted = -scaledKm * scaledD - scaledC * scaledW;
    const float residual = observationY - predicted;

    status.lastResidual = residual;

    // Outlier rejection against the running residual scale. Early on, when
    // the running scale is still tiny, allow the update through so the
    // estimator can actually start.
    const bool residualPlausible =
      (updateCount_ < WARMUP_UPDATES) ||
      (fabsf(residual) <=
        OUTLIER_SIGMA * fmaxf(filteredResidualMagnitude_, MIN_RESIDUAL_SCALE));

    if (!residualPlausible)
    {
      ++rejectCount_;
      status.dragCoefficientKm = dragCoefficientKm_;
      status.dragToThrustRatioM = dragCoefficientKm_ / ROTOR_THRUST_KF;
      status.filteredResidualMagnitude = filteredResidualMagnitude_;
      status.yawDampingC = yawDampingC_;
      status.kmSpreadFraction = kmSpreadFraction();
      status.updateCount = updateCount_;
      status.rejectCount = rejectCount_;
      status.converged = converged();
      return status;
    }

    filteredResidualMagnitude_ +=
      RESIDUAL_FILTER_ALPHA *
      (fabsf(residual) - filteredResidualMagnitude_);

    // Shared denominator over BOTH scaled regressors.
    const float denominator =
      scaledD * scaledD + scaledW * scaledW + NLMS_REGULARISATION_SCALED;

    scaledKm -= NLMS_STEP * residual * scaledD / denominator;
    scaledC -= NLMS_STEP * residual * scaledW / denominator;

    dragCoefficientKm_ = clampFloatFc(
      scaledKm / REGRESSOR_D_SCALE,
      ROTOR_THRUST_KF * DRAG_TO_THRUST_RATIO_MIN_M,
      ROTOR_THRUST_KF * DRAG_TO_THRUST_RATIO_MAX_M
    );

    /*
      Damping must be dissipative, so c is held non-negative. A negative
      value would mean the airframe adds energy to its own yaw, which is
      not a thing this aircraft does; letting the fit go there would only
      be it absorbing some other modelling error.
    */
    yawDampingC_ = clampFloatFc(
      scaledC / REGRESSOR_W_SCALE, 0.0f, MAX_YAW_DAMPING_C
    );

    /*
      Convergence tracking. See converged() for why a sample count is not
      good enough.
    */
    filteredKmMean_ +=
      KM_MEAN_ALPHA * (dragCoefficientKm_ - filteredKmMean_);
    filteredKmSpread_ +=
      KM_SPREAD_ALPHA *
      (fabsf(dragCoefficientKm_ - filteredKmMean_) - filteredKmSpread_);

    ++updateCount_;

    status.updatedThisCall = true;
    status.dragCoefficientKm = dragCoefficientKm_;
    status.dragToThrustRatioM = dragCoefficientKm_ / ROTOR_THRUST_KF;
    status.filteredResidualMagnitude = filteredResidualMagnitude_;
    status.yawDampingC = yawDampingC_;
    status.kmSpreadFraction = kmSpreadFraction();
    status.updateCount = updateCount_;
    status.rejectCount = rejectCount_;
    status.converged = converged();

    return status;
  }

  /*
    Convergence means THE ESTIMATE STOPPED MOVING, not that a lot of
    samples went by.

    This used to be `updateCount_ >= CONVERGED_UPDATES` and nothing else,
    which is a sample counter wearing a convergence badge. The 2026-08-15
    run reported converged=true at N=2000 and then went on to swing kM over
    a 10x range for another 6800 samples, spending long stretches pinned on
    the clamp floor. The flag was true the whole time.

    Now it also requires kM to have stayed inside a narrow band around its
    own slow mean, and requires that mean to be off the clamp rails - an
    estimate sitting on a limit is not an answer, it is the fit telling you
    it wants to go somewhere the model forbids.
  */
  bool converged() const
  {
    if (updateCount_ < CONVERGED_UPDATES) return false;
    if (!(filteredKmMean_ > 0.0f)) return false;

    const float lowRail = ROTOR_THRUST_KF * DRAG_TO_THRUST_RATIO_MIN_M;
    const float highRail = ROTOR_THRUST_KF * DRAG_TO_THRUST_RATIO_MAX_M;
    const float railMargin = 0.02f * (highRail - lowRail);

    if (dragCoefficientKm_ <= lowRail + railMargin) return false;
    if (dragCoefficientKm_ >= highRail - railMargin) return false;

    return (filteredKmSpread_ / filteredKmMean_) <= KM_CONVERGED_FRACTION;
  }

  float kmSpreadFraction() const
  {
    return (filteredKmMean_ > 0.0f)
      ? (filteredKmSpread_ / filteredKmMean_)
      : 1.0f;
  }

private:
  static constexpr float YAW_ACCEL_FILTER_ALPHA = 0.05f;
  static constexpr float MOMENTUM_RATE_FILTER_ALPHA = 0.05f;
  static constexpr float RESIDUAL_FILTER_ALPHA = 0.01f;

  static constexpr float NLMS_STEP = 0.002f;

  /*
    Regressor scales. D is order 1e5 at useful excitation, yaw rate is
    order 1 rad/s. Both are divided by these so the shared NLMS
    denominator sees comparable magnitudes and the damping parameter is
    not swamped by the rotor term.
  */
  static constexpr float REGRESSOR_D_SCALE = 1.0e5f;
  static constexpr float REGRESSOR_W_SCALE = 1.0f;
  static constexpr float NLMS_REGULARISATION_SCALED = 1.0e-3f;

  /*
    Upper bound on aerodynamic yaw damping. The airframe's whole yaw
    authority is about 0.041 N m, so a c of 0.05 would mean damping
    swallowing the entire control budget by 0.8 rad/s. The log-implied
    value is around 0.009, so this is roughly 5x headroom and still far
    from physically silly.
  */
  static constexpr float MAX_YAW_DAMPING_C = 0.05f;

  // Convergence: kM must sit within this fraction of its own slow mean.
  static constexpr float KM_MEAN_ALPHA = 0.002f;
  static constexpr float KM_SPREAD_ALPHA = 0.002f;
  static constexpr float KM_CONVERGED_FRACTION = 0.05f;

  // Requires roughly a 30 rad/s speed split at hover before it will learn.
  static constexpr float MIN_EXCITATION_D = 5.0e4f;

  static constexpr float MAX_LEVEL_RAD = 12.0f * FC_DEG_TO_RAD;
  static constexpr float MAX_YAW_RATE_RAD = 180.0f * FC_DEG_TO_RAD;

  static constexpr float OUTLIER_SIGMA = 4.0f;
  static constexpr float MIN_RESIDUAL_SCALE = 1.0e-4f;

  static constexpr uint32_t WARMUP_UPDATES = 50;
  static constexpr uint32_t CONVERGED_UPDATES = 2000;

  float dragCoefficientKm_ = 0.0f;
  float previousYawRate_ = 0.0f;
  float filteredYawAccel_ = 0.0f;
  float previousMomentumZ_ = 0.0f;
  float filteredMomentumRate_ = 0.0f;
  float filteredResidualMagnitude_ = 0.0f;
  float yawDampingC_ = 0.0f;
  float filteredKmMean_ = 0.0f;
  float filteredKmSpread_ = 0.0f;
  bool havePreviousMomentum_ = false;
  uint32_t updateCount_ = 0;
  uint32_t rejectCount_ = 0;
};
