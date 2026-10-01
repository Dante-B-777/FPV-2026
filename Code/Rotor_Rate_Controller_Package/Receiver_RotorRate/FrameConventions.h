#pragma once

#include <math.h>
#include <stdint.h>

/*
  ============================================================================
  FRAME CONVENTIONS - SINGLE SOURCE OF TRUTH
  ============================================================================

  This file exists because the previous build had no single place that defined
  what "yaw" meant. The result was that `measuredYawRateDps` physically held a
  COUNTER-CLOCKWISE-positive rate while being named `...Cw...` and consumed by
  three separate places that assumed CLOCKWISE-positive. That single mismatch
  produced the saturating yaw oscillation and the runaway XY hold.

  Nothing in this project may define a rotation sign anywhere else. Every
  angular quantity in the new controller is RIGHT-HANDED, and every variable
  name says so.

  ----------------------------------------------------------------------------
  BODY FRAME (right-handed)
  ----------------------------------------------------------------------------

      +X = drone RIGHT
      +Y = drone FORWARD
      +Z = drone UP

  Right-handed rotations about those axes, all positive as written:

      omegaX  = pitch rate,  NOSE UP positive        (about +X)
      omegaY  = roll rate,   RIGHT WING DOWN positive(about +Y)
      omegaZ  = yaw rate,    COUNTER-CLOCKWISE positive seen from above

  A clockwise turn seen from above is therefore NEGATIVE omegaZ. There is no
  "clockwise" variable anywhere in this build. If you find one, it is a bug.

  ----------------------------------------------------------------------------
  NAVIGATION FRAME
  ----------------------------------------------------------------------------

  Local level frame captured at each navigation reset:

      +Xn = body +X at reset time
      +Yn = body +Y at reset time
      +Zn = up

  yawRhRad is the right-handed rotation of the body frame relative to the
  navigation frame. Body -> navigation is the standard right-handed rotation:

      navX = cos(yaw)*bodyX - sin(yaw)*bodyY
      navY = sin(yaw)*bodyX + cos(yaw)*bodyY

  The old build used the transpose of this while feeding it a right-handed
  angle, which silently inverted every position correction once the heading
  left zero. See bodyToNavigation() below.

  ----------------------------------------------------------------------------
  MPU6050 MOUNTING - MEASURED, UPSIDE DOWN
  ----------------------------------------------------------------------------

  The IMU is installed upside down. Comparing the three gyro signs and the
  three accelerometer signs that were validated on this airframe shows they
  are all consistent with exactly one rigid transform: a 180-degree rotation
  about the body FORWARD axis (+Y).

      chip +X = body -X
      chip +Y = body +Y
      chip +Z = body -Z

  so, as a diagonal matrix,  R_body_from_chip = diag(-1, +1, -1)

  This reproduces the previously tuned signs exactly:

      old  ROLL_GYRO_SIGN  = -1 applied to gyroY   -> omegaY = +gyroY
      old  PITCH_GYRO_SIGN = +1 applied to gyroX   -> omegaX = -gyroX
      old  YAW_GYRO_SIGN   = -1 applied to gyroZ   -> omegaZ = -gyroZ
      old  ESKF_ACCEL_X/Y/Z_FROM_RAW_SIGN = -1/+1/-1

  The roll and pitch behaviour of the old build is therefore preserved bit for
  bit. Only the yaw NAMING and its three consumers change.

  ----------------------------------------------------------------------------
  QMC5883P MOUNTING - MEASURED, BACKWARDS
  ----------------------------------------------------------------------------

      sensor +X = body LEFT      -> body X = -sensor X
      sensor +Y = body BACKWARD  -> body Y = -sensor Y
      sensor +Z = body UP        -> body Z = +sensor Z

  Hard-iron bias and axis scale are applied in SENSOR coordinates BEFORE this
  transform. Unchanged from the previous build, which had this part right.

  ----------------------------------------------------------------------------
  PMW3901 MOUNTING - COMMISSIONED AS flowmap 1
  ----------------------------------------------------------------------------

      body X counts = +raw X
      body Y counts = -raw Y

  ----------------------------------------------------------------------------
  VL53L0X - points straight down, measures range to floor along body -Z.
  ----------------------------------------------------------------------------

  ============================================================================
*/

// ================================================================
// UNIT HELPERS
// ================================================================

static constexpr float FC_PI = 3.14159265358979323846f;
static constexpr float FC_DEG_TO_RAD = FC_PI / 180.0f;
static constexpr float FC_RAD_TO_DEG = 180.0f / FC_PI;
static constexpr float FC_GRAVITY_MPS2 = 9.80665f;

// ================================================================
// MPU6050 RAW -> RIGHT-HANDED BODY
// ================================================================

// R_body_from_chip = diag(IMU_CHIP_TO_BODY_X, ..._Y, ..._Z).
// Derived above from the 180-degree flip about body +Y. Do not edit these
// three numbers unless the IMU is physically remounted; they are the reason
// the validated roll/pitch loop still behaves identically.
static constexpr float IMU_CHIP_TO_BODY_X = -1.0f;
static constexpr float IMU_CHIP_TO_BODY_Y = +1.0f;
static constexpr float IMU_CHIP_TO_BODY_Z = -1.0f;

static constexpr float GYRO_LSB_PER_DPS = 131.0f;   // +/-250 dps range
static constexpr float ACCEL_LSB_PER_G = 16384.0f;  // +/-2 g range

/*
  Convert bias-corrected chip gyro readings (deg/s) into right-handed body
  angular rates (deg/s).

    outXPitchUpDps    right-handed about body +X, nose up positive
    outYRollRightDps  right-handed about body +Y, right wing down positive
    outZYawCcwDps     right-handed about body +Z, counter-clockwise positive
*/
inline void chipGyroToBodyRh(
  float chipGyroXDps,
  float chipGyroYDps,
  float chipGyroZDps,
  float &outXPitchUpDps,
  float &outYRollRightDps,
  float &outZYawCcwDps
)
{
  outXPitchUpDps   = IMU_CHIP_TO_BODY_X * chipGyroXDps;
  outYRollRightDps = IMU_CHIP_TO_BODY_Y * chipGyroYDps;
  outZYawCcwDps    = IMU_CHIP_TO_BODY_Z * chipGyroZDps;
}

/*
  Convert bias-corrected chip accelerometer readings (g) into right-handed
  body specific force (m/s^2). At rest and level this returns (0, 0, +g).
*/
inline void chipAccelToBodyRh(
  float chipAccelXG,
  float chipAccelYG,
  float chipAccelZG,
  float &outBodyX,
  float &outBodyY,
  float &outBodyZ
)
{
  outBodyX = IMU_CHIP_TO_BODY_X * chipAccelXG * FC_GRAVITY_MPS2;
  outBodyY = IMU_CHIP_TO_BODY_Y * chipAccelYG * FC_GRAVITY_MPS2;
  outBodyZ = IMU_CHIP_TO_BODY_Z * chipAccelZG * FC_GRAVITY_MPS2;
}

/*
  Right-handed tilt angles from body specific force.

    rollRhRad   about +Y, right wing down positive
    pitchRhRad  about +X, nose up positive

  Derivation: at rest with a right-handed roll phi about +Y the body specific
  force is (-g sin(phi), 0, g cos(phi)), so phi = atan2(-fx, fz). With a
  right-handed pitch theta about +X it is (0, g sin(theta), g cos(theta)), so
  theta = atan2(fy, fz).
*/
inline void bodySpecificForceToTiltRh(
  float forceBodyX,
  float forceBodyY,
  float forceBodyZ,
  float &rollRhRad,
  float &pitchRhRad
)
{
  rollRhRad  = atan2f(-forceBodyX, forceBodyZ);
  pitchRhRad = atan2f(forceBodyY, forceBodyZ);
}

// ================================================================
// QMC5883P SENSOR -> BODY
// ================================================================

static constexpr float MAG_SENSOR_TO_BODY_X = -1.0f;
static constexpr float MAG_SENSOR_TO_BODY_Y = -1.0f;
static constexpr float MAG_SENSOR_TO_BODY_Z = +1.0f;

inline void magSensorToBody(
  float sensorX,
  float sensorY,
  float sensorZ,
  float &bodyX,
  float &bodyY,
  float &bodyZ
)
{
  bodyX = MAG_SENSOR_TO_BODY_X * sensorX;
  bodyY = MAG_SENSOR_TO_BODY_Y * sensorY;
  bodyZ = MAG_SENSOR_TO_BODY_Z * sensorZ;
}

// ================================================================
// PMW3901 RAW COUNTS -> BODY (commissioned flowmap 1)
// ================================================================

static constexpr float FLOW_RAW_TO_BODY_X = +1.0f;
static constexpr float FLOW_RAW_TO_BODY_Y = -1.0f;

inline void flowCountsToBody(
  int16_t rawDeltaX,
  int16_t rawDeltaY,
  float &bodyXCounts,
  float &bodyYCounts
)
{
  bodyXCounts = FLOW_RAW_TO_BODY_X * static_cast<float>(rawDeltaX);
  bodyYCounts = FLOW_RAW_TO_BODY_Y * static_cast<float>(rawDeltaY);
}

// ================================================================
// BODY <-> NAVIGATION ROTATION (right-handed yaw)
// ================================================================

/*
  THE FIX. The previous build wrote these two functions as each other's
  transpose while feeding them a right-handed angle. At heading 0 that is
  invisible; at 90 degrees a north position error commanded an east
  correction; at 180 degrees the loop ran away. The hover log showed the
  target parked at X=0.123 m, Y=0.026 m while the estimate walked past 1.5 m
  and never returned.
*/
inline void bodyToNavigation(
  float yawRhRad,
  float bodyX,
  float bodyY,
  float &navX,
  float &navY
)
{
  const float c = cosf(yawRhRad);
  const float s = sinf(yawRhRad);

  navX = c * bodyX - s * bodyY;
  navY = s * bodyX + c * bodyY;
}

inline void navigationToBody(
  float yawRhRad,
  float navX,
  float navY,
  float &bodyX,
  float &bodyY
)
{
  const float c = cosf(yawRhRad);
  const float s = sinf(yawRhRad);

  bodyX =  c * navX + s * navY;
  bodyY = -s * navX + c * navY;
}

// ================================================================
// MOTOR GEOMETRY AND SPIN DIRECTION
// ================================================================

/*
  Confirmed by physical inspection on this airframe, viewed from above:

                         FRONT
                    M1             M2
                   CCW             CW

                    M4             M3
                    CW            CCW
                         REAR

  Motor index in code is zero-based: index 0 = M1 ... index 3 = M4.

  Position of each rotor in the body frame, metres:
*/
static constexpr float FRAME_SEPARATION_FRONT_BACK_M = 0.1645f;
static constexpr float FRAME_SEPARATION_SIDE_SIDE_M  = 0.1630f;

static constexpr float ROTOR_ARM_X_M = FRAME_SEPARATION_SIDE_SIDE_M * 0.5f;
static constexpr float ROTOR_ARM_Y_M = FRAME_SEPARATION_FRONT_BACK_M * 0.5f;

static constexpr float ROTOR_POSITION_X_M[4] = {
  -ROTOR_ARM_X_M,  // M1 front-left
  +ROTOR_ARM_X_M,  // M2 front-right
  +ROTOR_ARM_X_M,  // M3 rear-right
  -ROTOR_ARM_X_M   // M4 rear-left
};

static constexpr float ROTOR_POSITION_Y_M[4] = {
  +ROTOR_ARM_Y_M,  // M1 front-left
  +ROTOR_ARM_Y_M,  // M2 front-right
  -ROTOR_ARM_Y_M,  // M3 rear-right
  -ROTOR_ARM_Y_M   // M4 rear-left
};

/*
  Rotor spin direction as a right-handed sign about body +Z.
    +1 = counter-clockwise seen from above
    -1 = clockwise seen from above

  A rotor spinning counter-clockwise must be driven against aerodynamic drag,
  so the airframe receives the OPPOSITE reaction torque. The yaw torque
  contributed by rotor i is therefore

      tauZ_i = -ROTOR_SPIN_SIGN[i] * dragTorqueMagnitude_i

  This is the sign the old mixer had backwards: it added positive yaw mix to
  the two CCW rotors while the rate PID treated positive as counter-clockwise,
  closing a positive feedback loop that saturated at +/-12 DShot and reversed.
*/
static constexpr float ROTOR_SPIN_SIGN[4] = {
  +1.0f,  // M1 CCW
  -1.0f,  // M2 CW
  +1.0f,  // M3 CCW
  -1.0f   // M4 CW
};

// ================================================================
// AIRFRAME PHYSICAL PARAMETERS
// ================================================================

/*
  REVERTED to the pilot-measured 1 lb on 2026-08-15. It was briefly raised to
  0.4929 kg that same day; that change was wrong and the reasoning is kept
  below so the mistake is not repeated.

  The 0.4929 came from assuming steady hover means delivered thrust equals
  weight, and reading delivered thrust off the measured rotor speeds. Two
  things were wrong with it:

    1. The supporting "check" was circular. Commanded thrust and delivered
       thrust agreed to 0.2%, which was cited as proof that kF was sound. It
       proves nothing of the kind: the allocator converts commanded thrust to
       rotor speed by dividing by the MODEL kF, and delivered thrust is then
       recomputed by multiplying by the same MODEL kF. Those two numbers agree
       by construction whenever the rotor loop tracks its setpoint, no matter
       how wrong kF is.

    2. It failed its own prediction. Raising the modelled mass 8.7% raised
       commanded thrust 7.4% and the aircraft did not climb at all - the
       implied mass simply moved up with it, 0.480 -> 0.526 -> 0.533 kg. A
       constant does not follow the value you assign it. That is the
       signature of a scale error somewhere else in the chain.

  The 0.50 m climb attempt closed it out: with kF held at the assumed value,
  the mass required to explain the hover is 1.174 lb, outside the pilot's
  measured 1.0 +- 0.1 lb. Mass is ruled out by measurement, so the error is
  in kF - see PROP_THRUST_CT below.

  Original note follows.

  Taken from 133 steady-hover samples (|VZ| <= 0.02 m/s) in
  logs/phase8_20260815_005942.log. In equilibrium the delivered thrust is the
  weight, and delivered thrust comes straight off the measured rotor speeds:

      T = kF * sum(omega_i^2) = 4.835 N   ->   m = 4.835 / 9.81 = 0.4929 kg

  Widening to |VZ| <= 0.05 over 339 samples gives 4.866 N and 0.4960 kg, so
  the estimate is stable. Pilot's scale says 1 lb +- 0.1 lb; 1.087 lb sits
  inside that band, near the top.

  Cross-check that matters: mean COMMANDED thrust over the same samples was
  4.826 N against 4.835 N delivered - agreement to 0.2%. The allocator and the
  rotor speed loop are faithful, so kF is sound and the whole error was here.

  This is what pinned every flight at 0.14-0.20 m against a 0.25 m setpoint.
  Nothing in the vertical cascade has an integral term, so a mass error cannot
  be absorbed - it shows up directly as a permanent altitude sag. The loop was
  commanding 4.83 N, the aircraft weighed 4.83 N, and it sat there.

  CAVEAT: measured at 0.154 m mean altitude, inside ground effect, where a
  given rotor speed makes MORE thrust than free air. So 0.4929 kg is a lower
  bound on the true mass and this correction is deliberately conservative.
  If altitude still sags once it is hovering clear of the ground, that is the
  reason and the number should come up again.

  NOTE: INERTIA_XX/YY/ZZ below scale with this mass, so every rate loop now
  commands ~8.7% more torque for the same rate error. The roll/pitch gains
  tuned on 2026-08-15 were set at the old mass; watch for extra sharpness.
*/
static constexpr float AIRFRAME_MASS_KG = 0.45359237f;  // 1 lb, pilot-measured
static constexpr float PROP_RADIUS_M = 0.0635f;         // 2.5 inch
static constexpr float AIR_DENSITY_KGM3 = 1.225f;
/*
  0.18 -> 0.153, flight-identified 2026-08-15. 0.18 was a textbook guess for a
  2.5 inch prop and it was 17% optimistic, which made the whole thrust model
  optimistic by the same factor.

  Identified from 213 near-zero-vertical-velocity samples across
  logs/phase8_20260815_130420.log and logs/hover05_20260815_132424.log. At
  hover the MODELLED thrust from the measured rotor speeds read 5.224 N
  (sd 0.149) while the aircraft actually weighs 4.450 N, so

      kF_actual / kF_model = 4.450 / 5.224 = 0.852
      CT_actual = 0.18 * 0.852 = 0.153

  Independent cross-check that makes this convincing: hoverOmegaRadPerSec()
  predicted 875 rad/s at CT=0.18, but the aircraft actually hovers at roughly
  950. With CT=0.153 the prediction becomes 948. The two agree without being
  fitted to each other.

  This is what made the aircraft unable to climb. On the 0.50 m attempt it sat
  in Climb for the whole flight commanding +0.30 m/s and SANK at -0.05 m/s,
  because the controller believed it was producing 5.5 N when it was producing
  about 4.4 N - just under its own weight.

  0.153 -> 0.145 later the same day, once the aircraft could finally hover
  clear of the ground and be measured honestly.

  0.153 was identified inside ground effect at ~0.20 m, where a given rotor
  speed makes MORE thrust than in free air, so it was known at the time to be
  an under-correction. The 0.5 m hover confirmed it: holding steady at 0.377 m
  the modelled thrust from measured rotor speeds was 4.701 N while the aircraft
  weighs 4.450 N, giving a further ratio of 0.947 and CT = 0.153 * 0.947.

  What that residual 5% was doing, from logs/hover05_20260815_151437.log:

      Climb complete at 0.451 m
      ALT 0.45 -> 0.44 -> 0.40 -> 0.36 -> 0.32 -> 0.28 -> 0.25 -> 0.16
      VZ steady at about -0.20 m/s, T commanded 4.80 N
      Altitude hold captured at 0.250 m

  The vertical loop was asking for 4.80 N against a 4.450 N weight and still
  sinking, because real thrust was below what the model believed. Worse, the
  sink rate (0.20 m/s) exceeded the hold's settle threshold (0.15 m/s), so the
  hold could never settle, never captured, and nothing arrested the descent -
  it fell 30 cm until ground effect slowed it enough to latch, at the
  MIN_ALTITUDE_TARGET_M clamp. The 11 cm steady-state offset in the previous
  flight was the same error seen from the other side.

  Note kM is derived from kF, so yaw authority scales with this too.
*/
static constexpr float PROP_THRUST_CT = 0.145f;

/*
  Thrust per rotor:  T = ROTOR_THRUST_KF * omega^2   [N], omega in rad/s.
  Retained unchanged from the validated MotionModelFF port so the hover
  operating point does not shift with this rewrite.
*/
static constexpr float ROTOR_THRUST_KF =
  4.0f / (FC_PI * FC_PI) * PROP_THRUST_CT * AIR_DENSITY_KGM3 *
  (PROP_RADIUS_M * PROP_RADIUS_M * PROP_RADIUS_M * PROP_RADIUS_M);

// Rotor + prop polar moment of inertia about the shaft [kg m^2].
static constexpr float ROTOR_INERTIA_KGM2 =
  0.006f * (1.4e-2f * 1.4e-2f) +
  0.0038f * (PROP_RADIUS_M * PROP_RADIUS_M);

// Airframe yaw moment of inertia [kg m^2], slab approximation.
static constexpr float BODY_YAW_INERTIA_KGM2 =
  AIRFRAME_MASS_KG *
  (FRAME_SEPARATION_SIDE_SIDE_M * FRAME_SEPARATION_SIDE_SIDE_M +
   FRAME_SEPARATION_FRONT_BACK_M * FRAME_SEPARATION_FRONT_BACK_M) / 12.0f;

// Hover rotor speed implied by mass and thrust coefficient [rad/s].
inline float hoverOmegaRadPerSec()
{
  return sqrtf(
    AIRFRAME_MASS_KG * FC_GRAVITY_MPS2 / (4.0f * ROTOR_THRUST_KF)
  );
}

/*
  Rotor drag torque:  Q = ROTOR_DRAG_KM * omega^2   [N m].

  ROTOR_DRAG_KM is NOT known for these props. It is initialised from the
  torque-to-thrust ratio below and then identified in flight by
  YawTorqueIdentifier. Yaw authority scales linearly with it, so a wrong
  value shows up as sluggish or twitchy yaw, never as a sign inversion.

    kM = kF * DRAG_TO_THRUST_RATIO_M       [N m per N of thrust]

  0.010 m is a conservative starting value for a 2.5 inch prop.
*/
static constexpr float DRAG_TO_THRUST_RATIO_INITIAL_M = 0.010f;
static constexpr float DRAG_TO_THRUST_RATIO_MIN_M = 0.002f;
static constexpr float DRAG_TO_THRUST_RATIO_MAX_M = 0.040f;

inline float initialRotorDragKm()
{
  return ROTOR_THRUST_KF * DRAG_TO_THRUST_RATIO_INITIAL_M;
}

// ================================================================
// SMALL SHARED HELPERS
// ================================================================

inline float clampFloatFc(float value, float minimum, float maximum)
{
  if (!isfinite(value)) return 0.0f;
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

inline float wrapAngleRad(float angleRad)
{
  while (angleRad > FC_PI) angleRad -= 2.0f * FC_PI;
  while (angleRad < -FC_PI) angleRad += 2.0f * FC_PI;
  return angleRad;
}

inline float wrapAngleDeg(float angleDeg)
{
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}
