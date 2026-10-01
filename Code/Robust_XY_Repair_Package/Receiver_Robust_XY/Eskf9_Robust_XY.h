#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

/*
  ============================================================================
  Eskf9.h
  ============================================================================

  Fixed-size 9-error-state Error-State Kalman Filter for ESP32.

  Nominal state:

      position_N [m]
      velocity_N [m/s]
      q_NB

  where:

      N = local navigation frame
      B = drone body frame

  Navigation frame:

      +X = right at local-frame capture
      +Y = forward at local-frame capture
      +Z = up

  Body frame:

      +X = drone right
      +Y = drone forward
      +Z = drone up

  Quaternion:

      q_NB = body-to-navigation quaternion
      format = [w, x, y, z]

  Error state:

      dx =
      [
        dp_N,
        dv_N,
        dtheta_B
      ]

  or explicitly:

      state 0 = position X error
      state 1 = position Y error
      state 2 = position Z error

      state 3 = velocity X error
      state 4 = velocity Y error
      state 5 = velocity Z error

      state 6 = attitude-error X
      state 7 = attitude-error Y
      state 8 = attitude-error Z


  Attitude error convention:

      RIGHT-MULTIPLICATIVE body-frame attitude error.

  Therefore:

      R_true ~= R_nominal * Exp([dtheta_B]x)

  and the quaternion correction is injected on the RIGHT.


  Sensors currently supported:

      IMU accelerometer
          -> prediction
          -> gravity-direction correction

      IMU gyro
          -> quaternion prediction

      PMW3901 optical flow
          -> body X/Y velocity measurement

      VL53L0X
          -> altitude
          -> vertical velocity

      QMC5883P magnetometer
          -> absolute/relative yaw correction


  The magnetometer does NOT replace gyro propagation.

  Gyro:
      fast attitude prediction

  Magnetometer:
      slow absolute yaw correction

  ============================================================================
*/

class Eskf9
{
public:

  // ==============================================================
  // ERROR-STATE SIZE
  // ==============================================================

  static constexpr int STATE_DIM = 9;


  // ==============================================================
  // PARAMETERS
  // ==============================================================

  struct Params
  {
    float gravity = 9.80665f;


    // ------------------------------------------------------------
    // PROCESS-NOISE STANDARD DEVIATIONS
    // ------------------------------------------------------------

    float accelNoiseXY = 0.85f;       // m/s^2
    float accelNoiseZ = 1.20f;        // m/s^2

    float gyroNoiseRollPitch = 0.08f; // rad/s
    float gyroNoiseYaw = 0.12f;       // rad/s


    // ------------------------------------------------------------
    // INITIAL 1-SIGMA UNCERTAINTIES
    // ------------------------------------------------------------

    float initialPositionXY = 0.03f;   // m
    float initialPositionZ = 0.03f;    // m

    float initialVelocity = 0.08f;     // m/s

    float initialTilt = 0.04f;         // rad
    float initialYaw = 0.20f;          // rad


    // ------------------------------------------------------------
    // NUMERICAL PROTECTION
    // ------------------------------------------------------------

    float maxCovariance = 100.0f;
    float minCovariance = 1.0e-7f;
  };


  // ==============================================================
  // CONSTRUCTORS
  // ==============================================================

  Eskf9()
    : params_(Params{})
  {
    reset(
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f
    );
  }


  explicit Eskf9(const Params &params)
    : params_(params)
  {
    reset(
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f
    );
  }


  // ==============================================================
  // COMPLETE ESTIMATOR RESET
  // ==============================================================

  void reset(
    float x,
    float y,
    float z,
    float pitchAboutXRad,
    float rollAboutYRad,
    float yawRightHandRad
  )
  {
    // ------------------------------------------------------------
    // POSITION
    // ------------------------------------------------------------

    position_[0] = x;
    position_[1] = y;
    position_[2] = z;


    // ------------------------------------------------------------
    // VELOCITY
    // ------------------------------------------------------------

    velocity_[0] = 0.0f;
    velocity_[1] = 0.0f;
    velocity_[2] = 0.0f;


    // ------------------------------------------------------------
    // ATTITUDE
    //
    // Z-Y-X:
    //
    // R =
    // Rz(yaw)
    // Ry(rotation about Y)
    // Rx(rotation about X)
    // ------------------------------------------------------------

    quaternionFromZyx(
      yawRightHandRad,
      rollAboutYRad,
      pitchAboutXRad,
      quaternion_
    );

    normalizeQuaternion(quaternion_);

    updateRotationMatrix();


    // ------------------------------------------------------------
    // COVARIANCE
    // ------------------------------------------------------------

    memset(
      covariance_,
      0,
      sizeof(covariance_)
    );


    covariance_[0][0] =
      square(params_.initialPositionXY);

    covariance_[1][1] =
      square(params_.initialPositionXY);

    covariance_[2][2] =
      square(params_.initialPositionZ);


    covariance_[3][3] =
      square(params_.initialVelocity);

    covariance_[4][4] =
      square(params_.initialVelocity);

    covariance_[5][5] =
      square(params_.initialVelocity);


    covariance_[6][6] =
      square(params_.initialTilt);

    covariance_[7][7] =
      square(params_.initialTilt);

    covariance_[8][8] =
      square(params_.initialYaw);


    lastInnovationRatio_ = 0.0f;

    rejectedUpdates_ = 0;
    acceptedUpdates_ = 0;

    healthy_ = true;
  }


  // ==============================================================
  // RESET LOCAL POSITION AND YAW
  // ==============================================================

  void resetLocalPositionAndYaw(float z)
  {
    float pitchX = 0.0f;
    float rollY = 0.0f;
    float yawRh = 0.0f;


    getEulerRadians(
      pitchX,
      rollY,
      yawRh
    );


    reset(
      0.0f,
      0.0f,
      z,
      pitchX,
      rollY,
      0.0f
    );
  }


  // ==============================================================
  // ESKF PREDICTION
  // ==============================================================
  //
  // accelBody:
  //
  //     measured specific force in BODY coordinates
  //
  // gyroBody:
  //
  //     right-handed body angular rates [rad/s]
  //
  // processNoiseScale:
  //
  //     optional covariance inflation supplied by the rotor/RPM
  //     health model.
  //
  //     1.0 = nominal
  //     >1  = less certainty in the process model
  //
  // ==============================================================

  void predict(
    const float accelBody[3],
    const float gyroBody[3],
    float dt,
    float processNoiseScale = 1.0f
  )
  {
    // ------------------------------------------------------------
    // VALIDITY
    // ------------------------------------------------------------

    if (
      !finite3(accelBody) ||
      !finite3(gyroBody) ||
      !isfinite(dt)
    )
    {
      healthy_ = false;
      return;
    }


    dt = clamp(
      dt,
      0.001f,
      0.020f
    );


    if (!isfinite(processNoiseScale))
    {
      processNoiseScale = 1.0f;
    }


    // Do not allow corrupted RPM state to destroy covariance.

    processNoiseScale = clamp(
      processNoiseScale,
      0.25f,
      25.0f
    );


    // ------------------------------------------------------------
    // ATTITUDE PROPAGATION
    //
    // Body-frame incremental rotation.
    //
    // Because this ESKF uses right-multiplicative body error:
    //
    //     q_new = q_old * dq
    // ------------------------------------------------------------

    float deltaQuaternion[4];


    rotationVectorToQuaternion(
      gyroBody[0] * dt,
      gyroBody[1] * dt,
      gyroBody[2] * dt,
      deltaQuaternion
    );


    float propagatedQuaternion[4];


    quaternionMultiply(
      quaternion_,
      deltaQuaternion,
      propagatedQuaternion
    );


    memcpy(
      quaternion_,
      propagatedQuaternion,
      sizeof(quaternion_)
    );


    normalizeQuaternion(
      quaternion_
    );


    updateRotationMatrix();


    // ------------------------------------------------------------
    // SPECIFIC FORCE BODY -> NAVIGATION
    //
    // a_N =
    //
    //     R_NB f_B
    //
    // then subtract navigation-frame gravity.
    //
    // At rest:
    //
    //     f_B ~= +g upward
    //
    // therefore:
    //
    //     R f_B - g ~= 0
    // ------------------------------------------------------------

    float accelerationNav[3] =
    {
      rotation_[0][0] * accelBody[0] +
      rotation_[0][1] * accelBody[1] +
      rotation_[0][2] * accelBody[2],

      rotation_[1][0] * accelBody[0] +
      rotation_[1][1] * accelBody[1] +
      rotation_[1][2] * accelBody[2],

      rotation_[2][0] * accelBody[0] +
      rotation_[2][1] * accelBody[1] +
      rotation_[2][2] * accelBody[2] -
      params_.gravity
    };


    // ------------------------------------------------------------
    // TRANSLATIONAL STATE
    // ------------------------------------------------------------

    const float halfDtSquared =
      0.5f * dt * dt;


    for (int axis = 0; axis < 3; ++axis)
    {
      position_[axis] +=
        velocity_[axis] * dt +
        accelerationNav[axis] *
        halfDtSquared;


      velocity_[axis] +=
        accelerationNav[axis] * dt;
    }


    // ------------------------------------------------------------
    // FIRST-ORDER DISCRETE ERROR-STATE TRANSITION
    //
    // A ~= I + F dt
    // ------------------------------------------------------------

    float transition[STATE_DIM][STATE_DIM] = {};


    for (int i = 0; i < STATE_DIM; ++i)
    {
      transition[i][i] = 1.0f;
    }


    // dp / dv

    for (int axis = 0; axis < 3; ++axis)
    {
      transition[axis][axis + 3] = dt;
    }


    // ------------------------------------------------------------
    // dv / dtheta
    //
    // With right-multiplicative body-frame attitude error:
    //
    //     dv/dtheta = -R [a_B]x
    // ------------------------------------------------------------

    float skewAccel[3][3];

    skew(
      accelBody,
      skewAccel
    );


    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        float value = 0.0f;


        for (int k = 0; k < 3; ++k)
        {
          value -=
            rotation_[row][k] *
            skewAccel[k][col];
        }


        transition[row + 3][col + 6] +=
          value * dt;
      }
    }


    // ------------------------------------------------------------
    // dtheta dynamics
    //
    //     dtheta_dot =
    //
    //       -[omega_B]x dtheta
    // ------------------------------------------------------------

    float skewGyro[3][3];

    skew(
      gyroBody,
      skewGyro
    );


    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        transition[row + 6][col + 6] -=
          skewGyro[row][col] * dt;
      }
    }


    // ------------------------------------------------------------
    // COVARIANCE PROPAGATION
    // ------------------------------------------------------------

    propagateCovariance(
      transition,
      dt,
      processNoiseScale
    );


    sanitize();
  }


  // ==============================================================
  // ALTITUDE MEASUREMENT
  // ==============================================================

  bool updateAltitude(
    float altitudeMeters,
    float standardDeviationMeters,
    float gateSigma = 6.0f
  )
  {
    if (
      !isfinite(altitudeMeters) ||
      !isfinite(standardDeviationMeters) ||
      standardDeviationMeters <= 0.0f
    )
    {
      return false;
    }


    float H[STATE_DIM] = {};


    H[2] = 1.0f;


    return scalarUpdate(
      H,
      altitudeMeters - position_[2],
      square(standardDeviationMeters),
      gateSigma
    );
  }


  // ==============================================================
  // VERTICAL VELOCITY MEASUREMENT
  // ==============================================================

  bool updateVerticalVelocity(
    float velocityMetersPerSecond,
    float standardDeviationMetersPerSecond,
    float gateSigma = 6.0f
  )
  {
    if (
      !isfinite(velocityMetersPerSecond) ||
      !isfinite(standardDeviationMetersPerSecond) ||
      standardDeviationMetersPerSecond <= 0.0f
    )
    {
      return false;
    }


    float H[STATE_DIM] = {};


    H[5] = 1.0f;


    return scalarUpdate(
      H,
      velocityMetersPerSecond -
        velocity_[2],

      square(
        standardDeviationMetersPerSecond
      ),

      gateSigma
    );
  }


  // ==============================================================
  // BODY X/Y VELOCITY UPDATE
  // ==============================================================
  //
  // Optical flow supplies velocity relative to the body frame.
  //
  // Measurement:
  //
  //     v_B = R_NB^T v_N
  //
  // ==============================================================

  bool updateBodyVelocity(
    float bodyVelocityX,
    float bodyVelocityY,
    float standardDeviationX,
    float standardDeviationY,
    float gateSigma = 6.0f
  )
  {
    if (
      !isfinite(bodyVelocityX) ||
      !isfinite(bodyVelocityY) ||
      !isfinite(standardDeviationX) ||
      !isfinite(standardDeviationY) ||
      standardDeviationX <= 0.0f ||
      standardDeviationY <= 0.0f
    )
    {
      return false;
    }


    bool acceptedAny = false;


    // X and Y body velocity measurements are processed as
    // two scalar Kalman updates.

    for (
      int measuredAxis = 0;
      measuredAxis < 2;
      ++measuredAxis
    )
    {
      float velocityBody[3];


      getBodyVelocity(
        velocityBody
      );


      float H[STATE_DIM] = {};


      // ----------------------------------------------------------
      // VELOCITY JACOBIAN
      //
      // h =
      //
      //     e_i^T R_NB^T v_N
      // ----------------------------------------------------------

      for (
        int navAxis = 0;
        navAxis < 3;
        ++navAxis
      )
      {
        H[3 + navAxis] =
          rotation_[navAxis][measuredAxis];
      }


      // ----------------------------------------------------------
      // ATTITUDE JACOBIAN
      //
      // With right-multiplicative attitude error:
      //
      // R_true^T v
      //
      // ~= R^T v + [v_B]x dtheta
      // ----------------------------------------------------------

      float skewVelocityBody[3][3];


      skew(
        velocityBody,
        skewVelocityBody
      );


      for (
        int thetaAxis = 0;
        thetaAxis < 3;
        ++thetaAxis
      )
      {
        H[6 + thetaAxis] =
          skewVelocityBody
            [measuredAxis]
            [thetaAxis];
      }


      const float measurement =
        measuredAxis == 0
          ? bodyVelocityX
          : bodyVelocityY;


      const float standardDeviation =
        measuredAxis == 0
          ? standardDeviationX
          : standardDeviationY;


      const bool accepted =
        scalarUpdate(
          H,

          measurement -
            velocityBody[measuredAxis],

          square(
            standardDeviation
          ),

          gateSigma
        );


      acceptedAny =
        acceptedAny || accepted;
    }


    return acceptedAny;
  }



  // ==============================================================
  // BODY X/Y VELOCITY UPDATE -- TRANSLATION ONLY
  // ==============================================================
  //
  // Commissioning-safe optical-flow update used by the robust XY build.
  // The measurement still observes navigation-frame velocity through
  // v_B = R_NB^T v_N, but it deliberately does not use an attitude-error
  // Jacobian. This prevents an unverified PMW axis/sign/scale from changing
  // the quaternion. The returned bit mask reports which scalar axes passed
  // their innovation gates: bit0=X, bit1=Y.
  //
  // ==============================================================

  uint8_t updateBodyVelocityVelocityOnly(
    float bodyVelocityX,
    float bodyVelocityY,
    float standardDeviationX,
    float standardDeviationY,
    float gateSigma = 6.0f
  )
  {
    if (
      !isfinite(bodyVelocityX) ||
      !isfinite(bodyVelocityY) ||
      !isfinite(standardDeviationX) ||
      !isfinite(standardDeviationY) ||
      standardDeviationX <= 0.0f ||
      standardDeviationY <= 0.0f
    )
    {
      return 0U;
    }

    uint8_t acceptedMask = 0U;

    for (int measuredAxis = 0; measuredAxis < 2; ++measuredAxis)
    {
      float velocityBody[3] = {};
      getBodyVelocity(velocityBody);

      float H[STATE_DIM] = {};

      // h = e_i^T R_NB^T v_N. Only velocity-error states participate.
      for (int navAxis = 0; navAxis < 3; ++navAxis)
      {
        H[3 + navAxis] = rotation_[navAxis][measuredAxis];
      }

      const float measurement = measuredAxis == 0
        ? bodyVelocityX
        : bodyVelocityY;

      const float standardDeviation = measuredAxis == 0
        ? standardDeviationX
        : standardDeviationY;

      const bool accepted = scalarUpdate(
        H,
        measurement - velocityBody[measuredAxis],
        square(standardDeviation),
        gateSigma
      );

      if (accepted)
      {
        acceptedMask |= static_cast<uint8_t>(1U << measuredAxis);
      }
    }

    return acceptedMask;
  }

  // ==============================================================
  // GRAVITY-DIRECTION ATTITUDE UPDATE
  // ==============================================================
  //
  // Accelerometer specific force should point approximately along
  // navigation +Z while the drone is not undergoing strong linear
  // acceleration.
  //
  // This measurement observes roll/pitch but intentionally does not
  // provide absolute yaw.
  //
  // ==============================================================

  bool updateGravityDirection(
    const float accelBody[3],
    float standardDeviationUnitVector,
    float gateSigma = 5.0f
  )
  {
    if (
      !finite3(accelBody) ||
      !isfinite(standardDeviationUnitVector) ||
      standardDeviationUnitVector <= 0.0f
    )
    {
      return false;
    }


    const float magnitude =
      sqrtf(
        accelBody[0] * accelBody[0] +
        accelBody[1] * accelBody[1] +
        accelBody[2] * accelBody[2]
      );


    // ------------------------------------------------------------
    // REJECT DURING STRONG TRANSLATIONAL ACCELERATION
    // ------------------------------------------------------------

    if (
      magnitude < 0.70f * params_.gravity ||
      magnitude > 1.30f * params_.gravity
    )
    {
      return false;
    }


    const float measuredDirection[3] =
    {
      accelBody[0] / magnitude,
      accelBody[1] / magnitude,
      accelBody[2] / magnitude
    };


    bool acceptedAny = false;


    // ------------------------------------------------------------
    // Use X/Y normalized gravity components.
    //
    // The Z component becomes nearly redundant with normalized
    // X/Y and can unnecessarily over-constrain the update.
    // ------------------------------------------------------------

    for (
      int measuredAxis = 0;
      measuredAxis < 2;
      ++measuredAxis
    )
    {
      // ----------------------------------------------------------
      // Predicted body-frame direction of navigation +Z.
      //
      // Since:
      //
      //     v_B = R_NB^T v_N
      //
      // navigation +Z expressed in B is the third row of R_NB.
      // ----------------------------------------------------------

      const float predictedDirection[3] =
      {
        rotation_[2][0],
        rotation_[2][1],
        rotation_[2][2]
      };


      float skewDirection[3][3];


      skew(
        predictedDirection,
        skewDirection
      );


      float H[STATE_DIM] = {};


      for (
        int thetaAxis = 0;
        thetaAxis < 3;
        ++thetaAxis
      )
      {
        H[6 + thetaAxis] =
          skewDirection
            [measuredAxis]
            [thetaAxis];
      }


      const bool accepted =
        scalarUpdate(
          H,

          measuredDirection[measuredAxis] -
            predictedDirection[measuredAxis],

          square(
            standardDeviationUnitVector
          ),

          gateSigma
        );


      acceptedAny =
        acceptedAny || accepted;
    }


    return acceptedAny;
  }


  // ==============================================================
  // MAGNETOMETER YAW UPDATE
  // ==============================================================
  //
  // NEW FOR QMC5883P.
  //
  //
  // The flight controller:
  //
  //   1. Reads QMC5883P.
  //
  //   2. Applies hard-iron offsets.
  //
  //   3. Applies axis scale corrections.
  //
  //   4. Converts sensor frame:
  //
  //          sensor X = left
  //          sensor Y = backward
  //          sensor Z = up
  //
  //      into body frame:
  //
  //          body X = -sensor X
  //          body Y = -sensor Y
  //          body Z = +sensor Z
  //
  //   5. Uses the current ESKF quaternion to remove the vertical
  //      magnetic-field component.
  //
  //   6. Calculates a right-handed magnetic yaw.
  //
  //   7. Calls this function.
  //
  //
  // IMPORTANT:
  //
  // This is a MEASUREMENT UPDATE.
  //
  // It does not directly overwrite quaternion yaw.
  //
  // The ESKF computes:
  //
  //     innovation
  //
  //     innovation covariance
  //
  //     Kalman gain
  //
  //     attitude correction
  //
  // and then injects the small attitude correction into q_NB.
  //
  //
  // Right-handed convention:
  //
  //     +yaw = CCW viewed from above.
  //
  // The main flight controller uses clockwise-positive yaw and
  // performs the sign conversion before calling this method.
  //
  // ==============================================================

  bool updateYawRightHand(
    float yawRightHandRad,
    float standardDeviationRad,
    float gateSigma = 6.0f
  )
  {
    // ------------------------------------------------------------
    // INPUT VALIDITY
    // ------------------------------------------------------------

    if (
      !isfinite(yawRightHandRad) ||
      !isfinite(standardDeviationRad) ||
      standardDeviationRad <= 0.0f
    )
    {
      return false;
    }


    // ------------------------------------------------------------
    // CURRENT ESKF EULER ANGLES
    // ------------------------------------------------------------

    float pitchAboutXRad = 0.0f;
    float rollAboutYRad = 0.0f;

    float predictedYawRightHandRad =
      0.0f;


    getEulerRadians(
      pitchAboutXRad,
      rollAboutYRad,
      predictedYawRightHandRad
    );


    // ------------------------------------------------------------
    // EULER SINGULARITY PROTECTION
    //
    // For Z-Y-X Euler angles:
    //
    //     cos(theta) -> 0
    //
    // near +/-90 degrees rotation about body Y.
    //
    // A normal quadrotor should never intentionally operate there.
    // ------------------------------------------------------------

    const float cosineRollY =
      cosf(
        rollAboutYRad
      );


    if (
      !isfinite(cosineRollY) ||
      fabsf(cosineRollY) < 0.20f
    )
    {
      return false;
    }


    // ------------------------------------------------------------
    // YAW INNOVATION
    //
    // residual =
    //
    //     measured yaw - predicted yaw
    //
    // wrapped to [-pi,+pi].
    // ------------------------------------------------------------

    float residual =
      yawRightHandRad -
      predictedYawRightHandRad;


    static constexpr float PI_VALUE =
      3.14159265358979323846f;


    static constexpr float TWO_PI_VALUE =
      2.0f * PI_VALUE;


    while (residual > PI_VALUE)
    {
      residual -= TWO_PI_VALUE;
    }


    while (residual < -PI_VALUE)
    {
      residual += TWO_PI_VALUE;
    }


    // ------------------------------------------------------------
    // MAGNETOMETER MEASUREMENT JACOBIAN
    //
    //
    // This ESKF attitude error is:
    //
    //     dtheta_B =
    //
    //       [dtheta_x,
    //        dtheta_y,
    //        dtheta_z]
    //
    // in BODY coordinates.
    //
    //
    // For Z-Y-X Euler angles and a small right-multiplicative
    // body-frame error:
    //
    // dpsi ~=
    //
    //     sin(phi)
    //     -------- dtheta_y
    //     cos(theta)
    //
    //       +
    //
    //     cos(phi)
    //     -------- dtheta_z
    //     cos(theta)
    //
    //
    // Therefore:
    //
    // H =
    //
    // [0 0 0
    //  0 0 0
    //  0
    //  sin(phi)/cos(theta)
    //  cos(phi)/cos(theta)]
    //
    // ==============================================================

    float H[STATE_DIM] = {};


    H[7] =
      sinf(
        pitchAboutXRad
      ) /
      cosineRollY;


    H[8] =
      cosf(
        pitchAboutXRad
      ) /
      cosineRollY;


    // ------------------------------------------------------------
    // NORMAL SCALAR ESKF MEASUREMENT UPDATE
    // ------------------------------------------------------------

    return scalarUpdate(
      H,
      residual,
      square(
        standardDeviationRad
      ),
      gateSigma
    );
  }


  // ==============================================================
  // ZERO-VELOCITY UPDATE
  // ==============================================================
  //
  // Used while motors are disabled to keep translational state from
  // drifting while the aircraft is physically stationary.
  //
  // ==============================================================

  bool updateZeroVelocity(
    float standardDeviationMetersPerSecond = 0.03f,
    float gateSigma = 8.0f
  )
  {
    if (
      !isfinite(
        standardDeviationMetersPerSecond
      ) ||
      standardDeviationMetersPerSecond <= 0.0f
    )
    {
      return false;
    }


    bool acceptedAny = false;


    for (int axis = 0; axis < 3; ++axis)
    {
      float H[STATE_DIM] = {};


      H[3 + axis] = 1.0f;


      const bool accepted =
        scalarUpdate(
          H,
          -velocity_[axis],

          square(
            standardDeviationMetersPerSecond
          ),

          gateSigma
        );


      acceptedAny =
        acceptedAny || accepted;
    }


    return acceptedAny;
  }


  // ==============================================================
  // STATE ACCESSORS
  // ==============================================================

  void getPosition(
    float output[3]
  ) const
  {
    memcpy(
      output,
      position_,
      sizeof(position_)
    );
  }


  void getVelocityNavigation(
    float output[3]
  ) const
  {
    memcpy(
      output,
      velocity_,
      sizeof(velocity_)
    );
  }


  void getBodyVelocity(
    float output[3]
  ) const
  {
    // ------------------------------------------------------------
    // v_B = R_NB^T v_N
    // ------------------------------------------------------------

    for (
      int bodyAxis = 0;
      bodyAxis < 3;
      ++bodyAxis
    )
    {
      output[bodyAxis] =
        rotation_[0][bodyAxis] *
          velocity_[0] +

        rotation_[1][bodyAxis] *
          velocity_[1] +

        rotation_[2][bodyAxis] *
          velocity_[2];
    }
  }


  void getRotationMatrix(
    float output[3][3]
  ) const
  {
    memcpy(
      output,
      rotation_,
      sizeof(rotation_)
    );
  }


  // ==============================================================
  // EULER ANGLES
  // ==============================================================

  void getEulerRadians(
    float &pitchAboutXRad,
    float &rollAboutYRad,
    float &yawRightHandRad
  ) const
  {
    // ------------------------------------------------------------
    // Z-Y-X decomposition:
    //
    // R =
    //
    //     Rz(yaw)
    //     Ry(rotation about Y)
    //     Rx(rotation about X)
    // ------------------------------------------------------------

    const float sinRollY =
      clamp(
        -rotation_[2][0],
        -1.0f,
        1.0f
      );


    rollAboutYRad =
      asinf(
        sinRollY
      );


    pitchAboutXRad =
      atan2f(
        rotation_[2][1],
        rotation_[2][2]
      );


    yawRightHandRad =
      atan2f(
        rotation_[1][0],
        rotation_[0][0]
      );
  }


  // ==============================================================
  // SIMPLE ACCESSORS
  // ==============================================================

  float positionX() const
  {
    return position_[0];
  }


  float positionY() const
  {
    return position_[1];
  }


  float positionZ() const
  {
    return position_[2];
  }


  float velocityX() const
  {
    return velocity_[0];
  }


  float velocityY() const
  {
    return velocity_[1];
  }


  float velocityZ() const
  {
    return velocity_[2];
  }


  float lastInnovationRatio() const
  {
    return lastInnovationRatio_;
  }


  uint32_t rejectedUpdates() const
  {
    return rejectedUpdates_;
  }


  uint32_t acceptedUpdates() const
  {
    return acceptedUpdates_;
  }


  bool healthy() const
  {
    return healthy_;
  }


  float covarianceDiagonal(
    int index
  ) const
  {
    if (
      index < 0 ||
      index >= STATE_DIM
    )
    {
      return NAN;
    }


    return
      covariance_[index][index];
  }


private:

  // ==============================================================
  // NOMINAL STATE
  // ==============================================================

  Params params_;


  float position_[3] =
  {
    0.0f,
    0.0f,
    0.0f
  };


  float velocity_[3] =
  {
    0.0f,
    0.0f,
    0.0f
  };


  // q_NB = [w x y z]

  float quaternion_[4] =
  {
    1.0f,
    0.0f,
    0.0f,
    0.0f
  };


  // R_NB

  float rotation_[3][3] = {};


  // Error covariance

  float covariance_
    [STATE_DIM]
    [STATE_DIM] = {};


  float lastInnovationRatio_ =
    0.0f;


  uint32_t rejectedUpdates_ =
    0;


  uint32_t acceptedUpdates_ =
    0;


  bool healthy_ =
    true;


  // ==============================================================
  // BASIC MATH HELPERS
  // ==============================================================

  static float square(
    float value
  )
  {
    return value * value;
  }


  static float clamp(
    float value,
    float minimum,
    float maximum
  )
  {
    if (value < minimum)
    {
      return minimum;
    }


    if (value > maximum)
    {
      return maximum;
    }


    return value;
  }


  static bool finite3(
    const float value[3]
  )
  {
    return
      isfinite(value[0]) &&
      isfinite(value[1]) &&
      isfinite(value[2]);
  }


  // ==============================================================
  // SKEW-SYMMETRIC MATRIX
  // ==============================================================

  static void skew(
    const float vector[3],
    float output[3][3]
  )
  {
    output[0][0] = 0.0f;
    output[0][1] = -vector[2];
    output[0][2] = vector[1];


    output[1][0] = vector[2];
    output[1][1] = 0.0f;
    output[1][2] = -vector[0];


    output[2][0] = -vector[1];
    output[2][1] = vector[0];
    output[2][2] = 0.0f;
  }


// ================================================================
// QUATERNION MULTIPLICATION
// ================================================================

static void quaternionMultiply(
  const float left[4],
  const float right[4],
  float output[4]
)
{
  output[0] =
    left[0] * right[0] -
    left[1] * right[1] -
    left[2] * right[2] -
    left[3] * right[3];

  output[1] =
    left[0] * right[1] +
    left[1] * right[0] +
    left[2] * right[3] -
    left[3] * right[2];

  output[2] =
    left[0] * right[2] -
    left[1] * right[3] +
    left[2] * right[0] +
    left[3] * right[1];

  output[3] =
    left[0] * right[3] +
    left[1] * right[2] -
    left[2] * right[1] +
    left[3] * right[0];
}


  // ==============================================================
  // QUATERNION NORMALIZATION
  // ==============================================================

  static void normalizeQuaternion(
    float quaternion[4]
  )
  {
    const float norm =
      sqrtf(
        quaternion[0] * quaternion[0] +
        quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] +
        quaternion[3] * quaternion[3]
      );


    if (
      !isfinite(norm) ||
      norm < 1.0e-8f
    )
    {
      quaternion[0] = 1.0f;
      quaternion[1] = 0.0f;
      quaternion[2] = 0.0f;
      quaternion[3] = 0.0f;

      return;
    }


    const float inverseNorm =
      1.0f / norm;


    for (int i = 0; i < 4; ++i)
    {
      quaternion[i] *=
        inverseNorm;
    }
  }


  // ==============================================================
  // ROTATION VECTOR -> QUATERNION
  // ==============================================================

  static void rotationVectorToQuaternion(
    float x,
    float y,
    float z,
    float output[4]
  )
  {
    const float angle =
      sqrtf(
        x * x +
        y * y +
        z * z
      );


    // ------------------------------------------------------------
    // SMALL-ANGLE APPROXIMATION
    // ------------------------------------------------------------

    if (angle < 1.0e-6f)
    {
      output[0] = 1.0f;

      output[1] =
        0.5f * x;

      output[2] =
        0.5f * y;

      output[3] =
        0.5f * z;


      normalizeQuaternion(
        output
      );


      return;
    }


    // ------------------------------------------------------------
    // EXACT ROTATION-VECTOR CONVERSION
    // ------------------------------------------------------------

    const float halfAngle =
      0.5f * angle;


    const float scale =
      sinf(halfAngle) /
      angle;


    output[0] =
      cosf(halfAngle);


    output[1] =
      scale * x;


    output[2] =
      scale * y;


    output[3] =
      scale * z;
  }


  // ==============================================================
  // ZYX EULER -> QUATERNION
  // ==============================================================

  static void quaternionFromZyx(
    float yawZ,
    float rollY,
    float pitchX,
    float output[4]
  )
  {
    const float cy =
      cosf(
        0.5f * yawZ
      );


    const float sy =
      sinf(
        0.5f * yawZ
      );


    const float cr =
      cosf(
        0.5f * rollY
      );


    const float sr =
      sinf(
        0.5f * rollY
      );


    const float cp =
      cosf(
        0.5f * pitchX
      );


    const float sp =
      sinf(
        0.5f * pitchX
      );


    output[0] =
      cy * cr * cp +
      sy * sr * sp;


    output[1] =
      cy * cr * sp -
      sy * sr * cp;


    output[2] =
      cy * sr * cp +
      sy * cr * sp;


    output[3] =
      sy * cr * cp -
      cy * sr * sp;
  }


  // ==============================================================
  // QUATERNION -> ROTATION MATRIX
  // ==============================================================

  void updateRotationMatrix()
  {
    const float w =
      quaternion_[0];

    const float x =
      quaternion_[1];

    const float y =
      quaternion_[2];

    const float z =
      quaternion_[3];


    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;

    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;


    rotation_[0][0] =
      1.0f -
      2.0f * (yy + zz);


    rotation_[0][1] =
      2.0f * (xy - wz);


    rotation_[0][2] =
      2.0f * (xz + wy);


    rotation_[1][0] =
      2.0f * (xy + wz);


    rotation_[1][1] =
      1.0f -
      2.0f * (xx + zz);


    rotation_[1][2] =
      2.0f * (yz - wx);


    rotation_[2][0] =
      2.0f * (xz - wy);


    rotation_[2][1] =
      2.0f * (yz + wx);


    rotation_[2][2] =
      1.0f -
      2.0f * (xx + yy);
  }


  // ==============================================================
  // COVARIANCE PROPAGATION
  // ==============================================================

  void propagateCovariance(
    const float transition
      [STATE_DIM]
      [STATE_DIM],

    float dt,
    float processNoiseScale
  )
  {
    float temporary
      [STATE_DIM]
      [STATE_DIM] = {};


    float propagated
      [STATE_DIM]
      [STATE_DIM] = {};


    // ------------------------------------------------------------
    // temporary = A P
    // ------------------------------------------------------------

    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        float sum = 0.0f;


        for (
          int k = 0;
          k < STATE_DIM;
          ++k
        )
        {
          sum +=
            transition[row][k] *
            covariance_[k][col];
        }


        temporary[row][col] =
          sum;
      }
    }


    // ------------------------------------------------------------
    // propagated = A P A'
    // ------------------------------------------------------------

    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        float sum = 0.0f;


        for (
          int k = 0;
          k < STATE_DIM;
          ++k
        )
        {
          sum +=
            temporary[row][k] *
            transition[col][k];
        }


        propagated[row][col] =
          sum;
      }
    }


    memcpy(
      covariance_,
      propagated,
      sizeof(covariance_)
    );


    // ------------------------------------------------------------
    // PROCESS NOISE
    //
    // processNoiseScale multiplies PROCESS VARIANCE.
    //
    // So:
    //
    //     1.0 = nominal uncertainty
    //
    //     2.0 = twice the added covariance
    //
    // rather than doubling sigma and therefore quadrupling variance.
    // ------------------------------------------------------------

    const float dtSquared =
      dt * dt;


    covariance_[0][0] +=
      processNoiseScale *
      square(
        params_.accelNoiseXY *
        dtSquared
      );


    covariance_[1][1] +=
      processNoiseScale *
      square(
        params_.accelNoiseXY *
        dtSquared
      );


    covariance_[2][2] +=
      processNoiseScale *
      square(
        params_.accelNoiseZ *
        dtSquared
      );


    covariance_[3][3] +=
      processNoiseScale *
      square(
        params_.accelNoiseXY *
        dt
      );


    covariance_[4][4] +=
      processNoiseScale *
      square(
        params_.accelNoiseXY *
        dt
      );


    covariance_[5][5] +=
      processNoiseScale *
      square(
        params_.accelNoiseZ *
        dt
      );


    covariance_[6][6] +=
      processNoiseScale *
      square(
        params_.gyroNoiseRollPitch *
        dt
      );


    covariance_[7][7] +=
      processNoiseScale *
      square(
        params_.gyroNoiseRollPitch *
        dt
      );


    covariance_[8][8] +=
      processNoiseScale *
      square(
        params_.gyroNoiseYaw *
        dt
      );
  }


  // ==============================================================
  // GENERIC SCALAR KALMAN UPDATE
  // ==============================================================

  bool scalarUpdate(
    const float H[STATE_DIM],
    float residual,
    float measurementVariance,
    float gateSigma
  )
  {
    if (
      !isfinite(residual) ||
      !isfinite(measurementVariance) ||
      measurementVariance <= 0.0f
    )
    {
      return false;
    }


    // ------------------------------------------------------------
    // P H'
    // ------------------------------------------------------------

    float pTimesHt[STATE_DIM] = {};


    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        pTimesHt[row] +=
          covariance_[row][col] *
          H[col];
      }
    }


    // ------------------------------------------------------------
    // INNOVATION VARIANCE
    //
    // S =
    //
    //     H P H' + R
    // ------------------------------------------------------------

    float innovationVariance =
      measurementVariance;


    for (
      int i = 0;
      i < STATE_DIM;
      ++i
    )
    {
      innovationVariance +=
        H[i] *
        pTimesHt[i];
    }


    if (
      !isfinite(innovationVariance) ||
      innovationVariance < 1.0e-9f
    )
    {
      healthy_ = false;
      return false;
    }


    // ------------------------------------------------------------
    // NORMALIZED INNOVATION
    // ------------------------------------------------------------

    lastInnovationRatio_ =
      fabsf(residual) /
      sqrtf(
        innovationVariance
      );


    // ------------------------------------------------------------
    // INNOVATION GATE
    // ------------------------------------------------------------

    if (
      gateSigma > 0.0f &&
      lastInnovationRatio_ >
        gateSigma
    )
    {
      ++rejectedUpdates_;

      return false;
    }


    // ------------------------------------------------------------
    // KALMAN GAIN
    //
    // K =
    //
    //     P H' / S
    // ------------------------------------------------------------

    float gain[STATE_DIM] = {};


    float correction[STATE_DIM] = {};


    for (
      int i = 0;
      i < STATE_DIM;
      ++i
    )
    {
      gain[i] =
        pTimesHt[i] /
        innovationVariance;


      correction[i] =
        gain[i] *
        residual;
    }


    // ------------------------------------------------------------
    // JOSEPH COVARIANCE UPDATE
    //
    // P_new =
    //
    // (I-KH)P(I-KH)' + K R K'
    //
    // More numerically stable than:
    //
    // P = (I-KH)P
    // ------------------------------------------------------------

    float iMinusKh
      [STATE_DIM]
      [STATE_DIM] = {};


    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        iMinusKh[row][col] =
          (
            row == col
              ? 1.0f
              : 0.0f
          ) -
          gain[row] *
          H[col];
      }
    }


    float temporary
      [STATE_DIM]
      [STATE_DIM] = {};


    float updated
      [STATE_DIM]
      [STATE_DIM] = {};


    // ------------------------------------------------------------
    // temporary = (I-KH)P
    // ------------------------------------------------------------

    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        float sum = 0.0f;


        for (
          int k = 0;
          k < STATE_DIM;
          ++k
        )
        {
          sum +=
            iMinusKh[row][k] *
            covariance_[k][col];
        }


        temporary[row][col] =
          sum;
      }
    }


    // ------------------------------------------------------------
    // updated =
    //
    // temporary(I-KH)' + KRK'
    // ------------------------------------------------------------

    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        float sum =
          gain[row] *
          measurementVariance *
          gain[col];


        for (
          int k = 0;
          k < STATE_DIM;
          ++k
        )
        {
          sum +=
            temporary[row][k] *
            iMinusKh[col][k];
        }


        updated[row][col] =
          sum;
      }
    }


    memcpy(
      covariance_,
      updated,
      sizeof(covariance_)
    );


    // ------------------------------------------------------------
    // ERROR-STATE INJECTION
    // ------------------------------------------------------------

    injectAndReset(
      correction
    );


    ++acceptedUpdates_;


    sanitize();


    return healthy_;
  }


  // ==============================================================
  // ERROR-STATE INJECTION AND RESET
  // ==============================================================

  void injectAndReset(
    const float correction[STATE_DIM]
  )
  {
    // ------------------------------------------------------------
    // POSITION / VELOCITY
    // ------------------------------------------------------------

    for (
      int axis = 0;
      axis < 3;
      ++axis
    )
    {
      position_[axis] +=
        correction[axis];


      velocity_[axis] +=
        correction[axis + 3];
    }


    // ------------------------------------------------------------
    // ATTITUDE ERROR
    // ------------------------------------------------------------

    const float deltaTheta[3] =
    {
      correction[6],
      correction[7],
      correction[8]
    };


    float deltaQuaternion[4];


    rotationVectorToQuaternion(
      deltaTheta[0],
      deltaTheta[1],
      deltaTheta[2],
      deltaQuaternion
    );


    // Right-multiplicative correction:

    float correctedQuaternion[4];


    quaternionMultiply(
      quaternion_,
      deltaQuaternion,
      correctedQuaternion
    );


    memcpy(
      quaternion_,
      correctedQuaternion,
      sizeof(quaternion_)
    );


    normalizeQuaternion(
      quaternion_
    );


    updateRotationMatrix();


    // ------------------------------------------------------------
    // ESKF COVARIANCE RESET JACOBIAN
    //
    // For right-multiplicative error:
    //
    // G_theta ~=
    //
    //     I - 1/2 [dtheta]x
    // ------------------------------------------------------------

    float resetJacobian
      [STATE_DIM]
      [STATE_DIM] = {};


    for (
      int i = 0;
      i < STATE_DIM;
      ++i
    )
    {
      resetJacobian[i][i] =
        1.0f;
    }


    float skewDelta[3][3];


    skew(
      deltaTheta,
      skewDelta
    );


    for (
      int row = 0;
      row < 3;
      ++row
    )
    {
      for (
        int col = 0;
        col < 3;
        ++col
      )
      {
        resetJacobian
          [row + 6]
          [col + 6] -=

          0.5f *
          skewDelta[row][col];
      }
    }


    float temporary
      [STATE_DIM]
      [STATE_DIM] = {};


    float resetCovariance
      [STATE_DIM]
      [STATE_DIM] = {};


    // ------------------------------------------------------------
    // temporary = G P
    // ------------------------------------------------------------

    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        float sum = 0.0f;


        for (
          int k = 0;
          k < STATE_DIM;
          ++k
        )
        {
          sum +=
            resetJacobian[row][k] *
            covariance_[k][col];
        }


        temporary[row][col] =
          sum;
      }
    }


    // ------------------------------------------------------------
    // resetCovariance =
    //
    //     G P G'
    // ------------------------------------------------------------

    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = 0;
        col < STATE_DIM;
        ++col
      )
      {
        float sum = 0.0f;


        for (
          int k = 0;
          k < STATE_DIM;
          ++k
        )
        {
          sum +=
            temporary[row][k] *
            resetJacobian[col][k];
        }


        resetCovariance[row][col] =
          sum;
      }
    }


    memcpy(
      covariance_,
      resetCovariance,
      sizeof(covariance_)
    );
  }


  // ==============================================================
  // NUMERICAL SANITY
  // ==============================================================

  void sanitize()
  {
    healthy_ = true;


    // ------------------------------------------------------------
    // NOMINAL STATE
    // ------------------------------------------------------------

    for (
      int axis = 0;
      axis < 3;
      ++axis
    )
    {
      if (
        !isfinite(position_[axis]) ||
        !isfinite(velocity_[axis])
      )
      {
        healthy_ = false;
      }
    }


    // ------------------------------------------------------------
    // COVARIANCE
    //
    // Force symmetry and limit runaway values.
    // ------------------------------------------------------------

    for (
      int row = 0;
      row < STATE_DIM;
      ++row
    )
    {
      for (
        int col = row;
        col < STATE_DIM;
        ++col
      )
      {
        float value =
          0.5f *
          (
            covariance_[row][col] +
            covariance_[col][row]
          );


        if (!isfinite(value))
        {
          healthy_ = false;


          value =
            row == col
              ? params_.maxCovariance
              : 0.0f;
        }


        if (row == col)
        {
          value =
            clamp(
              value,
              params_.minCovariance,
              params_.maxCovariance
            );
        }
        else
        {
          value =
            clamp(
              value,
              -params_.maxCovariance,
              params_.maxCovariance
            );
        }


        covariance_[row][col] =
          value;


        covariance_[col][row] =
          value;
      }
    }


    // ------------------------------------------------------------
    // QUATERNION HEALTH
    // ------------------------------------------------------------

    const float quaternionNorm =
      sqrtf(
        quaternion_[0] * quaternion_[0] +
        quaternion_[1] * quaternion_[1] +
        quaternion_[2] * quaternion_[2] +
        quaternion_[3] * quaternion_[3]
      );


    if (
      !isfinite(quaternionNorm) ||
      quaternionNorm < 0.5f ||
      quaternionNorm > 1.5f
    )
    {
      healthy_ = false;
    }
  }
};