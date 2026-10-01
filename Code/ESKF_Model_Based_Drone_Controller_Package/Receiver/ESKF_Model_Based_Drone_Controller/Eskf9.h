#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

/*
  Fixed-size 9-error-state ESKF for the ESP32.

  Nominal state:
    position_N [m]       local navigation frame (+X right at arm, +Y forward, +Z up)
    velocity_N [m/s]
    q_NB                 body-to-navigation quaternion, [w x y z]

  Error state:
    dx = [dp_N, dv_N, dtheta_B]

  The implementation follows the predict -> asynchronous measurement update ->
  attitude-error injection/reset architecture used by koledickarlo/ESKF-ESP32,
  but uses fixed arrays instead of Eigen so it can be dropped directly into an
  Arduino IDE ESP32 sketch.
*/
class Eskf9
{
public:
  static constexpr int STATE_DIM = 9;

  struct Params
  {
    float gravity = 9.80665f;

    // Process-noise standard deviations.
    float accelNoiseXY = 0.85f;       // m/s^2
    float accelNoiseZ = 1.20f;        // m/s^2
    float gyroNoiseRollPitch = 0.08f; // rad/s
    float gyroNoiseYaw = 0.12f;       // rad/s

    // Initial 1-sigma uncertainties.
    float initialPositionXY = 0.03f;  // m
    float initialPositionZ = 0.03f;   // m
    float initialVelocity = 0.08f;    // m/s
    float initialTilt = 0.04f;        // rad
    float initialYaw = 0.20f;         // rad

    float maxCovariance = 100.0f;
    float minCovariance = 1.0e-7f;
  };

  Eskf9()
    : params_(Params{})
  {
    reset(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  explicit Eskf9(const Params &params)
    : params_(params)
  {
    reset(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  void reset(
    float x,
    float y,
    float z,
    float pitchAboutXRad,
    float rollAboutYRad,
    float yawRightHandRad
  )
  {
    position_[0] = x;
    position_[1] = y;
    position_[2] = z;

    velocity_[0] = 0.0f;
    velocity_[1] = 0.0f;
    velocity_[2] = 0.0f;

    quaternionFromZyx(
      yawRightHandRad,
      rollAboutYRad,
      pitchAboutXRad,
      quaternion_
    );
    normalizeQuaternion(quaternion_);
    updateRotationMatrix();

    memset(covariance_, 0, sizeof(covariance_));
    covariance_[0][0] = square(params_.initialPositionXY);
    covariance_[1][1] = square(params_.initialPositionXY);
    covariance_[2][2] = square(params_.initialPositionZ);
    covariance_[3][3] = square(params_.initialVelocity);
    covariance_[4][4] = square(params_.initialVelocity);
    covariance_[5][5] = square(params_.initialVelocity);
    covariance_[6][6] = square(params_.initialTilt);
    covariance_[7][7] = square(params_.initialTilt);
    covariance_[8][8] = square(params_.initialYaw);

    lastInnovationRatio_ = 0.0f;
    rejectedUpdates_ = 0;
    acceptedUpdates_ = 0;
    healthy_ = true;
  }

  void resetLocalPositionAndYaw(float z)
  {
    float pitchX = 0.0f;
    float rollY = 0.0f;
    float yawRh = 0.0f;
    getEulerRadians(pitchX, rollY, yawRh);
    reset(0.0f, 0.0f, z, pitchX, rollY, 0.0f);
  }

  void predict(
    const float accelBody[3],
    const float gyroBody[3],
    float dt
  )
  {
    if (!finite3(accelBody) || !finite3(gyroBody) || !isfinite(dt))
    {
      healthy_ = false;
      return;
    }

    dt = clamp(dt, 0.001f, 0.020f);

    // Body-frame incremental rotation, right-multiplied into q_NB.
    float deltaQuaternion[4];
    rotationVectorToQuaternion(
      gyroBody[0] * dt,
      gyroBody[1] * dt,
      gyroBody[2] * dt,
      deltaQuaternion
    );

    float propagatedQuaternion[4];
    quaternionMultiply(quaternion_, deltaQuaternion, propagatedQuaternion);
    memcpy(quaternion_, propagatedQuaternion, sizeof(quaternion_));
    normalizeQuaternion(quaternion_);
    updateRotationMatrix();

    float accelerationNav[3] = {
      rotation_[0][0] * accelBody[0] +
        rotation_[0][1] * accelBody[1] +
        rotation_[0][2] * accelBody[2],
      rotation_[1][0] * accelBody[0] +
        rotation_[1][1] * accelBody[1] +
        rotation_[1][2] * accelBody[2],
      rotation_[2][0] * accelBody[0] +
        rotation_[2][1] * accelBody[1] +
        rotation_[2][2] * accelBody[2] - params_.gravity
    };

    const float halfDtSquared = 0.5f * dt * dt;
    for (int axis = 0; axis < 3; ++axis)
    {
      position_[axis] += velocity_[axis] * dt +
        accelerationNav[axis] * halfDtSquared;
      velocity_[axis] += accelerationNav[axis] * dt;
    }

    // First-order discrete error-state transition A = I + F dt.
    float transition[STATE_DIM][STATE_DIM] = {};
    for (int i = 0; i < STATE_DIM; ++i)
    {
      transition[i][i] = 1.0f;
    }

    for (int axis = 0; axis < 3; ++axis)
    {
      transition[axis][axis + 3] = dt;
    }

    float skewAccel[3][3];
    skew(accelBody, skewAccel);

    // dv/dtheta = -R * [a_B]x
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        float value = 0.0f;
        for (int k = 0; k < 3; ++k)
        {
          value -= rotation_[row][k] * skewAccel[k][col];
        }
        transition[row + 3][col + 6] += value * dt;
      }
    }

    float skewGyro[3][3];
    skew(gyroBody, skewGyro);

    // dtheta_dot = -[omega_B]x dtheta
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        transition[row + 6][col + 6] -= skewGyro[row][col] * dt;
      }
    }

    propagateCovariance(transition, dt);
    sanitize();
  }

  bool updateAltitude(
    float altitudeMeters,
    float standardDeviationMeters,
    float gateSigma = 6.0f
  )
  {
    if (!isfinite(altitudeMeters) || !isfinite(standardDeviationMeters) ||
        standardDeviationMeters <= 0.0f)
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

  bool updateVerticalVelocity(
    float velocityMetersPerSecond,
    float standardDeviationMetersPerSecond,
    float gateSigma = 6.0f
  )
  {
    float H[STATE_DIM] = {};
    H[5] = 1.0f;
    return scalarUpdate(
      H,
      velocityMetersPerSecond - velocity_[2],
      square(standardDeviationMetersPerSecond),
      gateSigma
    );
  }

  bool updateBodyVelocity(
    float bodyVelocityX,
    float bodyVelocityY,
    float standardDeviationX,
    float standardDeviationY,
    float gateSigma = 6.0f
  )
  {
    if (!isfinite(bodyVelocityX) || !isfinite(bodyVelocityY) ||
        !isfinite(standardDeviationX) || !isfinite(standardDeviationY) ||
        standardDeviationX <= 0.0f || standardDeviationY <= 0.0f)
    {
      return false;
    }

    bool acceptedAny = false;

    for (int measuredAxis = 0; measuredAxis < 2; ++measuredAxis)
    {
      float velocityBody[3];
      getBodyVelocity(velocityBody);

      float H[STATE_DIM] = {};

      // h = e_i^T R_NB^T v_N
      for (int navAxis = 0; navAxis < 3; ++navAxis)
      {
        H[3 + navAxis] = rotation_[navAxis][measuredAxis];
      }

      // With right-multiplicative attitude error:
      // R_true^T v ~= R^T v + [v_B]x dtheta.
      float skewVelocityBody[3][3];
      skew(velocityBody, skewVelocityBody);
      for (int thetaAxis = 0; thetaAxis < 3; ++thetaAxis)
      {
        H[6 + thetaAxis] = skewVelocityBody[measuredAxis][thetaAxis];
      }

      const float measurement =
        measuredAxis == 0 ? bodyVelocityX : bodyVelocityY;
      const float standardDeviation =
        measuredAxis == 0 ? standardDeviationX : standardDeviationY;

      const bool accepted = scalarUpdate(
        H,
        measurement - velocityBody[measuredAxis],
        square(standardDeviation),
        gateSigma
      );
      acceptedAny = acceptedAny || accepted;
    }

    return acceptedAny;
  }

  bool updateGravityDirection(
    const float accelBody[3],
    float standardDeviationUnitVector,
    float gateSigma = 5.0f
  )
  {
    if (!finite3(accelBody) || !isfinite(standardDeviationUnitVector) ||
        standardDeviationUnitVector <= 0.0f)
    {
      return false;
    }

    const float magnitude = sqrtf(
      accelBody[0] * accelBody[0] +
      accelBody[1] * accelBody[1] +
      accelBody[2] * accelBody[2]
    );

    // Reject accelerometer tilt updates during strong translational motion.
    if (magnitude < 0.70f * params_.gravity ||
        magnitude > 1.30f * params_.gravity)
    {
      return false;
    }

    const float measuredDirection[3] = {
      accelBody[0] / magnitude,
      accelBody[1] / magnitude,
      accelBody[2] / magnitude
    };

    bool acceptedAny = false;

    // X and Y contain the useful near-level tilt information. Avoid a nearly
    // redundant Z update that can over-constrain the normalized vector.
    for (int measuredAxis = 0; measuredAxis < 2; ++measuredAxis)
    {
      // Predicted body-frame direction of navigation +Z.
      const float predictedDirection[3] = {
        rotation_[2][0],
        rotation_[2][1],
        rotation_[2][2]
      };

      float skewDirection[3][3];
      skew(predictedDirection, skewDirection);

      float H[STATE_DIM] = {};
      for (int thetaAxis = 0; thetaAxis < 3; ++thetaAxis)
      {
        H[6 + thetaAxis] = skewDirection[measuredAxis][thetaAxis];
      }

      const bool accepted = scalarUpdate(
        H,
        measuredDirection[measuredAxis] -
          predictedDirection[measuredAxis],
        square(standardDeviationUnitVector),
        gateSigma
      );
      acceptedAny = acceptedAny || accepted;
    }

    return acceptedAny;
  }

  bool updateZeroVelocity(
    float standardDeviationMetersPerSecond = 0.03f,
    float gateSigma = 8.0f
  )
  {
    bool acceptedAny = false;
    for (int axis = 0; axis < 3; ++axis)
    {
      float H[STATE_DIM] = {};
      H[3 + axis] = 1.0f;
      const bool accepted = scalarUpdate(
        H,
        -velocity_[axis],
        square(standardDeviationMetersPerSecond),
        gateSigma
      );
      acceptedAny = acceptedAny || accepted;
    }
    return acceptedAny;
  }

  void getPosition(float output[3]) const
  {
    memcpy(output, position_, sizeof(position_));
  }

  void getVelocityNavigation(float output[3]) const
  {
    memcpy(output, velocity_, sizeof(velocity_));
  }

  void getBodyVelocity(float output[3]) const
  {
    // v_B = R_NB^T v_N
    for (int bodyAxis = 0; bodyAxis < 3; ++bodyAxis)
    {
      output[bodyAxis] =
        rotation_[0][bodyAxis] * velocity_[0] +
        rotation_[1][bodyAxis] * velocity_[1] +
        rotation_[2][bodyAxis] * velocity_[2];
    }
  }

  void getRotationMatrix(float output[3][3]) const
  {
    memcpy(output, rotation_, sizeof(rotation_));
  }

  void getEulerRadians(
    float &pitchAboutXRad,
    float &rollAboutYRad,
    float &yawRightHandRad
  ) const
  {
    // Z-Y-X decomposition: R = Rz(yaw) * Ry(roll-about-Y) * Rx(pitch-about-X)
    const float sinRollY = clamp(-rotation_[2][0], -1.0f, 1.0f);
    rollAboutYRad = asinf(sinRollY);
    pitchAboutXRad = atan2f(rotation_[2][1], rotation_[2][2]);
    yawRightHandRad = atan2f(rotation_[1][0], rotation_[0][0]);
  }

  float positionX() const { return position_[0]; }
  float positionY() const { return position_[1]; }
  float positionZ() const { return position_[2]; }
  float velocityX() const { return velocity_[0]; }
  float velocityY() const { return velocity_[1]; }
  float velocityZ() const { return velocity_[2]; }

  float lastInnovationRatio() const { return lastInnovationRatio_; }
  uint32_t rejectedUpdates() const { return rejectedUpdates_; }
  uint32_t acceptedUpdates() const { return acceptedUpdates_; }
  bool healthy() const { return healthy_; }

  float covarianceDiagonal(int index) const
  {
    if (index < 0 || index >= STATE_DIM)
    {
      return NAN;
    }
    return covariance_[index][index];
  }

private:
  Params params_;
  float position_[3] = {};
  float velocity_[3] = {};
  float quaternion_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float rotation_[3][3] = {};
  float covariance_[STATE_DIM][STATE_DIM] = {};
  float lastInnovationRatio_ = 0.0f;
  uint32_t rejectedUpdates_ = 0;
  uint32_t acceptedUpdates_ = 0;
  bool healthy_ = true;

  static float square(float value)
  {
    return value * value;
  }

  static float clamp(float value, float minimum, float maximum)
  {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
  }

  static bool finite3(const float value[3])
  {
    return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
  }

  static void skew(const float vector[3], float output[3][3])
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

  static void quaternionMultiply(
    const float left[4],
    const float right[4],
    float output[4]
  )
  {
    output[0] =
      left[0] * right[0] - left[1] * right[1] -
      left[2] * right[2] - left[3] * right[3];
    output[1] =
      left[0] * right[1] + left[1] * right[0] +
      left[2] * right[3] - left[3] * right[2];
    output[2] =
      left[0] * right[2] - left[1] * right[3] +
      left[2] * right[0] + left[3] * right[1];
    output[3] =
      left[0] * right[3] + left[1] * right[2] -
      left[2] * right[1] + left[3] * right[0];
  }

  static void normalizeQuaternion(float quaternion[4])
  {
    const float norm = sqrtf(
      quaternion[0] * quaternion[0] +
      quaternion[1] * quaternion[1] +
      quaternion[2] * quaternion[2] +
      quaternion[3] * quaternion[3]
    );

    if (!isfinite(norm) || norm < 1.0e-8f)
    {
      quaternion[0] = 1.0f;
      quaternion[1] = 0.0f;
      quaternion[2] = 0.0f;
      quaternion[3] = 0.0f;
      return;
    }

    const float inverseNorm = 1.0f / norm;
    for (int i = 0; i < 4; ++i)
    {
      quaternion[i] *= inverseNorm;
    }
  }

  static void rotationVectorToQuaternion(
    float x,
    float y,
    float z,
    float output[4]
  )
  {
    const float angle = sqrtf(x * x + y * y + z * z);

    if (angle < 1.0e-6f)
    {
      output[0] = 1.0f;
      output[1] = 0.5f * x;
      output[2] = 0.5f * y;
      output[3] = 0.5f * z;
      normalizeQuaternion(output);
      return;
    }

    const float halfAngle = 0.5f * angle;
    const float scale = sinf(halfAngle) / angle;
    output[0] = cosf(halfAngle);
    output[1] = scale * x;
    output[2] = scale * y;
    output[3] = scale * z;
  }

  static void quaternionFromZyx(
    float yawZ,
    float rollY,
    float pitchX,
    float output[4]
  )
  {
    const float cy = cosf(0.5f * yawZ);
    const float sy = sinf(0.5f * yawZ);
    const float cr = cosf(0.5f * rollY);
    const float sr = sinf(0.5f * rollY);
    const float cp = cosf(0.5f * pitchX);
    const float sp = sinf(0.5f * pitchX);

    output[0] = cy * cr * cp + sy * sr * sp;
    output[1] = cy * cr * sp - sy * sr * cp;
    output[2] = cy * sr * cp + sy * cr * sp;
    output[3] = sy * cr * cp - cy * sr * sp;
  }

  void updateRotationMatrix()
  {
    const float w = quaternion_[0];
    const float x = quaternion_[1];
    const float y = quaternion_[2];
    const float z = quaternion_[3];

    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    rotation_[0][0] = 1.0f - 2.0f * (yy + zz);
    rotation_[0][1] = 2.0f * (xy - wz);
    rotation_[0][2] = 2.0f * (xz + wy);

    rotation_[1][0] = 2.0f * (xy + wz);
    rotation_[1][1] = 1.0f - 2.0f * (xx + zz);
    rotation_[1][2] = 2.0f * (yz - wx);

    rotation_[2][0] = 2.0f * (xz - wy);
    rotation_[2][1] = 2.0f * (yz + wx);
    rotation_[2][2] = 1.0f - 2.0f * (xx + yy);
  }

  void propagateCovariance(
    const float transition[STATE_DIM][STATE_DIM],
    float dt
  )
  {
    float temporary[STATE_DIM][STATE_DIM] = {};
    float propagated[STATE_DIM][STATE_DIM] = {};

    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        float sum = 0.0f;
        for (int k = 0; k < STATE_DIM; ++k)
        {
          sum += transition[row][k] * covariance_[k][col];
        }
        temporary[row][col] = sum;
      }
    }

    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        float sum = 0.0f;
        for (int k = 0; k < STATE_DIM; ++k)
        {
          sum += temporary[row][k] * transition[col][k];
        }
        propagated[row][col] = sum;
      }
    }

    memcpy(covariance_, propagated, sizeof(covariance_));

    // Diagonal process-noise approximation, intentionally matching the
    // lightweight structure of the reference ESP32 ESKF.
    const float dtSquared = dt * dt;
    covariance_[0][0] += square(params_.accelNoiseXY * dtSquared);
    covariance_[1][1] += square(params_.accelNoiseXY * dtSquared);
    covariance_[2][2] += square(params_.accelNoiseZ * dtSquared);
    covariance_[3][3] += square(params_.accelNoiseXY * dt);
    covariance_[4][4] += square(params_.accelNoiseXY * dt);
    covariance_[5][5] += square(params_.accelNoiseZ * dt);
    covariance_[6][6] += square(params_.gyroNoiseRollPitch * dt);
    covariance_[7][7] += square(params_.gyroNoiseRollPitch * dt);
    covariance_[8][8] += square(params_.gyroNoiseYaw * dt);
  }

  bool scalarUpdate(
    const float H[STATE_DIM],
    float residual,
    float measurementVariance,
    float gateSigma
  )
  {
    if (!isfinite(residual) || !isfinite(measurementVariance) ||
        measurementVariance <= 0.0f)
    {
      return false;
    }

    float pTimesHt[STATE_DIM] = {};
    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        pTimesHt[row] += covariance_[row][col] * H[col];
      }
    }

    float innovationVariance = measurementVariance;
    for (int i = 0; i < STATE_DIM; ++i)
    {
      innovationVariance += H[i] * pTimesHt[i];
    }

    if (!isfinite(innovationVariance) || innovationVariance < 1.0e-9f)
    {
      healthy_ = false;
      return false;
    }

    lastInnovationRatio_ =
      fabsf(residual) / sqrtf(innovationVariance);

    if (gateSigma > 0.0f && lastInnovationRatio_ > gateSigma)
    {
      ++rejectedUpdates_;
      return false;
    }

    float gain[STATE_DIM] = {};
    float correction[STATE_DIM] = {};
    for (int i = 0; i < STATE_DIM; ++i)
    {
      gain[i] = pTimesHt[i] / innovationVariance;
      correction[i] = gain[i] * residual;
    }

    // Joseph covariance update: (I-KH)P(I-KH)' + K R K'.
    float iMinusKh[STATE_DIM][STATE_DIM] = {};
    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        iMinusKh[row][col] =
          (row == col ? 1.0f : 0.0f) - gain[row] * H[col];
      }
    }

    float temporary[STATE_DIM][STATE_DIM] = {};
    float updated[STATE_DIM][STATE_DIM] = {};

    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        float sum = 0.0f;
        for (int k = 0; k < STATE_DIM; ++k)
        {
          sum += iMinusKh[row][k] * covariance_[k][col];
        }
        temporary[row][col] = sum;
      }
    }

    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        float sum = gain[row] * measurementVariance * gain[col];
        for (int k = 0; k < STATE_DIM; ++k)
        {
          sum += temporary[row][k] * iMinusKh[col][k];
        }
        updated[row][col] = sum;
      }
    }

    memcpy(covariance_, updated, sizeof(covariance_));
    injectAndReset(correction);
    ++acceptedUpdates_;
    sanitize();
    return healthy_;
  }

  void injectAndReset(const float correction[STATE_DIM])
  {
    for (int axis = 0; axis < 3; ++axis)
    {
      position_[axis] += correction[axis];
      velocity_[axis] += correction[axis + 3];
    }

    const float deltaTheta[3] = {
      correction[6], correction[7], correction[8]
    };

    float deltaQuaternion[4];
    rotationVectorToQuaternion(
      deltaTheta[0],
      deltaTheta[1],
      deltaTheta[2],
      deltaQuaternion
    );

    float correctedQuaternion[4];
    quaternionMultiply(quaternion_, deltaQuaternion, correctedQuaternion);
    memcpy(quaternion_, correctedQuaternion, sizeof(quaternion_));
    normalizeQuaternion(quaternion_);
    updateRotationMatrix();

    // Covariance reset Jacobian for right-multiplicative attitude error.
    float resetJacobian[STATE_DIM][STATE_DIM] = {};
    for (int i = 0; i < STATE_DIM; ++i)
    {
      resetJacobian[i][i] = 1.0f;
    }

    float skewDelta[3][3];
    skew(deltaTheta, skewDelta);
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        resetJacobian[row + 6][col + 6] -= 0.5f * skewDelta[row][col];
      }
    }

    float temporary[STATE_DIM][STATE_DIM] = {};
    float resetCovariance[STATE_DIM][STATE_DIM] = {};

    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        float sum = 0.0f;
        for (int k = 0; k < STATE_DIM; ++k)
        {
          sum += resetJacobian[row][k] * covariance_[k][col];
        }
        temporary[row][col] = sum;
      }
    }

    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = 0; col < STATE_DIM; ++col)
      {
        float sum = 0.0f;
        for (int k = 0; k < STATE_DIM; ++k)
        {
          sum += temporary[row][k] * resetJacobian[col][k];
        }
        resetCovariance[row][col] = sum;
      }
    }

    memcpy(covariance_, resetCovariance, sizeof(covariance_));
  }

  void sanitize()
  {
    healthy_ = true;

    for (int axis = 0; axis < 3; ++axis)
    {
      if (!isfinite(position_[axis]) || !isfinite(velocity_[axis]))
      {
        healthy_ = false;
      }
    }

    for (int row = 0; row < STATE_DIM; ++row)
    {
      for (int col = row; col < STATE_DIM; ++col)
      {
        float value = 0.5f *
          (covariance_[row][col] + covariance_[col][row]);

        if (!isfinite(value))
        {
          healthy_ = false;
          value = row == col ? params_.maxCovariance : 0.0f;
        }

        if (row == col)
        {
          value = clamp(
            value,
            params_.minCovariance,
            params_.maxCovariance
          );
        }
        else
        {
          value = clamp(
            value,
            -params_.maxCovariance,
            params_.maxCovariance
          );
        }

        covariance_[row][col] = value;
        covariance_[col][row] = value;
      }
    }

    const float quaternionNorm = sqrtf(
      quaternion_[0] * quaternion_[0] +
      quaternion_[1] * quaternion_[1] +
      quaternion_[2] * quaternion_[2] +
      quaternion_[3] * quaternion_[3]
    );

    if (!isfinite(quaternionNorm) || quaternionNorm < 0.5f ||
        quaternionNorm > 1.5f)
    {
      healthy_ = false;
    }
  }
};
