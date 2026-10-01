#include "Eskf9.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
constexpr float kGravity = 9.80665f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
}

int main()
{
  {
    Eskf9 estimator;
    float accelBody[3] = {0.0f, 0.0f, kGravity};
    float gyroBody[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 5000; ++i)
    {
      estimator.predict(accelBody, gyroBody, 0.004f);
      if ((i & 1) == 0)
      {
        estimator.updateGravityDirection(accelBody, 0.05f);
      }
      if ((i % 25) == 0)
      {
        estimator.updateZeroVelocity(0.03f);
      }
    }

    std::printf(
      "stationary p=(%.7f %.7f %.7f) v=(%.7f %.7f %.7f) healthy=%d\n",
      static_cast<double>(estimator.positionX()),
      static_cast<double>(estimator.positionY()),
      static_cast<double>(estimator.positionZ()),
      static_cast<double>(estimator.velocityX()),
      static_cast<double>(estimator.velocityY()),
      static_cast<double>(estimator.velocityZ()),
      estimator.healthy() ? 1 : 0
    );

    assert(estimator.healthy());
    assert(std::fabs(estimator.positionZ()) < 1.0e-4f);
    assert(std::fabs(estimator.velocityZ()) < 1.0e-4f);
  }

  {
    Eskf9 estimator;
    const float pitchX = 8.0f / kRadToDeg;

    // Start from a level estimate, then supply a gravity direction
    // corresponding to +8 degrees of right-handed body-X rotation.
    float accelBody[3] = {
      0.0f,
      std::sin(pitchX) * kGravity,
      std::cos(pitchX) * kGravity
    };
    float gyroBody[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 1000; ++i)
    {
      estimator.predict(accelBody, gyroBody, 0.004f);
      estimator.updateGravityDirection(accelBody, 0.03f);
      if ((i % 20) == 0)
      {
        estimator.updateZeroVelocity(0.05f);
      }
    }

    float pitchRh = 0.0f;
    float rollRh = 0.0f;
    float yawRh = 0.0f;
    estimator.getEulerRadians(pitchRh, rollRh, yawRh);

    std::printf(
      "tilt est pitchX=%.3f deg rollY=%.3f yaw=%.3f\n",
      static_cast<double>(pitchRh * kRadToDeg),
      static_cast<double>(rollRh * kRadToDeg),
      static_cast<double>(yawRh * kRadToDeg)
    );

    assert(std::fabs(pitchRh * kRadToDeg - 8.0f) < 0.6f);
  }

  {
    Eskf9 estimator;
    float accelBody[3] = {0.0f, 0.0f, kGravity};
    float gyroBody[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 500; ++i)
    {
      estimator.predict(accelBody, gyroBody, 0.004f);
      if ((i % 5) == 0)
      {
        estimator.updateBodyVelocity(0.50f, 0.0f, 0.05f, 0.05f);
      }
    }

    std::printf(
      "flow p=(%.3f %.3f) v=(%.3f %.3f)\n",
      static_cast<double>(estimator.positionX()),
      static_cast<double>(estimator.positionY()),
      static_cast<double>(estimator.velocityX()),
      static_cast<double>(estimator.velocityY())
    );

    assert(estimator.velocityX() > 0.45f);
    assert(estimator.velocityX() < 0.55f);
    assert(estimator.positionX() > 0.7f);
  }

  std::puts("ESKF tests passed");
  return 0;
}
