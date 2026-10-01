#include "Eskf9.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main()
{
  constexpr float gravity = 9.80665f;
  constexpr float pi = 3.14159265358979323846f;
  constexpr float angleRad = 10.0f * pi / 180.0f;

  // Existing controller convention, inferred from the validated mixer:
  //   controller roll  = -RH rotation about body +Y
  //   controller pitch = -RH rotation about body +X
  // A positive controller roll therefore gives +body-X gravity direction;
  // a positive controller pitch gives -body-Y gravity direction.
  for (int mode = 0; mode < 2; ++mode)
  {
    Eskf9 estimator;
    float accelBody[3] = {0.0f, 0.0f, gravity};
    float gyroBody[3] = {0.0f, 0.0f, 0.0f};

    if (mode == 0)
    {
      accelBody[0] = std::sin(angleRad) * gravity;
      accelBody[2] = std::cos(angleRad) * gravity;
    }
    else
    {
      accelBody[1] = -std::sin(angleRad) * gravity;
      accelBody[2] = std::cos(angleRad) * gravity;
    }

    for (int i = 0; i < 500; ++i)
    {
      estimator.predict(accelBody, gyroBody, 0.004f);
      estimator.updateGravityDirection(accelBody, 0.03f);
      estimator.updateZeroVelocity(0.05f);
    }

    float pitchRh = 0.0f;
    float rollRh = 0.0f;
    float yawRh = 0.0f;
    estimator.getEulerRadians(pitchRh, rollRh, yawRh);

    const float controllerRollDeg = -rollRh * 180.0f / pi;
    const float controllerPitchDeg = -pitchRh * 180.0f / pi;

    std::printf(
      "mode %d controller roll %.3f controller pitch %.3f\n",
      mode,
      static_cast<double>(controllerRollDeg),
      static_cast<double>(controllerPitchDeg)
    );

    if (mode == 0)
    {
      assert(std::fabs(controllerRollDeg - 10.0f) < 0.15f);
      assert(std::fabs(controllerPitchDeg) < 0.15f);
    }
    else
    {
      assert(std::fabs(controllerPitchDeg - 10.0f) < 0.15f);
      assert(std::fabs(controllerRollDeg) < 0.15f);
    }
  }

  std::puts("Controller/ESKF sign tests passed");
  return 0;
}
