#include "MotionModelFF.h"

#include <cassert>
#include <cmath>
#include <cstdio>

int main()
{
  {
    MotionModelFeedforward model;
    MotionModelFeedforward::Output output =
      model.update(0.0f, 0.0f, 0.0f, 0.0f, 0.004f, 505.0f);
    assert(output.valid);

    output = model.update(5.0f, 0.0f, 0.0f, 0.0f, 0.004f, 505.0f);
    assert(output.valid);

    const float front =
      0.5f * (output.motorDeltaDshot[0] + output.motorDeltaDshot[1]);
    const float rear =
      0.5f * (output.motorDeltaDshot[2] + output.motorDeltaDshot[3]);

    std::printf(
      "pitch onset front=%.3f rear=%.3f rate=%.3f\n",
      static_cast<double>(front),
      static_cast<double>(rear),
      static_cast<double>(output.desiredPitchRateDps)
    );

    assert(rear > front);
    assert(output.desiredPitchRateDps > 0.0f);
  }

  {
    MotionModelFeedforward model;
    MotionModelFeedforward::Output output =
      model.update(0.0f, 0.0f, 0.0f, 0.0f, 0.004f, 505.0f);
    assert(output.valid);

    output = model.update(0.0f, 5.0f, 0.0f, 0.0f, 0.004f, 505.0f);
    assert(output.valid);

    const float left =
      0.5f * (output.motorDeltaDshot[0] + output.motorDeltaDshot[3]);
    const float right =
      0.5f * (output.motorDeltaDshot[1] + output.motorDeltaDshot[2]);

    std::printf(
      "roll onset left=%.3f right=%.3f rate=%.3f\n",
      static_cast<double>(left),
      static_cast<double>(right),
      static_cast<double>(output.desiredRollRateDps)
    );

    assert(right > left);
    assert(output.desiredRollRateDps > 0.0f);
  }

  {
    MotionModelFeedforward model;
    MotionModelFeedforward::Output output =
      model.update(0.0f, 0.0f, 0.0f, 0.0f, 0.004f, 505.0f);
    assert(output.valid);

    for (int i = 0; i < 600; ++i)
    {
      output = model.update(5.0f, 0.0f, 0.0f, 0.0f, 0.004f, 505.0f);
    }

    std::printf(
      "settled collective=%.3f differentials=[%.3f %.3f %.3f %.3f]\n",
      static_cast<double>(output.collectiveDeltaDshot),
      static_cast<double>(output.motorDeltaDshot[0]),
      static_cast<double>(output.motorDeltaDshot[1]),
      static_cast<double>(output.motorDeltaDshot[2]),
      static_cast<double>(output.motorDeltaDshot[3])
    );

    assert(output.valid);
    assert(output.collectiveDeltaDshot > 0.0f);
    for (int motor = 0; motor < 4; ++motor)
    {
      assert(std::fabs(output.motorDeltaDshot[motor]) < 0.1f);
    }
  }

  std::puts("Motion model tests passed");
  return 0;
}
