#include "chassis_ctrl/mecanum.hpp"

#include <algorithm>
#include <cmath>

using namespace CanConfig_chassis_ctrl;

MecanumComponents calculateMecanumComponents(float vx, float vy, float wz) {
  MecanumComponents components = {};

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    components.forward[wheelIndex] =
      static_cast<float>(FWD_SIGN[wheelIndex]) * vx;
    components.strafe[wheelIndex] =
      static_cast<float>(STR_SIGN[wheelIndex]) * vy;
    components.yaw[wheelIndex] =
      static_cast<float>(YAW_SIGN[wheelIndex]) * wz;
  }

  return components;
}

MecanumOutput combineAndNormalizeMecanum(
  const MecanumComponents& components
) {
  MecanumOutput output = {};
  float maxMagnitude = 0.0f;

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    output.wheel[wheelIndex] =
      components.forward[wheelIndex] +
      components.strafe[wheelIndex] +
      components.yaw[wheelIndex];
    maxMagnitude = std::max(
      maxMagnitude,
      std::fabs(output.wheel[wheelIndex])
    );
  }

  if (maxMagnitude > 1.0f) {
    for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
      output.wheel[wheelIndex] /= maxMagnitude;
    }
  }

  return output;
}
