#include "chassis_ctrl/mecanum.hpp"

#include <algorithm>
#include <cmath>

using namespace CanConfig_chassis_ctrl;

MecanumComponents calculateMecanumComponents(float vx, float vy, float wz) {
  MecanumComponents components = {};

  // forwardは全輪を同方向、strafeとyawは車輪配置に応じた符号で加える。
  // 成分を分けたまま返すことで、呼び出し側が方向別wheel gainを適用できる。
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

  // 最大絶対値で全輪を同じ比率だけ縮小する。
  // これにより出力上限を守りながら、合成した移動方向の比率を保つ。
  if (maxMagnitude > 1.0f) {
    for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
      output.wheel[wheelIndex] /= maxMagnitude;
    }
  }

  return output;
}
