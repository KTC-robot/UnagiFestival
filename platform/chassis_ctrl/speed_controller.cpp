#include "chassis_ctrl/speed_controller.hpp"

#include "chassis_ctrl/constants.hpp"

#include <algorithm>
#include <cmath>

SpeedControllerResult SpeedController::update(
  float targetRpm,
  float actualRpm,
  float dt
) {
  // RPM差を電流指令へ変換する。P項は現在の誤差へ即座に反応し、
  // I項は残り続ける定常偏差を時間とともに補う。
  const float error = targetRpm - actualRpm;
  const float integralCandidate = std::clamp(
    integral_ + error * dt,
    -PID_INTEGRAL_LIMIT,
    PID_INTEGRAL_LIMIT
  );
  const float outputCandidate =
    SPEED_KP * error + SPEED_KI * integralCandidate;
  const bool saturatedHigh =
    outputCandidate >= static_cast<float>(MAX_CURRENT_COMMAND);
  const bool saturatedLow =
    outputCandidate <= -static_cast<float>(MAX_CURRENT_COMMAND);
  // 出力上限へ張り付いている間は、飽和を悪化させる向きの積分を止める。
  // 飽和から戻す向きの誤差は許可し、復帰を妨げない。
  const bool allowIntegral =
    (!saturatedHigh && !saturatedLow) ||
    (saturatedHigh && error < 0.0f) ||
    (saturatedLow && error > 0.0f);

  if (allowIntegral) {
    integral_ = integralCandidate;
  }

  const float output = SPEED_KP * error + SPEED_KI * integral_;
  // 最後にC620へ渡せる電流指令範囲へ制限する。
  const float limitedOutput = std::clamp(
    output,
    -static_cast<float>(MAX_CURRENT_COMMAND),
    static_cast<float>(MAX_CURRENT_COMMAND)
  );

  return {
    static_cast<int16_t>(std::lround(limitedOutput)),
    error,
    integral_,
    output
  };
}

void SpeedController::reset() {
  integral_ = 0.0f;
}
