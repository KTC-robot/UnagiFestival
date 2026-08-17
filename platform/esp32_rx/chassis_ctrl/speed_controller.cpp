#include "chassis_ctrl/speed_controller.hpp"

#include "chassis_ctrl/constants.h"

#include <algorithm>
#include <cmath>

SpeedControllerResult SpeedController::update(
  float targetRpm,
  float actualRpm,
  float dt
) {
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
  const bool allowIntegral =
    (!saturatedHigh && !saturatedLow) ||
    (saturatedHigh && error < 0.0f) ||
    (saturatedLow && error > 0.0f);

  if (allowIntegral) {
    integral_ = integralCandidate;
  }

  const float output = SPEED_KP * error + SPEED_KI * integral_;
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
