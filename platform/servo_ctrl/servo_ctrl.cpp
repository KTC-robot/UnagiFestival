#include "servo_ctrl/servo_ctrl.hpp"

#include <Arduino.h>

#include "servo_ctrl/constants.hpp"
#include "device/pca9685_driver.hpp"

using namespace CanConfig_servo_ctrl;

namespace {

bool setServoAngle(uint8_t channel, uint8_t requestedAngle) {
  if (channel >= SERVO_CHANNEL_COUNT) {
    return false;
  }

  const uint8_t minimum = SERVO_MIN_ANGLE[channel];
  const uint8_t maximum = SERVO_MAX_ANGLE[channel];

  if (minimum > maximum) {
    return false;
  }

  const uint8_t angle = constrain(requestedAngle, minimum, maximum);
  uint8_t physicalAngle = angle;

  if (SERVO_REVERSED[channel]) {
    physicalAngle = maximum - (angle - minimum);
  }

  uint16_t pulseUs = SERVO_MIN_US[channel];

  if (maximum != minimum) {
    pulseUs = map(
      physicalAngle,
      minimum,
      maximum,
      SERVO_MIN_US[channel],
      SERVO_MAX_US[channel]
    );
  }

  // servo_ctrlは角度だけを解釈し、PCA9685への物理出力はDriverへ委譲する。
  if (!pca9685DriverSetPulseUs(SERVO_PCA_CHANNEL[channel], pulseUs)) {
    return false;
  }

  Serial.print("SERVO CH=");
  Serial.print(channel);
  Serial.print(" PCA_CH=");
  Serial.print(SERVO_PCA_CHANNEL[channel]);
  Serial.print(" ANGLE=");
  Serial.print(angle);
  Serial.print(" PULSE_US=");
  Serial.println(pulseUs);

  return true;
}

}  // namespace

void servoCtrlBegin() {
  pca9685DriverBegin();
}

void servoCtrlDisableAll() {
  pca9685DriverDisableAll();
}

bool servoCtrlSetAngle(uint8_t channel, uint8_t angle) {
  if (angle > 180) return false;
  return setServoAngle(channel, angle);
}

bool servoCtrlSetAllAngles(uint8_t angle) {
  if (angle > 180) return false;

  Serial.print("[SERVO][ALL] ANGLE=");
  Serial.println(angle);

  // 個別packetと同じ角度制限・反転・pulse変換を全論理CHへ適用する。
  bool succeeded = true;
  for (uint8_t channel = 0; channel < SERVO_CHANNEL_COUNT; ++channel) {
    Serial.print("[SERVO][ALL] CH=");
    Serial.print(channel);
    if (setServoAngle(channel, angle)) {
      Serial.println(" SUCCESS");
    } else {
      Serial.println(" FAILED");
      succeeded = false;
    }
  }
  return succeeded;
}
