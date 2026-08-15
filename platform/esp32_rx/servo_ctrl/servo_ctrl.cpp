#include "servo_ctrl.h"

#include "servo_ctrl/constants.h"
#include "servo_manager/servo_manager.hpp"

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

  // servo_ctrlは角度だけを解釈し、TCA/PCA9685への物理出力はmanagerへ委譲する。
  if (!servoManagerSetPulseUs(SERVO_PCA_CHANNEL[channel], pulseUs)) {
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
  servoManagerBegin();
}

bool servoCtrlRestoreAfterI2cRecovery() {
  return servoManagerRestoreAfterI2cRecovery();
}

void servoCtrlDisableAll() {
  servoManagerDisableAll();
}

void servoCtrlHandlePacket(const String& hex)
{
  Serial.print("[SERVO][CTRL] RAW=");
  Serial.println(hex);

  if (hex.length() < 6) {
    Serial.print("[SERVO][CTRL] INVALID LENGTH=");
    Serial.println(hex.length());
    return;
  }

  const uint8_t channel =
    static_cast<uint8_t>(
      strtoul(
        hex.substring(2, 4).c_str(),
        nullptr,
        16
      )
    );

  const uint8_t angle =
    static_cast<uint8_t>(
      strtoul(
        hex.substring(4, 6).c_str(),
        nullptr,
        16
      )
    );

  Serial.print("[SERVO][CTRL] PARSED CH=");
  Serial.print(channel);
  Serial.print(" ANGLE=");
  Serial.println(angle);

  if (channel >= SERVO_CHANNEL_COUNT) {
    Serial.print("[SERVO][CTRL] INVALID CHANNEL=");
    Serial.println(channel);
    return;
  }

  if (angle > 180) {
    Serial.print("[SERVO][CTRL] INVALID ANGLE=");
    Serial.println(angle);
    return;
  }

  Serial.println("[SERVO][CTRL] setServoAngle() START");

  if (!setServoAngle(channel, angle)) {
    Serial.println("[SERVO][CTRL] ★ setServoAngle FAILED");
    return;
  }

  Serial.println("[SERVO][CTRL] setServoAngle SUCCESS");
}

void servoCtrlHandleAllPacket(const String& hex) {
  if (hex.length() < 4) {
    Serial.print("[SERVO][ALL] INVALID LENGTH=");
    Serial.println(hex.length());
    return;
  }

  const uint8_t angle = static_cast<uint8_t>(
    strtoul(hex.substring(2, 4).c_str(), nullptr, 16)
  );

  if (angle > 180) {
    Serial.print("[SERVO][ALL] INVALID ANGLE=");
    Serial.println(angle);
    return;
  }

  Serial.print("[SERVO][ALL] ANGLE=");
  Serial.println(angle);

  // 個別packetと同じ角度制限・反転・pulse変換を全論理CHへ適用する。
  for (uint8_t channel = 0; channel < SERVO_CHANNEL_COUNT; ++channel) {
    Serial.print("[SERVO][ALL] CH=");
    Serial.print(channel);
    if (setServoAngle(channel, angle)) {
      Serial.println(" SUCCESS");
    } else {
      Serial.println(" FAILED");
    }
  }
}
