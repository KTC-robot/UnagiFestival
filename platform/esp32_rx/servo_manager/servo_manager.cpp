#include "servo_manager/servo_manager.hpp"

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

#include "i2c/i2c_bus.hpp"
#include "servo_manager/constants.h"

using namespace CanConfig_servo_manager;

namespace {

Adafruit_PWMServoDriver servoDriver(PCA9685_ADDRESS);

uint16_t lastPulseUs[PCA9685_CHANNEL_COUNT] = {};
bool outputActive[PCA9685_CHANNEL_COUNT] = {};
bool managerInitialized = false;

uint16_t microsecondsToPcaTicks(uint16_t pulseUs) {
  const uint32_t periodUs = 1000000UL / PCA9685_PWM_FREQ_HZ;
  uint32_t ticks = (static_cast<uint32_t>(pulseUs) * 4096UL) / periodUs;

  if (ticks > 4095UL) {
    ticks = 4095UL;
  }

  return static_cast<uint16_t>(ticks);
}

bool selectServoChannelLocked() {
  // 呼び出し元のlockを保持したままCH3を選択することで、PCA9685アクセスまでに
  // VL53L0X側taskがTCAチャネルを切り替えることを防ぐ。
  return i2cBusSelectTcaChannel(SERVO_TCA9548A_CHANNEL);
}

void disableAllLocked() {
  for (uint8_t channel = 0; channel < PCA9685_CHANNEL_COUNT; ++channel) {
    servoDriver.setPWM(channel, 0, 4096);
    outputActive[channel] = false;
  }
}

bool initializeDriverLocked() {
  if (!selectServoChannelLocked()) {
    return false;
  }

  if (!servoDriver.begin()) {
    return false;
  }

  servoDriver.setPWMFreq(PCA9685_PWM_FREQ_HZ);
  delay(10);
  return true;
}

}  // namespace

bool servoManagerBegin() {
  I2cBusLockGuard lock;
  if (!lock.locked() || !initializeDriverLocked()) {
    Serial.println("PCA9685 initialization failed on TCA CH3");
    return false;
  }

  disableAllLocked();
  managerInitialized = true;
  Serial.println("PCA9685 ready: TCA CH3, ADDRESS=0x40, CH0-CH15 FULL OFF");
  return true;
}

bool servoManagerSetPulseUs(uint8_t channel, uint16_t pulseUs) {
  if (!managerInitialized || channel >= PCA9685_CHANNEL_COUNT) {
    return false;
  }

  I2cBusLockGuard lock;
  if (!lock.locked() || !selectServoChannelLocked()) {
    return false;
  }

  servoDriver.setPWM(channel, 0, microsecondsToPcaTicks(pulseUs));
  lastPulseUs[channel] = pulseUs;
  outputActive[channel] = true;
  return true;
}

bool servoManagerDisable(uint8_t channel) {
  if (!managerInitialized || channel >= PCA9685_CHANNEL_COUNT) {
    return false;
  }

  I2cBusLockGuard lock;
  if (!lock.locked() || !selectServoChannelLocked()) {
    return false;
  }

  servoDriver.setPWM(channel, 0, 4096);
  outputActive[channel] = false;
  return true;
}

void servoManagerDisableAll() {
  if (!managerInitialized) {
    return;
  }

  I2cBusLockGuard lock;
  if (!lock.locked() || !selectServoChannelLocked()) {
    Serial.println("PCA9685 disable all failed");
    return;
  }

  disableAllLocked();
}

bool servoManagerRestoreAfterI2cRecovery() {
  if (!managerInitialized) {
    return true;
  }

  I2cBusLockGuard lock;
  if (!lock.locked() || !initializeDriverLocked()) {
    Serial.println("PCA9685 recovery failed on TCA CH3");
    return false;
  }

  // I2C bus再初期化で失われたPCA9685状態を、managerが保持する物理出力単位で復元する。
  for (uint8_t channel = 0; channel < PCA9685_CHANNEL_COUNT; ++channel) {
    if (outputActive[channel]) {
      servoDriver.setPWM(
        channel,
        0,
        microsecondsToPcaTicks(lastPulseUs[channel])
      );
    } else {
      servoDriver.setPWM(channel, 0, 4096);
    }
  }

  Serial.println("PCA9685 state restored after I2C recovery");
  return true;
}
