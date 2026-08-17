#include "pca9685/pca9685_driver.hpp"

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "pca9685/constants.h"

using namespace Pca9685Config;

namespace {

// レーザー側のWireとは別のESP32 I2Cコントローラを使用する。
TwoWire servoWire(1);

Adafruit_PWMServoDriver servoDriver(
  PCA9685_ADDRESS,
  servoWire
);

uint16_t lastPulseUs[PCA9685_CHANNEL_COUNT] = {};
bool outputActive[PCA9685_CHANNEL_COUNT] = {};
bool driverInitialized = false;

uint16_t microsecondsToPcaTicks(uint16_t pulseUs)
{
  const uint32_t periodUs =
    1000000UL / PCA9685_PWM_FREQ_HZ;

  uint32_t ticks =
    (static_cast<uint32_t>(pulseUs) * 4096UL) /
    periodUs;

  if (ticks > 4095UL) {
    ticks = 4095UL;
  }

  return static_cast<uint16_t>(ticks);
}

bool probePca9685()
{
  Serial.println(
    "[PCA9685] PCA9685 0x40をprobeします"
  );

  servoWire.beginTransmission(PCA9685_ADDRESS);

  const uint8_t result =
    servoWire.endTransmission(true);

  Serial.print(
    "[PCA9685] PCA9685 probe result="
  );
  Serial.println(result);

  return result == 0;
}

void disableAll()
{
  for (
    uint8_t channel = 0;
    channel < PCA9685_CHANNEL_COUNT;
    ++channel
  ) {
    servoDriver.setPWM(channel, 0, 4096);
    outputActive[channel] = false;
  }
}

bool initializeDriver()
{
  Serial.print(
    "[PCA9685] 専用I2C初期化 SDA="
  );
  Serial.print(SERVO_I2C_SDA_PIN);
  Serial.print(" SCL=");
  Serial.print(SERVO_I2C_SCL_PIN);
  Serial.print(" clock=");
  Serial.print(SERVO_I2C_CLOCK_HZ);
  Serial.println("Hz");

  if (!servoWire.begin(
        SERVO_I2C_SDA_PIN,
        SERVO_I2C_SCL_PIN,
        SERVO_I2C_CLOCK_HZ
      )) {
    Serial.println(
      "[PCA9685] ★ 専用I2C初期化失敗"
    );
    return false;
  }

  Serial.println(
    "[PCA9685] 専用I2C初期化成功"
  );

  if (!probePca9685()) {
    Serial.println(
      "[PCA9685] ★ PCA9685が見つかりません"
    );
    return false;
  }

  Serial.println(
    "[PCA9685] PCA9685検出成功"
  );

  Serial.println(
    "[PCA9685] servoDriver.begin()"
  );

  if (!servoDriver.begin()) {
    Serial.println(
      "[PCA9685] ★ PCA9685 begin失敗"
    );
    return false;
  }

  Serial.println(
    "[PCA9685] PWM周波数=50Hz設定"
  );

  servoDriver.setPWMFreq(
    PCA9685_PWM_FREQ_HZ
  );

  delay(10);

  return true;
}

}  // namespace

bool pca9685DriverBegin()
{
  Serial.println(
    "[PCA9685] 初期化開始"
  );

  if (!initializeDriver()) {
    Serial.println(
      "[PCA9685] ★ 初期化失敗"
    );
    return false;
  }

  disableAll();

  driverInitialized = true;

  Serial.println(
    "[PCA9685] PCA9685準備完了"
  );

  return true;
}

bool pca9685DriverSetPulseUs(
  uint8_t channel,
  uint16_t pulseUs
)
{
  if (!driverInitialized) {
    Serial.println(
      "[PCA9685] 未初期化"
    );
    return false;
  }

  if (channel >= PCA9685_CHANNEL_COUNT) {
    Serial.println(
      "[PCA9685] 不正なCH"
    );
    return false;
  }

  const uint16_t ticks =
    microsecondsToPcaTicks(pulseUs);

  const uint8_t result =
    servoDriver.setPWM(
      channel,
      0,
      ticks
    );

  Serial.print("[PCA9685] CH=");
  Serial.print(channel);
  Serial.print(" pulse=");
  Serial.print(pulseUs);
  Serial.print("us ticks=");
  Serial.print(ticks);
  Serial.print(" result=");
  Serial.println(result);

  if (result != 0) {
    return false;
  }

  lastPulseUs[channel] = pulseUs;
  outputActive[channel] = true;

  return true;
}

bool pca9685DriverDisable(uint8_t channel)
{
  if (
    !driverInitialized ||
    channel >= PCA9685_CHANNEL_COUNT
  ) {
    return false;
  }

  const uint8_t result =
    servoDriver.setPWM(channel, 0, 4096);

  if (result != 0) {
    return false;
  }

  outputActive[channel] = false;

  return true;
}

void pca9685DriverDisableAll()
{
  if (!driverInitialized) {
    return;
  }

  disableAll();
}

bool pca9685DriverReinitialize()
{
  // Laser Sensor用Busとは独立した専用I2C/PCA9685だけを再初期化する。

  if (!driverInitialized) {
    return true;
  }

  servoWire.end();
  delay(100);

  if (!initializeDriver()) {
    Serial.println(
      "[PCA9685] ★ PCA9685復旧失敗"
    );
    return false;
  }

  for (
    uint8_t channel = 0;
    channel < PCA9685_CHANNEL_COUNT;
    ++channel
  ) {
    if (outputActive[channel]) {
      servoDriver.setPWM(
        channel,
        0,
        microsecondsToPcaTicks(
          lastPulseUs[channel]
        )
      );
    } else {
      servoDriver.setPWM(
        channel,
        0,
        4096
      );
    }
  }

  Serial.println(
    "[PCA9685] PCA9685状態復旧完了"
  );

  return true;
}
