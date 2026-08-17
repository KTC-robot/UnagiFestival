#include "device/pca9685_driver.hpp"

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr int SERVO_I2C_SDA_PIN = 26;
constexpr int SERVO_I2C_SCL_PIN = 27;
constexpr uint32_t SERVO_I2C_CLOCK_HZ = 100000;
constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint16_t PCA9685_PWM_FREQ_HZ = 50;
constexpr uint8_t PCA9685_CHANNEL_COUNT = 16;

// LaserSensor用Wireとは独立したServo専用I2C controllerを使用する。
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
  // 50Hzの1周期20msを4096分割したPCA9685 tick数へ変換する。
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
    "[PCA9685] address 0x40の応答を確認します"
  );

  servoWire.beginTransmission(PCA9685_ADDRESS);

  const uint8_t result =
    servoWire.endTransmission(true);

  Serial.print(
    "[PCA9685] 接続確認結果 error="
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
      "[PCA9685] Servo専用I2Cの初期化に失敗しました"
    );
    return false;
  }

  Serial.println(
    "[PCA9685] Servo専用I2Cの初期化が完了しました"
  );

  if (!probePca9685()) {
    Serial.println(
      "[PCA9685] deviceから応答がありません"
    );
    return false;
  }

  Serial.println(
    "[PCA9685] deviceの接続を確認しました"
  );

  Serial.println(
    "[PCA9685] device初期化を開始します"
  );

  if (!servoDriver.begin()) {
    Serial.println(
      "[PCA9685] device初期化に失敗しました"
    );
    return false;
  }

  Serial.println(
    "[PCA9685] PWM周波数を50Hzに設定します"
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
    "[PCA9685] 初期化を開始します"
  );

  if (!initializeDriver()) {
    Serial.println(
      "[PCA9685] 初期化に失敗しました"
    );
    return false;
  }

  disableAll();

  driverInitialized = true;

  Serial.println(
    "[PCA9685] 初期化が完了しました"
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
      "[PCA9685] 未初期化のためPWMを出力できません"
    );
    return false;
  }

  if (channel >= PCA9685_CHANNEL_COUNT) {
    Serial.println(
      "[PCA9685] channelが範囲外です"
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

  Serial.print("[PCA9685] PWM出力 channel=");
  Serial.print(channel);
  Serial.print(" pulse[us]=");
  Serial.print(pulseUs);
  Serial.print(" ticks=");
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
      "[PCA9685] 再初期化に失敗しました"
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
    "[PCA9685] 最後のPWM出力状態を復元しました"
  );

  return true;
}
