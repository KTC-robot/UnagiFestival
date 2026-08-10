#include "i2c_bus.hpp"

#include "../step_air_config.h"

#include <Wire.h>

namespace {
bool writeTcaControlByte(uint8_t controlByte, const char* failureLabel) {
  Wire.beginTransmission(STEP_AIR_TCA9548A_ADDRESS);
  Wire.write(controlByte);

  const uint8_t wireError = Wire.endTransmission();

  if (wireError == STEP_AIR_I2C_WIRE_ERROR_NONE) {
    return true;
  }

  Serial.print(failureLabel);
  Serial.print(" WireError=");
  Serial.println(wireError);
  return false;
}

/**
 * @brief TCA9548Aへ全チャネル無効control byteを書き込む。
 *
 * 0-byte probeではなく実際の1 byte書き込みで存在確認も兼ねる。
 *
 * @return TCA9548Aが書き込みにACKした場合true。
 */
bool initializeMultiplexer() {
  if (!writeTcaControlByte(
        STEP_AIR_TCA9548A_ALL_CHANNELS_DISABLED,
        "TCA9548A init failed:"
      )) {
    Serial.print("TCA9548A not found: address=0x");
    Serial.println(STEP_AIR_TCA9548A_ADDRESS, HEX);
    return false;
  }

  Serial.print("TCA9548A ready: address=0x");
  Serial.println(STEP_AIR_TCA9548A_ADDRESS, HEX);
  return true;
}
}  // namespace

void i2cBusApplySettings() {
  Wire.setClock(STEP_AIR_I2C_CLOCK_HZ);
  Wire.setTimeOut(STEP_AIR_I2C_TIMEOUT_MS);
}

bool i2cBusRestart() {
  const bool endOk = Wire.end();
  Serial.print("Wire.end(): ");
  Serial.println(endOk ? "OK" : "FAILED");

  delay(50);

  const bool beginOk = Wire.begin(
    STEP_AIR_I2C_SDA_PIN,
    STEP_AIR_I2C_SCL_PIN,
    STEP_AIR_I2C_CLOCK_HZ
  );
  Serial.print("Wire.begin(): ");
  Serial.println(beginOk ? "OK" : "FAILED");

  if (!beginOk) {
    return false;
  }

  i2cBusApplySettings();
  return initializeMultiplexer();
}

bool i2cBusBegin() {
  return i2cBusRestart();
}

bool i2cBusSelectTcaChannel(uint8_t channel) {
  if (channel >= STEP_AIR_TCA9548A_CHANNEL_COUNT) {
    Serial.print("TCA9548A channel select failed: CH=");
    Serial.print(channel);
    Serial.println(" out of range");
    return false;
  }

  Wire.beginTransmission(STEP_AIR_TCA9548A_ADDRESS);
  Wire.write(static_cast<uint8_t>(1U << channel));

  const uint8_t wireError = Wire.endTransmission();

  if (wireError == STEP_AIR_I2C_WIRE_ERROR_NONE) {
    return true;
  }

  Serial.print("TCA9548A channel select failed: CH=");
  Serial.print(channel);
  Serial.print(" WireError=");
  Serial.println(wireError);
  return false;
}

int i2cBusReadSda() {
  return digitalRead(STEP_AIR_I2C_SDA_PIN);
}

int i2cBusReadScl() {
  return digitalRead(STEP_AIR_I2C_SCL_PIN);
}
