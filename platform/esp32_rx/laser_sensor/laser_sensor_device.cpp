#include "laser_sensor_device.hpp"

#include "constants.h"
#include "../i2c/i2c_bus.hpp"
#include "laser_sensor_state.hpp"

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

namespace laserSensorInternal {
namespace {
Adafruit_VL53L0X sensors[LASER_SENSOR_COUNT];

enum class RangingStatusAction {
  USE_MEASUREMENT,
  IGNORE_MEASUREMENT,
  COUNT_FAILURE
};

bool prepareSensorChannel(int index) {
  if (i2cBusSelectTcaChannel(LASER_SENSOR_CHANNELS[index])) {
    return true;
  }

  Serial.print("TCA9548A チャネル選択失敗: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.println(LASER_SENSOR_CHANNELS[index]);
  return false;
}

bool probeSensor(int index) {
  Wire.beginTransmission(LASER_SENSOR_VL53L0X_ADDRESS);
  const uint8_t result = Wire.endTransmission(true);

  Serial.print("VL53L0X 接続確認: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(LASER_SENSOR_CHANNELS[index]);
  Serial.print(" address=0x");
  Serial.print(LASER_SENSOR_VL53L0X_ADDRESS, HEX);
  Serial.print(" result=");
  Serial.println(result);

  return result == 0;
}

void printI2CTimeoutDiagnostic(int index) {
  Serial.print("VL53L0X TIMEOUT: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(LASER_SENSOR_CHANNELS[index]);
  Serial.print(" SDA=");
  Serial.print(i2cBusReadSda());
  Serial.print(" SCL=");
  Serial.println(i2cBusReadScl());
}

RangingStatusAction handleRangingStatus(
  int index,
  VL53L0X_Error status
) {
  switch (status) {
    case VL53L0X_ERROR_NONE:
      return RangingStatusAction::USE_MEASUREMENT;

    case VL53L0X_ERROR_RANGE_ERROR:
      return RangingStatusAction::IGNORE_MEASUREMENT;

    case VL53L0X_ERROR_TIME_OUT:
      printI2CTimeoutDiagnostic(index);
      return RangingStatusAction::COUNT_FAILURE;

    case VL53L0X_ERROR_CONTROL_INTERFACE:
      Serial.print("VL53L0X I2Cエラー: ");
      Serial.println(LASER_SENSOR_NAMES[index]);
      return RangingStatusAction::COUNT_FAILURE;

    default:
      Serial.print("VL53L0X APIエラー: ");
      Serial.print(LASER_SENSOR_NAMES[index]);
      Serial.print(" code=");
      Serial.println(static_cast<int>(status));
      return RangingStatusAction::COUNT_FAILURE;
  }
}

void countSensorFailure(int index) {
  incrementSensorErrorCount(index);

  if (sensorErrorCount(index) < LASER_SENSOR_MAX_ERROR_COUNT) {
    return;
  }

  Serial.print("VL53L0X エラー閾値到達: ");
  Serial.println(LASER_SENSOR_NAMES[index]);
  disableSensor(index);
}
}  // namespace

bool initializeOneSensor(int index) {
  if (!sensorConfigured(index)) {
    return false;
  }

  Serial.println();
  Serial.print("VL53L0X 初期化開始: ");
  Serial.println(LASER_SENSOR_NAMES[index]);

  if (!prepareSensorChannel(index)) {
    disableSensor(index);
    return false;
  }

  if (!probeSensor(index)) {
    Serial.print("VL53L0X 応答なし: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
    disableSensor(index);
    return false;
  }

  clearOneSensorState(index);

  const bool initialized = sensors[index].begin(
    LASER_SENSOR_VL53L0X_ADDRESS,
    false,
    &Wire
  );

  i2cBusApplySettings();

  if (!initialized) {
    Serial.print("VL53L0X 初期化失敗: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
    disableSensor(index);
    return false;
  }

  markSensorAvailable(index);

  Serial.print("VL53L0X 初期化成功: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(LASER_SENSOR_CHANNELS[index]);
  Serial.print(" address=0x");
  Serial.println(LASER_SENSOR_VL53L0X_ADDRESS, HEX);

  return true;
}

void readOneSensor(int index) {
  if (!sensorConfigured(index) || !sensorAvailable(index)) {
    return;
  }

  if (!prepareSensorChannel(index)) {
    invalidateSensorReading(index);
    countSensorFailure(index);
    return;
  }

  VL53L0X_RangingMeasurementData_t measurement{};

  const VL53L0X_Error status =
    sensors[index].rangingTest(&measurement, false);

  switch (handleRangingStatus(index, status)) {
    case RangingStatusAction::USE_MEASUREMENT:
      break;

    case RangingStatusAction::COUNT_FAILURE:
      invalidateSensorReading(index);
      countSensorFailure(index);
      return;

    case RangingStatusAction::IGNORE_MEASUREMENT:
    default:
      return;
  }

  if (measurement.RangeStatus == 0) {
    storeSensorReading(index, measurement.RangeMilliMeter);

    Serial.print("VL53L0X 距離: ");
    Serial.print(LASER_SENSOR_NAMES[index]);
    Serial.print(" CH=");
    Serial.print(LASER_SENSOR_CHANNELS[index]);
    Serial.print(" raw=");
    Serial.print(measurement.RangeMilliMeter);
    Serial.print(" mm filtered=");
    Serial.print(sensorDistanceMm(index));
    Serial.println(" mm");
    return;
  }

  if (measurement.RangeStatus == 4) {
    Serial.print("VL53L0X 測定範囲外: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
    return;
  }

  Serial.print("VL53L0X 測距無効: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" RangeStatus=");
  Serial.println(measurement.RangeStatus);
}

}  // namespace laserSensorInternal
