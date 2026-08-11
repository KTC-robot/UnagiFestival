#include "laser_sensor/laser_sensor_device.hpp"

#include "laser_sensor/constants.h"
#include "i2c/i2c_bus.hpp"
#include "laser_sensor/laser_sensor_state.hpp"

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

namespace laserSensorInternal {
namespace {
Adafruit_VL53L0X sensors[LASER_SENSOR_COUNT];
uint8_t consecutiveBusFaults = 0;
bool recoveryRequired = false;

enum class RangingStatusAction {
  USE_MEASUREMENT,
  IGNORE_MEASUREMENT,
  COUNT_FAILURE
};

void recordBusFault() {
  if (consecutiveBusFaults < LASER_SENSOR_BUS_FAULT_THRESHOLD) {
    ++consecutiveBusFaults;
  }
  recoveryRequired =
    consecutiveBusFaults >= LASER_SENSOR_BUS_FAULT_THRESHOLD;
}

void recordBusSuccess() {
  consecutiveBusFaults = 0;
}

bool prepareSensorChannel(int index) {
  if (i2cBusSelectTcaChannel(LASER_SENSOR_CHANNELS[index])) {
    return true;
  }

  Serial.print("TCA9548A チャネル選択失敗: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.println(LASER_SENSOR_CHANNELS[index]);
  recordBusFault();
  return false;
}

bool probeSensor(int index) {
  const bool responding = i2cBusProbeDevice(
    LASER_SENSOR_VL53L0X_ADDRESS
  );

  Serial.print("VL53L0X 接続確認: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(LASER_SENSOR_CHANNELS[index]);
  Serial.print(" address=0x");
  Serial.print(LASER_SENSOR_VL53L0X_ADDRESS, HEX);
  Serial.print(" result=");
  Serial.println(responding ? 0 : 1);

  return responding;
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
      recordBusSuccess();
      return RangingStatusAction::USE_MEASUREMENT;

    case VL53L0X_ERROR_RANGE_ERROR:
      return RangingStatusAction::IGNORE_MEASUREMENT;

    case VL53L0X_ERROR_TIME_OUT:
      printI2CTimeoutDiagnostic(index);
      recordBusFault();
      return RangingStatusAction::COUNT_FAILURE;

    case VL53L0X_ERROR_CONTROL_INTERFACE:
      Serial.print("VL53L0X I2Cエラー: ");
      Serial.println(LASER_SENSOR_NAMES[index]);
      recordBusFault();
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

  I2cBusLockGuard lock;
  if (!lock.locked()) {
    disableSensor(index);
    return false;
  }

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

  I2cBusLockGuard lock;
  if (!lock.locked()) {
    countSensorFailure(index);
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
      invalidateSensorReading(index);
      return;

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
    invalidateSensorReading(index);
    return;
  }

  Serial.print("VL53L0X 測距無効: ");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" RangeStatus=");
  Serial.println(measurement.RangeStatus);
  invalidateSensorReading(index);
}

bool busRecoveryRequired() {
  return recoveryRequired;
}

void clearBusRecoveryRequest() {
  consecutiveBusFaults = 0;
  recoveryRequired = false;
}

}  // namespace laserSensorInternal
