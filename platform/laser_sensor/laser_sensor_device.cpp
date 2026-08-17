#include "laser_sensor/laser_sensor_device.hpp"

#include "laser_sensor/constants.hpp"
#include "device/i2c_bus.hpp"
#include "laser_sensor/laser_sensor_state.hpp"

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

namespace laserSensorInternal {
namespace {
Adafruit_VL53L0X sensors[LASER_SENSOR_COUNT];
uint8_t consecutiveBusFaults = 0;
bool recoveryRequired = false;

enum class RangingStatusAction {
  USE_MEASUREMENT,     ///< 正常値としてStateへ保存する。
  IGNORE_MEASUREMENT,  ///< 一時的な範囲外として値だけ破棄する。
  COUNT_FAILURE        ///< 通信/API障害として連続エラーへ加算する。
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

bool selectTcaChannel(uint8_t channel) {
  if (channel >= LASER_SENSOR_TCA9548A_CHANNEL_COUNT) return false;
  if (!i2cBusWriteByteLocked(
        LASER_SENSOR_TCA9548A_ADDRESS,
        static_cast<uint8_t>(1U << channel)
      )) {
    return false;
  }
  delay(LASER_SENSOR_TCA_CHANNEL_SETTLE_MS);
  return true;
}

bool prepareSensorChannel(int index) {
  // 呼び出し側が保持するlockの内側でTCA channelを選択し、
  // VL53L0Xアクセスまでに別taskがchannelを変更できないようにする。
  if (selectTcaChannel(LASER_SENSOR_CHANNELS[index])) {
    return true;
  }

  Serial.print("[LASER] TCA9548Aのchannel選択に失敗しました sensor=");
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

  Serial.print("[LASER] VL53L0X接続確認 sensor=");
  Serial.print(LASER_SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(LASER_SENSOR_CHANNELS[index]);
  Serial.print(" address=0x");
  Serial.print(LASER_SENSOR_VL53L0X_ADDRESS, HEX);
  Serial.print(" 応答=");
  Serial.println(responding ? "あり" : "なし");

  return responding;
}

void printI2CTimeoutDiagnostic(int index) {
  Serial.print("[LASER] VL53L0X通信timeout sensor=");
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
      Serial.print("[LASER] VL53L0XのI2Cエラー sensor=");
      Serial.println(LASER_SENSOR_NAMES[index]);
      recordBusFault();
      return RangingStatusAction::COUNT_FAILURE;

    default:
      Serial.print("[LASER] VL53L0XのAPIエラー sensor=");
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

  Serial.print("[LASER] 連続エラーが停止閾値に達しました sensor=");
  Serial.println(LASER_SENSOR_NAMES[index]);
  disableSensor(index);
}
}  // namespace

bool initializeOneSensor(int index) {
  if (!sensorConfigured(index)) {
    return false;
  }

  Serial.println();
  Serial.print("[LASER] VL53L0Xを初期化します sensor=");
  Serial.println(LASER_SENSOR_NAMES[index]);

  I2cBusConnection connection;
  if (!connection.locked()) {
    disableSensor(index);
    return false;
  }

  if (!prepareSensorChannel(index)) {
    disableSensor(index);
    return false;
  }

  if (!probeSensor(index)) {
    Serial.print("[LASER] VL53L0Xから応答がありません sensor=");
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
    Serial.print("[LASER] VL53L0Xの初期化に失敗しました sensor=");
    Serial.println(LASER_SENSOR_NAMES[index]);
    disableSensor(index);
    return false;
  }

  markSensorAvailable(index);

  Serial.print("[LASER] VL53L0Xの初期化に成功しました sensor=");
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

  I2cBusConnection connection;
  if (!connection.locked()) {
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

    // Serial.print("[LASER][DEBUG] 距離 sensor=");
    // Serial.print(LASER_SENSOR_NAMES[index]);
    // Serial.print(" CH=");
    // Serial.print(LASER_SENSOR_CHANNELS[index]);
    // Serial.print(" raw=");
    // Serial.print(measurement.RangeMilliMeter);
    // Serial.print(" mm filtered=");
    // Serial.print(sensorDistanceMm(index));
    // Serial.println(" mm");
    return;
  }

  if (measurement.RangeStatus == 4) {
    Serial.print("[LASER] 測距値が有効範囲外です sensor=");
    Serial.println(LASER_SENSOR_NAMES[index]);
    invalidateSensorReading(index);
    return;
  }

  Serial.print("[LASER] 測距結果が無効です sensor=");
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
