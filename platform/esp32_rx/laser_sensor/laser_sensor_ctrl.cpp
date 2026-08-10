#include "laser_sensor_ctrl.hpp"

#include "../i2c/i2c_bus.hpp"
#include "../step_air_config.h"

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

namespace {
Adafruit_VL53L0X sensors[LASER_SENSOR_COUNT];

const bool SENSOR_ENABLED[LASER_SENSOR_COUNT] = {
  STEP_AIR_USE_FRONT_SENSOR,
  STEP_AIR_USE_CENTER_SENSOR,
  STEP_AIR_USE_REAR_SENSOR
};

const uint8_t SENSOR_CHANNELS[LASER_SENSOR_COUNT] = {
  STEP_AIR_FRONT_SENSOR_CHANNEL,
  STEP_AIR_CENTER_SENSOR_CHANNEL,
  STEP_AIR_REAR_SENSOR_CHANNEL
};

const int SENSOR_OFFSETS_MM[LASER_SENSOR_COUNT] = {
  STEP_AIR_FRONT_SENSOR_OFFSET_MM,
  STEP_AIR_CENTER_SENSOR_OFFSET_MM,
  STEP_AIR_REAR_SENSOR_OFFSET_MM
};

const char* SENSOR_NAMES[LASER_SENSOR_COUNT] = {
  "FRONT",
  "CENTER",
  "REAR"
};

bool sensorAvailable[LASER_SENSOR_COUNT] = {};
bool sensorHasValue[LASER_SENSOR_COUNT] = {};
int sensorDistanceMm[LASER_SENSOR_COUNT] = {};
uint32_t sensorLastGoodMs[LASER_SENSOR_COUNT] = {};
uint32_t sensorUpdateCount[LASER_SENSOR_COUNT] = {};
uint32_t lastEvaluatedUpdateCount[LASER_SENSOR_COUNT] = {};
uint8_t sensorErrorCount[LASER_SENSOR_COUNT] = {};

uint32_t lastInitializationAttemptMs = 0;
uint32_t lastSensorReadMs = 0;
int nextSensorToRead = 0;

enum class RangingStatusAction {
  USE_MEASUREMENT,
  IGNORE_MEASUREMENT,
  COUNT_FAILURE
};

bool sensorConfigured(int index) {
  return (
    index >= 0 &&
    index < LASER_SENSOR_COUNT &&
    SENSOR_ENABLED[index]
  );
}

void clearOneSensorReading(int index) {
  sensorHasValue[index] = false;
  sensorDistanceMm[index] = 0;
  sensorLastGoodMs[index] = 0;
  sensorUpdateCount[index] = 0;
  lastEvaluatedUpdateCount[index] = 0;
  sensorErrorCount[index] = 0;
}

void clearSensorReadings() {
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    clearOneSensorReading(index);
  }
}

void markAllSensorsUnavailable() {
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    sensorAvailable[index] = false;
  }
}

void disableSensor(int index) {
  sensorAvailable[index] = false;
  clearOneSensorReading(index);

  Serial.print("VL53L0X disabled: ");
  Serial.println(SENSOR_NAMES[index]);
}

bool prepareSensorChannel(int index) {
  if (i2cBusSelectTcaChannel(SENSOR_CHANNELS[index])) {
    return true;
  }

  Serial.print("TCA9548A CHANNEL ERROR: ");
  Serial.print(SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.println(SENSOR_CHANNELS[index]);
  return false;
}

/**
 * @brief TIMEOUT発生時のI2Cライン状態をSerialへ出力する。
 *
 * センサー名、TCA9548Aチャネル、SDA/SCLの現在レベルを同時に残し、
 * センサー下流、TCAチャネル、上流I2Cバスの切り分けに使う。
 *
 * @param index TIMEOUTが発生したセンサーインデックス。
 */
void printI2CTimeoutDiagnostic(int index) {
  Serial.print("VL53L0X TIMEOUT: ");
  Serial.print(SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(SENSOR_CHANNELS[index]);
  Serial.print(" SDA=");
  Serial.print(i2cBusReadSda());
  Serial.print(" SCL=");
  Serial.println(i2cBusReadScl());
}

/**
 * @brief rangingTest()のAPIエラーを復旧方針へ分類する。
 *
 * RANGE_ERRORは測距結果側の一時的な問題として握りつぶす。
 * TIME_OUT、CONTROL_INTERFACE、その他APIエラーは通信系の重大エラーとして扱い、
 * 連続回数によるセンサー単位の復旧対象にする。
 *
 * @param index エラーが発生したセンサーインデックス。
 * @param status rangingTest()が返したAPIステータス。
 * @return 測距結果の利用可否と重大エラー加算要否。
 */
RangingStatusAction handleRangingStatus(int index, VL53L0X_Error status) {
  switch (status) {
    case VL53L0X_ERROR_NONE:
      return RangingStatusAction::USE_MEASUREMENT;

    case VL53L0X_ERROR_RANGE_ERROR:
      return RangingStatusAction::IGNORE_MEASUREMENT;

    case VL53L0X_ERROR_TIME_OUT:
      printI2CTimeoutDiagnostic(index);
      return RangingStatusAction::COUNT_FAILURE;

    case VL53L0X_ERROR_CONTROL_INTERFACE:
      Serial.print("VL53L0X I2C ERROR: ");
      Serial.println(SENSOR_NAMES[index]);
      return RangingStatusAction::COUNT_FAILURE;

    default:
      Serial.print("VL53L0X API ERROR: ");
      Serial.print(SENSOR_NAMES[index]);
      Serial.print(" code=");
      Serial.println(static_cast<int>(status));
      return RangingStatusAction::COUNT_FAILURE;
  }
}

/**
 * @brief 重大なセンサーエラーを連続回数として記録する。
 *
 * 閾値到達時は対象センサーだけを無効化し、他の正常センサーの測距は継続する。
 * RANGE_ERRORとRangeStatus異常はここへ入れず、復旧を過剰に走らせない。
 *
 * @param index エラーを記録するセンサーインデックス。
 */
void countSensorFailure(int index) {
  if (sensorErrorCount[index] < STEP_AIR_SENSOR_MAX_ERROR_COUNT) {
    ++sensorErrorCount[index];
  }

  if (sensorErrorCount[index] < STEP_AIR_SENSOR_MAX_ERROR_COUNT) {
    return;
  }

  Serial.print("VL53L0X error threshold reached: ");
  Serial.println(SENSOR_NAMES[index]);
  disableSensor(index);
}

bool isCorrectedDistanceValid(int distanceMm) {
  return (
    distanceMm >= STEP_AIR_SENSOR_MIN_VALID_MM &&
    distanceMm <= STEP_AIR_SENSOR_MAX_VALID_MM
  );
}

void storeSensorReading(int index, uint16_t rawDistanceMm) {
  const int correctedDistance =
    static_cast<int>(rawDistanceMm) + SENSOR_OFFSETS_MM[index];

  if (!isCorrectedDistanceValid(correctedDistance)) {
    return;
  }

  if (!sensorHasValue[index]) {
    sensorDistanceMm[index] = correctedDistance;
    sensorHasValue[index] = true;
  } else {
    const int oldWeight =
      STEP_AIR_FILTER_WEIGHT_DENOMINATOR -
      STEP_AIR_FILTER_NEW_WEIGHT_NUMERATOR;

    sensorDistanceMm[index] =
      (
        sensorDistanceMm[index] * oldWeight +
        correctedDistance * STEP_AIR_FILTER_NEW_WEIGHT_NUMERATOR
      ) /
      STEP_AIR_FILTER_WEIGHT_DENOMINATOR;
  }

  sensorLastGoodMs[index] = millis();
  ++sensorUpdateCount[index];
  sensorErrorCount[index] = 0;
}

/**
 * @brief 対象センサーをTCA9548A経由で初期化する。
 *
 * VL53L0Xは全台0x29のまま使うため、begin()前に対象チャネルだけを選択する。
 *
 * @param index 初期化対象のセンサーインデックス。
 * @return 初期化に成功した場合true。
 */
bool initializeOneSensor(int index) {
  if (!sensorConfigured(index)) {
    sensorAvailable[index] = false;
    clearOneSensorReading(index);
    return false;
  }

  Serial.println();
  Serial.print("VL53L0X initialize: ");
  Serial.println(SENSOR_NAMES[index]);

  if (!prepareSensorChannel(index)) {
    sensorAvailable[index] = false;
    clearOneSensorReading(index);
    return false;
  }

  clearOneSensorReading(index);

  if (!sensors[index].begin(
        STEP_AIR_VL53L0X_ADDRESS,
        false,
        &Wire
      )) {
    Serial.print("VL53L0X init failed: ");
    Serial.println(SENSOR_NAMES[index]);

    sensorAvailable[index] = false;
    return false;
  }

  i2cBusApplySettings();
  sensorAvailable[index] = true;

  Serial.print("VL53L0X ready: ");
  Serial.print(SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(SENSOR_CHANNELS[index]);
  Serial.print(" address=0x");
  Serial.println(STEP_AIR_VL53L0X_ADDRESS, HEX);

  return true;
}

void initializeAllSensors() {
  lastInitializationAttemptMs = millis();

  clearSensorReadings();
  markAllSensorsUnavailable();

  if (!i2cBusBegin()) {
    return;
  }

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      initializeOneSensor(index);
    }
  }
}

void readOneSensor(int index) {
  if (!sensorConfigured(index) || !sensorAvailable[index]) {
    return;
  }

  if (!prepareSensorChannel(index)) {
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
      countSensorFailure(index);
      return;

    case RangingStatusAction::IGNORE_MEASUREMENT:
    default:
      return;
  }

  if (measurement.RangeStatus == 0) {
    storeSensorReading(index, measurement.RangeMilliMeter);
    return;
  }

  // API自体は成功しているため復旧カウントには入れず、次回測距へ任せる。
}

void readNextAvailableSensorIfDue() {
  const uint32_t now = millis();

  if (now - lastSensorReadMs < STEP_AIR_SENSOR_PERIOD_MS) {
    return;
  }

  lastSensorReadMs = now;

  for (int attempt = 0; attempt < LASER_SENSOR_COUNT; ++attempt) {
    const int index = nextSensorToRead;
    nextSensorToRead =
      (nextSensorToRead + 1) % LASER_SENSOR_COUNT;

    if (sensorConfigured(index) && sensorAvailable[index]) {
      readOneSensor(index);
      return;
    }
  }
}

void markStaleSensorsUnavailable() {
  const uint32_t now = millis();

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (
      !sensorConfigured(index) ||
      !sensorAvailable[index]
    ) {
      continue;
    }

    const uint32_t referenceMs =
      sensorHasValue[index]
        ? sensorLastGoodMs[index]
        : lastInitializationAttemptMs;

    if (
      now - referenceMs >
      STEP_AIR_SENSOR_REINIT_AFTER_MS
    ) {
      Serial.print("VL53L0X stale: ");
      Serial.println(SENSOR_NAMES[index]);

      disableSensor(index);
    }
  }
}

/**
 * @brief staleまたは重大エラーで停止したセンサーを周期的に再初期化する。
 *
 * 復旧時はWire全体を再生成し、TCA9548Aへ全チャネル無効の実書き込みを行ってから、
 * 対象チャネルを選択してVL53L0Xを再初期化する。
 * 共有I2Cバスへの影響があるため、設定した間隔より短く連打しない。
 */
void retryMissingSensorsIfDue() {
  const uint32_t now = millis();

  if (
    now - lastInitializationAttemptMs <
    STEP_AIR_SENSOR_REINIT_INTERVAL_MS
  ) {
    return;
  }

  bool missingEnabledSensor = false;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (
      sensorConfigured(index) &&
      !sensorAvailable[index]
    ) {
      missingEnabledSensor = true;
      break;
    }
  }

  if (!missingEnabledSensor) {
    return;
  }

  lastInitializationAttemptMs = now;

  Serial.println("VL53L0X retry: restarting I2C bus and TCA9548A");

  if (!i2cBusRestart()) {
    return;
  }

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (
      sensorConfigured(index) &&
      !sensorAvailable[index]
    ) {
      initializeOneSensor(index);
    }
  }
}
}  // namespace

bool laserSensorCtrlBegin() {
  initializeAllSensors();

  const int configuredCount = laserSensorCtrlConfiguredCount();
  const int connectedCount = laserSensorCtrlConnectedCount();

  Serial.print("VL53L0X connected: ");
  Serial.print(connectedCount);
  Serial.print(" / ");
  Serial.println(configuredCount);

  if (
    configuredCount > 0 &&
    connectedCount == configuredCount
  ) {
    Serial.println(
      STEP_AIR_ENABLE_AUTO_CONTROL
        ? "VL53L0X sensors initialized - AUTO CONTROL READY"
        : "VL53L0X measurement test ready - AUTO CONTROL DISABLED"
    );
    return true;
  }

  Serial.println(
    "STEP AIR: one or more enabled sensors failed to initialize"
  );
  return false;
}

void laserSensorCtrlUpdate() {
  readNextAvailableSensorIfDue();
  markStaleSensorsUnavailable();
  retryMissingSensorsIfDue();
}

bool laserSensorCtrlReady() {
  const uint32_t now = millis();

  bool anyConfigured = false;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index)) {
      continue;
    }

    anyConfigured = true;

    if (
      !sensorAvailable[index] ||
      !sensorHasValue[index] ||
      now - sensorLastGoodMs[index] >
        STEP_AIR_SENSOR_STALE_MS
    ) {
      return false;
    }
  }

  return anyConfigured;
}

bool laserSensorCtrlFresh(int sensorIndex) {
  if (sensorIndex < 0 || sensorIndex >= LASER_SENSOR_COUNT) {
    return false;
  }

  return (
    sensorConfigured(sensorIndex) &&
    sensorAvailable[sensorIndex] &&
    sensorHasValue[sensorIndex] &&
    millis() - sensorLastGoodMs[sensorIndex] <=
      STEP_AIR_SENSOR_STALE_MS
  );
}

int laserSensorCtrlGetDistanceMm(int sensorIndex) {
  if (!laserSensorCtrlFresh(sensorIndex)) {
    return -1;
  }

  return sensorDistanceMm[sensorIndex];
}

int laserSensorCtrlConnectedCount() {
  int count = 0;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (
      sensorConfigured(index) &&
      sensorAvailable[index]
    ) {
      ++count;
    }
  }

  return count;
}

int laserSensorCtrlConfiguredCount() {
  int count = 0;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      ++count;
    }
  }

  return count;
}

bool laserSensorCtrlNewMeasurementSetReady() {
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (
      !sensorConfigured(index) ||
      sensorUpdateCount[index] == 0 ||
      sensorUpdateCount[index] ==
        lastEvaluatedUpdateCount[index]
    ) {
      return false;
    }
  }

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      lastEvaluatedUpdateCount[index] =
        sensorUpdateCount[index];
    }
  }

  return true;
}
