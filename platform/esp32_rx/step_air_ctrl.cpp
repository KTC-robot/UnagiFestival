#include "step_air_ctrl.h"

#include "chassis_ctrl.h"
#include "step_air_config.h"

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

namespace {
constexpr int SENSOR_COUNT = 3;
constexpr int FRONT_SENSOR = 0;
constexpr int CENTER_SENSOR = 1;
constexpr int REAR_SENSOR = 2;
constexpr uint8_t COUNTER_SATURATION_VALUE = 255;

// I2C速度と測距周期はstep_air_config.hで変更する。
constexpr uint32_t SENSOR_READ_INTERVAL_MS =
  STEP_AIR_SENSOR_PERIOD_MS;

Adafruit_VL53L0X sensors[SENSOR_COUNT];

const bool SENSOR_ENABLED[SENSOR_COUNT] = {
  STEP_AIR_USE_FRONT_SENSOR,
  STEP_AIR_USE_CENTER_SENSOR,
  STEP_AIR_USE_REAR_SENSOR
};

const uint8_t SENSOR_CHANNELS[SENSOR_COUNT] = {
  STEP_AIR_FRONT_SENSOR_CHANNEL,
  STEP_AIR_CENTER_SENSOR_CHANNEL,
  STEP_AIR_REAR_SENSOR_CHANNEL
};

const int SENSOR_OFFSETS_MM[SENSOR_COUNT] = {
  STEP_AIR_FRONT_SENSOR_OFFSET_MM,
  STEP_AIR_CENTER_SENSOR_OFFSET_MM,
  STEP_AIR_REAR_SENSOR_OFFSET_MM
};

const char* SENSOR_NAMES[SENSOR_COUNT] = {
  "FRONT",
  "CENTER",
  "REAR"
};

bool sensorAvailable[SENSOR_COUNT] = {};
bool sensorHasValue[SENSOR_COUNT] = {};
int sensorDistanceMm[SENSOR_COUNT] = {};
uint32_t sensorLastGoodMs[SENSOR_COUNT] = {};
uint32_t sensorUpdateCount[SENSOR_COUNT] = {};
uint32_t lastEvaluatedUpdateCount[SENSOR_COUNT] = {};
uint8_t sensorErrorCount[SENSOR_COUNT] = {};

bool frontValveOn = false;
bool rearValveOn = false;

StepAirState currentState = StepAirState::STARTUP;
StepAirState stateBeforeError = StepAirState::FLAT_NORMAL;

uint8_t transitionConfirmCount = 0;
uint8_t recoveryConfirmCount = 0;

uint32_t stateEnteredMs = 0;
uint32_t lastDebugPrintMs = 0;
uint32_t lastInitializationAttemptMs = 0;
uint32_t lastSensorReadMs = 0;
int nextSensorToRead = 0;

bool sensorConfigured(int index) {
  return (
    index >= 0 &&
    index < SENSOR_COUNT &&
    SENSOR_ENABLED[index]
  );
}

int configuredSensorCount() {
  int count = 0;

  for (int index = 0; index < SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      ++count;
    }
  }

  return count;
}

bool autoControlConfigured() {
  return (
    STEP_AIR_ENABLE_AUTO_CONTROL &&
    STEP_AIR_USE_FRONT_SENSOR &&
    STEP_AIR_USE_CENTER_SENSOR &&
    STEP_AIR_USE_REAR_SENSOR
  );
}

enum class RangingStatusAction {
  USE_MEASUREMENT,
  IGNORE_MEASUREMENT,
  COUNT_FAILURE
};

/**
 * @brief VL53L0X用I2C設定を現在の診断条件へ統一する。
 *
 * Adafruit_VL53L0X::begin() は内部でWire.begin()を呼ぶため、
 * begin()後にも再度この設定を適用する。
 */
void configureI2CBus() {
  Wire.setClock(STEP_AIR_I2C_CLOCK_HZ);
  Wire.setTimeOut(STEP_AIR_I2C_TIMEOUT_MS);
}

/**
 * @brief ESP32側I2CドライバをTCA9548A/VL53L0X用に作り直す。
 *
 * 実機で安定した単体コードと同じくWire.end()から開始する。
 * センサーの外部リセット端子による復旧やアドレス変更は行わない。
 * PCA9685を使う場合も同じ上流I2Cバスを共有するため、
 * クロックとtimeoutはここで一元管理する。
 *
 * @return I2Cドライバの初期化に成功した場合true。
 */
bool initializeI2CBus() {
  Wire.end();
  delay(50);

  if (!Wire.begin(
        STEP_AIR_I2C_SDA_PIN,
        STEP_AIR_I2C_SCL_PIN,
        STEP_AIR_I2C_CLOCK_HZ
      )) {
    Serial.println("[ERROR] I2C init failed");
    return false;
  }

  Wire.setTimeOut(STEP_AIR_I2C_TIMEOUT_MS);
  return true;
}

/**
 * @brief TCA9548Aが上流I2Cバス上に存在するか確認する。
 *
 * TCA9548Aが応答しない状態でVL53L0Xを初期化すると、
 * 0x29のセンサーへ到達できないため初期化を中止する。
 *
 * @return TCA9548AがACKを返した場合true。
 */
bool checkTca9548aReady() {
  Wire.beginTransmission(STEP_AIR_TCA9548A_ADDRESS);

  if (Wire.endTransmission() == 0) {
    Serial.print("TCA9548A ready: address=0x");
    Serial.println(STEP_AIR_TCA9548A_ADDRESS, HEX);
    return true;
  }

  Serial.print("TCA9548A not found: address=0x");
  Serial.println(STEP_AIR_TCA9548A_ADDRESS, HEX);
  return false;
}

/**
 * @brief TCA9548Aで使用するI2Cチャネルを選択する。
 *
 * 指定したチャネルだけをESP32側I2Cバスへ接続する。
 * 各VL53L0Xは同じI2Cアドレス0x29を使用するため、
 * センサーアクセス前に必ず対象チャネルを選択する必要がある。
 * 複数チャネルを同時に有効化すると0x29が衝突するため行わない。
 *
 * @param channel 選択するTCA9548Aチャネル番号。
 * @return チャネル選択に成功した場合true、失敗した場合false。
 */
bool selectSensorChannel(uint8_t channel) {
  if (channel >= STEP_AIR_TCA9548A_CHANNEL_COUNT) {
    return false;
  }

  Wire.beginTransmission(STEP_AIR_TCA9548A_ADDRESS);
  Wire.write(static_cast<uint8_t>(1U << channel));

  return Wire.endTransmission() == 0;
}

void printTcaChannelError(int index) {
  Serial.print("TCA9548A CHANNEL ERROR: ");
  Serial.print(SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.println(SENSOR_CHANNELS[index]);
}

/**
 * @brief TIMEOUT発生時のI2Cライン状態をSerialへ出力する。
 *
 * TCA9548Aチャネル、SDA/SCLがLOWに張り付いているか、
 * バス自体はidleなのかを実機で切り分ける。
 *
 * @param index TIMEOUTが発生したセンサーインデックス。
 */
void printI2CTimeoutDiagnostic(int index) {
  Serial.print("VL53L0X TIMEOUT: ");
  Serial.print(SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(SENSOR_CHANNELS[index]);
  Serial.print(" SDA=");
  Serial.print(digitalRead(STEP_AIR_I2C_SDA_PIN));
  Serial.print(" SCL=");
  Serial.println(digitalRead(STEP_AIR_I2C_SCL_PIN));
}

/**
 * @brief rangingTest()のAPIエラーを処理する。
 *
 * RANGE_ERRORは測距結果側の問題としてログを抑制し、
 * TIME_OUTとCONTROL_INTERFACEは連続エラー回数による復旧対象にする。
 *
 * @param index エラーが発生したセンサーインデックス。
 * @param status rangingTest()が返したAPIステータス。
 * @return 測距結果の扱いと復旧カウント要否。
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

int absoluteDifference(int a, int b) {
  const int difference = a - b;
  return difference >= 0 ? difference : -difference;
}

void setFrontValve(bool on) {
  if (frontValveOn == on) {
    return;
  }

  frontValveOn = on;
  digitalWrite(
    STEP_AIR_FRONT_VALVE_PIN,
    on ? STEP_AIR_VALVE_ON_LEVEL : STEP_AIR_VALVE_OFF_LEVEL
  );

  Serial.print("AIR FRONT VALVE -> ");
  Serial.println(on ? "ON (extend/down)" : "OFF (retract/up)");
}

void setRearValve(bool on) {
  if (rearValveOn == on) {
    return;
  }

  rearValveOn = on;
  digitalWrite(
    STEP_AIR_REAR_VALVE_PIN,
    on ? STEP_AIR_VALVE_ON_LEVEL : STEP_AIR_VALVE_OFF_LEVEL
  );

  Serial.print("AIR REAR VALVE -> ");
  Serial.println(on ? "ON (extend/down)" : "OFF (retract/up)");
}

void applyOutputsForState(StepAirState state) {
  switch (state) {
    case StepAirState::FLAT_NORMAL:
      // 通常時：ON → シリンダー伸長 → 機構DOWN
      setFrontValve(true);
      setRearValve(true);
      break;

    case StepAirState::CLIMB_FRONT_UP:
      // 上り開始：前だけOFF → 前シリンダー収縮 → 前機構UP
      setFrontValve(false);
      setRearValve(true);
      break;

    case StepAirState::TOP_BOTH_UP:
      // 段差上：前後ともOFF → 前後機構UP
      setFrontValve(false);
      setRearValve(false);
      break;

    case StepAirState::DESCEND_REAR_DOWN:
      // 下り開始：後だけON → 後シリンダー伸長 → 後機構DOWN
      setFrontValve(false);
      setRearValve(true);
      break;

    case StepAirState::STARTUP:
    case StepAirState::SENSOR_ERROR:
    default:
      // センサー異常時は安全側：前後OFF → 機構UP
      setFrontValve(false);
      setRearValve(false);
      break;
  }
}

void changeState(StepAirState nextState) {
  if (currentState == nextState) {
    return;
  }

  if (
    currentState != StepAirState::SENSOR_ERROR &&
    currentState != StepAirState::STARTUP
  ) {
    stateBeforeError = currentState;
  }

  currentState = nextState;
  transitionConfirmCount = 0;
  stateEnteredMs = millis();

  applyOutputsForState(currentState);

  Serial.print("STEP AIR STATE -> ");
  Serial.println(stepAirCtrlGetStateText());
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
  for (int index = 0; index < SENSOR_COUNT; ++index) {
    clearOneSensorReading(index);
  }

  recoveryConfirmCount = 0;
  transitionConfirmCount = 0;
}

void markAllSensorsUnavailable() {
  for (int index = 0; index < SENSOR_COUNT; ++index) {
    sensorAvailable[index] = false;
  }
}

void disableSensor(int index) {
  sensorAvailable[index] = false;
  clearOneSensorReading(index);

  Serial.print("VL53L0X disabled: ");
  Serial.println(SENSOR_NAMES[index]);
}

/**
 * @brief 重大なVL53L0X APIエラーを連続回数として記録する。
 *
 * TIMEOUT/I2C/その他APIエラーが短時間に連続する場合は、
 * stale時間を待たずに再初期化対象へ移す。
 * RANGE_ERRORとRangeStatus異常は測距不能として扱い、この回数には含めない。
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

bool prepareSensorChannel(int index) {
  if (selectSensorChannel(SENSOR_CHANNELS[index])) {
    return true;
  }

  printTcaChannelError(index);
  return false;
}

/**
 * @brief VL53L0Xを初期化する。
 *
 * 対象TCA9548Aチャネルを選択したうえで、
 * VL53L0Xをデフォルトアドレス0x29で初期化する。
 * 外部リセット端子によるハードウェアリセットやアドレス変更は行わない。
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

  applyOutputsForState(StepAirState::STARTUP);

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

  configureI2CBus();
  sensorAvailable[index] = true;

  Serial.print("VL53L0X ready: ");
  Serial.print(SENSOR_NAMES[index]);
  Serial.print(" CH=");
  Serial.print(SENSOR_CHANNELS[index]);
  Serial.print(" address=0x");
  Serial.println(STEP_AIR_VL53L0X_ADDRESS, HEX);

  return true;
}

int connectedSensorCount() {
  int count = 0;

  for (int index = 0; index < SENSOR_COUNT; ++index) {
    if (
      sensorConfigured(index) &&
      sensorAvailable[index]
    ) {
      ++count;
    }
  }

  return count;
}

bool initializeAllSensors() {
  lastInitializationAttemptMs = millis();

  clearSensorReadings();
  markAllSensorsUnavailable();

  if (!initializeI2CBus() || !checkTca9548aReady()) {
    changeState(StepAirState::SENSOR_ERROR);
    Serial.println("STEP AIR: TCA9548A initialization failed");
    return false;
  }

  for (int index = 0; index < SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      initializeOneSensor(index);
    }
  }

  const int configuredCount = configuredSensorCount();
  const int connectedCount = connectedSensorCount();

  Serial.print("VL53L0X connected: ");
  Serial.print(connectedCount);
  Serial.print(" / ");
  Serial.println(configuredCount);

  if (
    configuredCount > 0 &&
    connectedCount == configuredCount
  ) {
    // 測距テストモードではSTARTUPのまま。
    // バルブは安全側OFF、自動状態遷移はしない。
    currentState = StepAirState::STARTUP;
    applyOutputsForState(currentState);

    if (autoControlConfigured()) {
      stateBeforeError = StepAirState::FLAT_NORMAL;
      Serial.println(
        "VL53L0X sensors initialized - AUTO CONTROL READY"
      );
    } else {
      Serial.println(
        "VL53L0X measurement test ready - AUTO CONTROL DISABLED"
      );
    }

    return true;
  }

  changeState(StepAirState::SENSOR_ERROR);
  Serial.println(
    "STEP AIR: one or more enabled sensors failed to initialize"
  );

  return false;
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

  if (now - lastSensorReadMs < SENSOR_READ_INTERVAL_MS) {
    return;
  }

  lastSensorReadMs = now;

  for (int attempt = 0; attempt < SENSOR_COUNT; ++attempt) {
    const int index = nextSensorToRead;
    nextSensorToRead = (nextSensorToRead + 1) % SENSOR_COUNT;

    if (sensorConfigured(index) && sensorAvailable[index]) {
      readOneSensor(index);
      return;
    }
  }
}

bool allConfiguredSensorsFresh() {
  const uint32_t now = millis();

  bool anyConfigured = false;

  for (int index = 0; index < SENSOR_COUNT; ++index) {
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

bool allSensorsFresh() {
  // 自動段差制御には3台すべてが必要。
  if (!autoControlConfigured()) {
    return false;
  }

  return allConfiguredSensorsFresh();
}

void markStaleSensorsUnavailable() {
  const uint32_t now = millis();

  for (int index = 0; index < SENSOR_COUNT; ++index) {
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

      // 問題のセンサーだけ停止する。他のセンサーは動かし続ける。
      disableSensor(index);
    }
  }
}

void retryMissingSensorsIfDue() {
  const uint32_t now = millis();

  if (
    now - lastInitializationAttemptMs <
    STEP_AIR_SENSOR_REINIT_INTERVAL_MS
  ) {
    return;
  }

  bool missingEnabledSensor = false;

  for (int index = 0; index < SENSOR_COUNT; ++index) {
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

  Serial.println("VL53L0X retry: reinitializing sensor and I2C bus");

  for (int index = 0; index < SENSOR_COUNT; ++index) {
    if (
      sensorConfigured(index) &&
      !sensorAvailable[index]
    ) {
      initializeOneSensor(index);
    }
  }
}

bool newMeasurementSetReady() {
  if (!autoControlConfigured()) {
    return false;
  }

  for (int index = 0; index < SENSOR_COUNT; ++index) {
    if (
      sensorUpdateCount[index] == 0 ||
      sensorUpdateCount[index] ==
        lastEvaluatedUpdateCount[index]
    ) {
      return false;
    }
  }

  for (int index = 0; index < SENSOR_COUNT; ++index) {
    lastEvaluatedUpdateCount[index] =
      sensorUpdateCount[index];
  }

  return true;
}

bool confirmCondition(bool condition) {
  if (!condition) {
    transitionConfirmCount = 0;
    return false;
  }

  if (transitionConfirmCount < COUNTER_SATURATION_VALUE) {
    ++transitionConfirmCount;
  }

  return transitionConfirmCount >= STEP_AIR_CONFIRM_COUNT;
}

bool stateHeldLongEnough() {
  return millis() - stateEnteredMs >= STEP_AIR_STATE_MIN_HOLD_MS;
}

void updateStateMachine() {
  if (!autoControlConfigured()) {
    // 現在はTCA9548A経由の測距診断モード。
    // 自動バルブ制御は3台測距の実機安定確認後に別作業で有効化する。
    if (allConfiguredSensorsFresh()) {
      if (currentState != StepAirState::STARTUP) {
        changeState(StepAirState::STARTUP);
      } else {
        applyOutputsForState(StepAirState::STARTUP);
      }
    } else {
      changeState(StepAirState::SENSOR_ERROR);
    }

    return;
  }

  // 自動制御を有効化した場合も、3台すべて正常な場合だけ状態機械へ入る。
  if (!allSensorsFresh()) {
    recoveryConfirmCount = 0;

    if (
      currentState != StepAirState::SENSOR_ERROR &&
      currentState != StepAirState::STARTUP
    ) {
      stateBeforeError = currentState;
    }

    changeState(StepAirState::SENSOR_ERROR);
    return;
  }

  if (!newMeasurementSetReady()) {
    return;
  }

  if (
    currentState == StepAirState::STARTUP ||
    currentState == StepAirState::SENSOR_ERROR
  ) {
    if (recoveryConfirmCount < COUNTER_SATURATION_VALUE) {
      ++recoveryConfirmCount;
    }

    if (
      recoveryConfirmCount >=
      STEP_AIR_RECOVERY_CONFIRM_COUNT
    ) {
      StepAirState recoveredState = stateBeforeError;

      if (
        recoveredState == StepAirState::STARTUP ||
        recoveredState == StepAirState::SENSOR_ERROR
      ) {
        recoveredState = StepAirState::FLAT_NORMAL;
      }

      changeState(recoveredState);
    }

    return;
  }

  recoveryConfirmCount = 0;

  if (!stateHeldLongEnough()) {
    transitionConfirmCount = 0;
    return;
  }

  const int frontMm = sensorDistanceMm[FRONT_SENSOR];
  const int centerMm = sensorDistanceMm[CENTER_SENSOR];
  const int rearMm = sensorDistanceMm[REAR_SENSOR];

  const float longitudinalCommand =
    chassisCtrlGetLongitudinalCommand() *
    STEP_AIR_FORWARD_DIRECTION_SIGN;

  const bool movingForward =
    longitudinalCommand >= STEP_AIR_MOTION_DIRECTION_MIN;

  const bool movingBackward =
    longitudinalCommand <= -STEP_AIR_MOTION_DIRECTION_MIN;

  switch (currentState) {
    case StepAirState::FLAT_NORMAL: {
      const bool frontReachedUpperSurface =
        movingForward &&
        (centerMm - frontMm) >=
          STEP_AIR_CLIMB_FRONT_DETECT_DIFF_MM;

      if (confirmCondition(frontReachedUpperSurface)) {
        changeState(StepAirState::CLIMB_FRONT_UP);
      }
      break;
    }

    case StepAirState::CLIMB_FRONT_UP: {
      const bool frontAndCenterLevel =
        absoluteDifference(centerMm, frontMm) <=
          STEP_AIR_CLIMB_CENTER_LEVEL_DIFF_MM;

      if (movingForward) {
        if (confirmCondition(frontAndCenterLevel)) {
          changeState(StepAirState::TOP_BOTH_UP);
        }
      } else if (movingBackward) {
        if (confirmCondition(frontAndCenterLevel)) {
          changeState(StepAirState::FLAT_NORMAL);
        }
      } else {
        transitionConfirmCount = 0;
      }
      break;
    }

    case StepAirState::TOP_BOTH_UP: {
      const bool rearReachedDrop =
        movingBackward &&
        (rearMm - centerMm) >=
          STEP_AIR_DESCEND_REAR_DETECT_DIFF_MM;

      if (confirmCondition(rearReachedDrop)) {
        changeState(StepAirState::DESCEND_REAR_DOWN);
      }
      break;
    }

    case StepAirState::DESCEND_REAR_DOWN: {
      const bool rearAndCenterLevel =
        absoluteDifference(rearMm, centerMm) <=
          STEP_AIR_DESCEND_CENTER_LEVEL_DIFF_MM;

      if (movingBackward) {
        if (confirmCondition(rearAndCenterLevel)) {
          changeState(StepAirState::FLAT_NORMAL);
        }
      } else if (movingForward) {
        if (confirmCondition(rearAndCenterLevel)) {
          changeState(StepAirState::TOP_BOTH_UP);
        }
      } else {
        transitionConfirmCount = 0;
      }
      break;
    }

    case StepAirState::STARTUP:
    case StepAirState::SENSOR_ERROR:
    default:
      break;
  }
}

void printDebugStatus() {
  const uint32_t now = millis();

  if (
    now - lastDebugPrintMs <
    STEP_AIR_DEBUG_PRINT_INTERVAL_MS
  ) {
    return;
  }

  lastDebugPrintMs = now;

  Serial.print("STEP AIR F=");
  Serial.print(
    stepAirCtrlSensorFresh(FRONT_SENSOR)
      ? String(sensorDistanceMm[FRONT_SENSOR])
      : String("X")
  );

  Serial.print(" C=");
  Serial.print(
    stepAirCtrlSensorFresh(CENTER_SENSOR)
      ? String(sensorDistanceMm[CENTER_SENSOR])
      : String("X")
  );

  Serial.print(" R=");
  Serial.print(
    stepAirCtrlSensorFresh(REAR_SENSOR)
      ? String(sensorDistanceMm[REAR_SENSOR])
      : String("X")
  );

  Serial.print(" MODE=");
  Serial.print(
    autoControlConfigured()
      ? "AUTO_3SENSOR"
      : "MEASURE_TEST"
  );

  Serial.print(" CONNECT=");
  Serial.print(connectedSensorCount());
  Serial.print("/");
  Serial.print(configuredSensorCount());

  Serial.print(" VX=");
  Serial.print(chassisCtrlGetLongitudinalCommand(), 2);

  Serial.print(" VF=");
  Serial.print(frontValveOn ? "ON" : "OFF");

  Serial.print(" VR=");
  Serial.print(rearValveOn ? "ON" : "OFF");

  Serial.print(" STATE=");
  Serial.println(stepAirCtrlGetStateText());
}
}  // namespace

bool stepAirCtrlBegin() {
  pinMode(STEP_AIR_FRONT_VALVE_PIN, OUTPUT);
  pinMode(STEP_AIR_REAR_VALVE_PIN, OUTPUT);

  digitalWrite(
    STEP_AIR_FRONT_VALVE_PIN,
    STEP_AIR_VALVE_OFF_LEVEL
  );
  digitalWrite(
    STEP_AIR_REAR_VALVE_PIN,
    STEP_AIR_VALVE_OFF_LEVEL
  );

  frontValveOn = false;
  rearValveOn = false;

  currentState = StepAirState::STARTUP;
  stateBeforeError = StepAirState::FLAT_NORMAL;
  stateEnteredMs = millis();

  applyOutputsForState(currentState);

  const bool allInitialized = initializeAllSensors();

  Serial.print("Step air controller: front valve GPIO");
  Serial.print(STEP_AIR_FRONT_VALVE_PIN);
  Serial.print(", rear valve GPIO");
  Serial.println(STEP_AIR_REAR_VALVE_PIN);

  return allInitialized;
}

void stepAirCtrlUpdate() {
  readNextAvailableSensorIfDue();
  markStaleSensorsUnavailable();
  retryMissingSensorsIfDue();

  updateStateMachine();
  printDebugStatus();
}

bool stepAirCtrlSensorsReady() {
  return allConfiguredSensorsFresh();
}

bool stepAirCtrlSensorFresh(int sensorIndex) {
  if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT) {
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

int stepAirCtrlGetDistanceMm(int sensorIndex) {
  if (!stepAirCtrlSensorFresh(sensorIndex)) {
    return -1;
  }

  return sensorDistanceMm[sensorIndex];
}

bool stepAirCtrlFrontValveOn() {
  return frontValveOn;
}

bool stepAirCtrlRearValveOn() {
  return rearValveOn;
}

StepAirState stepAirCtrlGetState() {
  return currentState;
}

const char* stepAirCtrlGetStateText() {
  switch (currentState) {
    case StepAirState::STARTUP:
      return "STARTUP";
    case StepAirState::FLAT_NORMAL:
      return "FLAT_NORMAL";
    case StepAirState::CLIMB_FRONT_UP:
      return "CLIMB_FRONT_UP";
    case StepAirState::TOP_BOTH_UP:
      return "TOP_BOTH_UP";
    case StepAirState::DESCEND_REAR_DOWN:
      return "DESCEND_REAR_DOWN";
    case StepAirState::SENSOR_ERROR:
      return "SENSOR_ERROR";
  }

  return "UNKNOWN";
}

char stepAirCtrlGetStateCode() {
  switch (currentState) {
    case StepAirState::STARTUP: return 'I';
    case StepAirState::FLAT_NORMAL: return 'N';
    case StepAirState::CLIMB_FRONT_UP: return 'F';
    case StepAirState::TOP_BOTH_UP: return 'T';
    case StepAirState::DESCEND_REAR_DOWN: return 'R';
    case StepAirState::SENSOR_ERROR: return 'E';
  }

  return '?';
}

void stepAirCtrlForceSafe() {
  if (
    currentState != StepAirState::STARTUP &&
    currentState != StepAirState::SENSOR_ERROR
  ) {
    stateBeforeError = currentState;
  }

  changeState(StepAirState::SENSOR_ERROR);
}
