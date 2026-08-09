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

// I2C速度と測距周期はstep_air_config.hで変更する。
constexpr uint32_t SENSOR_READ_INTERVAL_MS =
  STEP_AIR_SENSOR_PERIOD_MS;

Adafruit_VL53L0X sensors[SENSOR_COUNT];

const int XSHUT_PINS[SENSOR_COUNT] = {
  STEP_AIR_FRONT_XSHUT_PIN,
  STEP_AIR_CENTER_XSHUT_PIN,
  STEP_AIR_REAR_XSHUT_PIN
};

const uint8_t SENSOR_ADDRESSES[SENSOR_COUNT] = {
  STEP_AIR_FRONT_SENSOR_ADDRESS,
  STEP_AIR_CENTER_SENSOR_ADDRESS,
  STEP_AIR_REAR_SENSOR_ADDRESS
};


const bool SENSOR_ENABLED[SENSOR_COUNT] = {
  STEP_AIR_USE_FRONT_SENSOR,
  STEP_AIR_USE_CENTER_SENSOR,
  STEP_AIR_USE_REAR_SENSOR
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
}

void clearSensorReadings() {
  for (int index = 0; index < SENSOR_COUNT; ++index) {
    clearOneSensorReading(index);
  }

  recoveryConfirmCount = 0;
  transitionConfirmCount = 0;
}

void shutdownAllSensors() {
  for (int index = 0; index < SENSOR_COUNT; ++index) {
    pinMode(XSHUT_PINS[index], OUTPUT);
    digitalWrite(XSHUT_PINS[index], LOW);
    sensorAvailable[index] = false;
  }

  delay(50);
}

void disableSensor(int index) {
  sensorAvailable[index] = false;
  clearOneSensorReading(index);

  pinMode(XSHUT_PINS[index], OUTPUT);
  digitalWrite(XSHUT_PINS[index], LOW);

  Serial.print("VL53L0X disabled: ");
  Serial.println(SENSOR_NAMES[index]);
}

bool initializeOneSensor(int index) {
  if (!sensorConfigured(index)) {
    pinMode(XSHUT_PINS[index], OUTPUT);
    digitalWrite(XSHUT_PINS[index], LOW);
    sensorAvailable[index] = false;
    clearOneSensorReading(index);
    return false;
  }
  pinMode(XSHUT_PINS[index], OUTPUT);

  digitalWrite(XSHUT_PINS[index], LOW);
  delay(10);

  digitalWrite(XSHUT_PINS[index], HIGH);
  delay(50);

  clearOneSensorReading(index);

  // 単体テストで成功したAdafruit方式
  if (!sensors[index].begin(
        SENSOR_ADDRESSES[index],
        false,
        &Wire
      )) {
    Serial.print("VL53L0X init failed: ");
    Serial.println(SENSOR_NAMES[index]);

    digitalWrite(XSHUT_PINS[index], LOW);
    sensorAvailable[index] = false;
    return false;
  }

  sensorAvailable[index] = true;

  Serial.print("VL53L0X ready: ");
  Serial.print(SENSOR_NAMES[index]);
  Serial.print(" address=0x");
  Serial.println(SENSOR_ADDRESSES[index], HEX);

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
  shutdownAllSensors();

  for (int index = 0; index < SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      initializeOneSensor(index);
    } else {
      // 未接続設定のセンサーはXSHUT LOWのまま完全に無視する。
      pinMode(XSHUT_PINS[index], OUTPUT);
      digitalWrite(XSHUT_PINS[index], LOW);
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
        "Three VL53L0X sensors initialized - AUTO CONTROL READY"
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
}

void readOneSensor(int index) {
  if (!sensorConfigured(index) || !sensorAvailable[index]) {
    return;
  }

  VL53L0X_RangingMeasurementData_t measurement;

  // 単体テストと同じ読み方
  sensors[index].rangingTest(&measurement, false);

  if (measurement.RangeStatus == 0) {
    storeSensorReading(index, measurement.RangeMilliMeter);
  }
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

void recoverI2CBus() {
  // SDA/SCLの一時的なハングから復帰するため、再初期化時だけ
  // ESP32側I2Cドライバを作り直す。
  // PCA9685と共有していても、同一loop内で順番に実行されるため
  // 再初期化中の同時アクセスは発生しない。
  Wire.end();
  delay(20);

  Wire.begin(
    STEP_AIR_I2C_SDA_PIN,
    STEP_AIR_I2C_SCL_PIN
  );
  Wire.setClock(STEP_AIR_I2C_CLOCK_HZ);

  delay(50);
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

  Serial.println("VL53L0X retry: recovering I2C bus");
  recoverI2CBus();

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

  if (transitionConfirmCount < 255) {
    ++transitionConfirmCount;
  }

  return transitionConfirmCount >= STEP_AIR_CONFIRM_COUNT;
}

bool stateHeldLongEnough() {
  return millis() - stateEnteredMs >= STEP_AIR_STATE_MIN_HOLD_MS;
}

void updateStateMachine() {
  if (!autoControlConfigured()) {
    // 現在は測距テストモード。
    // 接続すると設定したセンサーだけ監視し、エア自動制御はしない。
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

  // 自動制御は3台すべて正常な場合だけ
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
    if (recoveryConfirmCount < 255) {
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

  // main.cppでWire.begin()済み。単体テストと同じ100kHzへ。
  Wire.setClock(STEP_AIR_I2C_CLOCK_HZ);

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