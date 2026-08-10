#include "step_air_ctrl.h"

#include "chassis_ctrl.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"
#include "step_air_config.h"

namespace {
StepAirState currentState = StepAirState::STARTUP;
StepAirState stateBeforeError = StepAirState::FLAT_NORMAL;

uint8_t transitionConfirmCount = 0;
uint8_t recoveryConfirmCount = 0;

uint32_t stateEnteredMs = 0;
uint32_t lastDebugPrintMs = 0;

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

void applyOutputsForState(StepAirState state) {
  switch (state) {
    case StepAirState::FLAT_NORMAL:
      // 通常時：ON → シリンダー伸長 → 機構DOWN
      relayCtrlSetFront(true);
      relayCtrlSetRear(true);
      break;

    case StepAirState::CLIMB_FRONT_UP:
      // 上り開始：前だけOFF → 前シリンダー収縮 → 前機構UP
      relayCtrlSetFront(false);
      relayCtrlSetRear(true);
      break;

    case StepAirState::TOP_BOTH_UP:
      // 段差上：前後ともOFF → 前後機構UP
      relayCtrlForceOff();
      break;

    case StepAirState::DESCEND_REAR_DOWN:
      // 下り開始：後だけON → 後シリンダー伸長 → 後機構DOWN
      relayCtrlSetFront(false);
      relayCtrlSetRear(true);
      break;

    case StepAirState::STARTUP:
    case StepAirState::SENSOR_ERROR:
    default:
      // センサー異常時は安全側：前後OFF → 機構UP
      relayCtrlForceOff();
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

bool allSensorsFresh() {
  // 自動段差制御には3台すべてが必要。
  if (!autoControlConfigured()) {
    return false;
  }

  return laserSensorCtrlReady();
}

bool confirmCondition(bool condition) {
  if (!condition) {
    transitionConfirmCount = 0;
    return false;
  }

  if (transitionConfirmCount < STEP_AIR_COUNTER_SATURATION_VALUE) {
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
    if (laserSensorCtrlReady()) {
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

  if (!laserSensorCtrlNewMeasurementSetReady()) {
    return;
  }

  if (
    currentState == StepAirState::STARTUP ||
    currentState == StepAirState::SENSOR_ERROR
  ) {
    if (recoveryConfirmCount < STEP_AIR_COUNTER_SATURATION_VALUE) {
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

  const int frontMm =
    laserSensorCtrlGetDistanceMm(LASER_SENSOR_FRONT);
  const int centerMm =
    laserSensorCtrlGetDistanceMm(LASER_SENSOR_CENTER);
  const int rearMm =
    laserSensorCtrlGetDistanceMm(LASER_SENSOR_REAR);

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
    stepAirCtrlSensorFresh(LASER_SENSOR_FRONT)
      ? String(stepAirCtrlGetDistanceMm(LASER_SENSOR_FRONT))
      : String("X")
  );

  Serial.print(" C=");
  Serial.print(
    stepAirCtrlSensorFresh(LASER_SENSOR_CENTER)
      ? String(stepAirCtrlGetDistanceMm(LASER_SENSOR_CENTER))
      : String("X")
  );

  Serial.print(" R=");
  Serial.print(
    stepAirCtrlSensorFresh(LASER_SENSOR_REAR)
      ? String(stepAirCtrlGetDistanceMm(LASER_SENSOR_REAR))
      : String("X")
  );

  Serial.print(" MODE=");
  Serial.print(
    autoControlConfigured()
      ? "AUTO_3SENSOR"
      : "MEASURE_TEST"
  );

  Serial.print(" CONNECT=");
  Serial.print(laserSensorCtrlConnectedCount());
  Serial.print("/");
  Serial.print(laserSensorCtrlConfiguredCount());

  Serial.print(" VX=");
  Serial.print(chassisCtrlGetLongitudinalCommand(), 2);

  Serial.print(" VF=");
  Serial.print(relayCtrlFrontOn() ? "ON" : "OFF");

  Serial.print(" VR=");
  Serial.print(relayCtrlRearOn() ? "ON" : "OFF");

  Serial.print(" STATE=");
  Serial.println(stepAirCtrlGetStateText());
}
}  // namespace

bool stepAirCtrlBegin() {
  relayCtrlBegin();
  currentState = StepAirState::STARTUP;
  stateBeforeError = StepAirState::FLAT_NORMAL;
  stateEnteredMs = millis();
  transitionConfirmCount = 0;
  recoveryConfirmCount = 0;

  applyOutputsForState(currentState);

  const bool allInitialized = laserSensorCtrlBegin();

  if (!allInitialized) {
    changeState(StepAirState::SENSOR_ERROR);
  }

  Serial.print("Step air controller: front valve GPIO");
  Serial.print(STEP_AIR_FRONT_VALVE_PIN);
  Serial.print(", rear valve GPIO");
  Serial.println(STEP_AIR_REAR_VALVE_PIN);

  return allInitialized;
}

void stepAirCtrlUpdate() {
  laserSensorCtrlUpdate();
  updateStateMachine();
  printDebugStatus();
}

bool stepAirCtrlSensorsReady() {
  return laserSensorCtrlReady();
}

bool stepAirCtrlSensorFresh(int sensorIndex) {
  return laserSensorCtrlFresh(sensorIndex);
}

int stepAirCtrlGetDistanceMm(int sensorIndex) {
  return laserSensorCtrlGetDistanceMm(sensorIndex);
}

bool stepAirCtrlFrontValveOn() {
  return relayCtrlFrontOn();
}

bool stepAirCtrlRearValveOn() {
  return relayCtrlRearOn();
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
