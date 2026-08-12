#include "step_assist/step_assist_ctrl.hpp"

#include "laser_sensor/constants.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"
#include "step_assist/constants.h"

#include <Arduino.h>

namespace {

enum class StepAssistPhase {
  NORMAL,
  FRONT_LOWERED,
  BOTH_LOWERED,
  REAR_RAISED,
};

StepAssistPhase currentPhase = StepAssistPhase::NORMAL;

const char* phaseName(StepAssistPhase phase) {
  switch (phase) {
    case StepAssistPhase::NORMAL:
      return "通常";
    case StepAssistPhase::FRONT_LOWERED:
      return "前補助輪DOWN";
    case StepAssistPhase::BOTH_LOWERED:
      return "前後補助輪DOWN";
    case StepAssistPhase::REAR_RAISED:
      return "後補助輪UP";
    default:
      return "不明";
  }
}

void setFrontRaised(bool raised) {
  relayCtrlSetFront(!raised);
}

void setRearRaised(bool raised) {
  relayCtrlSetRear(!raised);
}

void applyPhaseOutputs(StepAssistPhase phase) {
  switch (phase) {
    case StepAssistPhase::NORMAL:
      setFrontRaised(true);
      setRearRaised(true);
      break;

    case StepAssistPhase::FRONT_LOWERED:
    case StepAssistPhase::REAR_RAISED:
      setFrontRaised(false);
      setRearRaised(true);
      break;

    case StepAssistPhase::BOTH_LOWERED:
      setFrontRaised(false);
      setRearRaised(false);
      break;
  }
}

void transitionTo(StepAssistPhase nextPhase) {
  Serial.print("段差制御 状態遷移: ");
  Serial.print(phaseName(currentPhase));
  Serial.print(" -> ");
  Serial.println(phaseName(nextPhase));

  currentPhase = nextPhase;
  applyPhaseOutputs(currentPhase);
}

}  // namespace

bool stepAssistCtrlBegin() {
  currentPhase = StepAssistPhase::NORMAL;
  applyPhaseOutputs(currentPhase);

  Serial.println(
    "段差制御 初期化完了: 前補助輪=UP 後補助輪=UP"
  );

  return true;
}

void stepAssistCtrlUpdate() {
  switch (currentPhase) {
    case StepAssistPhase::NORMAL:
      if (laserSensorCtrlFresh(LASER_SENSOR_FRONT)) {
        const int distance =
          laserSensorCtrlGetDistanceMm(LASER_SENSOR_FRONT);

        Serial.print("段差検出 FRONT 距離=");
        Serial.print(distance);
        Serial.println(" mm");

        if (
          distance <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
        ) {
          transitionTo(StepAssistPhase::FRONT_LOWERED);
        }
      } else {
        Serial.println(
          "段差検出 FRONT: 測距データ無効"
        );
      }
      break;

    case StepAssistPhase::FRONT_LOWERED:
      if (laserSensorCtrlFresh(LASER_SENSOR_CENTER)) {
        const int distance =
          laserSensorCtrlGetDistanceMm(LASER_SENSOR_CENTER);

        Serial.print("段差検出 CENTER 距離=");
        Serial.print(distance);
        Serial.println(" mm");

        if (
          distance <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
        ) {
          transitionTo(StepAssistPhase::BOTH_LOWERED);
        }
      } else {
        Serial.println(
          "段差検出 CENTER: 測距データ無効"
        );
      }
      break;

    case StepAssistPhase::BOTH_LOWERED:
      if (laserSensorCtrlFresh(LASER_SENSOR_REAR)) {
        const int distance =
          laserSensorCtrlGetDistanceMm(LASER_SENSOR_REAR);

        Serial.print("下降検出 REAR 距離=");
        Serial.print(distance);
        Serial.println(" mm");

        if (
          distance >= STEP_ASSIST_DROP_DETECT_THRESHOLD_MM
        ) {
          transitionTo(StepAssistPhase::REAR_RAISED);
        }
      } else {
        Serial.println(
          "下降検出 REAR: 測距データ無効"
        );
      }
      break;

    case StepAssistPhase::REAR_RAISED:
      if (laserSensorCtrlFresh(LASER_SENSOR_CENTER)) {
        const int distance =
          laserSensorCtrlGetDistanceMm(LASER_SENSOR_CENTER);

        Serial.print("下降検出 CENTER 距離=");
        Serial.print(distance);
        Serial.println(" mm");

        if (
          distance >= STEP_ASSIST_DROP_DETECT_THRESHOLD_MM
        ) {
          transitionTo(StepAssistPhase::NORMAL);
        }
      } else {
        Serial.println(
          "下降検出 CENTER: 測距データ無効"
        );
      }
      break;
  }
}