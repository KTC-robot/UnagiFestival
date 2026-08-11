#include "step_assist/step_assist_ctrl.hpp"

#include "laser_sensor/constants.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"

#include <Arduino.h>

namespace {

enum class StepAssistPhase {
  NORMAL,
  FRONT_LOWERED,
  BOTH_LOWERED,
  REAR_RAISED,
};

// 実機の取付高さと角度に合わせてここだけを調整する。
constexpr int STEP_DETECT_THRESHOLD_MM = 80;
constexpr int DROP_DETECT_THRESHOLD_MM = 120;

StepAssistPhase currentPhase = StepAssistPhase::NORMAL;

const char* phaseName(StepAssistPhase phase) {
  switch (phase) {
    case StepAssistPhase::NORMAL:
      return "NORMAL";
    case StepAssistPhase::FRONT_LOWERED:
      return "FRONT_LOWERED";
    case StepAssistPhase::BOTH_LOWERED:
      return "BOTH_LOWERED";
    case StepAssistPhase::REAR_RAISED:
      return "REAR_RAISED";
    default:
      return "UNKNOWN";
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

bool detectsStep(int sensorIndex) {
  return
    laserSensorCtrlFresh(sensorIndex) &&
    laserSensorCtrlGetDistanceMm(sensorIndex) <= STEP_DETECT_THRESHOLD_MM;
}

bool detectsDrop(int sensorIndex) {
  return
    laserSensorCtrlFresh(sensorIndex) &&
    laserSensorCtrlGetDistanceMm(sensorIndex) >= DROP_DETECT_THRESHOLD_MM;
}

void transitionTo(StepAssistPhase nextPhase) {
  Serial.print("STEP ASSIST: ");
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
  return true;
}

void stepAssistCtrlUpdate() {
  switch (currentPhase) {
    case StepAssistPhase::NORMAL:
      if (detectsStep(LASER_SENSOR_FRONT)) {
        transitionTo(StepAssistPhase::FRONT_LOWERED);
      }
      break;

    case StepAssistPhase::FRONT_LOWERED:
      if (detectsStep(LASER_SENSOR_CENTER)) {
        transitionTo(StepAssistPhase::BOTH_LOWERED);
      }
      break;

    case StepAssistPhase::BOTH_LOWERED:
      if (detectsDrop(LASER_SENSOR_REAR)) {
        transitionTo(StepAssistPhase::REAR_RAISED);
      }
      break;

    case StepAssistPhase::REAR_RAISED:
      if (detectsDrop(LASER_SENSOR_CENTER)) {
        transitionTo(StepAssistPhase::NORMAL);
      }
      break;
  }
}
