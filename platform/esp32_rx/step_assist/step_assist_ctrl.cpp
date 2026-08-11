#include "step_assist/step_assist_ctrl.hpp"

#include "laser_sensor/constants.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"

#include <Arduino.h>

namespace {

enum class StepAssistPhase {
  NORMAL,
  FRONT_RAISED,
  BOTH_RAISED,
  REAR_LOWERED,
};

// 実機の取付高さと角度に合わせてここだけを調整する。
constexpr int STEP_DETECT_THRESHOLD_MM = 80;
constexpr int DROP_DETECT_THRESHOLD_MM = 120;

StepAssistPhase currentPhase = StepAssistPhase::NORMAL;

const char* phaseName(StepAssistPhase phase) {
  switch (phase) {
    case StepAssistPhase::NORMAL:
      return "NORMAL";
    case StepAssistPhase::FRONT_RAISED:
      return "FRONT_RAISED";
    case StepAssistPhase::BOTH_RAISED:
      return "BOTH_RAISED";
    case StepAssistPhase::REAR_LOWERED:
      return "REAR_LOWERED";
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
      setFrontRaised(false);
      setRearRaised(false);
      break;
    case StepAssistPhase::FRONT_RAISED:
    case StepAssistPhase::REAR_LOWERED:
      setFrontRaised(true);
      setRearRaised(false);
      break;
    case StepAssistPhase::BOTH_RAISED:
      setFrontRaised(true);
      setRearRaised(true);
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
        transitionTo(StepAssistPhase::FRONT_RAISED);
      }
      break;

    case StepAssistPhase::FRONT_RAISED:
      if (detectsStep(LASER_SENSOR_CENTER)) {
        transitionTo(StepAssistPhase::BOTH_RAISED);
      }
      break;

    case StepAssistPhase::BOTH_RAISED:
      if (detectsDrop(LASER_SENSOR_REAR)) {
        transitionTo(StepAssistPhase::REAR_LOWERED);
      }
      break;

    case StepAssistPhase::REAR_LOWERED:
      if (detectsDrop(LASER_SENSOR_CENTER)) {
        transitionTo(StepAssistPhase::NORMAL);
      }
      break;
  }
}
