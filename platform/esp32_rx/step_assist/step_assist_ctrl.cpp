#include "step_assist/step_assist_ctrl.hpp"

#include "chassis_ctrl/chassis_ctrl.h"
#include "laser_sensor/constants.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"
#include "step_assist/constants.h"

#include <Arduino.h>

namespace
{

enum class StepAssistPhase
{
  NORMAL,
  FRONT_LOWERED,
  BOTH_LOWERED,
  REAR_SENSOR_LOWER,
  REAR_RAISED,
};

constexpr uint32_t STEP_ASSIST_DEBUG_LOG_INTERVAL_MS = 300;

StepAssistPhase currentPhase = StepAssistPhase::NORMAL;

uint32_t phaseEnteredMs = 0;
uint32_t lastDebugLogMs = 0;

const char *phaseName(StepAssistPhase phase)
{
  switch (phase)
  {
  case StepAssistPhase::NORMAL:
    return "通常";

  case StepAssistPhase::FRONT_LOWERED:
    return "前補助輪DOWN";

  case StepAssistPhase::BOTH_LOWERED:
    return "前後補助輪DOWN";

  case StepAssistPhase::REAR_SENSOR_LOWER:
    return "後センサー段差検出";

  case StepAssistPhase::REAR_RAISED:
    return "後補助輪UP";

  default:
    return "不明";
  }
}

bool shouldPrintDebugLog()
{
  const uint32_t now = millis();

  if (
      now - lastDebugLogMs <
      STEP_ASSIST_DEBUG_LOG_INTERVAL_MS)
  {
    return false;
  }

  lastDebugLogMs = now;
  return true;
}

void printDistanceDebug(
    const char *sensorName,
    const char *mode,
    int distance)
{
  Serial.print("[STEP][DEBUG] t=");
  Serial.print(millis());

  Serial.print(" phase=");
  Serial.print(phaseName(currentPhase));

  Serial.print(" sensor=");
  Serial.print(sensorName);

  Serial.print(" mode=");
  Serial.print(mode);

  Serial.print(" distance=");
  Serial.print(distance);

  Serial.println(" mm");
}

void setFrontRaised(bool raised)
{
  relayCtrlSetFront(!raised);
}

void setRearRaised(bool raised)
{
  relayCtrlSetRear(!raised);
}

void applyPhaseOutputs(StepAssistPhase phase)
{
  switch (phase)
  {
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
  case StepAssistPhase::REAR_SENSOR_LOWER:
    setFrontRaised(false);
    setRearRaised(false);
    break;
  }
}

/**
 * @brief 段差走破phaseに対応する車体全体の速度係数を適用する。
 *
 * 4輪間の差を補正するwheel gainとは分離し、phase遷移時の最大RPMだけを制限する。
 */
void applyPhaseDriveScale(StepAssistPhase phase)
{
  float forwardScale = STEP_ASSIST_NORMAL_FORWARD_SCALE;
  float backwardScale = STEP_ASSIST_NORMAL_BACKWARD_SCALE;
  float otherScale = STEP_ASSIST_NORMAL_OTHER_SCALE;

  switch (phase)
  {
  case StepAssistPhase::NORMAL:
    break;

  case StepAssistPhase::FRONT_LOWERED:
    forwardScale = STEP_ASSIST_FRONT_LOWERED_FORWARD_SCALE;
    backwardScale = STEP_ASSIST_FRONT_LOWERED_BACKWARD_SCALE;
    otherScale = STEP_ASSIST_FRONT_LOWERED_OTHER_SCALE;
    break;

  case StepAssistPhase::BOTH_LOWERED:
    forwardScale = STEP_ASSIST_BOTH_LOWERED_FORWARD_SCALE;
    backwardScale = STEP_ASSIST_BOTH_LOWERED_BACKWARD_SCALE;
    otherScale = STEP_ASSIST_BOTH_LOWERED_OTHER_SCALE;
    break;

  case StepAssistPhase::REAR_SENSOR_LOWER:
    forwardScale = STEP_ASSIST_REAR_SENSOR_LOWER_FORWARD_SCALE;
    backwardScale = STEP_ASSIST_REAR_SENSOR_LOWER_BACKWARD_SCALE;
    otherScale = STEP_ASSIST_REAR_SENSOR_LOWER_OTHER_SCALE;
    break;

  case StepAssistPhase::REAR_RAISED:
    forwardScale = STEP_ASSIST_REAR_RAISED_FORWARD_SCALE;
    backwardScale = STEP_ASSIST_REAR_RAISED_BACKWARD_SCALE;
    otherScale = STEP_ASSIST_REAR_RAISED_OTHER_SCALE;
    break;
  }

  chassisCtrlSetDriveScale(forwardScale, backwardScale, otherScale);
  Serial.print("[STEP][SCALE] FWD=");
  Serial.print(forwardScale, 2);
  Serial.print(" BWD=");
  Serial.print(backwardScale, 2);
  Serial.print(" OTHER=");
  Serial.println(otherScale, 2);
}

void transitionTo(StepAssistPhase nextPhase)
{
  const uint32_t now = millis();
  const uint32_t elapsedMs =
      now - phaseEnteredMs;

  Serial.print("[STEP] t=");
  Serial.print(now);

  Serial.print(" elapsed=");
  Serial.print(elapsedMs);

  Serial.print(" ms ");
  Serial.print(phaseName(currentPhase));

  Serial.print(" -> ");
  Serial.println(phaseName(nextPhase));

  currentPhase = nextPhase;
  phaseEnteredMs = now;

  applyPhaseOutputs(currentPhase);
  applyPhaseDriveScale(currentPhase);
}

} // namespace

bool stepAssistCtrlBegin()
{
  stepAssistCtrlReset();

  Serial.println(
      "段差制御 初期化完了: 前補助輪=UP 後補助輪=UP");

  return true;
}

void stepAssistCtrlReset()
{
  currentPhase = StepAssistPhase::NORMAL;

  phaseEnteredMs = millis();
  lastDebugLogMs = 0;

  applyPhaseOutputs(currentPhase);
  applyPhaseDriveScale(currentPhase);
}

void stepAssistCtrlUpdate()
{
  switch (currentPhase)
  {
  case StepAssistPhase::NORMAL:
    if (laserSensorCtrlFresh(LASER_SENSOR_FRONT))
    {
      const int distance =
          laserSensorCtrlGetDistanceMm(
              LASER_SENSOR_FRONT);

      if (shouldPrintDebugLog())
      {
        printDistanceDebug(
            "FRONT",
            "STEP_DETECT",
            distance);
      }

      if (
          distance <=
          STEP_ASSIST_STEP_DETECT_THRESHOLD_MM)
      {
        Serial.print(
            "[STEP][TRIGGER] FRONT step distance=");
        Serial.print(distance);
        Serial.println(" mm");

        transitionTo(
            StepAssistPhase::FRONT_LOWERED);
      }
    }
    else
    {
      if (shouldPrintDebugLog())
      {
        Serial.println(
            "[STEP][DEBUG] FRONT measurement invalid");
      }
    }

    break;

  case StepAssistPhase::FRONT_LOWERED:
    if (laserSensorCtrlFresh(LASER_SENSOR_CENTER))
    {
      const int distance =
          laserSensorCtrlGetDistanceMm(
              LASER_SENSOR_CENTER);

      if (shouldPrintDebugLog())
      {
        printDistanceDebug(
            "CENTER",
            "STEP_DETECT",
            distance);
      }

      if (
          distance <=
          STEP_ASSIST_STEP_DETECT_THRESHOLD_MM)
      {
        Serial.print(
            "[STEP][TRIGGER] CENTER step distance=");
        Serial.print(distance);
        Serial.println(" mm");

        transitionTo(
            StepAssistPhase::BOTH_LOWERED);
      }
    }
    else
    {
      if (shouldPrintDebugLog())
      {
        Serial.println(
            "[STEP][DEBUG] CENTER measurement invalid");
      }
    }

    break;

  case StepAssistPhase::BOTH_LOWERED:
    if (laserSensorCtrlFresh(LASER_SENSOR_REAR))
    {
      const int distance =
          laserSensorCtrlGetDistanceMm(
              LASER_SENSOR_REAR);

      if (shouldPrintDebugLog())
      {
        printDistanceDebug(
            "REAR",
            "STEP_DETECT",
            distance);
      }

      if (
          distance <=
          STEP_ASSIST_STEP_DETECT_THRESHOLD_MM)
      {
        Serial.print(
            "[STEP][TRIGGER] REAR step distance=");
        Serial.print(distance);
        Serial.println(" mm");

        transitionTo(
            StepAssistPhase::REAR_SENSOR_LOWER);
      }
    }
    else
    {
      if (shouldPrintDebugLog())
      {
        Serial.println(
            "[STEP][DEBUG] REAR measurement invalid");
      }
    }

    break;

  case StepAssistPhase::REAR_SENSOR_LOWER:
    if (laserSensorCtrlFresh(LASER_SENSOR_REAR))
    {
      const int distance =
          laserSensorCtrlGetDistanceMm(
              LASER_SENSOR_REAR);

      const uint32_t elapsedMs =
          millis() - phaseEnteredMs;

      if (shouldPrintDebugLog())
      {
        Serial.print("[STEP][DEBUG] t=");
        Serial.print(millis());

        Serial.print(" phase=");
        Serial.print(phaseName(currentPhase));

        Serial.print(" sensor=REAR");
        Serial.print(" mode=DROP_DETECT");

        Serial.print(" distance=");
        Serial.print(distance);

        Serial.print(" mm elapsed=");
        Serial.print(elapsedMs);

        Serial.print(" grace=");
        Serial.print(
            STEP_ASSIST_REAR_DROP_GRACE_MS);

        Serial.println(" ms");
      }

      if (
          elapsedMs <
          STEP_ASSIST_REAR_DROP_GRACE_MS)
      {
        break;
      }

      if (
          distance >=
          STEP_ASSIST_DROP_DETECT_THRESHOLD_MM)
      {
        Serial.print(
            "[STEP][TRIGGER] REAR drop distance=");
        Serial.print(distance);
        Serial.println(" mm");

        transitionTo(
            StepAssistPhase::REAR_RAISED);
      }
    }
    else
    {
      if (shouldPrintDebugLog())
      {
        Serial.println(
            "[STEP][DEBUG] REAR measurement invalid");
      }
    }

    break;

  case StepAssistPhase::REAR_RAISED:
    if (laserSensorCtrlFresh(LASER_SENSOR_FRONT))
    {
      const int distance =
          laserSensorCtrlGetDistanceMm(
              LASER_SENSOR_FRONT);

      if (shouldPrintDebugLog())
      {
        printDistanceDebug(
            "FRONT",
            "DROP_DETECT",
            distance);
      }

      if (
          distance >=
          STEP_ASSIST_DROP_DETECT_THRESHOLD_MM)
      {
        Serial.print(
            "[STEP][TRIGGER] FRONT drop distance=");
        Serial.print(distance);
        Serial.println(" mm");

        transitionTo(
            StepAssistPhase::NORMAL);
      }
    }
    else
    {
      if (shouldPrintDebugLog())
      {
        Serial.println(
            "[STEP][DEBUG] FRONT measurement invalid");
      }
    }

    break;
  }
}
