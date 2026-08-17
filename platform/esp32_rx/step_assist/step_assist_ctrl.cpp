#include "step_assist/step_assist_ctrl.hpp"

#include "chassis_ctrl/chassis_ctrl.h"
#include "laser_sensor/constants.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"
#include "step_assist/constants.h"
#include "step_assist/step_assist_logic.hpp"

#include <Arduino.h>

namespace
{

constexpr uint32_t STEP_ASSIST_DEBUG_LOG_INTERVAL_MS = 300;

StepAssistPhase currentPhase = StepAssistPhase::NORMAL;

uint32_t phaseEnteredMs = 0;
uint32_t lastDebugLogMs = 0;
bool resetGuardActive = false;
uint32_t resetGuardStartedMs = 0;

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

void applyPhaseForwardBlock(StepAssistPhase phase)
{
  // 後センサーが土台上にある期間だけ、通常走行の前進成分を禁止する。
  chassisCtrlSetForwardBlocked(
      phase == StepAssistPhase::REAR_SENSOR_LOWER);
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
  applyPhaseForwardBlock(currentPhase);
}

/**
 * @brief LaserSensor Facadeから遷移判断用の現在値を取得する。
 *
 * 現phaseの遷移判定で使用するSensorだけを参照し、不要なSensor stateには
 * アクセスしない。距離を取得しないSensorはfresh=false、distance=-1のまま返す。
 *
 * @param now phase経過時間の計算に使用するmillis()時刻。
 * @return fresh状態、距離、phase経過時間をまとめた入力値。
 */
StepAssistInput readStepAssistInput(uint32_t now)
{
  StepAssistInput input = {};
  input.frontDistanceMm = -1;
  input.centerDistanceMm = -1;
  input.rearDistanceMm = -1;
  input.phaseElapsedMs = now - phaseEnteredMs;

  switch (currentPhase)
  {
  case StepAssistPhase::NORMAL:
  case StepAssistPhase::REAR_RAISED:
    input.frontFresh = laserSensorCtrlFresh(LASER_SENSOR_FRONT);
    if (input.frontFresh)
    {
      input.frontDistanceMm =
        laserSensorCtrlGetDistanceMm(LASER_SENSOR_FRONT);
    }
    break;

  case StepAssistPhase::FRONT_LOWERED:
    input.centerFresh = laserSensorCtrlFresh(LASER_SENSOR_CENTER);
    if (input.centerFresh)
    {
      input.centerDistanceMm =
        laserSensorCtrlGetDistanceMm(LASER_SENSOR_CENTER);
    }
    break;

  case StepAssistPhase::BOTH_LOWERED:
  case StepAssistPhase::REAR_SENSOR_LOWER:
    input.rearFresh = laserSensorCtrlFresh(LASER_SENSOR_REAR);
    if (input.rearFresh)
    {
      input.rearDistanceMm =
        laserSensorCtrlGetDistanceMm(LASER_SENSOR_REAR);
    }
    break;
  }

  return input;
}

/**
 * @brief 現phaseが使用するSensor値を一定周期でdebug出力する。
 *
 * @param input 現在のSensor状態とphase経過時間。
 */
void printCurrentPhaseDebug(const StepAssistInput& input)
{
  if (!shouldPrintDebugLog())
  {
    return;
  }

  switch (currentPhase)
  {
  case StepAssistPhase::NORMAL:
  case StepAssistPhase::REAR_RAISED:
    if (input.frontFresh)
    {
      printDistanceDebug(
        "FRONT",
        currentPhase == StepAssistPhase::NORMAL
          ? "STEP_DETECT"
          : "DROP_DETECT",
        input.frontDistanceMm
      );
    }
    else
    {
      Serial.println("[STEP][DEBUG] FRONT measurement invalid");
    }
    break;

  case StepAssistPhase::FRONT_LOWERED:
    if (input.centerFresh)
    {
      printDistanceDebug("CENTER", "STEP_DETECT", input.centerDistanceMm);
    }
    else
    {
      Serial.println("[STEP][DEBUG] CENTER measurement invalid");
    }
    break;

  case StepAssistPhase::BOTH_LOWERED:
    if (input.rearFresh)
    {
      printDistanceDebug("REAR", "STEP_DETECT", input.rearDistanceMm);
    }
    else
    {
      Serial.println("[STEP][DEBUG] REAR measurement invalid");
    }
    break;

  case StepAssistPhase::REAR_SENSOR_LOWER:
    if (!input.rearFresh)
    {
      Serial.println("[STEP][DEBUG] REAR measurement invalid");
      break;
    }

    Serial.print("[STEP][DEBUG] t=");
    Serial.print(millis());
    Serial.print(" phase=");
    Serial.print(phaseName(currentPhase));
    Serial.print(" sensor=REAR mode=DROP_DETECT distance=");
    Serial.print(input.rearDistanceMm);
    Serial.print(" mm elapsed=");
    Serial.print(input.phaseElapsedMs);
    Serial.print(" grace=");
    Serial.print(STEP_ASSIST_REAR_DROP_GRACE_MS);
    Serial.println(" ms");
    break;
  }
}

/**
 * @brief phase遷移を発生させたSensor値を出力する。
 *
 * @param phase 遷移前のphase。
 * @param input 遷移判断に使用したSensor入力。
 */
void printTransitionTrigger(
  StepAssistPhase phase,
  const StepAssistInput& input
)
{
  switch (phase)
  {
  case StepAssistPhase::NORMAL:
    Serial.print("[STEP][TRIGGER] FRONT step distance=");
    Serial.print(input.frontDistanceMm);
    break;

  case StepAssistPhase::FRONT_LOWERED:
    Serial.print("[STEP][TRIGGER] CENTER step distance=");
    Serial.print(input.centerDistanceMm);
    break;

  case StepAssistPhase::BOTH_LOWERED:
    Serial.print("[STEP][TRIGGER] REAR step distance=");
    Serial.print(input.rearDistanceMm);
    break;

  case StepAssistPhase::REAR_SENSOR_LOWER:
    Serial.print("[STEP][TRIGGER] REAR drop distance=");
    Serial.print(input.rearDistanceMm);
    break;

  case StepAssistPhase::REAR_RAISED:
    Serial.print("[STEP][TRIGGER] FRONT drop distance=");
    Serial.print(input.frontDistanceMm);
    break;
  }

  Serial.println(" mm");
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

  const uint32_t now = millis();
  phaseEnteredMs = now;
  lastDebugLogMs = 0;

  applyPhaseOutputs(currentPhase);
  applyPhaseDriveScale(currentPhase);
  applyPhaseForwardBlock(currentPhase);

  resetGuardActive = true;
  resetGuardStartedMs = now;
  Serial.print("[STEP][RESET_GUARD] START ");
  Serial.print(STEP_ASSIST_RESET_GUARD_MS);
  Serial.println("ms");
}

void stepAssistCtrlUpdate()
{
  // reset直後も他のloop処理と測距は継続し、phase遷移判定だけを停止する。
  if (resetGuardActive)
  {
    if (millis() - resetGuardStartedMs < STEP_ASSIST_RESET_GUARD_MS)
    {
      return;
    }

    resetGuardActive = false;
    Serial.println("[STEP][RESET_GUARD] END");
  }

  const uint32_t now = millis();
  const StepAssistInput input = readStepAssistInput(now);
  printCurrentPhaseDebug(input);

  const StepAssistPhase nextPhase =
    evaluateStepAssistPhase(currentPhase, input);
  if (nextPhase == currentPhase)
  {
    return;
  }

  printTransitionTrigger(currentPhase, input);
  transitionTo(nextPhase);
}
