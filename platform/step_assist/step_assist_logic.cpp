#include "step_assist/step_assist_logic.hpp"

/**
 * @file step_assist_logic.cpp
 * @brief 現在状態と距離入力だけから次のStepAssist状態を判断する。
 */

#include "step_assist/constants.hpp"

StepAssistPhase evaluateStepAssistPhase(
  StepAssistPhase currentPhase,
  const StepAssistInput& input
) {
  switch (currentPhase) {
    case StepAssistPhase::NORMAL:
      // 前センサーが段差へ近づいたら、まず前補助輪を下げる。
      if (
        input.frontFresh &&
        input.frontDistanceMm <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::FRONT_LOWERED;
      }
      break;

    case StepAssistPhase::FRONT_LOWERED:
      // 中央センサーまで段差へ到達したら、後補助輪も下げる。
      if (
        input.centerFresh &&
        input.centerDistanceMm <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::BOTH_LOWERED;
      }
      break;

    case StepAssistPhase::BOTH_LOWERED:
      // 後センサーが段差へ乗った時点から、前進を制限するphaseへ進む。
      if (
        input.rearFresh &&
        input.rearDistanceMm <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::REAR_SENSOR_LOWER;
      }
      break;

    case StepAssistPhase::REAR_SENSOR_LOWER:
      // 乗り上げ直後の揺れをgrace時間で除外し、後センサーの下降を待つ。
      if (
        input.rearFresh &&
        input.phaseElapsedMs >= STEP_ASSIST_REAR_DROP_GRACE_MS &&
        input.rearDistanceMm >= STEP_ASSIST_DROP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::REAR_RAISED;
      }
      break;

    case StepAssistPhase::REAR_RAISED:
      // 前センサーが段差から離れたら一連の走破を完了しNORMALへ戻る。
      if (
        input.frontFresh &&
        input.frontDistanceMm >= STEP_ASSIST_DROP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::NORMAL;
      }
      break;
  }

  return currentPhase;
}
