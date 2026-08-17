#include "step_assist/step_assist_logic.hpp"

#include "step_assist/constants.h"

StepAssistPhase evaluateStepAssistPhase(
  StepAssistPhase currentPhase,
  const StepAssistInput& input
) {
  switch (currentPhase) {
    case StepAssistPhase::NORMAL:
      if (
        input.frontFresh &&
        input.frontDistanceMm <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::FRONT_LOWERED;
      }
      break;

    case StepAssistPhase::FRONT_LOWERED:
      if (
        input.centerFresh &&
        input.centerDistanceMm <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::BOTH_LOWERED;
      }
      break;

    case StepAssistPhase::BOTH_LOWERED:
      if (
        input.rearFresh &&
        input.rearDistanceMm <= STEP_ASSIST_STEP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::REAR_SENSOR_LOWER;
      }
      break;

    case StepAssistPhase::REAR_SENSOR_LOWER:
      if (
        input.rearFresh &&
        input.phaseElapsedMs >= STEP_ASSIST_REAR_DROP_GRACE_MS &&
        input.rearDistanceMm >= STEP_ASSIST_DROP_DETECT_THRESHOLD_MM
      ) {
        return StepAssistPhase::REAR_RAISED;
      }
      break;

    case StepAssistPhase::REAR_RAISED:
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
