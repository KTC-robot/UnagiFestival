#pragma once

#include <cstdint>

/**
 * @brief 段差走破制御の進行状態。
 */
enum class StepAssistPhase {
  NORMAL,             ///< 通常走行状態。
  FRONT_LOWERED,      ///< 前補助輪を下げた状態。
  BOTH_LOWERED,       ///< 前後補助輪を下げた状態。
  REAR_SENSOR_LOWER,  ///< 後センサーが段差上にある状態。
  REAR_RAISED,        ///< 後補助輪を上げた状態。
};

/**
 * @brief StepAssistのphase遷移判断に使用する入力値。
 */
struct StepAssistInput {
  bool frontFresh;          ///< FRONTの距離が利用可能か。
  int frontDistanceMm;      ///< FRONTの距離[mm]。
  bool centerFresh;         ///< CENTERの距離が利用可能か。
  int centerDistanceMm;     ///< CENTERの距離[mm]。
  bool rearFresh;           ///< REARの距離が利用可能か。
  int rearDistanceMm;       ///< REARの距離[mm]。
  uint32_t phaseElapsedMs;  ///< 現phaseへ遷移してからの経過時間[ms]。
};

/**
 * @brief 現phaseとSensor入力から次に適用するphaseを判断する。
 *
 * hardware入出力や時刻取得は行わず、遷移しない場合はcurrentPhaseを返す。
 *
 * @param currentPhase 現在のStepAssist phase。
 * @param input Sensor状態とphase経過時間。
 * @return 遷移条件を満たす場合は次phase、それ以外はcurrentPhase。
 */
StepAssistPhase evaluateStepAssistPhase(
  StepAssistPhase currentPhase,
  const StepAssistInput& input
);
