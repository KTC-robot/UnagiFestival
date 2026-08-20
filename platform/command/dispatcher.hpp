#pragma once

/**
 * @file dispatcher.hpp
 * @brief 復号済みCommandを対応する機体Systemへ引き渡す。
 */

#include <stdint.h>

#include "command/command.hpp"

/** @brief Command実行後にIM920側が行う返信・状態更新の種類。 */
enum class CommandReply : uint8_t {
  NONE,         ///< 返信を送らない。
  CTRL_STOP,    ///< 通常停止完了を返信する。
  CTRL_ESTOP,   ///< 緊急停止完了を返信する。
  WHEEL_GAIN,   ///< 適用した車輪gainを返信する。
  TUNE_START,   ///< RPM計測試験の開始を返信する。
  STEP_RESET,   ///< StepAssistのreset完了を返信する。
};

/**
 * @brief Dispatcherの実行結果をIM920 Coordinatorへ返す。
 *
 * wire返信とGain Tuning送信stateはIM920側がこの結果から更新する。
 */
struct CommandDispatchResult {
  bool executed = false;              ///< SystemがCommandを受理したか。
  bool driveExecuted = false;         ///< DRIVE ACK計数対象か。
  bool resetGainTuningTx = false;     ///< 旧結果送信stateを破棄するか。
  bool gainTuningResultAck = false;   ///< remote ACKを受信したか。
  uint8_t gainTuningResultIndex = 0;  ///< ACK対象のWG/WD index。
  CommandReply reply = CommandReply::NONE;  ///< 返信種別。
  uint8_t wheelGainDirection = 0;     ///< WGS返信の方向index。
  uint8_t wheelGainIndex = 0;         ///< WGS返信の車輪index。
  float wheelGain = 1.0f;             ///< WGS返信の適用gain。
};

/**
 * @brief decode済みCommandを対応する制御・deviceへ渡す。
 *
 * @param command 実行する意味的なCommand。
 * @return IM920側で必要な返信・通信state更新情報。
 */
CommandDispatchResult dispatchCommand(const Command& command);
