#pragma once

/**
 * @file command.hpp
 * @brief wire payloadから復号した、機体操作の意味を持つCommand型を定義する。
 */

#include <stdint.h>

/** @brief Decoderが復号した後の意味的なCommand種別。 */
enum class CommandType : uint8_t {
  STOP,                    ///< 通常停止を要求する。
  EMERGENCY_STOP,          ///< 緊急停止を要求する。
  CHANGE_POWER,            ///< 走行出力率の増減を要求する。
  DRIVE,                   ///< 前後・左右・旋回の走行を要求する。
  SET_WHEEL_GAIN,          ///< 方向別の車輪補正gain設定を要求する。
  GAIN_TUNE_START,         ///< 車輪RPM計測試験の開始を要求する。
  GAIN_TUNE_KEEPALIVE,     ///< 計測中の通信継続を通知する。
  GAIN_TUNE_RESULT_ACK,    ///< Piが計測結果を受信したことを通知する。
  STEP_ASSIST_RESET,       ///< 段差制御を通常状態へ戻す。
};

/** @brief 車体の前後・左右・旋回指令を保持する。 */
struct DriveCommand {
  int8_t vx = 0;  ///< 前後方向指令。
  int8_t vy = 0;  ///< 左右方向指令。
  int8_t wz = 0;  ///< 旋回方向指令。
};

/** @brief 方向別・車輪別の目標RPM補正gainを保持する。 */
struct SetWheelGainCommand {
  uint8_t direction = 0;   ///< 走行方向index。
  uint8_t wheelIndex = 0;  ///< 車輪index。
  float gain = 1.0f;       ///< 目標RPM補正係数。
};

/** @brief Gain Tuningの走行指令と計測時間を保持する。 */
struct GainTuneStartCommand {
  int8_t vx = 0;              ///< 前後方向指令。
  int8_t vy = 0;              ///< 左右方向指令。
  int8_t wz = 0;              ///< 旋回方向指令。
  uint32_t durationMs = 0;    ///< 計測時間（ms）。
};

/**
 * @brief wire payloadから復号したCommandとparameterを保持する。
 *
 * typeに対応するmemberだけをDispatcherが参照する。
 */
struct Command {
  CommandType type = CommandType::STOP;  ///< 実行するCommandの種類。
  int8_t powerDelta = 0;                 ///< 走行出力率へ加算する差分[%]。
  DriveCommand drive;                    ///< DRIVE用の車体指令。
  SetWheelGainCommand wheelGain;         ///< 車輪補正gainの設定値。
  GainTuneStartCommand gainTuneStart;    ///< RPM計測試験の条件。
  uint8_t gainTuneResultIndex = 0;       ///< ACK対象。0〜3=WG、4=WD。
};
