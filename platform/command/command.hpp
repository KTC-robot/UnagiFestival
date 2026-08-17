#pragma once

#include <stdint.h>

/** @brief decode後の意味的なCommand種別。 */
enum class CommandType : uint8_t {
  STOP,
  EMERGENCY_STOP,
  CHANGE_POWER,
  DRIVE,
  SET_WHEEL_GAIN,
  GAIN_TUNE_START,
  GAIN_TUNE_KEEPALIVE,
  GAIN_TUNE_RESULT_ACK,
  STEP_ASSIST_RESET,
  SERVO_SET,
  SERVO_SET_ALL,
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

/** @brief 論理サーボ番号と指定角度を保持する。 */
struct ServoSetCommand {
  uint8_t channel = 0;  ///< 論理サーボ番号。
  uint8_t angle = 0;    ///< 指定角度。
};

/**
 * @brief wire payloadから復号したCommandとparameterを保持する。
 *
 * typeに対応するmemberだけをDispatcherが参照する。
 */
struct Command {
  CommandType type = CommandType::STOP;
  int8_t powerDelta = 0;
  DriveCommand drive;
  SetWheelGainCommand wheelGain;
  GainTuneStartCommand gainTuneStart;
  uint8_t gainTuneResultIndex = 0;
  ServoSetCommand servo;
  uint8_t servoAllAngle = 0;
};
