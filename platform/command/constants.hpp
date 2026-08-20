#pragma once

/**
 * @file constants.hpp
 * @brief Raspberry PiとESP32間のCommand wire protocol定義を保持する。
 */

#include <stdint.h>

namespace CommandProtocol {

/** @brief wire packetの種別。 */
enum class PacketType : uint8_t {
  CONTROL = 0x43,  ///< 車体制御Commandを格納するpacket。
};

/** @brief CONTROL packet内のCommand ID。 */
enum class ControlCommand : uint8_t {
  STOP = 0x01,                    ///< 通常停止。
  EMERGENCY_STOP = 0x02,          ///< 緊急停止。
  RESERVED_03 = 0x03,             ///< 互換性維持のため欠番。
  DRIVE = 0x04,                   ///< vx、vy、wzによる走行。
  SET_WHEEL_GAIN = 0x05,          ///< 方向別・車輪別gainの設定。
  GAIN_TUNE_START = 0x06,         ///< RPM計測試験の開始。
  GAIN_TUNE_KEEPALIVE = 0x07,     ///< 計測中の通信維持。
  GAIN_TUNE_RESULT_ACK = 0x08,    ///< WG/WDに対する遠端ACK。
  STEP_ASSIST_RESET = 0x09,       ///< StepAssistの状態リセット。
  AIR_FIRE_START = 0x0A,          ///< Air Cylinder連射開始。
  AIR_FIRE_STOP = 0x0B,           ///< Air Cylinder連射停止。
  MD20A_SET_STATE = 0x0C,         ///< MD20Aのdesired state設定。
};

constexpr float GAIN_WIRE_SCALE = 1000.0f;
constexpr uint32_t GAIN_TUNING_DURATION_UNIT_MS = 100;
constexpr uint32_t GAIN_TUNING_MAX_DURATION_MS = 10000;
constexpr uint8_t GAIN_TUNING_WHEEL_COUNT = 4;

}  // namespace CommandProtocol
