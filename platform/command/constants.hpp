#pragma once

#include <stdint.h>

namespace CommandProtocol {

/** @brief wire packetの種別。 */
enum class PacketType : uint8_t {
  CONTROL = 0x43,
  SERVO_SET = 0x53,
  SERVO_SET_ALL = 0x54,
};

/** @brief CONTROL packet内のCommand ID。 */
enum class ControlCommand : uint8_t {
  STOP = 0x01,
  EMERGENCY_STOP = 0x02,
  CHANGE_POWER = 0x03,
  DRIVE = 0x04,
  SET_WHEEL_GAIN = 0x05,
  GAIN_TUNE_START = 0x06,
  GAIN_TUNE_KEEPALIVE = 0x07,
  GAIN_TUNE_RESULT_ACK = 0x08,
  STEP_ASSIST_RESET = 0x09,
};

constexpr float GAIN_WIRE_SCALE = 1000.0f;
constexpr uint32_t GAIN_TUNING_DURATION_UNIT_MS = 100;
constexpr uint32_t GAIN_TUNING_MAX_DURATION_MS = 10000;
constexpr uint8_t GAIN_TUNING_WHEEL_COUNT = 4;

}  // namespace CommandProtocol
