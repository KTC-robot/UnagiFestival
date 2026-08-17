#pragma once

/**
 * @file constants.hpp
 * @brief 論理Servoごとの角度範囲、反転、PCA9685 channel対応を定義する。
 */

#include <cstdint>

namespace CanConfig_servo_ctrl {

constexpr uint8_t SERVO_CHANNEL_COUNT = 7;

// 論理サーボ番号とPCA9685の物理チャネルを分離し、配線変更をmappingだけに留める。
const uint8_t SERVO_PCA_CHANNEL[SERVO_CHANNEL_COUNT] = {
  0, 1, 2, 3, 4, 5, 6
};

const uint8_t SERVO_MIN_ANGLE[SERVO_CHANNEL_COUNT] = {
  0, 0, 0, 0, 0, 0, 0
};

const uint8_t SERVO_MAX_ANGLE[SERVO_CHANNEL_COUNT] = {
  180, 180, 180, 180, 180, 180, 180
};

const uint16_t SERVO_MIN_US[SERVO_CHANNEL_COUNT] = {
  600, 600, 600, 600, 600, 600, 600
};

const uint16_t SERVO_MAX_US[SERVO_CHANNEL_COUNT] = {
  2400, 2400, 2400, 2400, 2400, 2400, 2400
};

const bool SERVO_REVERSED[SERVO_CHANNEL_COUNT] = {
  false, false, false, false, false, false, false
};

}  // namespace CanConfig_servo_ctrl
