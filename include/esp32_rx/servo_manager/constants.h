#pragma once

#include <cstdint>

namespace CanConfig_servo_manager {

constexpr uint8_t SERVO_TCA9548A_CHANNEL = 3;  ///< PCA9685を接続するTCAチャネル。
constexpr uint8_t PCA9685_ADDRESS = 0x40;      ///< PCA9685の7bit I2Cアドレス。
constexpr uint16_t PCA9685_PWM_FREQ_HZ = 50;  ///< サーボ用PWM周波数。
constexpr uint8_t PCA9685_CHANNEL_COUNT = 16; ///< PCA9685の物理チャネル数。

}  // namespace CanConfig_servo_manager
