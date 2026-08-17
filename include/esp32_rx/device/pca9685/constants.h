#pragma once

#include <cstdint>

namespace Pca9685Config {

constexpr int SERVO_I2C_SDA_PIN = 26;
constexpr int SERVO_I2C_SCL_PIN = 27;

constexpr uint32_t SERVO_I2C_CLOCK_HZ = 100000;

constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint16_t PCA9685_PWM_FREQ_HZ = 50;
constexpr uint8_t PCA9685_CHANNEL_COUNT = 16;

}  // namespace Pca9685Config
