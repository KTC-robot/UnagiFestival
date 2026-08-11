#pragma once

#include <Arduino.h>

// ========================================
// ESP32 I2C
// ========================================

constexpr int LASER_SENSOR_I2C_SDA_PIN = 21;
constexpr int LASER_SENSOR_I2C_SCL_PIN = 22;

// 現在実機で通信確認できている速度
constexpr uint32_t LASER_SENSOR_I2C_CLOCK_HZ = 50000;

// Wire timeout
constexpr uint16_t LASER_SENSOR_I2C_TIMEOUT_MS = 100;

// Wire.end() -> Wire.begin() 間の待機
constexpr uint32_t LASER_SENSOR_I2C_RESTART_DELAY_MS = 100;

// TCAチャネル選択後の安定待ち
constexpr uint32_t LASER_SENSOR_CHANNEL_SETTLE_MS = 50;

// ========================================
// TCA9548A
// ========================================

constexpr uint8_t LASER_SENSOR_TCA9548A_ADDRESS = 0x70;
constexpr uint8_t LASER_SENSOR_TCA9548A_CHANNEL_COUNT = 8;
