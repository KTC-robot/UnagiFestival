#pragma once

#include <Arduino.h>

// ========================================
// センサー構成
// ========================================

constexpr int LASER_SENSOR_COUNT = 3;

constexpr int LASER_SENSOR_FRONT = 1;
constexpr int LASER_SENSOR_CENTER = 2;
constexpr int LASER_SENSOR_REAR = 0;

constexpr bool LASER_SENSOR_ENABLED[LASER_SENSOR_COUNT] = {
    true,   // FRONT  CH1
    true,  // CENTER CH2
    true   // REAR   CH0
};

constexpr uint8_t LASER_SENSOR_CHANNELS[LASER_SENSOR_COUNT] = {
    0,
    1,
    2
};

constexpr int LASER_SENSOR_OFFSETS_MM[LASER_SENSOR_COUNT] = {
    0,
    0,
    0
};

constexpr const char* LASER_SENSOR_NAMES[LASER_SENSOR_COUNT] = {
    "FRONT",
    "CENTER",
    "REAR"
};

// ========================================
// VL53L0X
// ========================================

constexpr uint8_t LASER_SENSOR_VL53L0X_ADDRESS = 0x29;

// ========================================
// 測距周期
// ========================================

// 1センサーを読む周期
constexpr uint32_t LASER_SENSOR_PERIOD_MS = 100;

// この時間以上新しい値がなければfreshではない
constexpr uint32_t LASER_SENSOR_STALE_MS = 1000;

// この時間測距できなければ利用不可にする
constexpr uint32_t LASER_SENSOR_REINIT_AFTER_MS = 3000;

// 再初期化を連打しないための間隔
constexpr uint32_t LASER_SENSOR_REINIT_INTERVAL_MS = 3000;

// ========================================
// エラー
// ========================================

// I2C/APIエラーが連続した場合にセンサーを停止する閾値
constexpr uint8_t LASER_SENSOR_MAX_ERROR_COUNT = 5;
constexpr uint8_t LASER_SENSOR_BUS_FAULT_THRESHOLD = 3;

constexpr uint32_t LASER_SENSOR_TASK_DELAY_MS = 10;
constexpr uint32_t LASER_SENSOR_TASK_STACK_SIZE = 8192;
constexpr uint32_t LASER_SENSOR_TASK_PRIORITY = 1;
constexpr BaseType_t LASER_SENSOR_TASK_CORE = 0;

// ========================================
// 距離
// ========================================

// 有効な測距値として扱う範囲
constexpr int LASER_SENSOR_MIN_VALID_MM = 1;
constexpr int LASER_SENSOR_MAX_VALID_MM = 2000;

// ========================================
// 距離フィルタ
// ========================================

// new = 1/4, old = 3/4
constexpr int LASER_SENSOR_FILTER_NEW_WEIGHT_NUMERATOR = 1;
constexpr int LASER_SENSOR_FILTER_WEIGHT_DENOMINATOR = 4;
