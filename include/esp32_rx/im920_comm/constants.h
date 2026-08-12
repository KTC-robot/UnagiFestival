#pragma once

#include <cstdint>

namespace CanConfig_im920_comm {

constexpr int IM920_RX = 16;
constexpr int IM920_TX = 17;

constexpr int LED_PIN = 2;
constexpr uint32_t LED_PULSE_MS = 30;
constexpr uint32_t COMM_TIMEOUT_MS = 600;
constexpr uint32_t STATUS_TX_INTERVAL_MS = 1000;
constexpr int DRIVE_ACK_INTERVAL = 10;

constexpr bool ENABLE_REPLY_TO_PI = true;
constexpr bool ENABLE_RPM_TO_PI = true;
// Raspberry Pi側へ距離センサー状態を送る。
constexpr bool ENABLE_AIR_STATUS_TO_PI = true;
constexpr bool SHOW_RAW = true;
constexpr bool SHOW_DRIVE = true;

enum class PacketType : uint8_t {
  CONTROL = 0x43,
  SERVO_SET = 0x53
};

enum class ControlCommand : uint8_t {
  STOP = 0x01,
  EMERGENCY_STOP = 0x02,
  CHANGE_POWER = 0x03,
  DRIVE = 0x04,
  SET_GAIN = 0x05,
  GAIN_TUNE_START = 0x06,
  GAIN_TUNE_KEEPALIVE = 0x07
};

constexpr float GAIN_WIRE_SCALE = 1000.0f;
constexpr uint32_t GAIN_TUNING_DURATION_UNIT_MS = 100;
constexpr uint32_t GAIN_TUNING_MAX_DURATION_MS = 10000;
constexpr int GAIN_TUNING_MOTOR_COUNT = 4;

// Raspberry Pi側で確認済みのIM920設定。
// 自動書き換えはせず、起動時にESP32側の値を読み出して比較する。
constexpr const char* EXPECTED_IM920_GROUP = "0001C07B";
constexpr const char* EXPECTED_IM920_CHANNEL = "01";
constexpr uint32_t IM920_QUERY_TIMEOUT_MS = 500;
}