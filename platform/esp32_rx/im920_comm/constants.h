#pragma once

#include <cstdint>

namespace CanConfig_im920_comm {

constexpr int IM920_RX = 16;
constexpr int IM920_TX = 17;

constexpr int LED_PIN = 2;
constexpr uint32_t LED_PULSE_MS = 30;
constexpr uint32_t COMM_TIMEOUT_MS = 600;
constexpr uint32_t STATUS_TX_INTERVAL_MS = 1000;
constexpr int JOY_ACK_INTERVAL = 10;

constexpr int DRIVE_POWER_STEP = 5;
constexpr bool ENABLE_REPLY_TO_PI = true;
constexpr bool ENABLE_RPM_TO_PI = true;
// Raspberry Pi側へ距離センサー状態を送る。
constexpr bool ENABLE_AIR_STATUS_TO_PI = true;
constexpr bool SHOW_RAW = true;
constexpr bool SHOW_JOY = true;

// Raspberry Pi側で確認済みのIM920設定。
// 自動書き換えはせず、起動時にESP32側の値を読み出して比較する。
constexpr const char* EXPECTED_IM920_GROUP = "0001C07B";
constexpr const char* EXPECTED_IM920_CHANNEL = "01";
constexpr uint32_t IM920_QUERY_TIMEOUT_MS = 500;
}