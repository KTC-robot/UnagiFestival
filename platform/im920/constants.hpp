#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Im920Config {

constexpr int IM920_RX = 16;
constexpr int IM920_TX = 17;
constexpr uint32_t IM920_BAUD = 19200;
constexpr int LED_PIN = 2;
constexpr uint32_t LED_PULSE_MS = 30;
constexpr uint32_t COMM_TIMEOUT_MS = 600;
constexpr uint32_t DRIVE_ACK_INTERVAL = 10;
constexpr bool ENABLE_REPLY_TO_PI = true;
constexpr bool SHOW_RAW = true;
constexpr bool SHOW_DRIVE = true;
constexpr bool ENABLE_GAIN_TUNING_TX_LOG = false;
constexpr size_t IM920_TXDA_MAX_PAYLOAD_BYTES = 32;

constexpr uint32_t GAIN_TUNING_TX_RESPONSE_TIMEOUT_MS = 1000;
constexpr uint32_t GAIN_TUNING_RESULT_ACK_TIMEOUT_MS = 400;
constexpr uint32_t GAIN_TUNING_RESULT_TURNAROUND_GUARD_MS = 150;
constexpr uint8_t GAIN_TUNING_RESULT_MAX_RETRIES = 3;

constexpr const char* EXPECTED_IM920_GROUP = "0001C07B";
constexpr const char* EXPECTED_IM920_CHANNEL = "01";
constexpr uint32_t IM920_QUERY_TIMEOUT_MS = 500;

}  // namespace Im920Config
