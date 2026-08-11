#pragma once

#include <driver/twai.h>

namespace CanConfig_can_comm {
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_5;

constexpr uint32_t C620_COMMAND_ID = 0x200;
constexpr uint32_t C620_FEEDBACK_ID_BASE = 0x201;
constexpr uint32_t CAN_TX_INTERVAL_US = 2000;
constexpr uint32_t FEEDBACK_TIMEOUT_MS = 100;
}

