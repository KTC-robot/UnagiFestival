#pragma once

#include <stdint.h>

namespace Tca9548aConfig {

constexpr uint8_t TCA9548A_ADDRESS = 0x70;
constexpr uint8_t TCA9548A_CHANNEL_COUNT = 8;
constexpr uint32_t TCA9548A_CHANNEL_SETTLE_MS = 50;

}  // namespace Tca9548aConfig
