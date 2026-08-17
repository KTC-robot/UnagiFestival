#include "tca9548a/tca9548a_driver.hpp"

#include <Arduino.h>

#include "i2c/i2c_bus.hpp"
#include "tca9548a/constants.h"

using namespace Tca9548aConfig;

bool tca9548aDriverSelectChannel(uint8_t channel) {
  if (channel >= TCA9548A_CHANNEL_COUNT) return false;

  if (!i2cBusWriteByteLocked(
        TCA9548A_ADDRESS,
        static_cast<uint8_t>(1U << channel)
      )) {
    return false;
  }

  delay(TCA9548A_CHANNEL_SETTLE_MS);
  return true;
}

bool tca9548aDriverDisableAllChannels() {
  return i2cBusWriteByteLocked(TCA9548A_ADDRESS, 0x00);
}
