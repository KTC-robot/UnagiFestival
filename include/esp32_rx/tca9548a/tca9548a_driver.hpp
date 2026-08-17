#pragma once

#include <stdint.h>

/**
 * @brief lock保持中のI2C Busで指定チャネルだけを有効化する。
 *
 * 呼び出し側はチャネル選択後のdevice操作が終わるまで
 * I2cBusLockGuardを保持する必要がある。
 *
 * @param channel チャネル番号。0〜7。
 * @return チャネル選択に成功した場合true。
 */
bool tca9548aDriverSelectChannel(uint8_t channel);

/**
 * @brief lock保持中のI2C Busで全チャネルを無効化する。
 *
 * @return 無効化に成功した場合true。
 */
bool tca9548aDriverDisableAllChannels();
