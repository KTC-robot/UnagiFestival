#pragma once

#include <Arduino.h>

/**
 * @file i2c_bus.hpp
 * @brief VL53L0X/TCA9548A/PCA9685が共有するI2Cバス制御APIを提供する。
 */

class I2cBusLockGuard {
 public:
  I2cBusLockGuard();
  ~I2cBusLockGuard();

  I2cBusLockGuard(const I2cBusLockGuard&) = delete;
  I2cBusLockGuard& operator=(const I2cBusLockGuard&) = delete;

  bool locked() const;

 private:
  bool locked_;
};

bool i2cBusLock();
void i2cBusUnlock();

/**
 * @brief 共有I2Cバスを初期化する。
 *
 * 通常起動時に1回だけ呼び出す。Wire.end()は行わない。
 *
 * @return 初期化成功時true。
 */
bool i2cBusBegin();

/**
 * @brief bus-level fault発生時に共有I2Cバスを再初期化する。
 *
 * Wire.end()とWire.begin()を伴うため、個別デバイスの通常retryには使用しない。
 *
 * @return 再初期化成功時true。
 */
bool i2cBusRestart();

/**
 * @brief I2Cクロックとタイムアウト設定を再適用する。
 */
void i2cBusApplySettings();

/**
 * @brief TCA9548Aの全チャネルを無効化する。
 *
 * @return 成功時true。
 */
bool i2cBusDisableAllTcaChannels();

/**
 * @brief TCA9548Aの指定チャネルだけを有効化する。
 *
 * @param channel チャネル番号0〜7。
 * @return 成功時true。
 */
bool i2cBusSelectTcaChannel(uint8_t channel);

/**
 * @brief 指定I2Cアドレスへprobeを行う。
 *
 * @param address 7bit I2Cアドレス。
 * @return ACKを受信した場合true。
 */
bool i2cBusProbeDevice(uint8_t address);

/**
 * @brief SDAの現在の論理レベルを取得する。
 *
 * @return HIGHまたはLOW。
 */
int i2cBusReadSda();

/**
 * @brief SCLの現在の論理レベルを取得する。
 *
 * @return HIGHまたはLOW。
 */
int i2cBusReadScl();
