#pragma once

#include <Arduino.h>

/**
 * @file i2c_bus.hpp
 * @brief Laser Sensor用I2Cバスの初期化・排他制御APIを提供する。
 */

/**
 * @brief LaserSensor用I2C Busへの接続をscope単位で保持する。
 *
 * constructorでlockし、scopeを抜ける際に自動unlockすることで、
 * TCA channel選択からVL53L0X操作までの排他を崩さない。
 */
class I2cBusConnection {
 public:
  /** @brief I2C Busのrecursive mutex(I2cBusの1コネクション)を取得する。 */
  I2cBusConnection();
  /** @brief 取得済みの場合にI2C Busのmutexを解放する。 */
  ~I2cBusConnection();

  I2cBusConnection(const I2cBusConnection&) = delete;
  I2cBusConnection& operator=(const I2cBusConnection&) = delete;

  /** @return mutexを取得できている場合true。 */
  bool locked() const;

 private:
  bool locked_;
};

/** @return mutexの取得に成功した場合true。 */
bool i2cBusLock();
/** @brief 現在のtaskが保持するI2C mutexを1段階解放する。 */
void i2cBusUnlock();

/**
 * @brief 共有I2Cバスを初期化する。
 *
 * 通常起動時に1回だけ呼び出す。Wire.end()は行わない。
 *
 * @return mutex生成とWire初期化に成功した場合true。失敗時false。
 */
bool i2cBusBegin();

/**
 * @brief bus-level fault発生時に共有I2Cバスを再初期化する。
 *
 * Wire.end()とWire.begin()を伴うため、個別デバイスの通常retryには使用しない。
 *
 * @return Wire.end()後の再初期化に成功した場合true。失敗時false。
 */
bool i2cBusRestart();

/**
 * @brief I2Cクロックとタイムアウト設定を再適用する。
 */
void i2cBusApplySettings();

/**
 * @brief lock保持中(接続中)のBusへ1byte(SYN)を書き込む。
 *
 * 呼び出し側がI2cBusConnectionを保持している必要がある。
 *
 * @param address 7bit I2Cアドレス。
 * @param value 書き込む1byte。
 * @return ACKを受信した場合true。
 */
bool i2cBusWriteByteLocked(uint8_t address, uint8_t value);

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
