#pragma once

#include <Arduino.h>

/**
 * @brief レーザーセンサー用I2Cバスを初期化する。
 *
 * @return 初期化成功時true。
 */
bool i2cBusBegin();

/**
 * @brief レーザーセンサー用I2Cバスを再初期化する。
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