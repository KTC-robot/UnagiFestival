#pragma once

#include <cstdint>

/**
 * @file servo_manager.hpp
 * @brief TCA9548A配下のPCA9685を管理する低レイヤAPIを提供する。
 */

/**
 * @brief TCA9548A CH3配下のPCA9685を初期化する。
 *
 * PWM周波数を50Hzへ設定し、全16チャネルをFULL OFFにする。
 * 共有I2Cバスは事前にi2cBusBegin()で初期化されている必要がある。
 *
 * @return 初期化に成功した場合true。
 */
bool servoManagerBegin();

/**
 * @brief PCA9685の指定チャネルへパルス幅を出力する。
 *
 * @param channel PCA9685チャネル。0〜15。
 * @param pulseUs パルス幅。マイクロ秒単位。
 * @return 出力に成功した場合true。
 */
bool servoManagerSetPulseUs(uint8_t channel, uint16_t pulseUs);

/**
 * @brief PCA9685の指定チャネルをFULL OFFにする。
 *
 * @param channel PCA9685チャネル。0〜15。
 * @return 出力停止に成功した場合true。
 */
bool servoManagerDisable(uint8_t channel);

/**
 * @brief PCA9685の全16チャネルをFULL OFFにする。
 */
void servoManagerDisableAll();

/**
 * @brief I2C bus recovery後にPCA9685と直前の出力状態を復元する。
 *
 * activeだったチャネルは最後のパルス幅を再出力し、inactiveだった
 * チャネルはFULL OFFにする。
 *
 * @return 全設定と出力の復元に成功した場合true。
 */
bool servoManagerRestoreAfterI2cRecovery();
