#pragma once

#include <cstdint>

/**
 * @file pca9685_driver.hpp
 * @brief Servo専用I2C上のPCA9685を操作するDriver APIを提供する。
 */

/**
 * @brief Servo専用I2CとPCA9685を初期化する。
 *
 * PWM周波数を50Hzへ設定し、全16チャネルをFULL OFFにする。
 * Laser Sensor用I2C Busとは独立したTwoWireを初期化する。
 *
 * @return Servo専用I2C、接続確認、50Hz設定に成功した場合true。
 *         いずれかに失敗した場合false。
 */
bool pca9685DriverBegin();

/**
 * @brief PCA9685の指定チャネルへパルス幅を出力する。
 *
 * @param channel PCA9685チャネル。0〜15。
 * @param pulseUs パルス幅。マイクロ秒単位。
 * @return 有効channelへPWMを書き込めた場合true。未初期化や範囲外ではfalse。
 */
bool pca9685DriverSetPulseUs(uint8_t channel, uint16_t pulseUs);

/**
 * @brief PCA9685の指定チャネルをFULL OFFにする。
 *
 * @param channel PCA9685チャネル。0〜15。
 * @return 出力停止に成功した場合true。
 */
bool pca9685DriverDisable(uint8_t channel);

/**
 * @brief PCA9685の全16チャネルをFULL OFFにする。
 */
void pca9685DriverDisableAll();

/**
 * @brief Servo専用I2CとPCA9685を再初期化して直前の出力状態を復元する。
 *
 * activeだったチャネルは最後のパルス幅を再出力し、inactiveだった
 * チャネルはFULL OFFにする。
 *
 * @return 全設定と出力の復元に成功した場合true。
 */
bool pca9685DriverReinitialize();
