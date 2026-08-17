#pragma once

#include <stdint.h>


/**
 * @file servo_ctrl.h
 * @brief 論理サーボ番号と角度を扱うサーボ制御APIを提供する。
 */

/**
 * @brief PCA9685 Driverを初期化し、全サーボ出力を停止する。
 *
 * PCA9685 DriverがServo専用I2Cも同時に初期化する。
 */
void servoCtrlBegin();

/**
 * @brief PCA9685 Driverへ全サーボ出力の停止を指示する。
 */
void servoCtrlDisableAll();

/**
 * @brief 指定した論理サーボへ角度を出力する。
 *
 * @param channel 論理サーボ番号。
 * @param angle 指定角度。0〜180度。
 * @return 出力に成功した場合true。
 */
bool servoCtrlSetAngle(uint8_t channel, uint8_t angle);

/**
 * @brief 全論理サーボへ同じ角度を出力する。
 *
 * @param angle 指定角度。0〜180度。
 * @return 全チャネルへの出力に成功した場合true。
 */
bool servoCtrlSetAllAngles(uint8_t angle);
