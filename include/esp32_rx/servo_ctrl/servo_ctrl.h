#pragma once

#include <stdint.h>

#include <Arduino.h>

/**
 * @file servo_ctrl.h
 * @brief 論理サーボ番号と角度を扱うサーボ制御APIを提供する。
 */

/**
 * @brief servo_managerを初期化し、全サーボ出力を停止する。
 *
 * 共有I2Cバスは事前にi2cBusBegin()で初期化されていることを前提とする。
 */
void servoCtrlBegin();

/**
 * @brief servo_managerへ全サーボ出力の停止を指示する。
 */
void servoCtrlDisableAll();

/**
 * @brief I2C bus recovery後の出力復元をservo_managerへ委譲する。
 *
 * @return 全チャネルの復元に成功した場合true。
 */
bool servoCtrlRestoreAfterI2cRecovery();

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

/** @brief 移行期間中の個別サーボpacket入口。 */
void servoCtrlHandlePacket(const String& hex);

/** @brief 移行期間中の全サーボpacket入口。 */
void servoCtrlHandleAllPacket(const String& hex);
