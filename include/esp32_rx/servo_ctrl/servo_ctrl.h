#pragma once

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
 * @brief サーボ制御パケットを解析し、指定チャネルへ角度を出力する。
 *
 * パケットは先頭の種別1バイトに続くチャネル1バイト、角度1バイトを
 * 16進文字列で表す。短いパケット、範囲外の論理サーボ番号、または180度を
 * 超える角度は出力しない。角度はサーボ別の許容範囲へ制限される。
 *
 * @param hex 解析対象の16進文字列。
 */
void servoCtrlHandlePacket(const String& hex);

/**
 * @brief 全論理サーボ角度設定パケットを解析して一括出力する。
 *
 * packet形式は種別1バイトと角度1バイトを表す16進文字列（54AA）。
 * 短いpacketまたは180度を超える角度は出力しない。
 *
 * @param hex 解析対象の16進文字列。
 */
void servoCtrlHandleAllPacket(const String& hex);
