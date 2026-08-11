#pragma once

#include <Arduino.h>

/**
 * @file servo_ctrl.h
 * @brief PCA9685を使用したサーボ出力制御APIを提供する。
 */

/**
 * @brief PCA9685を初期化し、全サーボチャネルをFULL OFFにする。
 */
void servoCtrlBegin();

/**
 * @brief 全サーボチャネルをFULL OFFにする。
 *
 * PCA9685が未初期化の場合は何も行わない。
 */
void servoCtrlDisableAll();

/**
 * @brief サーボ制御パケットを解析し、指定チャネルへ角度を出力する。
 *
 * パケットは先頭の種別1バイトに続くチャネル1バイト、角度1バイトを
 * 16進文字列で表す。短いパケット、範囲外のチャネル、180度を超える角度、
 * またはPCA9685未初期化時は出力しない。角度はチャネル別の許容範囲へ制限される。
 *
 * @param hex 解析対象の16進文字列。
 */
void servoCtrlHandlePacket(const String& hex);