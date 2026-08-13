#pragma once

#include <Arduino.h>

/**
 * @file im920_comm.h
 * @brief IM920とのUART通信および受信コマンド処理APIを提供する。
 */

/**
 * @brief IM920用UARTと受信表示LEDを初期化し、無線設定を確認する。
 */
void im920CommBegin();

/**
 * @brief LED状態、IM920受信、PCシリアル入力を処理する。
 */
void im920CommUpdate();

/**
 * @brief 無線パケットの受信タイムアウトを確認し、必要なら足回りを停止する。
 *
 * タイムアウト時点で足回りが動作中の場合だけ停止処理を実行する。
 */
void im920CommCheckTimeout();

/**
 * @brief ASCII文字列を16進文字列へ変換し、IM920のTXDAコマンドで送信する。
 *
 * 返信機能が無効な場合、または空文字列の場合は送信しない。
 *
 * @param text 送信するASCII文字列。
 */
void im920CommSendText(const String& text);
