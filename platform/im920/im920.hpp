#pragma once

/**
 * @file im920.hpp
 * @brief IM920のUART通信、timeout監視、text送信を提供する。
 */

#include <Arduino.h>

/**
 * @brief IM920 UARTと受信表示用LEDを初期化する。
 *
 * 起動後にmodule設定を問い合わせ、期待するgroupとchannelも確認する。
 */
void im920Begin();

/**
 * @brief UART受信・PC passthrough・Gain Tuning結果送信を更新する。
 *
 * blocking待機を行わず、main loopから繰り返し呼び出して通信を進める。
 */
void im920Update();

/**
 * @brief 有効Commandの受信timeoutを確認して足回りを安全停止する。
 *
 * IM920自身のOK/NGや設定応答では受信時刻を更新しない。
 */
void im920CheckTimeout();

/**
 * @brief textをTXDA payloadへ変換してIM920から送信する。
 *
 * textは大文字16進文字列へ変換し、IM920のTXDA commandとしてUARTへ渡す。
 * @param text 送信するASCII text。
 */
void im920SendText(const String& text);
