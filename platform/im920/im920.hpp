#pragma once

#include <Arduino.h>

/** @brief IM920 UARTと周辺状態を初期化する。 */
void im920Begin();

/** @brief UART受信・PC passthrough・結果送信を更新する。 */
void im920Update();

/** @brief 有効Commandの受信timeoutを確認して安全停止する。 */
void im920CheckTimeout();

/**
 * @brief textをTXDA payloadへ変換してIM920から送信する。
 * @param text 送信するASCII text。
 */
void im920SendText(const String& text);
