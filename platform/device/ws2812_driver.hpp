#pragma once

#include <stdint.h>

/** @brief GPIO13のWS2812B data出力を初期化する。 */
void ws2812DriverBegin();

/**
 * @brief 接続された全WS2812Bを指定RGB色へ更新する。
 *
 * @param red 赤成分。0〜255。
 * @param green 緑成分。0〜255。
 * @param blue 青成分。0〜255。
 */
void ws2812DriverSetRgb(uint8_t red, uint8_t green, uint8_t blue);
