#pragma once

#include <Arduino.h>

void chassisCtrlBegin();
void chassisCtrlUpdate();

void chassisCtrlSetFromJoy(
  int8_t lx,
  int8_t ly,
  int8_t rx,
  int8_t dpadX,
  int8_t dpadY
);

void chassisCtrlStop();
void chassisCtrlChangePower(int delta);

int chassisCtrlGetPowerPercent();
bool chassisCtrlIsActive();

/**
 * @brief 現在の前後方向コマンドを取得する。
 *
 * デッドゾーンと車体側の反転設定を反映した値を返す。
 *
 * @return 前後方向コマンド。
 */
float chassisCtrlGetLongitudinalCommand();
