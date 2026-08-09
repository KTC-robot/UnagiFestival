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
 * @brief 段差制御が参照する現在の前後方向コマンドを取得する。
 *
 * デッドゾーンと車体側の反転設定を反映した値を返す。
 *
 * @return 前後方向コマンド。正負の向きは実機に合わせてstep_air_config.hで補正する。
 */
float chassisCtrlGetLongitudinalCommand();
