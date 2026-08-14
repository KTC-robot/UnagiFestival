#pragma once

#include <Arduino.h>

/**
 * @file stepper_ctrl.hpp
 * @brief TB6600 x2を共通PUL/DIR信号で同期制御する。
 */

enum class StepperMotion : uint8_t {
  STOP = 0,
  UP = 1,
  DOWN = 2
};

/** GPIOを初期化し、安全側STOPにする。 */
bool stepperCtrlBegin();

/** 2台を共通信号でUP方向へ回転開始する。 */
void stepperCtrlStartUp();

/** 2台をUPと逆方向へ回転開始する。 */
void stepperCtrlStartDown();

/** 共通PUL出力を停止し、2台とも停止する。 */
void stepperCtrlStop();

/** loop()から毎回呼び、共通STEPパルスを生成する。 */
void stepperCtrlUpdate();

/** ステップパルス周波数[Hz]を変更する。 */
bool stepperCtrlSetPulseHz(uint32_t pulseHz);

/** 現在動作中かを返す。 */
bool stepperCtrlIsRunning();

/** 現在の動作方向を返す。 */
StepperMotion stepperCtrlGetMotion();

/** IM920のSTEPPER packet (54 xx) を処理する。 */
void stepperCtrlHandlePacket(const String& hex);
