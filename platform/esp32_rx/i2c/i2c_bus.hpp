#pragma once

#include <Arduino.h>

/**
 * @brief ESP32側I2CバスとTCA9548Aを初期化する。
 *
 * Wireを安定動作確認済みのSDA/SCL/50 kHz/timeout 50 msで開始し、
 * TCA9548Aへ全チャネル無効のcontrol byteを実書き込みする。
 *
 * @return I2CバスとTCA9548Aの初期化に成功した場合true。
 */
bool i2cBusBegin();

/**
 * @brief ESP32側I2Cバスを再生成し、TCA9548Aを初期状態へ戻す。
 *
 * 復旧用途の高レベル処理。Wire.end()/Wire.begin()の結果をSerialへ出力する。
 *
 * @return I2CバスとTCA9548Aの再初期化に成功した場合true。
 */
bool i2cBusRestart();

/**
 * @brief 現在のWireへ標準I2C設定を再適用する。
 *
 * Adafruit_VL53L0X::begin()内部でWire.begin()が呼ばれるため、
 * センサー初期化後に50 kHz/timeout 50 msへ戻す。
 */
void i2cBusApplySettings();

/**
 * @brief TCA9548Aで指定したI2Cチャネルを選択する。
 *
 * VL53L0Xは3台とも同じI2Cアドレス0x29を使用するため、
 * センサーアクセス前に対象チャネルのみを有効化する。
 *
 * @param channel 選択するTCA9548Aチャネル。
 * @return チャネル選択に成功した場合true。
 */
bool i2cBusSelectTcaChannel(uint8_t channel);

/**
 * @brief ESP32側SDAピンの現在レベルを読む。
 *
 * @return digitalRead()で取得したSDAレベル。
 */
int i2cBusReadSda();

/**
 * @brief ESP32側SCLピンの現在レベルを読む。
 *
 * @return digitalRead()で取得したSCLレベル。
 */
int i2cBusReadScl();
