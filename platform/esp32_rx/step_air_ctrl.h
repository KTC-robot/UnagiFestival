#pragma once

#include <Arduino.h>

enum class StepAirState {
  STARTUP,
  FLAT_NORMAL,
  CLIMB_FRONT_UP,
  TOP_BOTH_UP,
  DESCEND_REAR_DOWN,
  SENSOR_ERROR
};

/**
 * @brief 段差エア制御のGPIO、I2C、VL53L0Xを初期化する。
 *
 * 初期化失敗時もバルブは安全側のOFFに維持する。
 *
 * @return 有効化された全VL53L0Xの初期化に成功した場合はtrue。
 */
bool stepAirCtrlBegin();

/**
 * @brief VL53L0X測距、再初期化、段差エア状態遷移を周期更新する。
 */
void stepAirCtrlUpdate();

/**
 * @brief 設定上有効なVL53L0Xがすべて新鮮な測距値を持つか確認する。
 *
 * @return すべての有効センサーが利用可能な場合はtrue。
 */
bool stepAirCtrlSensorsReady();

/**
 * @brief 指定したVL53L0Xの測距値が利用可能か確認する。
 *
 * TCA9548A経由で初期化済み、かつstale timeout内に有効値を取得した場合だけtrue。
 *
 * @param sensorIndex 確認するセンサーインデックス。
 * @return 測距値が新鮮な場合true。
 */
bool stepAirCtrlSensorFresh(int sensorIndex);

/**
 * @brief 指定したVL53L0Xの補正・フィルタ後距離を取得する。
 *
 * 値が古い、未初期化、または無効なセンサーの場合は-1を返す。
 *
 * @param sensorIndex 取得するセンサーインデックス。
 * @return 距離[mm]。取得できない場合は-1。
 */
int stepAirCtrlGetDistanceMm(int sensorIndex);

bool stepAirCtrlFrontValveOn();
bool stepAirCtrlRearValveOn();
StepAirState stepAirCtrlGetState();
const char* stepAirCtrlGetStateText();
char stepAirCtrlGetStateCode();

/**
 * @brief センサー異常時と同じ安全側状態へ強制的に遷移する。
 */
void stepAirCtrlForceSafe();
