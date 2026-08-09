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

bool stepAirCtrlSensorFresh(int sensorIndex);
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
