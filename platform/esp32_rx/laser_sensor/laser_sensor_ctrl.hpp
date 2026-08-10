#pragma once

#include <Arduino.h>

constexpr int LASER_SENSOR_COUNT = 3;
constexpr int LASER_SENSOR_FRONT = 0;
constexpr int LASER_SENSOR_CENTER = 1;
constexpr int LASER_SENSOR_REAR = 2;

/**
 * @brief TCA9548A経由のVL53L0X群を初期化する。
 *
 * I2Cバス再生成、TCA9548A初期化、設定上有効な各センサーの初期化を行う。
 *
 * @return 有効化された全センサーの初期化に成功した場合true。
 */
bool laserSensorCtrlBegin();

/**
 * @brief VL53L0X測距、stale判定、再初期化を周期更新する。
 */
void laserSensorCtrlUpdate();

/**
 * @brief 設定上有効なVL53L0Xがすべて新鮮な測距値を持つか確認する。
 *
 * @return すべての有効センサーが利用可能な場合true。
 */
bool laserSensorCtrlReady();

/**
 * @brief 指定したVL53L0Xの測距値が利用可能か確認する。
 *
 * @param sensorIndex 確認するセンサーインデックス。
 * @return 測距値が新鮮な場合true。
 */
bool laserSensorCtrlFresh(int sensorIndex);

/**
 * @brief 指定したVL53L0Xの補正・フィルタ後距離を取得する。
 *
 * @param sensorIndex 取得するセンサーインデックス。
 * @return 距離[mm]。取得できない場合は-1。
 */
int laserSensorCtrlGetDistanceMm(int sensorIndex);

/**
 * @brief 初期化済みで利用可能なVL53L0X数を取得する。
 *
 * @return 利用可能な有効センサー数。
 */
int laserSensorCtrlConnectedCount();

/**
 * @brief 設定上有効なVL53L0X数を取得する。
 *
 * @return 有効設定のセンサー数。
 */
int laserSensorCtrlConfiguredCount();

/**
 * @brief 自動制御用に全有効センサーの新しい測距セットが揃ったか確認する。
 *
 * @return 前回確認後に全有効センサーが更新されている場合true。
 */
bool laserSensorCtrlNewMeasurementSetReady();
