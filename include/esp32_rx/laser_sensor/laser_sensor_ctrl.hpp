#pragma once

#include "constants.h"

/**
 * @file laser_sensor_ctrl.hpp
 * @brief レーザーセンサーモジュールの公開制御APIを提供する。
 *
 * TCA9548A経由で接続されたVL53L0X群について、初期化、周期更新、
 * 測距値取得、接続状態確認を行うための公開インターフェースを定義する。
 */

/**
 * @brief VL53L0X初期化と更新を行う低優先度Taskを起動する。
 *
 * I2CバスとTCA9548Aを初期化した後、設定上有効な各VL53L0Xを順番に初期化する。
 *
 * @return Taskの起動に成功した場合true。
 */
bool laserSensorCtrlBegin();

/**
 * @brief レーザーセンサー群の周期処理を更新する。
 *
 * 測距、stale判定、通信異常後の再初期化を内部設定の周期に従って実行する。
 */
void laserSensorCtrlUpdate();

/**
 * @brief 有効設定されている全センサーが利用可能か確認する。
 *
 * 各センサーが初期化済みで、かつstale判定時間内の測距値を保持しているか確認する。
 *
 * @return 全有効センサーが新鮮な測距値を持つ場合true。
 */
bool laserSensorCtrlReady();

/**
 * @brief 指定したセンサーの測距値が利用可能か確認する。
 *
 * @param sensorIndex 確認対象のセンサーインデックス。
 * @return 指定センサーが有効設定され、新鮮な測距値を保持している場合true。
 */
bool laserSensorCtrlFresh(int sensorIndex);

/**
 * @brief 指定したセンサーの補正・フィルタ後距離を取得する。
 *
 * @param sensorIndex 取得対象のセンサーインデックス。
 * @return 距離[mm]。有効な測距値を取得できない場合は-1。
 */
int laserSensorCtrlGetDistanceMm(int sensorIndex);

/**
 * @brief 現在利用可能なセンサー数を取得する。
 *
 * @return 有効設定され、初期化済みとして利用可能なセンサー数。
 */
int laserSensorCtrlConnectedCount();

/**
 * @brief 設定上有効なセンサー数を取得する。
 *
 * @return LASER_SENSOR_ENABLEDで有効化されているセンサー数。
 */
int laserSensorCtrlConfiguredCount();

/**
 * @brief 全有効センサーの新しい測距セットが揃ったか確認する。
 *
 * 前回trueを返した時点以降に、すべての有効センサーが少なくとも1回ずつ
 * 新しい有効測距値へ更新されたかを確認する。
 *
 * @return 全有効センサーが新しい測距値へ更新済みの場合true。
 */
bool laserSensorCtrlNewMeasurementSetReady();
