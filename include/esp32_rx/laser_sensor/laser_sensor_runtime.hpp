#pragma once

/**
 * @file laser_sensor_runtime.hpp
 * @brief レーザーセンサー群の初期化と周期実行を管理する。
 *
 * センサー群全体の起動処理、測距スケジューリング、stale判定、
 * 利用不能センサーの再初期化を行う内部ランタイムAPIを定義する。
 */

namespace laserSensorInternal {

/**
 * @brief 有効設定されている全レーザーセンサーを初期化する。
 *
 * 共有I2Cバスが初期化済みであることを前提に、TCA9548Aと内部状態を整え、
 * 設定上有効なセンサーを順番に初期化する。
 */
void initializeAllSensors();

/**
 * @brief レーザーセンサー群の周期処理を1回実行する。
 *
 * 測距周期に応じたセンサー読み取り、stale判定、必要に応じた再初期化を行う。
 * Laser Sensor専用FreeRTOS Taskから継続的に呼び出す。
 */
void updateSensors();

}  // namespace laserSensorInternal
