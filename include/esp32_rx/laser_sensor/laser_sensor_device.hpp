#pragma once

/**
 * @file laser_sensor_device.hpp
 * @brief 個々のVL53L0Xに対する初期化・測距処理を提供する。
 *
 * TCA9548Aチャネル選択、VL53L0Xの接続確認、初期化、測距をまとめた
 * レーザーセンサーモジュール内部用インターフェースを定義する。
 */

namespace laserSensorInternal {

/**
 * @brief 指定したVL53L0XをTCA9548A経由で初期化する。
 *
 * 対象チャネルを選択し、VL53L0XのI2C応答を確認した後にセンサー初期化を行う。
 * 初期化結果は内部状態へ反映される。
 *
 * @param index 初期化対象のセンサーインデックス。
 * @return 初期化に成功した場合true。
 */
bool initializeOneSensor(int index);

/**
 * @brief 指定したVL53L0Xから1回分の測距を行う。
 *
 * 対象TCA9548Aチャネルを選択して測距し、正常値の場合は補正・フィルタ後の
 * 距離を内部状態へ保存する。通信エラー時はエラーカウンタと利用状態を更新する。
 *
 * @param index 測距対象のセンサーインデックス。
 */
void readOneSensor(int index);

}  // namespace laserSensorInternal
