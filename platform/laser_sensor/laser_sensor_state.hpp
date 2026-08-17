#pragma once

#include "laser_sensor/constants.hpp"

#include <Arduino.h>

/**
 * @file laser_sensor_state.hpp
 * @brief VL53L0X群の内部状態と測距値管理APIを提供する。
 *
 * 各センサーの利用可否、測距値、最終更新時刻、更新回数、エラー回数を保持し、
 * fresh判定、距離フィルタ、接続数集計などの状態操作を行う。
 */

namespace laserSensorInternal {

/**
 * @brief 1台分のレーザーセンサー状態を保持する。
 */
struct SensorState {
  /** @brief センサーが初期化済みで利用可能か。 */
  bool available = false;

  /** @brief 有効な測距値を保持しているか。 */
  bool hasValue = false;

  /** @brief 補正・フィルタ後の距離[mm]。 */
  int distanceMm = 0;

  /** @brief 最後に有効な測距値を取得したmillis()時刻。 */
  uint32_t lastGoodMs = 0;

  /** @brief 有効な測距値を保存した累積回数。 */
  uint32_t updateCount = 0;

  /** @brief 新規測距セット確認時に記録した更新回数。 */
  uint32_t lastEvaluatedUpdateCount = 0;

  /** @brief 連続する重大エラーの回数。 */
  uint8_t errorCount = 0;
};

/**
 * @brief 指定したセンサーが設定上有効か確認する。
 *
 * @param index 確認対象のセンサーインデックス。
 * @return インデックスが有効範囲内でLASER_SENSOR_ENABLEDがtrueの場合true。
 */
bool sensorConfigured(int index);

/**
 * @brief 指定したセンサー状態を初期値へ戻す。
 *
 * @param index 初期化対象のセンサーインデックス。
 */
void clearOneSensorState(int index);

/**
 * @brief 全センサー状態を初期値へ戻す。
 */
void clearAllSensorStates();

/**
 * @brief 全センサーを利用不可状態へ変更する。
 *
 * 測距値など他の状態は維持し、availableのみfalseへ変更する。
 */
void markAllSensorsUnavailable();

/**
 * @brief 指定したセンサーが保持する測距値を無効化する。
 *
 * availableやエラーカウンタは変更せず、測距値と最終正常測距時刻を破棄する。
 *
 * @param index 対象のセンサーインデックス。
 */
void invalidateSensorReading(int index);

/**
 * @brief 指定したセンサーを利用不可状態へ変更する。
 *
 * 測距値を無効化し、エラーカウンタをリセットする。
 *
 * @param index 対象のセンサーインデックス。
 */
void disableSensor(int index);

/**
 * @brief 指定したセンサーを利用可能状態へ変更する。
 *
 * @param index 対象のセンサーインデックス。
 */
void markSensorAvailable(int index);

/**
 * @brief 指定したセンサーが利用可能か確認する。
 *
 * @param index 確認対象のセンサーインデックス。
 * @return 利用可能な場合true。
 */
bool sensorAvailable(int index);

/**
 * @brief 指定したセンサーが有効な測距値を保持しているか確認する。
 *
 * @param index 確認対象のセンサーインデックス。
 * @return 有効な測距値を保持している場合true。
 */
bool sensorHasValue(int index);

/**
 * @brief 指定したセンサーの最終正常測距時刻を取得する。
 *
 * @param index 取得対象のセンサーインデックス。
 * @return 最後に有効な測距値を保存したmillis()時刻。無効なindexの場合は0。
 */
uint32_t sensorLastGoodMs(int index);

/**
 * @brief 指定したセンサーの連続エラー回数を取得する。
 *
 * @param index 取得対象のセンサーインデックス。
 * @return 現在のエラーカウント。無効なindexの場合は0。
 */
uint8_t sensorErrorCount(int index);

/**
 * @brief 指定したセンサーのエラーカウンタを1増加させる。
 *
 * カウンタはLASER_SENSOR_MAX_ERROR_COUNTを上限として飽和する。
 *
 * @param index 対象のセンサーインデックス。
 */
void incrementSensorErrorCount(int index);

/**
 * @brief 生の測距値を補正・フィルタして内部状態へ保存する。
 *
 * オフセット補正後に有効距離範囲を確認し、有効な場合のみフィルタ処理、
 * 最終更新時刻、更新回数、エラー状態を更新する。
 *
 * @param index 保存対象のセンサーインデックス。
 * @param rawDistanceMm VL53L0Xから取得した生の距離[mm]。
 */
void storeSensorReading(int index, uint16_t rawDistanceMm);

/**
 * @brief 有効設定されている全センサーがfreshか確認する。
 *
 * @return 全有効センサーが利用可能かつstale時間内の測距値を持つ場合true。
 */
bool allConfiguredSensorsFresh();

/**
 * @brief 指定したセンサーがfreshか確認する。
 *
 * @param index 確認対象のセンサーインデックス。
 * @return 有効設定され、利用可能で、stale時間内の測距値を持つ場合true。
 */
bool sensorFresh(int index);

/**
 * @brief 指定したセンサーの補正・フィルタ後距離を取得する。
 *
 * @param index 取得対象のセンサーインデックス。
 * @return 距離[mm]。freshでない場合は-1。
 */
int sensorDistanceMm(int index);

/**
 * @brief 現在利用可能な有効センサー数を取得する。
 *
 * @return 有効設定され、availableであるセンサー数。
 */
int connectedSensorCount();

/**
 * @brief 設定上有効なセンサー数を取得する。
 *
 * @return LASER_SENSOR_ENABLEDがtrueのセンサー数。
 */
int configuredSensorCount();

/**
 * @brief 全有効センサーの新しい測距セットが揃ったか確認する。
 *
 * 前回trueを返した時点から、すべての有効センサーでupdateCountが更新されたか確認する。
 * trueを返す場合は現在のupdateCountを確認済み値として記録する。
 *
 * @return 全有効センサーに新しい有効測距値が揃った場合true。
 */
bool newMeasurementSetReady();

}  // namespace laserSensorInternal
