#pragma once

#include <Arduino.h>

/**
 * @file chassis_ctrl.h
 * @brief メカナム足回りの目標生成、固定PI制御、wheel gain調整APIを提供する。
 */

/**
 * @brief 1車輪分のwheel gain調整試験結果。
 */
struct ChassisGainTuningResult {
  uint32_t sampleCount;          ///< RPM集計に使用した有効サンプル数。
  float meanAbsoluteRpm;         ///< 実測RPM絶対値の平均。
  float standardDeviationRpm;    ///< 実測RPM絶対値の母標準偏差。
};

/**
 * @brief wheel RPM補正ゲインを選択する、ユーザー指令基準の走行方向。
 */
enum class ChassisGainDirection : uint8_t {
  FORWARD = 0,   ///< 前進。
  BACKWARD = 1,  ///< 後退。
  RIGHT = 2,     ///< 右平行移動。
  LEFT = 3       ///< 左平行移動。
};

/**
 * @brief 足回り制御の周期処理用時刻を初期化する。
 */
void chassisCtrlBegin();

/**
 * @brief モーター回転数フィードバックに基づく速度PI制御を更新する。
 *
 * CAN通信が未開始の場合や制御周期に達していない場合は更新しない。
 */
void chassisCtrlUpdate();

/**
 * @brief ロボットの移動指令から各車輪の目標回転数を設定する。
 *
 * 入力値は内部で正規化、デッドゾーン処理、方向反転、車輪ゲイン補正を受ける。
 *
 * @param vx 前後方向の移動指令。範囲は-127〜127。
 * @param vy 左右方向の移動指令。範囲は-127〜127。
 * @param wz 回転方向の移動指令。範囲は-127〜127。
 */
void chassisCtrlSetDriveCommand(
  int8_t vx,
  int8_t vy,
  int8_t wz
);

/**
 * @brief 足回りを停止し、全モーターへゼロ電流指令を即時送信する。
 *
 * 目標回転数、ランプ後回転数、PI積分値、動作中状態もリセットする。
 */
void chassisCtrlStop();

/**
 * @brief 走行出力率を指定量だけ増減する。
 *
 * 変更後の値は設定された最小値から最大値の範囲へ制限される。
 *
 * @param delta 現在値へ加算する出力率の差分[%]。
 */
void chassisCtrlChangePower(int delta);

/**
 * @brief 指定方向・車輪の目標RPM補正ゲインを設定する。
 *
 * @param direction 補正対象の走行方向。
 * @param wheelIndex 車輪インデックス。0=FL、1=FR、2=RL、3=RR。
 * @param gain 目標RPMへ掛ける補正係数。0.50〜1.50。
 * @return true 設定に成功した場合。
 * @return false direction、wheelIndex、gainのいずれかが不正な場合。
 */
bool chassisCtrlSetWheelGain(
  ChassisGainDirection direction,
  int wheelIndex,
  float gain
);

/**
 * @brief 車輪ごとの実測RPM集計用の自動走行試験を開始する。
 *
 * @param vx 前後方向指令。範囲は-127〜127。
 * @param vy 左右方向指令。範囲は-127〜127。
 * @param wz 回転方向指令。範囲は-127〜127。
 * @param durationMs 試験時間[ms]。
 */
void chassisCtrlStartGainTuning(
  int8_t vx,
  int8_t vy,
  int8_t wz,
  uint32_t durationMs
);

/**
 * @brief ゲイン調整試験が終了し、未送信の結果があるか確認する。
 *
 * @return true 未送信結果がある場合。
 * @return false 試験中または結果送信済みの場合。
 */
bool chassisCtrlGainTuningResultReady();

/**
 * @brief 指定車輪の直近のゲイン調整結果を取得する。
 *
 * @param wheelIndex 車輪インデックス。0=FL、1=FR、2=RL、3=RR。
 * @return 指定車輪の集計結果。wheelIndexが不正な場合は全memberが0。
 */
ChassisGainTuningResult chassisCtrlGetGainTuningResult(int wheelIndex);

/**
 * @brief ゲイン調整結果を送信済みとしてマークする。
 */
void chassisCtrlClearGainTuningResultReady();

/**
 * @brief 現在の走行出力率を取得する。
 *
 * @return 設定中の走行出力率[%]。
 */
int chassisCtrlGetPowerPercent();

/**
 * @brief いずれかの車輪に非ゼロの目標回転数が設定されているか確認する。
 *
 * @return 1台以上の車輪が動作対象の場合true。
 */
bool chassisCtrlIsActive();

/**
 * @brief 現在の前後方向コマンドを取得する。
 *
 * デッドゾーンと車体側の反転設定を反映した値を返す。
 *
 * @return 前後方向コマンド。
 */
float chassisCtrlGetLongitudinalCommand();
