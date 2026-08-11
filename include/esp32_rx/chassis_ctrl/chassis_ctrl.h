#pragma once

#include <Arduino.h>

/**
 * @file chassis_ctrl.h
 * @brief メカナム足回りの目標生成と速度PI制御APIを提供する。
 */

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
 * @brief ジョイスティックまたは十字キー入力から各車輪の目標回転数を設定する。
 *
 * 十字キー入力がある場合はジョイスティック入力より優先する。値は内部で正規化、
 * デッドゾーン処理、方向反転、車輪ゲイン補正を受ける。
 *
 * @param lx 左スティックX軸の入力値。
 * @param ly 左スティックY軸の入力値。
 * @param rx 右スティックX軸の入力値。
 * @param dpadX 十字キーX軸の入力値。
 * @param dpadY 十字キーY軸の入力値。
 */
void chassisCtrlSetFromJoy(
  int8_t lx,
  int8_t ly,
  int8_t rx,
  int8_t dpadX,
  int8_t dpadY
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
