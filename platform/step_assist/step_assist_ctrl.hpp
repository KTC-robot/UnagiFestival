#pragma once

/**
 * @brief StepAssistの操作モード。
 *
 * AUTOはLaserSensorによる既存state machine、MANUALはDPADによる
 * 前後補助輪の個別操作を行う。
 */
enum class StepAssistMode {
  AUTO,
  MANUAL,
};

/**
 * @file step_assist_ctrl.hpp
 * @brief 3台の距離センサーと前後補助輪による段差制御APIを提供する。
 */

/**
 * @brief 段差制御をNORMAL状態で初期化する。
 *
 * @return 初期化成功時true。
 */
bool stepAssistCtrlBegin();

/**
 * @brief 段差制御をNORMAL状態へ戻す。
 *
 * 前後補助輪の出力、走行速度係数、phase経過時間もNORMAL用に初期化する。
 * chassisの停止処理は行わない。
 */
void stepAssistCtrlReset();

/**
 * @brief 最新の距離に基づき段差制御を1回更新する。
 */
void stepAssistCtrlUpdate();

/** @brief 現在のAUTO/MANUALモードを取得する。 */
StepAssistMode stepAssistCtrlGetMode();

/**
 * @brief AUTOとMANUALを切り替える。
 *
 * MANUAL移行時は補助輪位置を維持し、AUTO復帰時はNORMALへresetする。
 */
void stepAssistCtrlToggleMode();

/**
 * @brief MANUAL時だけ前補助輪のUP/DOWNを切り替える。
 *
 * AUTOではLaserSensorによるstate machineを保護するため何もしない。
 */
void stepAssistCtrlToggleManualFront();

/**
 * @brief MANUAL時だけ後補助輪のUP/DOWNを切り替える。
 *
 * AUTOではLaserSensorによるstate machineを保護するため何もしない。
 */
void stepAssistCtrlToggleManualRear();
