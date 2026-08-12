#pragma once

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
 * @brief 最新の距離に基づき段差制御を1回更新する。
 */
void stepAssistCtrlUpdate();
