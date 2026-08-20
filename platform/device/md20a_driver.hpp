#pragma once

#include <cstdint>

/**
 * @file md20a_driver.hpp
 * @brief ラックレール押し出し用775モーターをMD20A経由で制御する。
 *
 * このdriverは速度制御を行わず、正転・逆転・停止の3状態だけを管理する。
 */

enum class Md20aState : uint8_t {
  STOPPED,  ///< モーターを停止している状態。
  FORWARD,  ///< ラックレールを押し出す向きへ回転している状態。
  REVERSE,  ///< ラックレールを戻す向きへ回転している状態。
};

/**
 * @brief MD20AのGPIOを初期化し、起動時の状態を停止へ設定する。
 */
void md20aDriverBegin();

/**
 * @brief MD20Aを指定した回転状態へ切り替える。
 *
 * @param state 設定する状態。正転・逆転・停止のいずれか。
 */
void md20aDriverSetState(Md20aState state);

/**
 * @brief 現在MD20Aへ設定している状態を取得する。
 *
 * @return 現在の正転・逆転・停止状態。
 */
Md20aState md20aDriverGetState();
