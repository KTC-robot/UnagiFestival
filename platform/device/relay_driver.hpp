#pragma once

/**
 * @file relay_driver.hpp
 * @brief 前後およびAir用GPIOリレーを操作するDriver APIを提供する。
 */

/**
 * @brief リレーGPIOを初期化し、両電磁弁を安全側OFFにする。
 *
 * @return 初期化が完了した場合true。
 */
bool relayDriverBegin();

/**
 * @brief 前側電磁弁のリレー状態を変更する。
 *
 * @param on trueの場合は電磁弁を励磁する。
 */
void relayDriverSetFront(bool on);

/**
 * @brief 後側電磁弁のリレー状態を変更する。
 *
 * @param on trueの場合は電磁弁を励磁する。
 */
void relayDriverSetRear(bool on);

/**
 * @brief 前側電磁弁がONか確認する。
 *
 * @return ONの場合true。
 */
bool relayDriverFrontOn();

/**
 * @brief 後側電磁弁がONか確認する。
 *
 * @return ONの場合true。
 */
bool relayDriverRearOn();

/** @brief Air Cylinder用リレー状態を変更する。 */
void relayDriverSetAir(bool on);

/** @brief Air Cylinder用リレーがONか確認する。 */
bool relayDriverAirOn();

/**
 * @brief 全電磁弁を安全側OFFへ強制する。
 */
void relayDriverForceOff();
