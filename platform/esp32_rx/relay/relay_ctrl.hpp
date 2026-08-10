#pragma once

/**
 * @brief リレーGPIOを初期化し、両電磁弁を安全側OFFにする。
 *
 * @return 初期化が完了した場合true。
 */
bool relayCtrlBegin();

/**
 * @brief 前側電磁弁のリレー状態を変更する。
 *
 * @param on trueの場合は電磁弁を励磁する。
 */
void relayCtrlSetFront(bool on);

/**
 * @brief 後側電磁弁のリレー状態を変更する。
 *
 * @param on trueの場合は電磁弁を励磁する。
 */
void relayCtrlSetRear(bool on);

/**
 * @brief 前側電磁弁がONか確認する。
 *
 * @return ONの場合true。
 */
bool relayCtrlFrontOn();

/**
 * @brief 後側電磁弁がONか確認する。
 *
 * @return ONの場合true。
 */
bool relayCtrlRearOn();

/**
 * @brief 両電磁弁を安全側OFFへ強制する。
 */
void relayCtrlForceOff();
