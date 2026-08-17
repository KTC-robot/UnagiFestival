#pragma once

#include <Arduino.h>

/**
 * @file c620_driver.hpp
 * @brief ESP32のTWAIを使用したC620向けCAN通信APIを提供する。
 */

/** @brief CAN通信で管理するモーター数。 */
constexpr int C620_MOTOR_COUNT = 4;

/**
 * @brief TWAIドライバーを1 Mbpsで初期化して開始する。
 *
 * @return ドライバーのインストールと開始に成功した場合true。
 */
bool c620DriverBegin();

/**
 * @brief CAN通信が開始済みか確認する。
 *
 * @return c620DriverBegin()が成功している場合true。
 */
bool c620DriverIsReady();

/**
 * @brief 受信キューにあるCANフレームを読み取り、C620のフィードバックを更新する。
 *
 * CAN通信が開始されていない場合は何も行わない。
 */
void c620DriverReadFrames();

/**
 * @brief 設定済みの電流指令値を所定周期でC620へ送信する。
 *
 * CAN通信が未開始の場合、または送信周期に達していない場合は送信しない。
 */
void c620DriverSendPeriodically();

/**
 * @brief 指定したモーターの電流指令値を内部状態へ設定する。
 *
 * この関数だけではCANフレームを即時送信しない。
 *
 * @param motorIndex モーターインデックス。範囲は0以上C620_MOTOR_COUNT未満。
 * @param command C620へ送信する電流指令値。無効なインデックスの場合は設定しない。
 */
void c620DriverSetCurrentCommand(int motorIndex, int16_t command);

/**
 * @brief 指定したモーターに設定されている電流指令値を取得する。
 *
 * @param motorIndex モーターインデックス。範囲は0以上C620_MOTOR_COUNT未満。
 * @return 電流指令値。無効なインデックスの場合は0。
 */
int16_t c620DriverGetCurrentCommand(int motorIndex);

/**
 * @brief 全モーターの電流指令値を0にしてCANフレームを即時送信する。
 *
 * CAN通信が開始されていない場合も内部の電流指令値は0へ変更する。
 */
void c620DriverZeroAllImmediate();

/**
 * @brief 指定したモーターのフィードバックが有効期間内か確認する。
 *
 * @param motorIndex モーターインデックス。範囲は0以上C620_MOTOR_COUNT未満。
 * @return フィードバック受信済みで、最終受信からタイムアウト時間以内の場合true。
 */
bool c620DriverFeedbackFresh(int motorIndex);

/**
 * @brief 指定したモーターの現在の回転数を取得する。
 *
 * @param motorIndex モーターインデックス。範囲は0以上C620_MOTOR_COUNT未満。
 * @return 最後に受信したモーター回転数[rpm]。無効なインデックスの場合は0。
 */
int16_t c620DriverGetMotorRpm(int motorIndex);

/**
 * @brief 指定したモーターの測定電流値を取得する。
 *
 * @param motorIndex モーターインデックス。範囲は0以上C620_MOTOR_COUNT未満。
 * @return 最後に受信したC620の測定電流値。無効なインデックスの場合は0。
 */
int16_t c620DriverGetMeasuredCurrent(int motorIndex);

/**
 * @brief 指定したモーターの温度を取得する。
 *
 * @param motorIndex モーターインデックス。範囲は0以上C620_MOTOR_COUNT未満。
 * @return 最後に受信したモーター温度[℃]。無効なインデックスの場合は0。
 */
uint8_t c620DriverGetMotorTemperature(int motorIndex);

/**
 * @brief 指定したモーターのローター角度を取得する。
 *
 * @param motorIndex モーターインデックス。範囲は0以上C620_MOTOR_COUNT未満。
 * @return 最後に受信したC620のローター角度値。無効なインデックスの場合は0。
 */
uint16_t c620DriverGetRotorAngle(int motorIndex);

/**
 * @brief freshなモーターフィードバックをビットマスクで取得する。
 *
 * @return ビットNがモーターインデックスNのfresh状態を表すビットマスク。
 */
uint8_t c620DriverGetFeedbackMask();

/**
 * @brief CAN電流指令フレームの送信成功回数を取得する。
 *
 * @return TWAI送信に成功した累積回数。
 */
uint32_t c620DriverGetTxCount();

/**
 * @brief 有効なC620フィードバックの受信回数を取得する。
 *
 * @return 認識したフィードバックフレームの累積回数。
 */
uint32_t c620DriverGetFeedbackCount();
