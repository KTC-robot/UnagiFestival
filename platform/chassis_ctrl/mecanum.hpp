#pragma once

#include "chassis_ctrl/constants.hpp"

/**
 * @brief Mecanum合成前の車輪別成分。
 *
 * wheel gainをchassis_ctrl側で既存順序のまま適用できるよう、
 * 前後・横移動・旋回の成分を分けて保持する。
 */
struct MecanumComponents {
  float forward[CanConfig_chassis_ctrl::NUM_WHEELS];  ///< 前後移動成分。
  float strafe[CanConfig_chassis_ctrl::NUM_WHEELS];   ///< 横移動成分。
  float yaw[CanConfig_chassis_ctrl::NUM_WHEELS];      ///< 旋回成分。
};

/**
 * @brief normalization後の車輪別Mecanum出力。
 */
struct MecanumOutput {
  float wheel[CanConfig_chassis_ctrl::NUM_WHEELS];  ///< 正規化済み出力。
};

/**
 * @brief 車体指令から車輪別のMecanum成分を計算する。
 *
 * @param vx 前後方向指令。
 * @param vy 横方向指令。
 * @param wz 旋回方向指令。
 * @return 車輪ごとの前後・横移動・旋回成分。
 */
MecanumComponents calculateMecanumComponents(float vx, float vy, float wz);

/**
 * @brief 車輪別成分を合成し、最大絶対値が1.0以下になるよう正規化する。
 *
 * @param components 合成対象の車輪別成分。
 * @return 正規化済みの4輪出力。
 */
MecanumOutput combineAndNormalizeMecanum(const MecanumComponents& components);
