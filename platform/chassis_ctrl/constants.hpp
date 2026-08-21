#pragma once

/**
 * @file constants.hpp
 * @brief Chassisの座標補正、制御上限、車輪とmotorの対応を定義する。
 */

#include <cstdint>

constexpr bool VX_INVERT = true;  ///< 車体取付方向に合わせて前後成分を反転する。
constexpr bool WZ_INVERT = true;  ///< 車体取付方向に合わせて旋回成分を反転する。

constexpr int16_t MAX_CURRENT_COMMAND = 3000;  ///< C620へ渡す電流指令の絶対上限。
constexpr int16_t CHASSIS_MAX_RPM = 8000;      ///< power 100%時の最大目標RPM。
constexpr float CHASSIS_DEADZONE = 0.08f;      ///< 正規化後の微小入力を0にする範囲。

constexpr bool ENABLE_MIN_RUN_RPM = false;
constexpr int16_t MIN_RUN_RPM = 1200;
constexpr int16_t DEAD_RPM = 200;

constexpr float SPEED_KP = 1.00f;  ///< RPM誤差へ掛ける比例gain。
constexpr float SPEED_KI = 0.20f;  ///< RPM誤差の積分値へ掛けるgain。
constexpr float PID_INTEGRAL_LIMIT = 8000.0f;  ///< 積分値の絶対上限。
constexpr float TARGET_RPM_SLEW_PER_SEC = 6000.0f;  ///< 目標RPMの秒間最大変化量。

constexpr bool ENABLE_GAIN_TUNING_LOG = false;
constexpr uint32_t GAIN_TUNING_LOG_INTERVAL_MS = 100;
constexpr uint32_t GAIN_TUNING_MAX_DURATION_MS = 10000;
constexpr float GAIN_TUNING_RAMP_TOLERANCE_RPM = 50.0f;

constexpr uint32_t MOTOR_CONTROL_INTERVAL_US = 5000;

constexpr int DRIVE_POWER_PERCENT = 80;

namespace CanConfig_chassis_ctrl {
constexpr int NUM_MOTORS = 4;
constexpr int NUM_WHEELS = 4;

// wheel配列はFL, FR, RL, RRの順、motor配列はID1, ID2, ID3, ID4の順。
// 配線順が異なるため、feedback参照時だけこの対応表でmotor indexへ変換する。
const uint8_t WHEEL_TO_MOTOR[NUM_WHEELS] = {0, 2, 1, 3};
const uint8_t WHEEL_ESC_ID[NUM_WHEELS] = {1, 3, 2, 4};
const char* const WHEEL_NAME[NUM_WHEELS] = {"FL", "FR", "RL", "RR"};

// C620の物理取付方向を車輪座標へ揃えるための符号反転設定。
const bool MOTOR_REVERSED[NUM_MOTORS] = {
  true,
  true,
  false,
  false
};

// 前後・横・旋回の各成分が4輪へ寄与する符号。
const int8_t FWD_SIGN[NUM_WHEELS] = {+1, +1, +1, +1};
const int8_t STR_SIGN[NUM_WHEELS] = {+1, -1, -1, +1};
const int8_t YAW_SIGN[NUM_WHEELS] = {-1, +1, -1, +1};

constexpr float DEFAULT_WHEEL_GAIN_FWD[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

constexpr float DEFAULT_WHEEL_GAIN_BWD[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

constexpr float DEFAULT_WHEEL_GAIN_RIGHT[NUM_WHEELS] = {
  1.200f, 1.200f, 1.000f, 1.000f
};

constexpr float DEFAULT_WHEEL_GAIN_LEFT[NUM_WHEELS] = {
  1.200f, 1.200f, 1.000f, 1.000f
};

}
