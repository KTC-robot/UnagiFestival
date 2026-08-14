#pragma once

#include <cstdint>

namespace CanConfig_stepper_ctrl {

// ============================================================
// TB6600 x 2 + 17PM-F438CP06CA x 2
// PUL / DIR 共通配線仕様
//
// 実機テストで確認した配線:
//   ESP32 3V3 -> TB6600 x2 の PUL+ / DIR+
//   GPIO25    -> TB6600 x2 の PUL-
//   GPIO26    -> TB6600 x2 の DIR-
//   ENA+ / ENA- は未接続
//
// 2台は常に同じ速度・同じ方向・同じタイミングで動作する。
// ============================================================
constexpr int STEPPER_PUL_PIN = 25;
constexpr int STEPPER_DIR_PIN = 26;

// 貼り付けてもらった共通ピン単体テストと同じ初期速度。
// TB6600を200 pulse/rev設定にしている場合、理論上約90 rpm。
constexpr uint32_t STEPPER_PULSE_HZ_DEFAULT = 300;
constexpr uint32_t STEPPER_PULSE_HZ_MIN = 100;
constexpr uint32_t STEPPER_PULSE_HZ_MAX = 2500;

// DIR切替後、最初のPULを出すまでの待ち時間。
constexpr uint32_t STEPPER_DIR_SETUP_US = 100;

}  // namespace CanConfig_stepper_ctrl
