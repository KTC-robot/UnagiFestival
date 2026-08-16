#pragma once

// 段差へ接近したと判定する距離。
constexpr int STEP_ASSIST_STEP_DETECT_THRESHOLD_MM = 120;

// 段差から離れたと判定する距離。
constexpr int STEP_ASSIST_DROP_DETECT_THRESHOLD_MM = 150;

// REARが段差へ接近した直後の誤判定を防ぐ猶予時間。
constexpr uint32_t STEP_ASSIST_REAR_DROP_GRACE_MS = 300;

// reset直後に補助輪・車体・センサーが安定するまでphase遷移を禁止する時間。
constexpr uint32_t STEP_ASSIST_RESET_GUARD_MS = 500;

// ============================================================
// Step assist drive scale
// ============================================================

// 通常走行
constexpr float STEP_ASSIST_NORMAL_FORWARD_SCALE = 1.00f;
constexpr float STEP_ASSIST_NORMAL_BACKWARD_SCALE = 1.00f;
constexpr float STEP_ASSIST_NORMAL_OTHER_SCALE = 1.00f;

// 前補助輪DOWN
constexpr float STEP_ASSIST_FRONT_LOWERED_FORWARD_SCALE = 0.50f;
constexpr float STEP_ASSIST_FRONT_LOWERED_BACKWARD_SCALE = 0.50f;
constexpr float STEP_ASSIST_FRONT_LOWERED_OTHER_SCALE = 0.50f;

// 前後補助輪DOWN
constexpr float STEP_ASSIST_BOTH_LOWERED_FORWARD_SCALE = 0.40f;
constexpr float STEP_ASSIST_BOTH_LOWERED_BACKWARD_SCALE = 0.40f;
constexpr float STEP_ASSIST_BOTH_LOWERED_OTHER_SCALE = 0.40f;

// 後センサーが段差を検出している状態
// 出力状態としては前後補助輪DOWNなので同じ0.40とする。
constexpr float STEP_ASSIST_REAR_SENSOR_LOWER_FORWARD_SCALE = 0.40f;
constexpr float STEP_ASSIST_REAR_SENSOR_LOWER_BACKWARD_SCALE = 0.40f;
constexpr float STEP_ASSIST_REAR_SENSOR_LOWER_OTHER_SCALE = 0.40f;

// 後補助輪UP
constexpr float STEP_ASSIST_REAR_RAISED_FORWARD_SCALE = 0.20f;
constexpr float STEP_ASSIST_REAR_RAISED_BACKWARD_SCALE = 0.20f;
constexpr float STEP_ASSIST_REAR_RAISED_OTHER_SCALE = 0.20f;    ///< 横移動・旋回速度係数。
