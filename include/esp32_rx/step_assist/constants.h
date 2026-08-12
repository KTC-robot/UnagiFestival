#pragma once

// 段差へ接近したと判定する距離。
constexpr int STEP_ASSIST_STEP_DETECT_THRESHOLD_MM = 120;

// 段差から離れたと判定する距離。
constexpr int STEP_ASSIST_DROP_DETECT_THRESHOLD_MM = 180;

// REARが段差へ接近した直後の誤判定を防ぐ猶予時間。
constexpr uint32_t STEP_ASSIST_REAR_DROP_GRACE_MS = 300;