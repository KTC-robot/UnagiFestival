#pragma once

#include <Arduino.h>

// ============================================================
// 3 x VL53L0X + 2 x air cylinder configuration
//
// センサー配置（全センサーは床方向）:
//   robot front -> FRONT, CENTER, REAR -> robot rear
//
// この機体で必要なバルブ動作:
//   Valve ON  -> シリンダー伸長 -> 機構DOWN
//   Valve OFF -> シリンダー収縮 -> 機構UP
//
// 重要:
//   24 V電磁弁は必ずMOSFET/ドライバ回路経由で駆動する。
//   電磁弁コイルをESP32 GPIOへ直接接続しない。
// ============================================================


// PCA9685とVL53L0Xで共有するI2Cバス。
constexpr int STEP_AIR_I2C_SDA_PIN = 21;
constexpr int STEP_AIR_I2C_SCL_PIN = 22;

// 電磁弁動作時のノイズ影響を確認するため、
// 単体テストで安定していた50 kHzまでI2Cクロックを下げる。
constexpr uint32_t STEP_AIR_I2C_CLOCK_HZ = 50000UL;

// 電磁弁動作直後のI2Cハングを早めに検出し、stale判定へ渡す。
constexpr uint32_t STEP_AIR_I2C_TIMEOUT_MS = 50;

// VL53L0XのI2Cアドレス割り当てで使用するXSHUTピン。
constexpr int STEP_AIR_FRONT_XSHUT_PIN = 25;
constexpr int STEP_AIR_CENTER_XSHUT_PIN = 26;
constexpr int STEP_AIR_REAR_XSHUT_PIN = 27;

// 現在は電磁弁動作直後のTIMEOUT原因がXSHUT配線ノイズかを切り分けるため、
// FRONT 1台だけを0x29のまま使い、ESP32からXSHUT GPIOを一切操作しない。
// 将来3台構成へ戻す場合、VL53L0Xは全台が起動時0x29で衝突するため、
// XSHUTで1台ずつ起動してFRONT/CENTER/REARへ別アドレスを割り当てる必要がある。
constexpr bool STEP_AIR_USE_XSHUT = false;

// MOSFET/電磁弁ドライバ入力。
constexpr int STEP_AIR_FRONT_VALVE_PIN = 23;
constexpr int STEP_AIR_REAR_VALVE_PIN = 32;


// HIGHで対応する電磁弁を励磁する。
// アクティブLOWのドライバを使う場合だけLOWへ変更する。
constexpr uint8_t STEP_AIR_VALVE_ON_LEVEL = HIGH;
constexpr uint8_t STEP_AIR_VALVE_OFF_LEVEL = LOW;


// XSHUTなしの単体診断ではFRONTをVL53L0Xの初期アドレス0x29で使う。
// XSHUTを有効化した3台構成では、起動後に0x30/0x31/0x32へ分離する。
constexpr uint8_t STEP_AIR_FRONT_SENSOR_ADDRESS =
  STEP_AIR_USE_XSHUT ? 0x30 : 0x29;
constexpr uint8_t STEP_AIR_CENTER_SENSOR_ADDRESS = 0x31;
constexpr uint8_t STEP_AIR_REAR_SENSOR_ADDRESS = 0x32;


// ============================================================
// 現在接続しているセンサー
// 現在はFRONT 1台だけを使う診断構成。
// XSHUT無効時は未接続センサーのXSHUTも含め、ESP32からGPIO操作しない。
// ============================================================
constexpr bool STEP_AIR_USE_FRONT_SENSOR = true;
constexpr bool STEP_AIR_USE_CENTER_SENSOR = false;
constexpr bool STEP_AIR_USE_REAR_SENSOR = false;

static_assert(
  STEP_AIR_USE_XSHUT ||
  (
    STEP_AIR_USE_FRONT_SENSOR &&
    !STEP_AIR_USE_CENTER_SENSOR &&
    !STEP_AIR_USE_REAR_SENSOR
  ),
  "Multiple VL53L0X sensors require XSHUT address assignment"
);


// ============================================================
// 段差の自動エア制御
//
// 現在はFRONT 1台・XSHUTなしの測距診断中なので false。
// falseでも、接続済みセンサーの距離取得・Serial表示・PiへのAIR送信は行う。
// 3台すべて取り付けて動作確認が終わったら true に変更する。
// ============================================================
constexpr bool STEP_AIR_ENABLE_AUTO_CONTROL = false;


// ============================================================
// センサー測距設定
// ============================================================

// Adafruit版では現在この値を直接使っていないが、本番調整用に残す。
constexpr uint32_t STEP_AIR_SENSOR_TIMING_BUDGET_US = 20000;

// 1回のloopで1台ずつ読む。現在のFRONT単体診断では約200 msごとに更新される。
constexpr uint32_t STEP_AIR_SENSOR_PERIOD_MS = 200;

// 一瞬の読み取り失敗ですぐエラー扱いにしない。
constexpr uint32_t STEP_AIR_SENSOR_STALE_MS = 1500;

// 3秒以上有効値が取れない場合、そのセンサーだけ停止して再初期化対象にする。
constexpr uint32_t STEP_AIR_SENSOR_REINIT_AFTER_MS = 3000;

// 再初期化を連打しない。5秒に1回まで。
constexpr uint32_t STEP_AIR_SENSOR_REINIT_INTERVAL_MS = 5000;


// 補正後の測距値として採用する範囲。
constexpr int STEP_AIR_SENSOR_MIN_VALID_MM = 20;
constexpr int STEP_AIR_SENSOR_MAX_VALID_MM = 1500;


// センサー取付高さの補正値。
// corrected distance = measured distance + offset
// 取付後の実測に合わせて調整する。
constexpr int STEP_AIR_FRONT_SENSOR_OFFSET_MM = 0;
constexpr int STEP_AIR_CENTER_SENSOR_OFFSET_MM = 0;
constexpr int STEP_AIR_REAR_SENSOR_OFFSET_MM = 0;


// ローパスフィルタ:
// new filtered value =
//   old * (denominator - numerator) / denominator
//   + new * numerator / denominator
constexpr int STEP_AIR_FILTER_NEW_WEIGHT_NUMERATOR = 1;
constexpr int STEP_AIR_FILTER_WEIGHT_DENOMINATOR = 4;


// ------------------------------------------------------------
// Thresholds to tune later
// ------------------------------------------------------------

// 実測:
//   通常床      約200～220 mm
//   10 cm段差上 約100～120 mm
//
// まずは差70 mmを目安にする。
// 必要なら実走行で調整する。
constexpr int STEP_AIR_CLIMB_FRONT_DETECT_DIFF_MM = 70;


// 前側が上がっている間:
// abs(CENTER - FRONT) がこの値以下ならCENTERも上面に到達したとみなし、
// 後シリンダーも収縮して後機構をUPする。
constexpr int STEP_AIR_CLIMB_CENTER_LEVEL_DIFF_MM = 25;


// 段差上で前後機構UPのまま後退中:
// REAR - CENTER がこの値以上なら後シリンダーを伸長して後機構をDOWNする。
// 下り側は後センサーの実測後に調整する。
constexpr int STEP_AIR_DESCEND_REAR_DETECT_DIFF_MM = 70;


// 後側が下がっている間:
// abs(REAR - CENTER) がこの値以下ならCENTERも下面に到達したとみなし、
// 前シリンダーも伸長して前機構をDOWNする。
constexpr int STEP_AIR_DESCEND_CENTER_LEVEL_DIFF_MM = 25;


// 状態遷移前に連続して必要な判定回数。
constexpr uint8_t STEP_AIR_CONFIRM_COUNT = 3;
constexpr uint8_t STEP_AIR_RECOVERY_CONFIRM_COUNT = 5;


// 微小な前後入力は停止扱いにする。
constexpr float STEP_AIR_MOTION_DIRECTION_MIN = 0.15f;


// chassisCtrlGetLongitudinalCommand() は前進時に正になる想定。
// 実機で前後判定が逆なら -1.0f に変更する。
constexpr float STEP_AIR_FORWARD_DIRECTION_SIGN = 1.0f;


// 連続した状態遷移を抑制する最小保持時間。
constexpr uint32_t STEP_AIR_STATE_MIN_HOLD_MS = 200;


// Serial Monitorへ測距状態を出す周期。
constexpr uint32_t STEP_AIR_DEBUG_PRINT_INTERVAL_MS = 250;


// 起動中またはセンサー異常時は両バルブOFFで両機構をUP側に倒す。
constexpr bool STEP_AIR_FAIL_SAFE_VALVES_OFF = true;
