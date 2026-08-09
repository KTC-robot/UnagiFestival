#pragma once

#include <Arduino.h>

// ============================================================
// 3 x VL53L0X + 2 x air cylinder configuration
//
// Sensor layout (all sensors point toward the floor):
//   robot front -> FRONT, CENTER, REAR -> robot rear
//
// Valve behavior required by this robot:
//   Valve ON  -> cylinder extends  -> mechanism goes DOWN
//   Valve OFF -> cylinder retracts -> mechanism goes UP
//
// IMPORTANT:
//   The 24 V solenoid valves must be driven through MOSFET/driver circuits.
//   Never connect a solenoid coil directly to an ESP32 GPIO.
// ============================================================


// Shared I2C bus with PCA9685
constexpr int STEP_AIR_I2C_SDA_PIN = 21;
constexpr int STEP_AIR_I2C_SCL_PIN = 22;


// XSHUT pins used to assign a different I2C address to each VL53L0X
constexpr int STEP_AIR_FRONT_XSHUT_PIN = 25;
constexpr int STEP_AIR_CENTER_XSHUT_PIN = 26;
constexpr int STEP_AIR_REAR_XSHUT_PIN = 27;


// MOSFET/solenoid driver inputs
constexpr int STEP_AIR_FRONT_VALVE_PIN = 23;
constexpr int STEP_AIR_REAR_VALVE_PIN = 32;


// HIGH means the corresponding valve is energized.
// Change to LOW only when using an active-low driver module.
constexpr uint8_t STEP_AIR_VALVE_ON_LEVEL = HIGH;
constexpr uint8_t STEP_AIR_VALVE_OFF_LEVEL = LOW;


// VL53L0X addresses assigned at each ESP32 boot
constexpr uint8_t STEP_AIR_FRONT_SENSOR_ADDRESS = 0x30;
constexpr uint8_t STEP_AIR_CENTER_SENSOR_ADDRESS = 0x31;
constexpr uint8_t STEP_AIR_REAR_SENSOR_ADDRESS = 0x32;


// ============================================================
// 現在接続しているセンサー
// false のセンサーはXSHUTをLOWのままにして、初期化・再初期化を
// 一切行わない。そのため未接続センサーが原因のI2C Error 263を防ぐ。
// ============================================================
constexpr bool STEP_AIR_USE_FRONT_SENSOR = true;
constexpr bool STEP_AIR_USE_CENTER_SENSOR = false;
constexpr bool STEP_AIR_USE_REAR_SENSOR = false;


// ============================================================
// 段差の自動エア制御
//
// 現在は2センサーの測距確認中なので false。
// falseでも、接続済みセンサーの距離取得・Serial表示・PiへのAIR送信は行う。
// 3台すべて取り付けて動作確認が終わったら true に変更する。
// ============================================================
constexpr bool STEP_AIR_ENABLE_AUTO_CONTROL = false;


// ============================================================
// Sensor measurement settings
// ============================================================

// 単体テストで安定していた条件に合わせて100 kHzで使用する。
// step_air_ctrl.cpp側でWire.setClock()に使用。
constexpr uint32_t STEP_AIR_I2C_CLOCK_HZ = 50000UL;

// Adafruit版では現在この値を直接使っていないが、本番調整用に残す。
constexpr uint32_t STEP_AIR_SENSOR_TIMING_BUDGET_US = 20000;

// 1回のloopで1台ずつ読む。
// 2台接続時は各センサーがおよそ400 msごとに更新される。
constexpr uint32_t STEP_AIR_SENSOR_PERIOD_MS = 200;

// 一瞬の読み取り失敗ですぐエラー扱いにしない。
constexpr uint32_t STEP_AIR_SENSOR_STALE_MS = 1500;

// 3秒以上有効値が取れない場合、そのセンサーだけ停止して再初期化対象にする。
constexpr uint32_t STEP_AIR_SENSOR_REINIT_AFTER_MS = 3000;

// 再初期化を連打しない。5秒に1回まで。
constexpr uint32_t STEP_AIR_SENSOR_REINIT_INTERVAL_MS = 5000;


// Accepted corrected measurement range
constexpr int STEP_AIR_SENSOR_MIN_VALID_MM = 20;
constexpr int STEP_AIR_SENSOR_MAX_VALID_MM = 1500;


// Installation-height correction for each sensor.
// corrected distance = measured distance + offset
// Adjust these after mounting the sensors.
constexpr int STEP_AIR_FRONT_SENSOR_OFFSET_MM = 0;
constexpr int STEP_AIR_CENTER_SENSOR_OFFSET_MM = 0;
constexpr int STEP_AIR_REAR_SENSOR_OFFSET_MM = 0;


// Low-pass filter: new filtered value =
// old * (denominator - numerator) / denominator
// + new * numerator / denominator
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


// While the front side is raised:
// abs(CENTER - FRONT) <= this value -> center has reached the
// upper surface, so the rear cylinder also retracts/up.
constexpr int STEP_AIR_CLIMB_CENTER_LEVEL_DIFF_MM = 25;


// Moving backward while both mechanisms are raised on the top:
// REAR - CENTER >= this value -> rear cylinder extends/down.
// 下り側は後センサーの実測後に調整する。
constexpr int STEP_AIR_DESCEND_REAR_DETECT_DIFF_MM = 70;


// While the rear side is lowered:
// abs(REAR - CENTER) <= this value -> center has reached the
// lower surface, so the front cylinder also extends/down.
constexpr int STEP_AIR_DESCEND_CENTER_LEVEL_DIFF_MM = 25;


// Number of consecutive control evaluations required before changing state
constexpr uint8_t STEP_AIR_CONFIRM_COUNT = 3;
constexpr uint8_t STEP_AIR_RECOVERY_CONFIRM_COUNT = 5;


// Ignore very small forward/backward commands.
constexpr float STEP_AIR_MOTION_DIRECTION_MIN = 0.15f;


// chassisCtrlGetLongitudinalCommand() should be positive during forward travel.
// Change this to -1.0f if actual forward/backward detection is reversed.
constexpr float STEP_AIR_FORWARD_DIRECTION_SIGN = 1.0f;


// Prevent two state transitions from occurring almost simultaneously.
constexpr uint32_t STEP_AIR_STATE_MIN_HOLD_MS = 200;


// Serial-monitor diagnostics interval
constexpr uint32_t STEP_AIR_DEBUG_PRINT_INTERVAL_MS = 250;


// On startup or sensor failure, both valves are switched OFF so both
// cylinders retract and both mechanisms move UP.
constexpr bool STEP_AIR_FAIL_SAFE_VALVES_OFF = true;