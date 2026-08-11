#pragma once

#include <cstdint>

constexpr bool VX_INVERT = true;
constexpr bool WZ_INVERT = true;

constexpr int16_t MAX_CURRENT_COMMAND = 3000;
constexpr int16_t CHASSIS_MAX_RPM = 8000;
constexpr float CHASSIS_DEADZONE = 0.08f;

constexpr bool ENABLE_MIN_RUN_RPM = false;
constexpr int16_t MIN_RUN_RPM = 1200;
constexpr int16_t DEAD_RPM = 200;

constexpr float SPEED_KP = 1.00f;
constexpr float SPEED_KI = 0.20f;
constexpr float PID_INTEGRAL_LIMIT = 8000.0f;
constexpr float TARGET_RPM_SLEW_PER_SEC = 6000.0f;

constexpr uint32_t MOTOR_CONTROL_INTERVAL_US = 5000;

constexpr int DRIVE_POWER_MIN = 10;
constexpr int DRIVE_POWER_MAX = 80;
constexpr int DRIVE_POWER_STEP = 5;

namespace CanConfig_chassis_ctrl {
constexpr int NUM_MOTORS = 4;
constexpr int NUM_WHEELS = 4;

// wheel: FL, FR, RL, RR
// motor: ID1, ID2, ID3, ID4
const uint8_t WHEEL_TO_MOTOR[NUM_WHEELS] = {0, 2, 1, 3};
const uint8_t WHEEL_ESC_ID[NUM_WHEELS] = {1, 3, 2, 4};
const char* WHEEL_NAME[NUM_WHEELS] = {"FL", "FR", "RL", "RR"};

const bool MOTOR_REVERSED[NUM_MOTORS] = {
  false,
  false,
  true,
  true
};

const int8_t FWD_SIGN[NUM_WHEELS] = {+1, +1, +1, +1};
const int8_t STR_SIGN[NUM_WHEELS] = {+1, -1, -1, +1};
const int8_t YAW_SIGN[NUM_WHEELS] = {-1, +1, -1, +1};

const float WHEEL_GAIN_FWD[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

const float WHEEL_GAIN_BWD[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

const float WHEEL_GAIN_RIGHT[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

const float WHEEL_GAIN_LEFT[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

}