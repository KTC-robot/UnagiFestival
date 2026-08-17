#include "chassis_ctrl/chassis_ctrl.hpp"

#include "device/c620_driver.hpp"
#include "chassis_ctrl/constants.hpp"
#include "chassis_ctrl/mecanum.hpp"
#include "chassis_ctrl/speed_controller.hpp"

#include <math.h>

using namespace CanConfig_chassis_ctrl;

namespace {
constexpr int COMMAND_MAX = 127;
constexpr int COMMAND_DEAD = 8;

float commandToFloat(int value) {
  if (abs(value) < COMMAND_DEAD) value = 0;
  value = constrain(value, -COMMAND_MAX, COMMAND_MAX);
  return static_cast<float>(value) / static_cast<float>(COMMAND_MAX);
}

int drivePowerPercent = 80;
bool motorsActive = false;
float longitudinalCommand = 0.0f;

float requestedMotorRpm[NUM_MOTORS] = {};
float rampedMotorRpm[NUM_MOTORS] = {};
SpeedController speedControllers[NUM_MOTORS] = {};
float wheelGainFwd[NUM_WHEELS] = {};
float wheelGainBwd[NUM_WHEELS] = {};
float wheelGainRight[NUM_WHEELS] = {};
float wheelGainLeft[NUM_WHEELS] = {};
float driveScaleForward = 1.0f;
float driveScaleBackward = 1.0f;
float driveScaleOther = 1.0f;
bool forwardBlocked = false;
int8_t lastDriveVx = 0;
int8_t lastDriveVy = 0;
int8_t lastDriveWz = 0;

struct GainTuningAccumulator {
  uint32_t sampleCount;
  double absoluteRpmSum;
  double absoluteRpmSquaredSum;
};

GainTuningAccumulator tuningStats[NUM_MOTORS] = {};
ChassisGainTuningResult tuningResults[NUM_MOTORS] = {};
bool tuningActive = false;
bool tuningResultReady = false;
bool tuningSamplingStarted = false;
uint32_t tuningStartedMs = 0;
uint32_t tuningDurationMs = 0;
uint32_t lastTuningLogMs = 0;

uint32_t lastMotorControlUs = 0;
uint32_t lastMotorPrintMs = 0;

void clearTuningStats() {
  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    tuningStats[wheelIndex] = {};
    tuningResults[wheelIndex] = {};
  }
}

/**
 * @brief 4輪すべての目標RPMランプが要求値へ収束したか確認する。
 *
 * @return true 全車輪が許容差以内の場合。
 */
bool allTuningTargetsSettled() {
  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
    if (fabsf(rampedMotorRpm[motorIndex] - requestedMotorRpm[motorIndex]) >
        GAIN_TUNING_RAMP_TOLERANCE_RPM) {
      return false;
    }
  }
  return true;
}

void finishGainTuning() {
  tuningActive = false;
  chassisCtrlStop();

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const GainTuningAccumulator& stats = tuningStats[wheelIndex];
    ChassisGainTuningResult& result = tuningResults[wheelIndex];

    result.sampleCount = stats.sampleCount;
    if (stats.sampleCount > 0) {
      result.meanAbsoluteRpm = static_cast<float>(
        stats.absoluteRpmSum / static_cast<double>(stats.sampleCount)
      );
      const double variance = max(
        0.0,
        stats.absoluteRpmSquaredSum / static_cast<double>(stats.sampleCount) -
          static_cast<double>(result.meanAbsoluteRpm) *
          static_cast<double>(result.meanAbsoluteRpm)
      );
      result.standardDeviationRpm = static_cast<float>(sqrt(variance));
    }
  }

  tuningResultReady = true;
  Serial.println("GAIN TUNING DONE -> STOP");
}

float applyMotorInverse(int motorIndex, float value) {
  return MOTOR_REVERSED[motorIndex] ? -value : value;
}

int16_t snapTargetRpm(int16_t target) {
  if (!ENABLE_MIN_RUN_RPM) {
    return target;
  }

  const int magnitude = abs(static_cast<int>(target));

  if (magnitude < DEAD_RPM) {
    return 0;
  }

  if (magnitude < MIN_RUN_RPM) {
    return target >= 0 ? MIN_RUN_RPM : -MIN_RUN_RPM;
  }

  return target;
}

float moveToward(float current, float target, float maxChange) {
  const float difference = target - current;

  if (difference > maxChange) {
    return current + maxChange;
  }

  if (difference < -maxChange) {
    return current - maxChange;
  }

  return target;
}

float getWheelTargetRpm(int wheelIndex) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  return applyMotorInverse(motorIndex, requestedMotorRpm[motorIndex]);
}

float getWheelMeasuredRpm(int wheelIndex) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

  return applyMotorInverse(
    motorIndex,
    static_cast<float>(c620DriverGetMotorRpm(motorIndex))
  );
}

int16_t getWheelCurrentCommand(int wheelIndex) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

  return static_cast<int16_t>(
    applyMotorInverse(
      motorIndex,
      static_cast<float>(c620DriverGetCurrentCommand(motorIndex))
    )
  );
}

void printMotorValues(float vx, float vy, float wz) {
  const uint32_t now = millis();

  if (now - lastMotorPrintMs < 200) {
    return;
  }

  lastMotorPrintMs = now;

  Serial.print("MECANUM VX=");
  Serial.print(vx, 2);
  Serial.print(" VY=");
  Serial.print(vy, 2);
  Serial.print(" WZ=");
  Serial.print(wz, 2);
  Serial.print(" PWR=");
  Serial.print(drivePowerPercent);
  Serial.print("% | ");

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

    Serial.print(WHEEL_NAME[wheelIndex]);
    Serial.print("(ID");
    Serial.print(WHEEL_ESC_ID[wheelIndex]);
    Serial.print(") T=");
    Serial.print(static_cast<int>(getWheelTargetRpm(wheelIndex)));
    Serial.print(" A=");
    Serial.print(static_cast<int>(getWheelMeasuredRpm(wheelIndex)));
    Serial.print(" I=");
    Serial.print(static_cast<int>(getWheelCurrentCommand(wheelIndex)));
    Serial.print(" C=");
    Serial.print(static_cast<int>(c620DriverGetMotorTemperature(motorIndex)));

    if (wheelIndex < NUM_WHEELS - 1) {
      Serial.print(" | ");
    }
  }

  Serial.println();
}

void setChassisSpringLogic(float vx, float vy, float wz) {
  vx = constrain(vx, -1.0f, 1.0f);
  vy = constrain(vy, -1.0f, 1.0f);
  wz = constrain(wz, -1.0f, 1.0f);

  if (fabsf(vx) < CHASSIS_DEADZONE) vx = 0.0f;
  if (fabsf(vy) < CHASSIS_DEADZONE) vy = 0.0f;
  if (fabsf(wz) < CHASSIS_DEADZONE) wz = 0.0f;

  // 前進禁止はユーザー指令基準の正方向vxだけへ適用する。
  // gain tuning、後退、横移動、旋回には影響させない。
  if (!tuningActive && forwardBlocked && vx > 0.0f) {
    vx = 0.0f;
  }

  // gainの方向はユーザー指令基準で判定する。VX_INVERTは車体座標系の
  // 補正なので、forward/backwardのgain選択には反転前のvxを使用する。
  const float commandDirectionVx = vx;

  if (VX_INVERT) vx = -vx;
  if (WZ_INVERT) wz = -wz;

  longitudinalCommand = vx;

  if (vx == 0.0f && vy == 0.0f && wz == 0.0f) {
    chassisCtrlStop();
    return;
  }

  MecanumComponents mecanum = calculateMecanumComponents(vx, vy, wz);

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    if (commandDirectionVx > 0.0f) {
      mecanum.forward[wheelIndex] *= wheelGainFwd[wheelIndex];
    } else if (commandDirectionVx < 0.0f) {
      mecanum.forward[wheelIndex] *= wheelGainBwd[wheelIndex];
    }

    if (vy > 0.0f) {
      mecanum.strafe[wheelIndex] *= wheelGainRight[wheelIndex];
    } else if (vy < 0.0f) {
      mecanum.strafe[wheelIndex] *= wheelGainLeft[wheelIndex];
    }
  }

  const MecanumOutput mecanumOutput = combineAndNormalizeMecanum(mecanum);

  float selectedDriveScale = driveScaleOther;
  if (commandDirectionVx > 0.0f) {
    selectedDriveScale = driveScaleForward;
  } else if (commandDirectionVx < 0.0f) {
    selectedDriveScale = driveScaleBackward;
  }

  // gain tuningではstep assist状態に依存しない同一条件でRPMを比較する。
  if (tuningActive) {
    selectedDriveScale = 1.0f;
  }

  const float maxRpm =
    static_cast<float>(CHASSIS_MAX_RPM) *
    (static_cast<float>(drivePowerPercent) / 100.0f) *
    selectedDriveScale;

  bool anyWheelActive = false;

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int16_t wheelRpm =
      static_cast<int16_t>(lroundf(mecanumOutput.wheel[wheelIndex] * maxRpm));

    const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

    const int16_t rawMotorTarget =
      static_cast<int16_t>(
        applyMotorInverse(motorIndex, static_cast<float>(wheelRpm))
      );

    requestedMotorRpm[motorIndex] = snapTargetRpm(rawMotorTarget);

    if (requestedMotorRpm[motorIndex] != 0.0f) {
      anyWheelActive = true;
    }
  }

  motorsActive = anyWheelActive;
  printMotorValues(vx, vy, wz);
}
}

void chassisCtrlBegin() {
  lastMotorControlUs = micros();

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    wheelGainFwd[wheelIndex] = DEFAULT_WHEEL_GAIN_FWD[wheelIndex];
    wheelGainBwd[wheelIndex] = DEFAULT_WHEEL_GAIN_BWD[wheelIndex];
    wheelGainRight[wheelIndex] = DEFAULT_WHEEL_GAIN_RIGHT[wheelIndex];
    wheelGainLeft[wheelIndex] = DEFAULT_WHEEL_GAIN_LEFT[wheelIndex];
  }

  Serial.print("Chassis ready: Kp=");
  Serial.print(SPEED_KP, 2);
  Serial.print(" Ki=");
  Serial.print(SPEED_KI, 2);
  Serial.print(" current_limit=+/-");
  Serial.println(MAX_CURRENT_COMMAND);
}

void chassisCtrlUpdate() {
  const uint32_t nowMs = millis();
  if (tuningActive && nowMs - tuningStartedMs >= tuningDurationMs) {
    finishGainTuning();
    return;
  }

  if (!c620DriverIsReady()) {
    return;
  }

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastMotorControlUs;

  if (elapsedUs < MOTOR_CONTROL_INTERVAL_US) {
    return;
  }

  lastMotorControlUs = nowUs;

  float dt = static_cast<float>(elapsedUs) / 1000000.0f;

  if (dt <= 0.0f || dt > 0.05f) {
    dt = static_cast<float>(MOTOR_CONTROL_INTERVAL_US) / 1000000.0f;
  }

  const float maxRpmChange = TARGET_RPM_SLEW_PER_SEC * dt;

  const bool printTuningLog =
    tuningActive && ENABLE_GAIN_TUNING_LOG &&
    nowMs - lastTuningLogMs >= GAIN_TUNING_LOG_INTERVAL_MS;

  // 4輪すべてのtarget rampが収束した次の制御周期から同時に集計する。
  // wheelごとに開始時刻がずれると、RPM比較の測定区間が揃わないため。
  if (tuningActive && !tuningSamplingStarted && allTuningTargetsSettled()) {
    tuningSamplingStarted = true;
  }

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    rampedMotorRpm[motorIndex] = moveToward(
      rampedMotorRpm[motorIndex],
      requestedMotorRpm[motorIndex],
      maxRpmChange
    );

    if (!c620DriverFeedbackFresh(motorIndex)) {
      c620DriverSetCurrentCommand(motorIndex, 0);
      speedControllers[motorIndex].reset();
      continue;
    }

    if (fabsf(rampedMotorRpm[motorIndex]) < 1.0f) {
      c620DriverSetCurrentCommand(motorIndex, 0);
      speedControllers[motorIndex].reset();
      continue;
    }

    const SpeedControllerResult speedResult =
      speedControllers[motorIndex].update(
        rampedMotorRpm[motorIndex],
        static_cast<float>(c620DriverGetMotorRpm(motorIndex)),
        dt
      );

    if (tuningActive && tuningSamplingStarted) {
      for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
        if (WHEEL_TO_MOTOR[wheelIndex] != motorIndex) {
          continue;
        }
        GainTuningAccumulator& stats = tuningStats[wheelIndex];
        const float absoluteRpm = fabsf(
          static_cast<float>(c620DriverGetMotorRpm(motorIndex))
        );
        ++stats.sampleCount;
        stats.absoluteRpmSum += static_cast<double>(absoluteRpm);
        stats.absoluteRpmSquaredSum +=
          static_cast<double>(absoluteRpm) * static_cast<double>(absoluteRpm);
        break;
      }

      if (printTuningLog) {
        Serial.print("GAIN M");
        Serial.print(motorIndex + 1);
        Serial.print(" KP=");
        Serial.print(SPEED_KP, 3);
        Serial.print(" KI=");
        Serial.print(SPEED_KI, 3);
        Serial.print(" TARGET=");
        Serial.print(rampedMotorRpm[motorIndex], 0);
        Serial.print(" ACTUAL=");
        Serial.print(c620DriverGetMotorRpm(motorIndex));
        Serial.print(" ERROR=");
        Serial.print(speedResult.error, 0);
        Serial.print(" P=");
        Serial.print(SPEED_KP * speedResult.error, 0);
        Serial.print(" I=");
        Serial.print(SPEED_KI * speedResult.integral, 0);
        Serial.print(" OUT=");
        Serial.print(speedResult.output, 0);
        Serial.print(" SAT=");
        Serial.println(
          fabsf(speedResult.output) >=
            static_cast<float>(MAX_CURRENT_COMMAND) ? 1 : 0
        );
      }
    }

    c620DriverSetCurrentCommand(
      motorIndex,
      speedResult.currentCommand
    );
  }

  if (printTuningLog) {
    lastTuningLogMs = nowMs;
  }
}

void chassisCtrlSetDriveCommand(
  int8_t vx,
  int8_t vy,
  int8_t wz
) {
  // phase変更時に次のPi packetを待たず再計算できるよう、生の指令を保持する。
  lastDriveVx = vx;
  lastDriveVy = vy;
  lastDriveWz = wz;
  setChassisSpringLogic(
    commandToFloat(vx),
    commandToFloat(vy),
    commandToFloat(wz)
  );
}

void chassisCtrlSetDriveScale(
  float forwardScale,
  float backwardScale,
  float otherScale
) {
  driveScaleForward = constrain(forwardScale, 0.0f, 1.0f);
  driveScaleBackward = constrain(backwardScale, 0.0f, 1.0f);
  driveScaleOther = constrain(otherScale, 0.0f, 1.0f);

  // tuning中は測定条件を1.0に固定する。通常走行中だけ現在の指令へ即時反映する。
  // STOPでlast commandとmotorsActiveを消すため、停止後のphase変更では再始動しない。
  if (!tuningActive && motorsActive) {
    setChassisSpringLogic(
      commandToFloat(lastDriveVx),
      commandToFloat(lastDriveVy),
      commandToFloat(lastDriveWz)
    );
  }
}

void chassisCtrlSetForwardBlocked(bool blocked) {
  if (forwardBlocked == blocked) {
    return;
  }

  forwardBlocked = blocked;
  Serial.print("[CHASSIS][FORWARD_BLOCK] ");
  Serial.println(blocked ? "ON" : "OFF");

  // block開始時は次のPi packetを待たず、保持中の指令から前進成分だけを除去する。
  // unblock時は保存指令を再始動せず、次の通常DRIVE commandから前進を許可する。
  if (blocked && !tuningActive && lastDriveVx > 0) {
    setChassisSpringLogic(
      commandToFloat(lastDriveVx),
      commandToFloat(lastDriveVy),
      commandToFloat(lastDriveWz)
    );
  }
}

void chassisCtrlStop() {
  tuningActive = false;
  tuningResultReady = false;
  tuningSamplingStarted = false;
  longitudinalCommand = 0.0f;
  lastDriveVx = 0;
  lastDriveVy = 0;
  lastDriveWz = 0;

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    requestedMotorRpm[motorIndex] = 0.0f;
    rampedMotorRpm[motorIndex] = 0.0f;
    speedControllers[motorIndex].reset();
    c620DriverSetCurrentCommand(motorIndex, 0);
  }

  motorsActive = false;
  c620DriverZeroAllImmediate();
}

void chassisCtrlChangePower(int delta) {
  drivePowerPercent = constrain(
    drivePowerPercent + delta,
    DRIVE_POWER_MIN,
    DRIVE_POWER_MAX
  );

  Serial.print("DRIVE POWER=");
  Serial.print(drivePowerPercent);
  Serial.println("%");
}

bool chassisCtrlSetWheelGain(
  ChassisGainDirection direction,
  int wheelIndex,
  float gain
) {
  if (wheelIndex < 0 || wheelIndex >= NUM_WHEELS ||
      gain < 0.50f || gain > 1.50f) {
    return false;
  }
  switch (direction) {
    case ChassisGainDirection::FORWARD: wheelGainFwd[wheelIndex] = gain; break;
    case ChassisGainDirection::BACKWARD: wheelGainBwd[wheelIndex] = gain; break;
    case ChassisGainDirection::RIGHT: wheelGainRight[wheelIndex] = gain; break;
    case ChassisGainDirection::LEFT: wheelGainLeft[wheelIndex] = gain; break;
    default: return false;
  }
  return true;
}

void chassisCtrlStartGainTuning(
  int8_t vx,
  int8_t vy,
  int8_t wz,
  uint32_t durationMs
) {
  if (tuningActive) {
    chassisCtrlStop();
  }

  clearTuningStats();
  tuningResultReady = false;
  tuningSamplingStarted = false;
  tuningDurationMs = min(durationMs, GAIN_TUNING_MAX_DURATION_MS);
  tuningStartedMs = millis();
  lastTuningLogMs = tuningStartedMs;
  // drive command生成時からstep assist scaleを無効化するため、先に有効化する。
  tuningActive = true;
  chassisCtrlSetDriveCommand(vx, vy, wz);

  Serial.print("GAIN TUNING START duration_ms=");
  Serial.println(tuningDurationMs);
}

bool chassisCtrlGainTuningResultReady() {
  return tuningResultReady;
}

ChassisGainTuningResult chassisCtrlGetGainTuningResult(int wheelIndex) {
  if (wheelIndex < 0 || wheelIndex >= NUM_WHEELS) {
    return {};
  }

  return tuningResults[wheelIndex];
}

void chassisCtrlClearGainTuningResultReady() {
  tuningResultReady = false;
}

int chassisCtrlGetPowerPercent() {
  return drivePowerPercent;
}

bool chassisCtrlIsActive() {
  return motorsActive;
}

float chassisCtrlGetLongitudinalCommand() {
  return longitudinalCommand;
}
