#include "chassis_ctrl.h"

#include "can_comm.h"
#include "util.h"
#include "chassis_ctrl/constants.h"

#include <math.h>

using namespace CanConfig_chassis_ctrl;

namespace {
int drivePowerPercent = 80;
bool motorsActive = false;
float longitudinalCommand = 0.0f;

float requestedMotorRpm[NUM_MOTORS] = {};
float rampedMotorRpm[NUM_MOTORS] = {};
float pidIntegral[NUM_MOTORS] = {};
float speedKp[NUM_MOTORS] = {};
float speedKi[NUM_MOTORS] = {};

struct GainTuningAccumulator {
  uint32_t sampleCount;
  double absoluteErrorSum;
  double squaredErrorSum;
  float maximumAbsoluteError;
  float finalError;
  uint32_t saturationCount;
};

GainTuningAccumulator tuningStats[NUM_MOTORS] = {};
ChassisGainTuningResult tuningResults[NUM_MOTORS] = {};
bool tuningActive = false;
bool tuningResultReady = false;
uint32_t tuningStartedMs = 0;
uint32_t tuningDurationMs = 0;
uint32_t lastTuningLogMs = 0;

uint32_t lastMotorControlUs = 0;
uint32_t lastMotorPrintMs = 0;

void clearTuningStats() {
  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    tuningStats[motorIndex] = {};
    tuningResults[motorIndex] = {};
  }
}

void finishGainTuning() {
  tuningActive = false;
  chassisCtrlStop();

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    const GainTuningAccumulator& stats = tuningStats[motorIndex];
    ChassisGainTuningResult& result = tuningResults[motorIndex];

    result.sampleCount = stats.sampleCount;
    result.maximumAbsoluteError = stats.maximumAbsoluteError;
    result.finalError = stats.finalError;
    result.saturationCount = stats.saturationCount;

    if (stats.sampleCount > 0) {
      result.meanAbsoluteError = static_cast<float>(
        stats.absoluteErrorSum / static_cast<double>(stats.sampleCount)
      );
      result.rootMeanSquaredError = static_cast<float>(sqrt(
        stats.squaredErrorSum / static_cast<double>(stats.sampleCount)
      ));
    }
  }

  tuningResultReady = true;
  Serial.println("GAIN TUNING DONE -> STOP");
}

float applyMotorInverse(int motorIndex, float value) {
  return MOTOR_REVERSED[motorIndex] ? -value : value;
}

int16_t clampCurrentCommand(float value) {
  value = constrain(
    value,
    -static_cast<float>(MAX_CURRENT_COMMAND),
    static_cast<float>(MAX_CURRENT_COMMAND)
  );

  return static_cast<int16_t>(lroundf(value));
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
    static_cast<float>(canCommGetMotorRpm(motorIndex))
  );
}

int16_t getWheelCurrentCommand(int wheelIndex) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

  return static_cast<int16_t>(
    applyMotorInverse(
      motorIndex,
      static_cast<float>(canCommGetCurrentCommand(motorIndex))
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
    Serial.print(static_cast<int>(canCommGetMotorTemperature(motorIndex)));

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

  if (VX_INVERT) vx = -vx;
  if (WZ_INVERT) wz = -wz;

  longitudinalCommand = vx;

  if (vx == 0.0f && vy == 0.0f && wz == 0.0f) {
    chassisCtrlStop();
    return;
  }

  float wheel[NUM_WHEELS] = {};

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    float forward = static_cast<float>(FWD_SIGN[wheelIndex]) * vx;
    float strafe = static_cast<float>(STR_SIGN[wheelIndex]) * vy;
    const float yaw = static_cast<float>(YAW_SIGN[wheelIndex]) * wz;

    if (vx > 0.0f) {
      forward *= WHEEL_GAIN_FWD[wheelIndex];
    } else if (vx < 0.0f) {
      forward *= WHEEL_GAIN_BWD[wheelIndex];
    }

    if (vy > 0.0f) {
      strafe *= WHEEL_GAIN_RIGHT[wheelIndex];
    } else if (vy < 0.0f) {
      strafe *= WHEEL_GAIN_LEFT[wheelIndex];
    }

    wheel[wheelIndex] = forward + strafe + yaw;
  }

  float maxMagnitude = 0.0f;

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    maxMagnitude = max(maxMagnitude, fabsf(wheel[wheelIndex]));
  }

  if (maxMagnitude > 1.0f) {
    for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
      wheel[wheelIndex] /= maxMagnitude;
    }
  }

  const float maxRpm =
    static_cast<float>(CHASSIS_MAX_RPM) *
    (static_cast<float>(drivePowerPercent) / 100.0f);

  bool anyWheelActive = false;

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int16_t wheelRpm =
      static_cast<int16_t>(lroundf(wheel[wheelIndex] * maxRpm));

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

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    speedKp[motorIndex] = SPEED_KP;
    speedKi[motorIndex] = SPEED_KI;
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

  if (!canCommIsReady()) {
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

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    rampedMotorRpm[motorIndex] = moveToward(
      rampedMotorRpm[motorIndex],
      requestedMotorRpm[motorIndex],
      maxRpmChange
    );

    if (!canCommFeedbackFresh(motorIndex)) {
      canCommSetCurrentCommand(motorIndex, 0);
      pidIntegral[motorIndex] = 0.0f;
      continue;
    }

    if (fabsf(rampedMotorRpm[motorIndex]) < 1.0f) {
      canCommSetCurrentCommand(motorIndex, 0);
      pidIntegral[motorIndex] = 0.0f;
      continue;
    }

    const float error =
      rampedMotorRpm[motorIndex] -
      static_cast<float>(canCommGetMotorRpm(motorIndex));

    const float integralOld = pidIntegral[motorIndex];

    float integralCandidate = integralOld + error * dt;
    integralCandidate = constrain(
      integralCandidate,
      -PID_INTEGRAL_LIMIT,
      PID_INTEGRAL_LIMIT
    );

    const float outputCandidate =
      speedKp[motorIndex] * error +
      speedKi[motorIndex] * integralCandidate;

    const bool saturatedHigh =
      outputCandidate >= static_cast<float>(MAX_CURRENT_COMMAND);

    const bool saturatedLow =
      outputCandidate <= -static_cast<float>(MAX_CURRENT_COMMAND);

    const bool allowIntegral =
      (!saturatedHigh && !saturatedLow) ||
      (saturatedHigh && error < 0.0f) ||
      (saturatedLow && error > 0.0f);

    const float integralUsed =
      allowIntegral ? integralCandidate : integralOld;

    const float output =
      speedKp[motorIndex] * error + speedKi[motorIndex] * integralUsed;

    const int16_t currentCommand = clampCurrentCommand(output);

    const bool saturated =
      currentCommand >= MAX_CURRENT_COMMAND ||
      currentCommand <= -MAX_CURRENT_COMMAND;

    if (tuningActive) {
      GainTuningAccumulator& stats = tuningStats[motorIndex];
      const float absoluteError = fabsf(error);
      ++stats.sampleCount;
      stats.absoluteErrorSum += static_cast<double>(absoluteError);
      stats.squaredErrorSum +=
        static_cast<double>(error) * static_cast<double>(error);
      stats.maximumAbsoluteError = max(
        stats.maximumAbsoluteError,
        absoluteError
      );
      stats.finalError = error;
      if (saturated) {
        ++stats.saturationCount;
      }

      if (printTuningLog) {
        Serial.print("GAIN M");
        Serial.print(motorIndex + 1);
        Serial.print(" KP=");
        Serial.print(speedKp[motorIndex], 3);
        Serial.print(" KI=");
        Serial.print(speedKi[motorIndex], 3);
        Serial.print(" TARGET=");
        Serial.print(rampedMotorRpm[motorIndex], 0);
        Serial.print(" ACTUAL=");
        Serial.print(canCommGetMotorRpm(motorIndex));
        Serial.print(" ERROR=");
        Serial.print(error, 0);
        Serial.print(" P=");
        Serial.print(speedKp[motorIndex] * error, 0);
        Serial.print(" I=");
        Serial.print(speedKi[motorIndex] * integralUsed, 0);
        Serial.print(" OUT=");
        Serial.print(output, 0);
        Serial.print(" SAT=");
        Serial.println(saturated ? 1 : 0);
      }
    }

    canCommSetCurrentCommand(
      motorIndex,
      currentCommand
    );

    pidIntegral[motorIndex] = integralUsed;
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
  setChassisSpringLogic(
    utilCommandToFloat(vx),
    utilCommandToFloat(vy),
    utilCommandToFloat(wz)
  );
}

void chassisCtrlStop() {
  tuningActive = false;
  tuningResultReady = false;
  longitudinalCommand = 0.0f;

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    requestedMotorRpm[motorIndex] = 0.0f;
    rampedMotorRpm[motorIndex] = 0.0f;
    pidIntegral[motorIndex] = 0.0f;
    canCommSetCurrentCommand(motorIndex, 0);
  }

  motorsActive = false;
  canCommZeroAllImmediate();
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

bool chassisCtrlSetSpeedGain(int motorIndex, float kp, float ki) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) {
    return false;
  }

  speedKp[motorIndex] = kp;
  speedKi[motorIndex] = ki;
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
  tuningDurationMs = min(durationMs, GAIN_TUNING_MAX_DURATION_MS);
  tuningStartedMs = millis();
  lastTuningLogMs = tuningStartedMs;
  chassisCtrlSetDriveCommand(vx, vy, wz);
  tuningActive = true;

  Serial.print("GAIN TUNING START duration_ms=");
  Serial.println(tuningDurationMs);
}

bool chassisCtrlGainTuningResultReady() {
  return tuningResultReady;
}

ChassisGainTuningResult chassisCtrlGetGainTuningResult(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) {
    return {};
  }

  return tuningResults[motorIndex];
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
