#include "chassis_ctrl/chassis_ctrl.hpp"

#include "device/c620_driver.hpp"
#include "chassis_ctrl/constants.hpp"
#include "chassis_ctrl/mecanum.hpp"
#include "chassis_ctrl/speed_controller.hpp"

#include <math.h>

using namespace CanConfig_chassis_ctrl;

namespace {
// Controllerから届く符号付き指令を、計算で扱いやすい-1.0〜1.0へ変換する。
// 小さな入力はdeadzoneとして0にし、スティックの微小な揺れを走行へ伝えない。
constexpr int COMMAND_MAX = 127;
constexpr int COMMAND_DEAD = 8;

float commandToFloat(int value) {
  if (abs(value) < COMMAND_DEAD) value = 0;
  value = constrain(value, -COMMAND_MAX, COMMAND_MAX);
  return static_cast<float>(value) / static_cast<float>(COMMAND_MAX);
}

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
  uint32_t sampleCount;          ///< 平均計算に使用した有効サンプル数。
  double absoluteRpmSum;         ///< 実測RPM絶対値の合計。
  double absoluteRpmSquaredSum;  ///< 標準偏差計算用のRPM絶対値二乗和。
};

/** @brief Gain Tuning途中の統計表示に使用する読み取り専用snapshot。 */
struct GainTuningStatisticsSnapshot {
  double meanAbsoluteRpm;  ///< 現時点までの実測RPM絶対値平均。
  double standardDeviationRpm;  ///< 現時点までの実測RPM絶対値の母標準偏差。
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

GainTuningStatisticsSnapshot calculateTuningStatisticsSnapshot(
  const GainTuningAccumulator& stats
) {
  if (stats.sampleCount == 0) {
    return {};
  }

  const double mean =
    stats.absoluteRpmSum / static_cast<double>(stats.sampleCount);
  const double variance = max(
    0.0,
    stats.absoluteRpmSquaredSum / static_cast<double>(stats.sampleCount) -
      mean * mean
  );
  return {mean, sqrt(variance)};
}

void printGainTuningMapping() {
  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
    Serial.print("[GAIN][MAP] wheel=");
    Serial.print(WHEEL_NAME[wheelIndex]);
    Serial.print(" motor=M");
    Serial.print(motorIndex + 1);
    Serial.print(" ESC_ID=");
    Serial.print(WHEEL_ESC_ID[wheelIndex]);
    Serial.print(" reversed=");
    Serial.println(MOTOR_REVERSED[motorIndex] ? "true" : "false");
  }
}

void printGainTuningStart(
  int8_t vx,
  int8_t vy,
  int8_t wz
) {
  Serial.println("[GAIN][START] Gain Tuningを開始します");
  Serial.print("[GAIN][START] duration=");
  Serial.print(tuningDurationMs);
  Serial.print("ms command=(vx=");
  Serial.print(vx);
  Serial.print(", vy=");
  Serial.print(vy);
  Serial.print(", wz=");
  Serial.print(wz);
  Serial.print(") power=");
  Serial.print(DRIVE_POWER_PERCENT);
  Serial.println("%");

  Serial.print("[GAIN][START] Kp=");
  Serial.print(SPEED_KP, 3);
  Serial.print(" Ki=");
  Serial.print(SPEED_KI, 3);
  Serial.print(" current_limit=");
  Serial.print(MAX_CURRENT_COMMAND);
  Serial.print(" integral_limit=");
  Serial.println(PID_INTEGRAL_LIMIT, 0);

  Serial.print("[GAIN][START] slew=");
  Serial.print(TARGET_RPM_SLEW_PER_SEC, 0);
  Serial.print("rpm/s ramp_tolerance=");
  Serial.print(GAIN_TUNING_RAMP_TOLERANCE_RPM, 0);
  Serial.print("rpm log_interval=");
  Serial.print(GAIN_TUNING_LOG_INTERVAL_MS);
  Serial.println("ms");
  printGainTuningMapping();
}

void printGainTuningResults() {
  double maximumMeanRpm = 0.0;
  double minimumMeanRpm = 0.0;

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
    const ChassisGainTuningResult& result = tuningResults[wheelIndex];
    const double variationPercent = result.meanAbsoluteRpm > 0.0f
      ? static_cast<double>(result.standardDeviationRpm) /
        static_cast<double>(result.meanAbsoluteRpm) * 100.0
      : 0.0;

    if (wheelIndex == 0) {
      maximumMeanRpm = result.meanAbsoluteRpm;
      minimumMeanRpm = result.meanAbsoluteRpm;
    } else {
      maximumMeanRpm = max(
        maximumMeanRpm,
        static_cast<double>(result.meanAbsoluteRpm)
      );
      minimumMeanRpm = min(
        minimumMeanRpm,
        static_cast<double>(result.meanAbsoluteRpm)
      );
    }

    Serial.print("[GAIN][RESULT] wheel=");
    Serial.print(WHEEL_NAME[wheelIndex]);
    Serial.print(" motor=M");
    Serial.print(motorIndex + 1);
    Serial.print(" samples=");
    Serial.print(result.sampleCount);
    Serial.print(" mean=");
    Serial.print(result.meanAbsoluteRpm, 0);
    Serial.print("rpm stddev=");
    Serial.print(result.standardDeviationRpm, 0);
    Serial.print("rpm variation=");
    Serial.print(variationPercent, 2);
    Serial.println("%");
  }

  const double differenceRpm = maximumMeanRpm - minimumMeanRpm;
  const double balancePercent = maximumMeanRpm > 0.0
    ? minimumMeanRpm / maximumMeanRpm * 100.0
    : 0.0;
  Serial.print("[GAIN][SUMMARY] max_mean=");
  Serial.print(maximumMeanRpm, 0);
  Serial.print("rpm min_mean=");
  Serial.print(minimumMeanRpm, 0);
  Serial.print("rpm difference=");
  Serial.print(differenceRpm, 0);
  Serial.print("rpm balance=");
  Serial.print(balancePercent, 1);
  Serial.println("%");
}

/**
 * @brief Gain Tuningを終了し、4輪のRPM統計を確定する。
 *
 * 車体を先に安全停止してから、平均絶対RPMと母標準偏差を計算する。
 * 確定した結果はIM920側が順次送信できるようready状態にする。
 */
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

  printGainTuningResults();
  tuningResultReady = true;
  Serial.println("[GAIN][RESULT] Gain Tuningが完了したため停止しました");
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

// 目標RPMを1周期で変化できる量に制限し、急な電流変化を避ける。
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

float getWheelRampedRpm(int wheelIndex) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  return applyMotorInverse(motorIndex, rampedMotorRpm[motorIndex]);
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

int16_t getWheelMeasuredCurrent(int wheelIndex) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  return static_cast<int16_t>(
    applyMotorInverse(
      motorIndex,
      static_cast<float>(c620DriverGetMeasuredCurrent(motorIndex))
    )
  );
}

void printGainTuningWheelRpm(
  const char* stage,
  uint32_t elapsedMs,
  int wheelIndex
) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  const float requestedRpm = getWheelTargetRpm(wheelIndex);
  const float rampedRpm = getWheelRampedRpm(wheelIndex);

  Serial.print("[GAIN][");
  Serial.print(stage);
  Serial.print("] t=");
  Serial.print(elapsedMs);
  Serial.print("ms wheel=");
  Serial.print(WHEEL_NAME[wheelIndex]);
  Serial.print(" motor=M");
  Serial.print(motorIndex + 1);
  Serial.print(" requested=");
  Serial.print(requestedRpm, 0);
  Serial.print("rpm ramped=");
  Serial.print(rampedRpm, 0);
  Serial.print("rpm actual=");
  Serial.print(getWheelMeasuredRpm(wheelIndex), 0);
  Serial.print("rpm remaining=");
  Serial.print(requestedRpm - rampedRpm, 0);
  Serial.print("rpm fresh=");
  Serial.println(c620DriverFeedbackFresh(motorIndex) ? "true" : "false");
}

void printGainTuningSamplingStarted(uint32_t elapsedMs) {
  Serial.print(
    "[GAIN][SAMPLE] 全wheelのRampが収束したため測定を開始します t="
  );
  Serial.print(elapsedMs);
  Serial.println("ms");
  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    printGainTuningWheelRpm("SAMPLE", elapsedMs, wheelIndex);
  }
}

void printGainTuningSample(
  uint32_t elapsedMs,
  float dt,
  int wheelIndex,
  const SpeedControllerResult* speedResult
) {
  const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  const bool fresh = c620DriverFeedbackFresh(motorIndex);
  const GainTuningAccumulator& stats = tuningStats[wheelIndex];
  const GainTuningStatisticsSnapshot snapshot =
    calculateTuningStatisticsSnapshot(stats);

  Serial.print("[GAIN][SAMPLE] t=");
  Serial.print(elapsedMs);
  Serial.print("ms dt=");
  Serial.print(dt, 4);
  Serial.print("s wheel=");
  Serial.print(WHEEL_NAME[wheelIndex]);
  Serial.print(" motor=M");
  Serial.print(motorIndex + 1);
  Serial.print(" ESC_ID=");
  Serial.println(WHEEL_ESC_ID[wheelIndex]);

  Serial.print("  requested=");
  Serial.print(getWheelTargetRpm(wheelIndex), 0);
  Serial.print("rpm ramped=");
  Serial.print(getWheelRampedRpm(wheelIndex), 0);
  Serial.print("rpm actual=");
  Serial.print(getWheelMeasuredRpm(wheelIndex), 0);
  Serial.print("rpm error=");
  if (speedResult != nullptr) {
    Serial.print(applyMotorInverse(motorIndex, speedResult->error), 0);
  } else {
    Serial.print("N/A");
  }
  Serial.println("rpm");

  Serial.print("  Kp=");
  Serial.print(SPEED_KP, 3);
  Serial.print(" Ki=");
  Serial.print(SPEED_KI, 3);
  if (speedResult != nullptr) {
    const float wheelError = applyMotorInverse(motorIndex, speedResult->error);
    const float wheelIntegral =
      applyMotorInverse(motorIndex, speedResult->integral);
    Serial.print(" P=");
    Serial.print(SPEED_KP * wheelError, 0);
    Serial.print(" integral=");
    Serial.print(wheelIntegral, 0);
    Serial.print(" I=");
    Serial.println(SPEED_KI * wheelIntegral, 0);
  } else {
    Serial.println(" P=N/A integral=N/A I=N/A");
  }

  Serial.print("  output_raw=");
  if (speedResult != nullptr) {
    Serial.print(applyMotorInverse(motorIndex, speedResult->output), 0);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" current_cmd=");
  Serial.print(getWheelCurrentCommand(wheelIndex));
  Serial.print(" current_measured=");
  Serial.print(getWheelMeasuredCurrent(wheelIndex));
  Serial.print(" saturated=");
  Serial.println(
    speedResult != nullptr &&
      fabsf(speedResult->output) >= static_cast<float>(MAX_CURRENT_COMMAND)
      ? "true" : "false"
  );

  Serial.print("  fresh=");
  Serial.print(fresh ? "true" : "false");
  Serial.print(" temperature=");
  Serial.print(c620DriverGetMotorTemperature(motorIndex));
  Serial.print("C samples=");
  Serial.print(stats.sampleCount);
  Serial.print(" mean_abs=");
  Serial.print(snapshot.meanAbsoluteRpm, 0);
  Serial.print("rpm stddev=");
  Serial.print(snapshot.standardDeviationRpm, 0);
  Serial.println("rpm");
}

void printMotorValues(float vx, float vy, float wz) {
  const uint32_t now = millis();

  if (now - lastMotorPrintMs < 200) {
    return;
  }

  lastMotorPrintMs = now;

  Serial.print("[CHASSIS] 指令 vx=");
  Serial.print(vx, 2);
  Serial.print(" vy=");
  Serial.print(vy, 2);
  Serial.print(" wz=");
  Serial.print(wz, 2);
  Serial.print(" PWR=");
  Serial.print(DRIVE_POWER_PERCENT);
  Serial.print("% | ");

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

    Serial.print(WHEEL_NAME[wheelIndex]);
    Serial.print("(ID");
    Serial.print(WHEEL_ESC_ID[wheelIndex]);
    Serial.print(") 目標RPM=");
    Serial.print(static_cast<int>(getWheelTargetRpm(wheelIndex)));
    Serial.print(" 実測RPM=");
    Serial.print(static_cast<int>(getWheelMeasuredRpm(wheelIndex)));
    Serial.print(" 電流指令=");
    Serial.print(static_cast<int>(getWheelCurrentCommand(wheelIndex)));
    Serial.print(" 温度[℃]=");
    Serial.print(static_cast<int>(c620DriverGetMotorTemperature(motorIndex)));

    if (wheelIndex < NUM_WHEELS - 1) {
      Serial.print(" | ");
    }
  }

  Serial.println();
}

void setChassisSpringLogic(float vx, float vy, float wz) {
  // 入力値の範囲とdeadzoneを先に確定し、以降の方向判定を統一する。
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

  // 前後・横・旋回を車輪別成分へ分解した後、走行方向ごとのwheel gainを
  // 対応成分だけへ掛ける。wheel gainは車輪間の個体差補正にのみ使う。
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

  // 正規化済み車輪出力を、power設定とStepAssistの車体全体scaleを反映した
  // 実際の目標RPMへ変換する。
  const float maxRpm =
    static_cast<float>(CHASSIS_MAX_RPM) *
    (static_cast<float>(DRIVE_POWER_PERCENT) / 100.0f) *
    selectedDriveScale;

  bool anyWheelActive = false;

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
    const int16_t wheelRpm =
      static_cast<int16_t>(lroundf(mecanumOutput.wheel[wheelIndex] * maxRpm));

    const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

    // 物理的な取付方向が逆のmotorだけ符号を反転し、車輪座標から
    // C620が受け取るmotor座標へ変換する。
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

  Serial.print("[CHASSIS] 初期化完了 Kp=");
  Serial.print(SPEED_KP, 2);
  Serial.print(" Ki=");
  Serial.print(SPEED_KI, 2);
  Serial.print(" 電流指令上限=±");
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
  SpeedControllerResult tuningSpeedResults[NUM_MOTORS] = {};
  bool tuningSpeedResultValid[NUM_MOTORS] = {};

  // 4輪すべてのtarget rampが収束した次の制御周期から同時に集計する。
  // wheelごとに開始時刻がずれると、RPM比較の測定区間が揃わないため。
  if (tuningActive && !tuningSamplingStarted && allTuningTargetsSettled()) {
    tuningSamplingStarted = true;
    printGainTuningSamplingStarted(nowMs - tuningStartedMs);
  }

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; ++motorIndex) {
    rampedMotorRpm[motorIndex] = moveToward(
      rampedMotorRpm[motorIndex],
      requestedMotorRpm[motorIndex],
      maxRpmChange
    );

    // 古いfeedbackで制御を続けると実際の回転状態を誤認するため、
    // freshnessを失ったmotorは電流を0にして積分状態も破棄する。
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

    // ramp後の目標RPMとC620の実測RPMから、今回送る電流指令を求める。
    const SpeedControllerResult speedResult =
      speedControllers[motorIndex].update(
        rampedMotorRpm[motorIndex],
        static_cast<float>(c620DriverGetMotorRpm(motorIndex)),
        dt
      );
    tuningSpeedResults[motorIndex] = speedResult;
    tuningSpeedResultValid[motorIndex] = true;

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

    }

    c620DriverSetCurrentCommand(
      motorIndex,
      speedResult.currentCommand
    );
  }

  if (printTuningLog) {
    const uint32_t tuningElapsedMs = nowMs - tuningStartedMs;
    for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; ++wheelIndex) {
      if (tuningSamplingStarted) {
        const int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
        printGainTuningSample(
          tuningElapsedMs,
          dt,
          wheelIndex,
          tuningSpeedResultValid[motorIndex]
            ? &tuningSpeedResults[motorIndex]
            : nullptr
        );
      } else {
        printGainTuningWheelRpm("RAMP", tuningElapsedMs, wheelIndex);
      }
    }
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
  Serial.print("[CHASSIS] 前進禁止=");
  Serial.println(blocked ? "有効" : "解除");

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
  // STOP後のscale変更やphase遷移で古い指令が再適用されないよう、
  // 目標・保持Command・PI積分・Gain Tuning状態をまとめて破棄する。
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
  printGainTuningStart(vx, vy, wz);
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
  return DRIVE_POWER_PERCENT;
}

bool chassisCtrlIsActive() {
  return motorsActive;
}

float chassisCtrlGetLongitudinalCommand() {
  return longitudinalCommand;
}
