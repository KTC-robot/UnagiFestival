#include "stepper_ctrl/stepper_ctrl.hpp"

#include "stepper_ctrl/constants.h"
#include "util/util.h"

using namespace CanConfig_stepper_ctrl;

namespace {

bool initialized = false;
bool running = false;
bool stepLevel = HIGH;
StepperMotion currentMotion = StepperMotion::STOP;
uint32_t currentPulseHz = STEPPER_PULSE_HZ_DEFAULT;
uint32_t halfPeriodUs = 1000000UL / (STEPPER_PULSE_HZ_DEFAULT * 2UL);
uint32_t lastStepUs = 0;

void setPulseIdle() {
  // PUL+がESP32 3.3V固定なので、PUL-をHIGHにすると入力OFF。
  digitalWrite(STEPPER_PUL_PIN, HIGH);
  stepLevel = HIGH;
}

void applyDirection(StepperMotion motion) {
  // 共通DIR信号なので2台のTB6600へ同じ方向信号が入る。
  // UP   : DIR- = LOW
  // DOWN : DIR- = HIGH
  const uint8_t dirLevel = motion == StepperMotion::UP ? LOW : HIGH;
  digitalWrite(STEPPER_DIR_PIN, dirLevel);

  delayMicroseconds(STEPPER_DIR_SETUP_US);
}

void startMotion(StepperMotion motion) {
  if (!initialized) {
    Serial.println("STEPPER ignored: controller is not initialized");
    return;
  }

  if (motion == StepperMotion::STOP) {
    stepperCtrlStop();
    return;
  }

  // 同じ開始命令が来た場合はそのまま回転を継続する。
  if (running && currentMotion == motion) {
    return;
  }

  // 回転中に方向を変える場合は一度パルスを停止してからDIRを変更。
  if (running) {
    setPulseIdle();
    delayMicroseconds(STEPPER_DIR_SETUP_US);
  }

  applyDirection(motion);

  setPulseIdle();
  lastStepUs = micros();
  currentMotion = motion;
  running = true;

  Serial.print("STEPPER START ");
  Serial.print(motion == StepperMotion::UP ? "UP" : "DOWN");
  Serial.print(" PULSE_HZ=");
  Serial.println(currentPulseHz);
}

}  // namespace

bool stepperCtrlBegin() {
  pinMode(STEPPER_PUL_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);

  // 実機単体テストと同じ初期状態。
  setPulseIdle();
  digitalWrite(STEPPER_DIR_PIN, HIGH);

  initialized = true;
  running = false;
  currentMotion = StepperMotion::STOP;
  lastStepUs = micros();

  Serial.println("STEPPER ready (shared PUL/DIR for 2 drivers)");
  Serial.print("  PUL = GPIO");
  Serial.println(STEPPER_PUL_PIN);
  Serial.print("  DIR = GPIO");
  Serial.println(STEPPER_DIR_PIN);
  Serial.print("  PULSE_HZ = ");
  Serial.println(currentPulseHz);

  return true;
}

void stepperCtrlStartUp() {
  startMotion(StepperMotion::UP);
}

void stepperCtrlStartDown() {
  startMotion(StepperMotion::DOWN);
}

void stepperCtrlStop() {
  const bool wasRunning = running;

  running = false;
  currentMotion = StepperMotion::STOP;
  setPulseIdle();

  if (wasRunning) {
    Serial.println("STEPPER STOP");
  }
}

void stepperCtrlUpdate() {
  if (!initialized || !running) {
    return;
  }

  const uint32_t now = micros();

  if (static_cast<uint32_t>(now - lastStepUs) < halfPeriodUs) {
    return;
  }

  // 単体テストと同じmicros() + digitalWrite()方式。
  lastStepUs = now;
  stepLevel = !stepLevel;
  digitalWrite(STEPPER_PUL_PIN, stepLevel);
}

bool stepperCtrlSetPulseHz(uint32_t pulseHz) {
  if (pulseHz < STEPPER_PULSE_HZ_MIN || pulseHz > STEPPER_PULSE_HZ_MAX) {
    return false;
  }

  currentPulseHz = pulseHz;
  halfPeriodUs = 1000000UL / (currentPulseHz * 2UL);

  if (halfPeriodUs == 0) {
    halfPeriodUs = 1;
  }

  Serial.print("STEPPER PULSE_HZ=");
  Serial.println(currentPulseHz);
  return true;
}

bool stepperCtrlIsRunning() {
  return running;
}

StepperMotion stepperCtrlGetMotion() {
  return currentMotion;
}

void stepperCtrlHandlePacket(const String& hex) {
  // packet: 54 CC
  // CC=00 STOP, 01 UP, 02 DOWN
  if (hex.length() < 4) {
    return;
  }

  const StepperMotion motion = static_cast<StepperMotion>(
    utilHexByteToUint8(hex.substring(2, 4))
  );

  switch (motion) {
    case StepperMotion::STOP:
      stepperCtrlStop();
      break;

    case StepperMotion::UP:
      stepperCtrlStartUp();
      break;

    case StepperMotion::DOWN:
      stepperCtrlStartDown();
      break;

    default:
      Serial.print("STEPPER invalid motion=");
      Serial.println(static_cast<uint8_t>(motion));
      break;
  }
}
