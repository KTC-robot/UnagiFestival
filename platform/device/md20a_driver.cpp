#include "device/md20a_driver.hpp"

#include <Arduino.h>

namespace {
constexpr int MD20A_PWM_PIN = 25;
constexpr int MD20A_DIR_PIN = 26;
constexpr int MD20A_MOTOR_PWM = 120;

Md20aState currentState = Md20aState::STOPPED;

void applyState(Md20aState state) {
  switch (state) {
    case Md20aState::FORWARD:
      digitalWrite(MD20A_DIR_PIN, HIGH);
      analogWrite(MD20A_PWM_PIN, MD20A_MOTOR_PWM);
      Serial.println("[MD20A] 正転へ切り替えました");
      break;

    case Md20aState::REVERSE:
      digitalWrite(MD20A_DIR_PIN, LOW);
      analogWrite(MD20A_PWM_PIN, MD20A_MOTOR_PWM);
      Serial.println("[MD20A] 逆転へ切り替えました");
      break;

    case Md20aState::STOPPED:
    default:
      analogWrite(MD20A_PWM_PIN, 0);
      Serial.println("[MD20A] 停止しました");
      break;
  }
}
}  // namespace

void md20aDriverBegin() {
  pinMode(MD20A_PWM_PIN, OUTPUT);
  pinMode(MD20A_DIR_PIN, OUTPUT);

  // 起動時に意図せずラックレールが動かないよう、必ず停止状態から開始する。
  currentState = Md20aState::STOPPED;
  applyState(currentState);
  Serial.println("[MD20A] 初期化が完了しました");
}

void md20aDriverSetState(Md20aState state) {
  if (currentState == state) {
    return;
  }

  currentState = state;
  applyState(currentState);
}

Md20aState md20aDriverGetState() {
  return currentState;
}
