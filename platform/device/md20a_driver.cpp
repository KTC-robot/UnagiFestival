#include "device/md20a_driver.hpp"

#include <Arduino.h>

namespace {
constexpr int MD20A_PWM_PIN = 25;
constexpr int MD20A_DIR_PIN = 26;
Md20aState currentState = Md20aState::STOPPED;
}  // namespace

void md20aDriverBegin() {
  pinMode(MD20A_PWM_PIN, OUTPUT);
  pinMode(MD20A_DIR_PIN, OUTPUT);
  digitalWrite(MD20A_PWM_PIN, LOW);
  digitalWrite(MD20A_DIR_PIN, LOW);
  currentState = Md20aState::STOPPED;
  Serial.println("[MD20A] 停止状態で初期化しました");
}

void md20aDriverSetState(Md20aState state) {
  if (state == currentState) return;
  currentState = state;
  switch (state) {
    case Md20aState::STOPPED:
      digitalWrite(MD20A_PWM_PIN, LOW);
      Serial.println("[MD20A] 停止しました");
      break;
    case Md20aState::FORWARD:
      digitalWrite(MD20A_DIR_PIN, HIGH);
      digitalWrite(MD20A_PWM_PIN, HIGH);
      Serial.println("[MD20A] 正転へ切り替えました");
      break;
    case Md20aState::REVERSE:
      digitalWrite(MD20A_DIR_PIN, LOW);
      digitalWrite(MD20A_PWM_PIN, HIGH);
      Serial.println("[MD20A] 逆転へ切り替えました");
      break;
  }
}

Md20aState md20aDriverGetState() {
  return currentState;
}
