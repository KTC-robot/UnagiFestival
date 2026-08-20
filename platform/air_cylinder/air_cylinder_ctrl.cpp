#include "air_cylinder/air_cylinder_ctrl.hpp"

#include <Arduino.h>

#include "device/relay_driver.hpp"

namespace {
// 暫定値。実機のAir Cylinder動作に合わせて後で調整する。
constexpr uint32_t AIR_FIRE_ON_MS = 100;
constexpr uint32_t AIR_FIRE_OFF_MS = 200;

enum class AirCylinderPhase : uint8_t {
  STOPPED,
  AIR_ON,
  AIR_OFF,
};

AirCylinderPhase phase = AirCylinderPhase::STOPPED;
uint32_t lastChangedMs = 0;
}  // namespace

void airCylinderCtrlBegin() {
  phase = AirCylinderPhase::STOPPED;
  relayDriverSetAir(false);
  lastChangedMs = millis();
}

void airCylinderCtrlStart() {
  if (phase != AirCylinderPhase::STOPPED) return;
  phase = AirCylinderPhase::AIR_ON;
  relayDriverSetAir(true);
  lastChangedMs = millis();
  Serial.println("[AIR] 連射を開始します");
}

void airCylinderCtrlStop() {
  if (phase == AirCylinderPhase::STOPPED && !relayDriverAirOn()) return;
  phase = AirCylinderPhase::STOPPED;
  relayDriverSetAir(false);
  lastChangedMs = millis();
  Serial.println("[AIR] 連射を停止します");
}

void airCylinderCtrlUpdate() {
  const uint32_t now = millis();
  if (phase == AirCylinderPhase::AIR_ON && now - lastChangedMs >= AIR_FIRE_ON_MS) {
    phase = AirCylinderPhase::AIR_OFF;
    relayDriverSetAir(false);
    lastChangedMs = now;
  } else if (
    phase == AirCylinderPhase::AIR_OFF && now - lastChangedMs >= AIR_FIRE_OFF_MS
  ) {
    phase = AirCylinderPhase::AIR_ON;
    relayDriverSetAir(true);
    lastChangedMs = now;
  }
}

bool airCylinderCtrlActive() {
  return phase != AirCylinderPhase::STOPPED;
}
