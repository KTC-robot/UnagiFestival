#include "relay_ctrl.hpp"

#include "../step_air_config.h"

#include <Arduino.h>

namespace {
bool frontValveOn = false;
bool rearValveOn = false;
}  // namespace

bool relayCtrlBegin() {
  pinMode(STEP_AIR_FRONT_VALVE_PIN, OUTPUT);
  pinMode(STEP_AIR_REAR_VALVE_PIN, OUTPUT);

  relayCtrlForceOff();
  return true;
}

void relayCtrlSetFront(bool on) {
  if (frontValveOn == on) {
    return;
  }

  frontValveOn = on;
  digitalWrite(
    STEP_AIR_FRONT_VALVE_PIN,
    on ? STEP_AIR_VALVE_ON_LEVEL : STEP_AIR_VALVE_OFF_LEVEL
  );

  Serial.print("AIR FRONT VALVE -> ");
  Serial.println(on ? "ON (extend/down)" : "OFF (retract/up)");
}

void relayCtrlSetRear(bool on) {
  if (rearValveOn == on) {
    return;
  }

  rearValveOn = on;
  digitalWrite(
    STEP_AIR_REAR_VALVE_PIN,
    on ? STEP_AIR_VALVE_ON_LEVEL : STEP_AIR_VALVE_OFF_LEVEL
  );

  Serial.print("AIR REAR VALVE -> ");
  Serial.println(on ? "ON (extend/down)" : "OFF (retract/up)");
}

bool relayCtrlFrontOn() {
  return frontValveOn;
}

bool relayCtrlRearOn() {
  return rearValveOn;
}

void relayCtrlForceOff() {
  frontValveOn = false;
  rearValveOn = false;

  digitalWrite(STEP_AIR_FRONT_VALVE_PIN, STEP_AIR_VALVE_OFF_LEVEL);
  digitalWrite(STEP_AIR_REAR_VALVE_PIN, STEP_AIR_VALVE_OFF_LEVEL);
}
