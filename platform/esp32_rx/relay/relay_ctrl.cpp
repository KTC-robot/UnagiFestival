#include "relay_ctrl.hpp"
#include "constants.h"

#include <Arduino.h>

namespace {
bool frontValveOn = false;
bool rearValveOn = false;
}  // namespace

bool relayCtrlBegin() {
  pinMode(RELAY_FRONT_VALVE_PIN, OUTPUT);
  pinMode(RELAY_REAR_VALVE_PIN, OUTPUT);

  relayCtrlForceOff();
  return true;
}

void relayCtrlSetFront(bool on) {
  if (frontValveOn == on) {
    return;
  }

  frontValveOn = on;
  digitalWrite(
    RELAY_FRONT_VALVE_PIN,
    on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL
  );

  Serial.print("RELAY FRONT VALVE -> ");
  Serial.println(on ? "ON (extend/down)" : "OFF (retract/up)");
}

void relayCtrlSetRear(bool on) {
  if (rearValveOn == on) {
    return;
  }

  rearValveOn = on;
  digitalWrite(
    RELAY_REAR_VALVE_PIN,
    on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL
  );

  Serial.print("RELAY REAR VALVE -> ");
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

  digitalWrite(RELAY_FRONT_VALVE_PIN, RELAY_OFF_LEVEL);
  digitalWrite(RELAY_REAR_VALVE_PIN, RELAY_OFF_LEVEL);
}
