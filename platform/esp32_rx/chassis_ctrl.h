#pragma once

#include <Arduino.h>

void chassisCtrlBegin();
void chassisCtrlUpdate();

void chassisCtrlSetFromJoy(
  int8_t lx,
  int8_t ly,
  int8_t rx,
  int8_t dpadX,
  int8_t dpadY
);

void chassisCtrlStop();
void chassisCtrlChangePower(int delta);

int chassisCtrlGetPowerPercent();
bool chassisCtrlIsActive();
