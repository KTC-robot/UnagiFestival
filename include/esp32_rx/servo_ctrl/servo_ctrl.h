#pragma once

#include <Arduino.h>

void servoCtrlBegin();
void servoCtrlDisableAll();
void servoCtrlHandlePacket(const String& hex);