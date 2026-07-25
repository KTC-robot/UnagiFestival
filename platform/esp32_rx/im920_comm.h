#pragma once

#include <Arduino.h>

void im920CommBegin();
void im920CommUpdate();

void im920CommCheckTimeout();
void im920CommSendPeriodicStatus();
void im920CommSendText(const String& text);
