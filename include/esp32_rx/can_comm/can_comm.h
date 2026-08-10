#pragma once

#include <Arduino.h>

constexpr int CAN_MOTOR_COUNT = 4;

bool canCommBegin();
bool canCommIsReady();

void canCommReadFrames();
void canCommSendPeriodically();

void canCommSetCurrentCommand(int motorIndex, int16_t command);
int16_t canCommGetCurrentCommand(int motorIndex);
void canCommZeroAllImmediate();

bool canCommFeedbackFresh(int motorIndex);
int16_t canCommGetMotorRpm(int motorIndex);
int16_t canCommGetMeasuredCurrent(int motorIndex);
uint8_t canCommGetMotorTemperature(int motorIndex);
uint16_t canCommGetRotorAngle(int motorIndex);

uint8_t canCommGetFeedbackMask();
uint32_t canCommGetTxCount();
uint32_t canCommGetFeedbackCount();
