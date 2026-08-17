#include "c620/c620_driver.hpp"

#include <driver/twai.h>
#include "c620/constants.h"

using namespace C620Config;

namespace {

bool canReady = false;

int16_t currentCommands[C620_MOTOR_COUNT] = {};
uint16_t rotorAngle[C620_MOTOR_COUNT] = {};
int16_t motorRpm[C620_MOTOR_COUNT] = {};
int16_t measuredCurrent[C620_MOTOR_COUNT] = {};
uint8_t motorTemperature[C620_MOTOR_COUNT] = {};
bool feedbackValid[C620_MOTOR_COUNT] = {};
uint32_t feedbackMs[C620_MOTOR_COUNT] = {};

uint32_t canTxCount = 0;
uint32_t canFeedbackCount = 0;
uint32_t lastCanTxUs = 0;
uint32_t lastCanErrorPrintMs = 0;

int16_t readInt16BigEndian(uint8_t highByte, uint8_t lowByte) {
  return static_cast<int16_t>(
    (static_cast<uint16_t>(highByte) << 8) | lowByte
  );
}

int motorIndexFromFeedbackId(uint32_t identifier) {
  if (
    identifier < C620_FEEDBACK_ID_BASE ||
    identifier >= C620_FEEDBACK_ID_BASE + C620_MOTOR_COUNT
  ) {
    return -1;
  }

  return static_cast<int>(identifier - C620_FEEDBACK_ID_BASE);
}

bool sendCurrentFrame() {
  if (!canReady) {
    return false;
  }

  twai_message_t message = {};
  message.identifier = C620_COMMAND_ID;
  message.data_length_code = 8;

  for (int motorIndex = 0; motorIndex < C620_MOTOR_COUNT; ++motorIndex) {
    const int16_t command = currentCommands[motorIndex];

    message.data[motorIndex * 2] =
      static_cast<uint8_t>((command >> 8) & 0xFF);
    message.data[motorIndex * 2 + 1] =
      static_cast<uint8_t>(command & 0xFF);
  }

  const esp_err_t result = twai_transmit(&message, 0);

  if (result != ESP_OK) {
    const uint32_t now = millis();

    if (now - lastCanErrorPrintMs >= 1000) {
      lastCanErrorPrintMs = now;
      Serial.print("C620 CAN transmit failed: ");
      Serial.println(static_cast<int>(result));
    }

    return false;
  }

  ++canTxCount;
  return true;
}

void handleFeedback(const twai_message_t& message) {
  if (message.data_length_code < 7) {
    return;
  }

  const int motorIndex = motorIndexFromFeedbackId(message.identifier);

  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return;
  }

  rotorAngle[motorIndex] =
    (static_cast<uint16_t>(message.data[0]) << 8) | message.data[1];

  motorRpm[motorIndex] =
    readInt16BigEndian(message.data[2], message.data[3]);

  measuredCurrent[motorIndex] =
    readInt16BigEndian(message.data[4], message.data[5]);

  motorTemperature[motorIndex] = message.data[6];
  feedbackValid[motorIndex] = true;
  feedbackMs[motorIndex] = millis();

  ++canFeedbackCount;
}
}

bool c620DriverBegin() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN,
    CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

  generalConfig.tx_queue_len = 10;
  generalConfig.rx_queue_len = 64;

  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(
    &generalConfig,
    &timingConfig,
    &filterConfig
  );

  if (result != ESP_OK) {
    Serial.print("TWAI driver install failed: ");
    Serial.println(static_cast<int>(result));
    return false;
  }

  result = twai_start();

  if (result != ESP_OK) {
    Serial.print("TWAI start failed: ");
    Serial.println(static_cast<int>(result));
    twai_driver_uninstall();
    return false;
  }

  canReady = true;
  lastCanTxUs = micros();

  Serial.println("TWAI/CAN ready: 1 Mbps, TX=GPIO4, RX=GPIO5");
  return true;
}

bool c620DriverIsReady() {
  return canReady;
}

void c620DriverReadFrames() {
  if (!canReady) {
    return;
  }

  twai_message_t message;

  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.rtr) {
      continue;
    }

    handleFeedback(message);
  }
}

void c620DriverSendPeriodically() {
  if (!canReady) {
    return;
  }

  const uint32_t nowUs = micros();

  if (nowUs - lastCanTxUs < CAN_TX_INTERVAL_US) {
    return;
  }

  lastCanTxUs = nowUs;
  sendCurrentFrame();
}

void c620DriverSetCurrentCommand(int motorIndex, int16_t command) {
  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return;
  }

  currentCommands[motorIndex] = command;
}

int16_t c620DriverGetCurrentCommand(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return 0;
  }

  return currentCommands[motorIndex];
}

void c620DriverZeroAllImmediate() {
  for (int motorIndex = 0; motorIndex < C620_MOTOR_COUNT; ++motorIndex) {
    currentCommands[motorIndex] = 0;
  }

  sendCurrentFrame();
}

bool c620DriverFeedbackFresh(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return false;
  }

  return feedbackValid[motorIndex] &&
    millis() - feedbackMs[motorIndex] <= FEEDBACK_TIMEOUT_MS;
}

int16_t c620DriverGetMotorRpm(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return 0;
  }

  return motorRpm[motorIndex];
}

int16_t c620DriverGetMeasuredCurrent(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return 0;
  }

  return measuredCurrent[motorIndex];
}

uint8_t c620DriverGetMotorTemperature(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return 0;
  }

  return motorTemperature[motorIndex];
}

uint16_t c620DriverGetRotorAngle(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= C620_MOTOR_COUNT) {
    return 0;
  }

  return rotorAngle[motorIndex];
}

uint8_t c620DriverGetFeedbackMask() {
  uint8_t mask = 0;

  for (int motorIndex = 0; motorIndex < C620_MOTOR_COUNT; ++motorIndex) {
    if (c620DriverFeedbackFresh(motorIndex)) {
      mask |= static_cast<uint8_t>(1U << motorIndex);
    }
  }

  return mask;
}

uint32_t c620DriverGetTxCount() {
  return canTxCount;
}

uint32_t c620DriverGetFeedbackCount() {
  return canFeedbackCount;
}
