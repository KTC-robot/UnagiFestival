#include "laser_sensor/laser_sensor_ctrl.hpp"

#include "device/i2c_bus.hpp"
#include "laser_sensor/laser_sensor_device.hpp"
#include "laser_sensor/laser_sensor_state.hpp"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace laserSensorInternal;

namespace {
TaskHandle_t laserSensorTaskHandle = nullptr;
uint32_t lastInitializationAttemptMs[LASER_SENSOR_COUNT] = {};
uint32_t lastSensorReadMs = 0;
int nextSensorToRead = 0;

bool disableAllTcaChannels() {
  I2cBusLockGuard lock;
  return lock.locked() && i2cBusWriteByteLocked(
    LASER_SENSOR_TCA9548A_ADDRESS,
    0x00
  );
}

void readNextAvailableSensorIfDue() {
  const uint32_t now = millis();
  if (now - lastSensorReadMs < LASER_SENSOR_PERIOD_MS) return;
  lastSensorReadMs = now;

  for (int attempt = 0; attempt < LASER_SENSOR_COUNT; ++attempt) {
    const int index = nextSensorToRead;
    nextSensorToRead = (nextSensorToRead + 1) % LASER_SENSOR_COUNT;
    if (sensorConfigured(index) && sensorAvailable(index)) {
      readOneSensor(index);
      return;
    }
  }
}

void invalidateStaleSensorReadings() {
  const uint32_t now = millis();
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index) || !sensorAvailable(index)) continue;
    if (!sensorHasValue(index) ||
        now - sensorLastGoodMs(index) <= LASER_SENSOR_STALE_MS) continue;
    Serial.print("VL53L0X stale: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
    invalidateSensorReading(index);
  }
}

void retryMissingSensorsIfDue() {
  const uint32_t now = millis();
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index) || sensorAvailable(index) ||
        now - lastInitializationAttemptMs[index] <
          LASER_SENSOR_REINIT_INTERVAL_MS) continue;

    lastInitializationAttemptMs[index] = now;
    Serial.print("VL53L0X retry: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
    Serial.print(initializeOneSensor(index)
      ? "VL53L0X retry success: "
      : "VL53L0X retry failed: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
  }
}

void recoverI2cBusIfRequired() {
  if (!busRecoveryRequired()) return;
  clearBusRecoveryRequest();
  Serial.println("I2C BUS RECOVERY START");
  if (!i2cBusRestart()) {
    Serial.println("I2C BUS RECOVERY FAILED");
    return;
  }

  Serial.println("I2C BUS RESTART SUCCESS");
  markAllSensorsUnavailable();
  const uint32_t now = millis();
  const bool tcaReady = disableAllTcaChannels();
  bool allSensorsRecovered = tcaReady;
  if (!tcaReady) Serial.println("TCA9548A recovery failed");

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index)) continue;
    lastInitializationAttemptMs[index] = now;
    if (tcaReady && !initializeOneSensor(index)) {
      allSensorsRecovered = false;
    }
  }
  Serial.println(allSensorsRecovered
    ? "I2C DEVICE RECOVERY SUCCESS"
    : "I2C DEVICE RECOVERY PARTIAL");
}

void initializeAllSensors() {
  lastSensorReadMs = 0;
  nextSensorToRead = 0;
  const uint32_t now = millis();
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    lastInitializationAttemptMs[index] = now;
  }

  clearAllSensorStates();
  markAllSensorsUnavailable();
  if (!disableAllTcaChannels()) {
    Serial.println("TCA9548A 初期化失敗");
    return;
  }
  Serial.println("TCA9548A 接続成功");
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) initializeOneSensor(index);
  }
}

void updateSensors() {
  readNextAvailableSensorIfDue();
  invalidateStaleSensorReadings();
  retryMissingSensorsIfDue();
  recoverI2cBusIfRequired();
}

void laserSensorTask(void*) {
  initializeAllSensors();

  const int configuredCount = configuredSensorCount();
  const int connectedCount = connectedSensorCount();

  Serial.print("VL53L0X connected: ");
  Serial.print(connectedCount);
  Serial.print(" / ");
  Serial.println(configuredCount);

  for (;;) {
    updateSensors();
    vTaskDelay(pdMS_TO_TICKS(LASER_SENSOR_TASK_DELAY_MS));
  }
}
}  // namespace

bool laserSensorCtrlBegin() {
  if (laserSensorTaskHandle != nullptr) {
    return true;
  }

  const BaseType_t result = xTaskCreatePinnedToCore(
    laserSensorTask,
    "laser_sensor",
    LASER_SENSOR_TASK_STACK_SIZE,
    nullptr,
    LASER_SENSOR_TASK_PRIORITY,
    &laserSensorTaskHandle,
    LASER_SENSOR_TASK_CORE
  );

  if (result != pdPASS) {
    laserSensorTaskHandle = nullptr;
    Serial.println("VL53L0X task start failed");
    return false;
  }

  Serial.println("VL53L0X task started");
  return true;
}

bool laserSensorCtrlReady() {
  return allConfiguredSensorsFresh();
}

bool laserSensorCtrlFresh(int sensorIndex) {
  return sensorFresh(sensorIndex);
}

int laserSensorCtrlGetDistanceMm(int sensorIndex) {
  return sensorDistanceMm(sensorIndex);
}

int laserSensorCtrlConnectedCount() {
  return connectedSensorCount();
}

int laserSensorCtrlConfiguredCount() {
  return configuredSensorCount();
}

bool laserSensorCtrlNewMeasurementSetReady() {
  return newMeasurementSetReady();
}
