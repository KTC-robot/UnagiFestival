#include "laser_sensor_runtime.hpp"

#include "constants.h"
#include "../i2c/i2c_bus.hpp"
#include "laser_sensor_device.hpp"
#include "laser_sensor_state.hpp"

#include <Arduino.h>

namespace laserSensorInternal {
namespace {
uint32_t lastInitializationAttemptMs = 0;
uint32_t lastSensorReadMs = 0;
int nextSensorToRead = 0;

void readNextAvailableSensorIfDue() {
  const uint32_t now = millis();

  if (now - lastSensorReadMs < LASER_SENSOR_PERIOD_MS) {
    return;
  }

  lastSensorReadMs = now;

  for (int attempt = 0; attempt < LASER_SENSOR_COUNT; ++attempt) {
    const int index = nextSensorToRead;
    nextSensorToRead =
      (nextSensorToRead + 1) % LASER_SENSOR_COUNT;

    if (sensorConfigured(index) && sensorAvailable(index)) {
      readOneSensor(index);
      return;
    }
  }
}

void markStaleSensorsUnavailable() {
  const uint32_t now = millis();

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index) || !sensorAvailable(index)) {
      continue;
    }

    const uint32_t referenceMs =
      sensorHasValue(index)
        ? sensorLastGoodMs(index)
        : lastInitializationAttemptMs;

    if (now - referenceMs <= LASER_SENSOR_REINIT_AFTER_MS) {
      continue;
    }

    Serial.print("VL53L0X stale: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
    disableSensor(index);
  }
}

void retryMissingSensorsIfDue() {
  const uint32_t now = millis();

  if (
    now - lastInitializationAttemptMs <
    LASER_SENSOR_REINIT_INTERVAL_MS
  ) {
    return;
  }

  bool missingEnabledSensor = false;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index) && !sensorAvailable(index)) {
      missingEnabledSensor = true;
      break;
    }
  }

  if (!missingEnabledSensor) {
    return;
  }

  lastInitializationAttemptMs = now;

  Serial.println(
    "VL53L0X 再初期化: I2CバスとTCA9548Aを再起動"
  );

  if (!i2cBusRestart()) {
    return;
  }

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index) && !sensorAvailable(index)) {
      initializeOneSensor(index);
    }
  }
}
}  // namespace

void initializeAllSensors() {
  lastInitializationAttemptMs = millis();
  lastSensorReadMs = 0;
  nextSensorToRead = 0;

  clearAllSensorStates();
  markAllSensorsUnavailable();

  if (!i2cBusBegin()) {
    return;
  }

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      initializeOneSensor(index);
    }
  }
}

void updateSensors() {
  readNextAvailableSensorIfDue();
  markStaleSensorsUnavailable();
  retryMissingSensorsIfDue();
}

}  // namespace laserSensorInternal
