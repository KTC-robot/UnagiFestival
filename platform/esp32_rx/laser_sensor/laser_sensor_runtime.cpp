#include "laser_sensor_runtime.hpp"

#include "constants.h"
#include "../i2c/i2c_bus.hpp"
#include "laser_sensor_device.hpp"
#include "laser_sensor_state.hpp"
#include "servo_ctrl/servo_ctrl.h"

#include <Arduino.h>

namespace laserSensorInternal {
namespace {
uint32_t lastInitializationAttemptMs[LASER_SENSOR_COUNT] = {};
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

    if (
      !sensorHasValue(index) ||
      now - sensorLastGoodMs(index) <= LASER_SENSOR_STALE_MS
    ) {
      continue;
    }

    Serial.print("VL53L0X stale: ");
    Serial.println(LASER_SENSOR_NAMES[index]);
    invalidateSensorReading(index);
  }
}

void retryMissingSensorsIfDue() {
  const uint32_t now = millis();

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (
      !sensorConfigured(index) ||
      sensorAvailable(index) ||
      now - lastInitializationAttemptMs[index] <
        LASER_SENSOR_REINIT_INTERVAL_MS
    ) {
      continue;
    }

    lastInitializationAttemptMs[index] = now;
    Serial.print("VL53L0X retry: ");
    Serial.println(LASER_SENSOR_NAMES[index]);

    if (initializeOneSensor(index)) {
      Serial.print("VL53L0X retry success: ");
    } else {
      Serial.print("VL53L0X retry failed: ");
    }
    Serial.println(LASER_SENSOR_NAMES[index]);
  }
}

void recoverI2cBusIfRequired() {
  if (!busRecoveryRequired()) {
    return;
  }

  clearBusRecoveryRequest();
  Serial.println("I2C BUS RECOVERY START");

  if (!i2cBusRestart()) {
    Serial.println("I2C BUS RECOVERY FAILED");
    return;
  }

  servoCtrlRestoreAfterI2cRecovery();
  markAllSensorsUnavailable();

  const uint32_t now = millis();

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      lastInitializationAttemptMs[index] = now;
      initializeOneSensor(index);
    }
  }

  Serial.println("I2C BUS RECOVERY SUCCESS");
}
}  // namespace

void initializeAllSensors() {
  lastSensorReadMs = 0;
  nextSensorToRead = 0;

  const uint32_t now = millis();
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    lastInitializationAttemptMs[index] = now;
  }

  clearAllSensorStates();
  markAllSensorsUnavailable();

  if (!i2cBusBegin()) {
    return;
  }

  servoCtrlRestoreAfterI2cRecovery();

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
  recoverI2cBusIfRequired();
}

}  // namespace laserSensorInternal
