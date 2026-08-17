#include "laser_sensor_runtime.hpp"

#include "constants.h"
#include "../i2c/i2c_bus.hpp"
#include "laser_sensor_device.hpp"
#include "laser_sensor_state.hpp"

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

void invalidateStaleSensorReadings() {
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

  Serial.println("I2C BUS RESTART SUCCESS");

  markAllSensorsUnavailable();

  const uint32_t now = millis();
  bool tcaReady = i2cBusDisableAllTcaChannels();
  bool allSensorsRecovered = tcaReady;

  if (!tcaReady) {
    Serial.println("TCA9548A recovery failed");
  }

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index)) {
      continue;
    }

    lastInitializationAttemptMs[index] = now;

    if (tcaReady && !initializeOneSensor(index)) {
      allSensorsRecovered = false;
    }
  }

  if (allSensorsRecovered) {
    Serial.println("I2C DEVICE RECOVERY SUCCESS");
  } else {
    Serial.println("I2C DEVICE RECOVERY PARTIAL");
  }
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

  if (!i2cBusDisableAllTcaChannels()) {
    Serial.println("TCA9548A 初期化失敗");
    return;
  }

  Serial.println("TCA9548A 接続成功");

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      initializeOneSensor(index);
    }
  }
}

void updateSensors() {
  readNextAvailableSensorIfDue();
  invalidateStaleSensorReadings();
  retryMissingSensorsIfDue();
  recoverI2cBusIfRequired();
}

}  // namespace laserSensorInternal
