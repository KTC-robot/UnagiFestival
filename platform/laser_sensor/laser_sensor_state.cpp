#include "laser_sensor/laser_sensor_state.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace laserSensorInternal {
namespace {
SensorState sensorStates[LASER_SENSOR_COUNT];
SemaphoreHandle_t stateMutex = nullptr;

class StateLock {
 public:
  StateLock() : locked_(false) {
    if (stateMutex == nullptr) {
      stateMutex = xSemaphoreCreateRecursiveMutex();
    }
    if (stateMutex != nullptr) {
      locked_ =
        xSemaphoreTakeRecursive(stateMutex, portMAX_DELAY) == pdTRUE;
    }
  }

  ~StateLock() {
    if (locked_) {
      xSemaphoreGiveRecursive(stateMutex);
    }
  }

 private:
  bool locked_;
};

bool correctedDistanceValid(int distanceMm) {
  return (
    distanceMm >= LASER_SENSOR_MIN_VALID_MM &&
    distanceMm <= LASER_SENSOR_MAX_VALID_MM
  );
}
}  // namespace

bool sensorConfigured(int index) {
  return (
    index >= 0 &&
    index < LASER_SENSOR_COUNT &&
    LASER_SENSOR_ENABLED[index]
  );
}

void clearOneSensorState(int index) {
  StateLock lock;
  if (index < 0 || index >= LASER_SENSOR_COUNT) {
    return;
  }

  sensorStates[index] = SensorState{};
}

void clearAllSensorStates() {
  StateLock lock;
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    clearOneSensorState(index);
  }
}

void markAllSensorsUnavailable() {
  StateLock lock;
  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    sensorStates[index].available = false;
  }
}

void invalidateSensorReading(int index) {
  StateLock lock;
  if (index < 0 || index >= LASER_SENSOR_COUNT) {
    return;
  }

  sensorStates[index].hasValue = false;
  sensorStates[index].distanceMm = 0;
  sensorStates[index].lastGoodMs = 0;
}

void markSensorAvailable(int index) {
  StateLock lock;
  if (index < 0 || index >= LASER_SENSOR_COUNT) {
    return;
  }

  sensorStates[index].available = true;
}

void disableSensor(int index) {
  StateLock lock;
  if (index < 0 || index >= LASER_SENSOR_COUNT) {
    return;
  }

  sensorStates[index].available = false;
  invalidateSensorReading(index);
  sensorStates[index].errorCount = 0;

  Serial.print("VL53L0X 無効化: ");
  Serial.println(LASER_SENSOR_NAMES[index]);
}

bool sensorAvailable(int index) {
  StateLock lock;
  return (
    index >= 0 &&
    index < LASER_SENSOR_COUNT &&
    sensorStates[index].available
  );
}

bool sensorHasValue(int index) {
  StateLock lock;
  return (
    index >= 0 &&
    index < LASER_SENSOR_COUNT &&
    sensorStates[index].hasValue
  );
}

uint32_t sensorLastGoodMs(int index) {
  StateLock lock;
  if (index < 0 || index >= LASER_SENSOR_COUNT) {
    return 0;
  }

  return sensorStates[index].lastGoodMs;
}

uint8_t sensorErrorCount(int index) {
  StateLock lock;
  if (index < 0 || index >= LASER_SENSOR_COUNT) {
    return 0;
  }

  return sensorStates[index].errorCount;
}

void incrementSensorErrorCount(int index) {
  StateLock lock;
  if (index < 0 || index >= LASER_SENSOR_COUNT) {
    return;
  }

  if (sensorStates[index].errorCount < LASER_SENSOR_MAX_ERROR_COUNT) {
    ++sensorStates[index].errorCount;
  }
}

void storeSensorReading(int index, uint16_t rawDistanceMm) {
  StateLock lock;
  if (!sensorConfigured(index)) {
    return;
  }

  const int correctedDistance =
    static_cast<int>(rawDistanceMm) + LASER_SENSOR_OFFSETS_MM[index];

  if (!correctedDistanceValid(correctedDistance)) {
    return;
  }

  SensorState& state = sensorStates[index];

  if (!state.hasValue) {
    state.distanceMm = correctedDistance;
    state.hasValue = true;
  } else {
    const int oldWeight =
      LASER_SENSOR_FILTER_WEIGHT_DENOMINATOR -
      LASER_SENSOR_FILTER_NEW_WEIGHT_NUMERATOR;

    state.distanceMm =
      (
        state.distanceMm * oldWeight +
        correctedDistance * LASER_SENSOR_FILTER_NEW_WEIGHT_NUMERATOR
      ) /
      LASER_SENSOR_FILTER_WEIGHT_DENOMINATOR;
  }

  state.lastGoodMs = millis();
  ++state.updateCount;
  state.errorCount = 0;
  state.available = true;
}

bool allConfiguredSensorsFresh() {
  StateLock lock;
  const uint32_t now = millis();
  bool anyConfigured = false;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index)) {
      continue;
    }

    anyConfigured = true;
    const SensorState& state = sensorStates[index];

    if (
      !state.available ||
      !state.hasValue ||
      now - state.lastGoodMs > LASER_SENSOR_STALE_MS
    ) {
      return false;
    }
  }

  return anyConfigured;
}

bool sensorFresh(int index) {
  StateLock lock;
  if (!sensorConfigured(index)) {
    return false;
  }

  const SensorState& state = sensorStates[index];

  return (
    state.available &&
    state.hasValue &&
    millis() - state.lastGoodMs <= LASER_SENSOR_STALE_MS
  );
}

int sensorDistanceMm(int index) {
  StateLock lock;
  if (!sensorFresh(index)) {
    return -1;
  }

  return sensorStates[index].distanceMm;
}

int connectedSensorCount() {
  StateLock lock;
  int count = 0;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index) && sensorStates[index].available) {
      ++count;
    }
  }

  return count;
}

int configuredSensorCount() {
  int count = 0;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      ++count;
    }
  }

  return count;
}

bool newMeasurementSetReady() {
  StateLock lock;
  bool anyConfigured = false;

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (!sensorConfigured(index)) {
      continue;
    }

    anyConfigured = true;
    const SensorState& state = sensorStates[index];

    if (
      !state.available ||
      !state.hasValue ||
      state.updateCount == 0 ||
      state.updateCount == state.lastEvaluatedUpdateCount
    ) {
      return false;
    }
  }

  if (!anyConfigured) {
    return false;
  }

  for (int index = 0; index < LASER_SENSOR_COUNT; ++index) {
    if (sensorConfigured(index)) {
      sensorStates[index].lastEvaluatedUpdateCount =
        sensorStates[index].updateCount;
    }
  }

  return true;
}

}  // namespace laserSensorInternal
