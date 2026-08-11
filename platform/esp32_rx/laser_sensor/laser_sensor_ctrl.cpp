#include "laser_sensor/laser_sensor_ctrl.hpp"

#include "laser_sensor/laser_sensor_runtime.hpp"
#include "laser_sensor/laser_sensor_state.hpp"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace laserSensorInternal;

namespace {
TaskHandle_t laserSensorTaskHandle = nullptr;

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
}

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

void laserSensorCtrlUpdate() {
  updateSensors();
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
