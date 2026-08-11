#include "laser_sensor/laser_sensor_ctrl.hpp"

#include "laser_sensor/laser_sensor_runtime.hpp"
#include "laser_sensor/laser_sensor_state.hpp"

#include <Arduino.h>

using namespace laserSensorInternal;

bool laserSensorCtrlBegin() {
  initializeAllSensors();

  const int configuredCount = configuredSensorCount();
  const int connectedCount = connectedSensorCount();

  Serial.print("VL53L0X connected: ");
  Serial.print(connectedCount);
  Serial.print(" / ");
  Serial.println(configuredCount);

  if (
    configuredCount > 0 &&
    connectedCount == configuredCount
  ) {
    Serial.println("VL53L0X センサー初期化完了");
    return true;
  }

  Serial.println(
    "VL53L0X: 有効なセンサーの一部または全部を初期化できませんでした"
  );
  return false;
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
