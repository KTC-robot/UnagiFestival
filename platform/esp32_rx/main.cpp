#include <Arduino.h>

#include "can_comm/can_comm.h"
#include "chassis_ctrl/chassis_ctrl.h"
#include "im920_comm/im920_comm.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"

void setup() {
  Serial.begin(115200);
  delay(1000);

  delay(500);

  relayCtrlBegin();

  if (!canCommBegin()) {
    Serial.println(
      "WARNING: CAN initialization failed. Motors remain stopped."
    );
  }

  if (!laserSensorCtrlBegin()) {
    Serial.println("WARNING: laser sensor initialization failed.");
  }

  chassisCtrlBegin();
  im920CommBegin();
  chassisCtrlStop();

  Serial.println();
  Serial.println("READY");
  Serial.println();
}

void loop() {
  im920CommUpdate();

  canCommReadFrames();
  chassisCtrlUpdate();
  canCommSendPeriodically();

  laserSensorCtrlUpdate();

  im920CommCheckTimeout();
  im920CommSendPeriodicStatus();
}
