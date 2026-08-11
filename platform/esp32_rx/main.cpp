#include <Arduino.h>

#include "can_comm/can_comm.h"
#include "chassis_ctrl/chassis_ctrl.h"
#include "im920_comm/im920_comm.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"
#include "servo_ctrl/servo_ctrl.h"

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

  servoCtrlBegin();
  chassisCtrlBegin();
  im920CommBegin();
  chassisCtrlStop();

  if (!laserSensorCtrlBegin()) {
    Serial.println("WARNING: laser sensor task startup failed.");
  }

  Serial.println();
  Serial.println("READY");
  Serial.println();
}

void loop() {
  im920CommUpdate();

  canCommReadFrames();
  chassisCtrlUpdate();
  canCommSendPeriodically();

  im920CommCheckTimeout();
  im920CommSendPeriodicStatus();
}
