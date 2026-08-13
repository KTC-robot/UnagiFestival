#include <Arduino.h>

#include "can_comm/can_comm.h"
#include "chassis_ctrl/chassis_ctrl.h"
#include "i2c/i2c_bus.hpp"
#include "im920_comm/im920_comm.h"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "relay/relay_ctrl.hpp"
#include "servo_ctrl/servo_ctrl.h"
#include "step_assist/step_assist_ctrl.hpp"

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

  if (!i2cBusBegin()) {
    Serial.println("WARNING: shared I2C bus initialization failed.");
  }

  servoCtrlBegin();
  chassisCtrlBegin();
  im920CommBegin();
  chassisCtrlStop();

  if (!laserSensorCtrlBegin()) {
    Serial.println("WARNING: laser sensor task startup failed.");
  }

  stepAssistCtrlBegin();

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
  // stepAssistCtrlUpdate();
}
