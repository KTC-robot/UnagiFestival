#include <Arduino.h>

#include "can_comm.h"
#include "chassis_ctrl.h"
#include "im920_comm.h"
#include "servo_ctrl.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================================");
  Serial.println("ESP32 Robot Controller - Modular Version");
  Serial.println("================================================");

  servoCtrlBegin();

  if (!canCommBegin()) {
    Serial.println(
      "WARNING: CAN initialization failed. Motors remain stopped."
    );
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

  im920CommCheckTimeout();
  im920CommSendPeriodicStatus();
}
