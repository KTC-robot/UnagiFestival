#include <Arduino.h>

/**
 * @file main.cpp
 * @brief ESP32 firmwareの初期化順序と周期処理をまとめるentry point。
 */

#include "device/c620_driver.hpp"
#include "chassis_ctrl/chassis_ctrl.hpp"
#include "device/i2c_bus.hpp"
#include "im920/im920.hpp"
#include "laser_sensor/laser_sensor_ctrl.hpp"
#include "device/relay_driver.hpp"
#include "servo_ctrl/servo_ctrl.hpp"
#include "step_assist/step_assist_ctrl.hpp"

void setup() {
  Serial.begin(115200);
  delay(1000);

  delay(500);

  // 安全側出力を最初に確定し、device driver、制御、通信、sensor task、
  // StepAssistの順に初期化する。起動途中でもmotorは停止状態を維持する。
  relayDriverBegin();

  if (!c620DriverBegin()) {
    Serial.println(
      "[C620] 警告: CAN初期化に失敗したためmotorを停止状態に保ちます"
    );
  }

  if (!i2cBusBegin()) {
    Serial.println("[I2C] 警告: LaserSensor用Busの初期化に失敗しました");
  }

  servoCtrlBegin();
  chassisCtrlBegin();
  im920Begin();
  chassisCtrlStop();

  if (!laserSensorCtrlBegin()) {
    Serial.println("[LASER] 警告: 測距taskの起動に失敗しました");
  }

  stepAssistCtrlBegin();

  Serial.println();
  Serial.println("[SYSTEM] 初期化が完了しました");
  Serial.println();
}

void loop() {
  // 通信受信、CAN feedback、速度制御、電流送信の順序を毎loopで保つ。
  // timeout監視とStepAssistもblockingせず更新し、安全停止と状態遷移を行う。
  im920Update();

  c620DriverReadFrames();
  chassisCtrlUpdate();
  c620DriverSendPeriodically();

  im920CheckTimeout();
  stepAssistCtrlUpdate();
}
