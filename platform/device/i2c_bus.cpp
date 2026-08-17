#include "device/i2c_bus.hpp"

#include <Wire.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {
constexpr int LASER_SENSOR_I2C_SDA_PIN = 21;
constexpr int LASER_SENSOR_I2C_SCL_PIN = 22;
constexpr uint32_t LASER_SENSOR_I2C_CLOCK_HZ = 50000;
constexpr uint16_t LASER_SENSOR_I2C_TIMEOUT_MS = 100;
constexpr uint32_t LASER_SENSOR_I2C_RESTART_DELAY_MS = 100;

SemaphoreHandle_t i2cMutex = nullptr;

bool ensureMutex() {
  if (i2cMutex == nullptr) {
    i2cMutex = xSemaphoreCreateRecursiveMutex();
  }

  return i2cMutex != nullptr;
}

void applySettingsUnlocked() {
  Wire.setClock(LASER_SENSOR_I2C_CLOCK_HZ);
  Wire.setTimeOut(LASER_SENSOR_I2C_TIMEOUT_MS);
}

bool beginWireUnlocked(bool restart) {
  if (restart) {
    Wire.end();
    delay(LASER_SENSOR_I2C_RESTART_DELAY_MS);
  }

  const bool result = Wire.begin(
    LASER_SENSOR_I2C_SDA_PIN,
    LASER_SENSOR_I2C_SCL_PIN,
    LASER_SENSOR_I2C_CLOCK_HZ
  );

  if (!result) {
    Serial.println("I2C初期化失敗");
    return false;
  }

  applySettingsUnlocked();

  Serial.print("I2C初期化成功 SDA=");
  Serial.print(LASER_SENSOR_I2C_SDA_PIN);
  Serial.print(" SCL=");
  Serial.print(LASER_SENSOR_I2C_SCL_PIN);
  Serial.print(" clock=");
  Serial.print(LASER_SENSOR_I2C_CLOCK_HZ);
  Serial.println("Hz");

  return true;
}

}  // namespace

I2cBusLockGuard::I2cBusLockGuard()
    : locked_(i2cBusLock()) {}

I2cBusLockGuard::~I2cBusLockGuard() {
  if (locked_) {
    i2cBusUnlock();
  }
}

bool I2cBusLockGuard::locked() const {
  return locked_;
}

bool i2cBusLock() {
  return ensureMutex() &&
    xSemaphoreTakeRecursive(i2cMutex, portMAX_DELAY) == pdTRUE;
}

void i2cBusUnlock() {
  if (i2cMutex != nullptr) {
    xSemaphoreGiveRecursive(i2cMutex);
  }
}

void i2cBusApplySettings() {
  I2cBusLockGuard lock;
  if (!lock.locked()) {
    return;
  }

  applySettingsUnlocked();
}

bool i2cBusBegin() {
  I2cBusLockGuard lock;
  if (!lock.locked()) {
    return false;
  }

  return beginWireUnlocked(false);
}

bool i2cBusRestart() {
  I2cBusLockGuard lock;
  if (!lock.locked()) {
    return false;
  }

  Serial.println("I2Cバスを再初期化します");
  return beginWireUnlocked(true);
}

bool i2cBusWriteByteLocked(uint8_t address, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool i2cBusProbeDevice(uint8_t address) {
  I2cBusLockGuard lock;
  if (!lock.locked()) {
    return false;
  }

  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

int i2cBusReadSda() {
  return digitalRead(LASER_SENSOR_I2C_SDA_PIN);
}

int i2cBusReadScl() {
  return digitalRead(LASER_SENSOR_I2C_SCL_PIN);
}
