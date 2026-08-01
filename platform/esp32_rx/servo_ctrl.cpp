#include "servo_ctrl.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace {
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint8_t SERVO_CHANNEL_COUNT = 16;
constexpr uint16_t SERVO_PWM_FREQ = 50;

Adafruit_PWMServoDriver servoDriver(PCA9685_ADDRESS);

const uint8_t SERVO_MIN_ANGLE[SERVO_CHANNEL_COUNT] = {
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

const uint8_t SERVO_MAX_ANGLE[SERVO_CHANNEL_COUNT] = {
  180, 180, 180, 180,
  180, 180, 180, 180,
  180, 180, 180, 180,
  180, 180, 180, 180
};

const uint16_t SERVO_MIN_US[SERVO_CHANNEL_COUNT] = {
  600, 600, 600, 600,
  600, 600, 600, 600,
  600, 600, 600, 600,
  600, 600, 600, 600
};

const uint16_t SERVO_MAX_US[SERVO_CHANNEL_COUNT] = {
  2400, 2400, 2400, 2400,
  2400, 2400, 2400, 2400,
  2400, 2400, 2400, 2400,
  2400, 2400, 2400, 2400
};

const bool SERVO_REVERSED[SERVO_CHANNEL_COUNT] = {
  false, false, false, false,
  false, false, false, false,
  false, false, false, false,
  false, false, false, false
};

bool servoOutputActive[SERVO_CHANNEL_COUNT] = {};
uint8_t servoLastAngle[SERVO_CHANNEL_COUNT] = {
  90, 90, 90, 90,
  90, 90, 90, 90,
  90, 90, 90, 90,
  90, 90, 90, 90
};

uint16_t microsecondsToPcaTicks(uint16_t pulseUs) {
  const uint32_t periodUs = 1000000UL / SERVO_PWM_FREQ;
  uint32_t ticks = (static_cast<uint32_t>(pulseUs) * 4096UL) / periodUs;

  if (ticks > 4095UL) {
    ticks = 4095UL;
  }

  return static_cast<uint16_t>(ticks);
}

void disableServo(uint8_t channel) {
  if (channel >= SERVO_CHANNEL_COUNT) {
    return;
  }

  servoDriver.setPWM(channel, 0, 4096);
  servoOutputActive[channel] = false;
}

bool setServoAngle(uint8_t channel, uint8_t requestedAngle) {
  if (channel >= SERVO_CHANNEL_COUNT) {
    return false;
  }

  const uint8_t minimum = SERVO_MIN_ANGLE[channel];
  const uint8_t maximum = SERVO_MAX_ANGLE[channel];

  if (minimum > maximum) {
    return false;
  }

  const uint8_t angle = constrain(requestedAngle, minimum, maximum);
  uint8_t physicalAngle = angle;

  if (SERVO_REVERSED[channel]) {
    physicalAngle = maximum - (angle - minimum);
  }

  uint16_t pulseUs = SERVO_MIN_US[channel];

  if (maximum != minimum) {
    pulseUs = map(
      physicalAngle,
      minimum,
      maximum,
      SERVO_MIN_US[channel],
      SERVO_MAX_US[channel]
    );
  }

  const uint16_t ticks = microsecondsToPcaTicks(pulseUs);
  servoDriver.setPWM(channel, 0, ticks);

  servoOutputActive[channel] = true;
  servoLastAngle[channel] = angle;

  Serial.print("SERVO CH=");
  Serial.print(channel);
  Serial.print(" ANGLE=");
  Serial.print(angle);
  Serial.print(" PULSE_US=");
  Serial.print(pulseUs);
  Serial.print(" TICKS=");
  Serial.println(ticks);

  return true;
}
}

void servoCtrlBegin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_PWM_FREQ);
  delay(10);

  servoCtrlDisableAll();

  Serial.println(
    "PCA9685 ready: SDA=21 SCL=22 ADDRESS=0x40, CH0-CH15 FULL OFF"
  );
}

void servoCtrlDisableAll() {
  for (uint8_t channel = 0; channel < SERVO_CHANNEL_COUNT; ++channel) {
    disableServo(channel);
  }
}

void servoCtrlHandlePacket(const String& hex) {
  if (hex.length() < 6) {
    return;
  }

  const uint8_t channel =
    static_cast<uint8_t>(strtoul(hex.substring(2, 4).c_str(), nullptr, 16));
  const uint8_t angle =
    static_cast<uint8_t>(strtoul(hex.substring(4, 6).c_str(), nullptr, 16));

  if (channel >= SERVO_CHANNEL_COUNT || angle > 180) {
    Serial.print("SERVO INVALID CH=");
    Serial.print(channel);
    Serial.print(" ANGLE=");
    Serial.println(angle);
    return;
  }

  if (!setServoAngle(channel, angle)) {
    Serial.println("SERVO SET FAILED");
  }
}
