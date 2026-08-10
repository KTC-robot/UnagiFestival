#include <Adafruit_PWMServoDriver.h>

namespace CanConfig_servo_ctrl {  
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
}