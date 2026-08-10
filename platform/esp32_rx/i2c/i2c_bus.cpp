#include "i2c_bus.hpp"

#include "constants.h"

#include <Wire.h>

void i2cBusApplySettings()
{
    Wire.setClock(LASER_SENSOR_I2C_CLOCK_HZ);
    Wire.setTimeOut(LASER_SENSOR_I2C_TIMEOUT_MS);
}

bool i2cBusDisableAllTcaChannels()
{
    Wire.beginTransmission(
        LASER_SENSOR_TCA9548A_ADDRESS
    );

    Wire.write(0x00);

    const uint8_t result =
        Wire.endTransmission(true);

    if (result != 0) {
        Serial.print(
            "TCA9548A 全CH無効化失敗 result="
        );
        Serial.println(result);

        return false;
    }

    return true;
}

bool i2cBusBegin()
{
    Wire.end();

    delay(
        LASER_SENSOR_I2C_RESTART_DELAY_MS
    );

    const bool result = Wire.begin(
        LASER_SENSOR_I2C_SDA_PIN,
        LASER_SENSOR_I2C_SCL_PIN,
        LASER_SENSOR_I2C_CLOCK_HZ
    );

    if (!result) {
        Serial.println(
            "I2C初期化失敗"
        );

        return false;
    }

    i2cBusApplySettings();

    Serial.print("I2C初期化成功 SDA=");
    Serial.print(LASER_SENSOR_I2C_SDA_PIN);
    Serial.print(" SCL=");
    Serial.print(LASER_SENSOR_I2C_SCL_PIN);
    Serial.print(" clock=");
    Serial.print(LASER_SENSOR_I2C_CLOCK_HZ);
    Serial.println("Hz");

    // TCA9548A存在確認
    Wire.beginTransmission(
        LASER_SENSOR_TCA9548A_ADDRESS
    );

    const uint8_t probeResult =
        Wire.endTransmission(true);

    if (probeResult != 0) {
        Serial.print(
            "TCA9548A応答なし result="
        );
        Serial.println(probeResult);

        return false;
    }

    Serial.println(
        "TCA9548A 接続成功"
    );

    return i2cBusDisableAllTcaChannels();
}

bool i2cBusRestart()
{
    Serial.println(
        "I2Cバスを再初期化します"
    );

    return i2cBusBegin();
}

bool i2cBusSelectTcaChannel(uint8_t channel)
{
    if (
        channel >=
        LASER_SENSOR_TCA9548A_CHANNEL_COUNT
    ) {
        return false;
    }

    Wire.beginTransmission(
        LASER_SENSOR_TCA9548A_ADDRESS
    );

    Wire.write(
        static_cast<uint8_t>(
            1U << channel
        )
    );

    const uint8_t result =
        Wire.endTransmission(true);

    if (result != 0) {
        return false;
    }

    delay(
        LASER_SENSOR_CHANNEL_SETTLE_MS
    );

    return true;
}

int i2cBusReadSda()
{
    return digitalRead(
        LASER_SENSOR_I2C_SDA_PIN
    );
}

int i2cBusReadScl()
{
    return digitalRead(
        LASER_SENSOR_I2C_SCL_PIN
    );
}