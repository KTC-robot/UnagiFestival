#include "device/relay_driver.hpp"

#include <Arduino.h>

namespace {

constexpr int RELAY_FRONT_VALVE_PIN = 23;
constexpr int RELAY_REAR_VALVE_PIN = 32;
constexpr int RELAY_AIR_VALVE_PIN = 33;
constexpr uint8_t RELAY_ON_LEVEL = HIGH;
constexpr uint8_t RELAY_OFF_LEVEL = LOW;

bool frontValveOn = false;
bool rearValveOn = false;
bool airValveOn = false;

}  // namespace

bool relayDriverBegin()
{
    pinMode(RELAY_FRONT_VALVE_PIN, OUTPUT);
    pinMode(RELAY_REAR_VALVE_PIN, OUTPUT);
    pinMode(RELAY_AIR_VALVE_PIN, OUTPUT);

    relayDriverForceOff();

    Serial.println("[RELAY] GPIOの初期化が完了しました");
    Serial.print("[RELAY] 前側電磁弁 GPIO=");
    Serial.println(RELAY_FRONT_VALVE_PIN);
    Serial.print("[RELAY] 後側電磁弁 GPIO=");
    Serial.println(RELAY_REAR_VALVE_PIN);
    Serial.print("[RELAY] Air電磁弁 GPIO=");
    Serial.println(RELAY_AIR_VALVE_PIN);

    return true;
}

void relayDriverSetFront(bool on)
{
    if (frontValveOn == on) {
        return;
    }

    frontValveOn = on;

    digitalWrite(
        RELAY_FRONT_VALVE_PIN,
        on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL
    );

    Serial.print("[RELAY] 前側出力=");
    Serial.println(on ? "ON" : "OFF");
}

void relayDriverSetRear(bool on)
{
    if (rearValveOn == on) {
        return;
    }

    rearValveOn = on;

    digitalWrite(
        RELAY_REAR_VALVE_PIN,
        on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL
    );

    Serial.print("[RELAY] 後側出力=");
    Serial.println(on ? "ON" : "OFF");
}

bool relayDriverFrontOn()
{
    return frontValveOn;
}

bool relayDriverRearOn()
{
    return rearValveOn;
}

void relayDriverSetAir(bool on)
{
    if (airValveOn == on) return;
    airValveOn = on;
    digitalWrite(RELAY_AIR_VALVE_PIN, on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
}

bool relayDriverAirOn()
{
    return airValveOn;
}

void relayDriverForceOff()
{
    frontValveOn = false;
    rearValveOn = false;
    airValveOn = false;

    digitalWrite(
        RELAY_FRONT_VALVE_PIN,
        RELAY_OFF_LEVEL
    );

    digitalWrite(
        RELAY_REAR_VALVE_PIN,
        RELAY_OFF_LEVEL
    );

    digitalWrite(RELAY_AIR_VALVE_PIN, RELAY_OFF_LEVEL);

    Serial.println("[RELAY] 全電磁弁を安全側OFFにしました");
}
