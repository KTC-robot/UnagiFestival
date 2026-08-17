#include "relay/relay_driver.hpp"

#include "relay/constants.h"

#include <Arduino.h>

namespace {

bool frontValveOn = false;
bool rearValveOn = false;

}  // namespace

bool relayDriverBegin()
{
    pinMode(RELAY_FRONT_VALVE_PIN, OUTPUT);
    pinMode(RELAY_REAR_VALVE_PIN, OUTPUT);

    relayDriverForceOff();

    Serial.println("リレー制御を初期化しました");
    Serial.print("前側電磁弁 GPIO=");
    Serial.println(RELAY_FRONT_VALVE_PIN);
    Serial.print("後側電磁弁 GPIO=");
    Serial.println(RELAY_REAR_VALVE_PIN);

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

    Serial.print("前側リレー: ");
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

    Serial.print("後側リレー: ");
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

void relayDriverForceOff()
{
    frontValveOn = false;
    rearValveOn = false;

    digitalWrite(
        RELAY_FRONT_VALVE_PIN,
        RELAY_OFF_LEVEL
    );

    digitalWrite(
        RELAY_REAR_VALVE_PIN,
        RELAY_OFF_LEVEL
    );

    Serial.println(
        "前後の電磁弁を安全側OFFにしました"
    );
}
