#include "relay_ctrl.hpp"

#include "relay/constants.h"

#include <Arduino.h>

namespace {

bool frontValveOn = false;
bool rearValveOn = false;

}  // namespace

bool relayCtrlBegin()
{
    pinMode(RELAY_FRONT_VALVE_PIN, OUTPUT);
    pinMode(RELAY_REAR_VALVE_PIN, OUTPUT);

    relayCtrlForceOff();

    Serial.println("リレー制御を初期化しました");
    Serial.print("前側電磁弁 GPIO=");
    Serial.println(RELAY_FRONT_VALVE_PIN);
    Serial.print("後側電磁弁 GPIO=");
    Serial.println(RELAY_REAR_VALVE_PIN);

    return true;
}

void relayCtrlSetFront(bool on)
{
    if (frontValveOn == on) {
        return;
    }

    frontValveOn = on;

    digitalWrite(
        RELAY_FRONT_VALVE_PIN,
        on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL
    );

    Serial.print("前側電磁弁: ");
    Serial.println(
        on
            ? "ON（シリンダー伸長 / 機構DOWN）"
            : "OFF（シリンダー収縮 / 機構UP）"
    );
}

void relayCtrlSetRear(bool on)
{
    if (rearValveOn == on) {
        return;
    }

    rearValveOn = on;

    digitalWrite(
        RELAY_REAR_VALVE_PIN,
        on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL
    );

    Serial.print("後側電磁弁: ");
    Serial.println(
        on
            ? "ON（シリンダー伸長 / 機構DOWN）"
            : "OFF（シリンダー収縮 / 機構UP）"
    );
}

bool relayCtrlFrontOn()
{
    return frontValveOn;
}

bool relayCtrlRearOn()
{
    return rearValveOn;
}

void relayCtrlForceOff()
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