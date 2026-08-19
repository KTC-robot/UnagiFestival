#include <Arduino.h>

// ============================================================
// MD20A 接続ピン
// ============================================================
constexpr int PWM_PIN = 25;
constexpr int DIR_PIN = 26;

// ============================================================
// モーター速度
// 0 ～ 255
// ============================================================
constexpr int MOTOR_PWM = 120;


// ============================================================
// 正転
// ============================================================
void motorForward()
{
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, MOTOR_PWM);

    Serial.println("MOTOR : FORWARD");
}


// ============================================================
// 逆転
// ============================================================
void motorReverse()
{
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, MOTOR_PWM);

    Serial.println("MOTOR : REVERSE");
}


// ============================================================
// 停止
// ============================================================
void motorStop()
{
    analogWrite(PWM_PIN, 0);

    Serial.println("MOTOR : STOP");
}


// ============================================================
// setup
// ============================================================
void setup()
{
    Serial.begin(115200);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    // 起動時は必ず停止
    motorStop();

    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println("775 + MD20A MOTOR TEST");
    Serial.println("==============================");
    Serial.println();
    Serial.println("F    -> 正転");
    Serial.println("R    -> 逆転");
    Serial.println("STOP -> 停止");
    Serial.println();
}


// ============================================================
// loop
// ============================================================
void loop()
{
    if (Serial.available()) {

        String command = Serial.readStringUntil('\n');

        command.trim();
        command.toUpperCase();

        // ----------------------------
        // 正転
        // ----------------------------
        if (command == "F") {

            motorForward();

        }

        // ----------------------------
        // 逆転
        // ----------------------------
        else if (command == "R") {

            motorReverse();

        }

        // ----------------------------
        // 停止
        // ----------------------------
        else if (command == "STOP") {

            motorStop();

        }

        // ----------------------------
        // 不明なコマンド
        // ----------------------------
        else {

            Serial.print("UNKNOWN COMMAND : ");
            Serial.println(command);
        }
    }
}