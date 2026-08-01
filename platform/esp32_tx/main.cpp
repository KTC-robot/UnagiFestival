#include "Arduino.h"

#define LED_PIN 2

void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.printf("%s - run\n",__func__);
  pinMode(LED_PIN, OUTPUT);
}

void loop(){
  if (Serial.available() > 0) {
    char reception = Serial.read();
    Serial.printf("%c", reception);
    digitalWrite(LED_PIN, HIGH);
    delay(800);
    digitalWrite(LED_PIN, LOW);
    delay(800);
  }
}