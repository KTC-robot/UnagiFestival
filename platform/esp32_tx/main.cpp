#include "Arduino.h"

#define LED_PIN 2

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.printf("%s - run\n",__func__);
  pinMode(LED_PIN, OUTPUT);
}

void loop(){
  digitalWrite(LED_PIN, HIGH);
  Serial.printf("%s - LED_PIN - HIGH\n",__func__);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.printf("%s - LED_PIN - LOW\n",__func__);
  delay(3000);
}