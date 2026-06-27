#include <Arduino.h>

HardwareSerial IM920(2);

const int IM920_RX = 16;  // ESP32 RX2 ← IM920sL TXD
const int IM920_TX = 17;  // ESP32 TX2 → IM920sL RXD

String pcInput = "";
String imInput = "";

String toHex(String text) {
  String hex = "";

  for (int i = 0; i < text.length(); i++) {
    char buf[3];
    sprintf(buf, "%02X", (uint8_t)text[i]);
    hex += buf;
  }

  return hex;
}

void sendCommand(String cmd) {
  IM920.print(cmd);
  IM920.print('\r');

  Serial.print("CMD -> ");
  Serial.println(cmd);
}

void sendText(String text) {
  text.trim();
  if (text.length() == 0) return;

  String hexData = toHex(text);
  String cmd = "TXDA " + hexData;

  sendCommand(cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  IM920.begin(19200, SERIAL_8N1, IM920_RX, IM920_TX);

  Serial.println("ESP32 + IM920sL Send/Receive Test");
  Serial.println("Type text and press Enter to send.");
  Serial.println();

  delay(500);

  sendCommand("RDVR");
  delay(200);
  sendCommand("RDID");
}

void loop() {
  // PCシリアルモニタ → ESP32 → IM920sLへ送信
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') {
      continue;
    } else if (c == '\n') {
      sendText(pcInput);
      pcInput = "";
    } else {
      pcInput += c;
    }
  }

  // IM920sL → ESP32 → PCシリアルモニタへ表示
  while (IM920.available()) {
    char c = IM920.read();

    if (c == '\r') {
      continue;
    } else if (c == '\n') {
      if (imInput.length() > 0) {
        Serial.print("IM920 RX: ");
        Serial.println(imInput);
        imInput = "";
      }
    } else {
      imInput += c;
    }
  }
}