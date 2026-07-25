#include "im920_comm.h"

#include "can_comm.h"
#include "chassis_ctrl.h"
#include "servo_ctrl.h"
#include "util.h"

namespace {
HardwareSerial IM920(2);

constexpr int IM920_RX = 16;
constexpr int IM920_TX = 17;

constexpr int LED_PIN = 2;
constexpr uint32_t LED_PULSE_MS = 30;
constexpr uint32_t COMM_TIMEOUT_MS = 600;
constexpr uint32_t STATUS_TX_INTERVAL_MS = 1000;
constexpr int JOY_ACK_INTERVAL = 10;

constexpr int DRIVE_POWER_STEP = 5;
constexpr bool ENABLE_REPLY_TO_PI = true;
constexpr bool SHOW_RAW = false;
constexpr bool SHOW_JOY = true;

String imLine;
String pcLine;

bool ledOn = false;
uint32_t ledOffAt = 0;

uint32_t lastRxMs = 0;
uint32_t lastStatusTxMs = 0;
uint32_t joyPacketCount = 0;

int8_t lastDpadX = 0;
int8_t lastDpadY = 0;

bool isKnownPacketType(uint8_t type) {
  return (
    type == 0x42 ||
    type == 0x44 ||
    type == 0x4A ||
    type == 0x53
  );
}

String extractPayloadHex(String line) {
  line.trim();

  if (line.length() == 0) {
    return "";
  }

  const int colonIndex = line.indexOf(':');

  if (colonIndex >= 0) {
    const String payload = line.substring(colonIndex + 1);
    const String hex = utilCollectHexChars(payload);

    if (hex.length() >= 2) {
      const uint8_t type =
        utilHexByteToUint8(hex.substring(0, 2));

      if (isKnownPacketType(type)) {
        return hex;
      }
    }

    return "";
  }

  const String hex = utilCollectHexChars(line);

  if (hex.length() >= 2) {
    const uint8_t type =
      utilHexByteToUint8(hex.substring(0, 2));

    if (isKnownPacketType(type)) {
      return hex;
    }
  }

  return "";
}

void pulseLed() {
  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOffAt = millis() + LED_PULSE_MS;
}

void updateLed() {
  if (ledOn && static_cast<int32_t>(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}

void sendCommandToIm920(const String& command) {
  IM920.print(command);
  IM920.print("\r\n");

  Serial.print("CMD -> ");
  Serial.println(command);
}

void handleButtonPacket(const String& hex) {
  if (hex.length() < 6) {
    return;
  }

  const uint8_t id = utilHexByteToUint8(hex.substring(2, 4));
  const uint8_t state = utilHexByteToUint8(hex.substring(4, 6));

  Serial.print("BUTTON <- ");
  Serial.print(utilButtonName(id));
  Serial.print(" state=");
  Serial.println(state);

  String reply = "BTN ";
  reply += utilButtonName(id);
  reply += " ";
  reply += String(state);
  im920CommSendText(reply);

  if (state != 1) {
    return;
  }

  if (id == 10) {
    Serial.println("EMERGENCY STOP by PS button");
    chassisCtrlStop();
    lastDpadX = 0;
    lastDpadY = 0;
  } else if (id == 4) {
    chassisCtrlChangePower(-DRIVE_POWER_STEP);
  } else if (id == 5) {
    chassisCtrlChangePower(DRIVE_POWER_STEP);
  } else if (id == 0) {
    Serial.println("STOP by CROSS button");
    chassisCtrlStop();
  }
}

void handleDpadPacket(const String& hex) {
  if (hex.length() < 6) {
    return;
  }

  const uint8_t axis = utilHexByteToUint8(hex.substring(2, 4));
  const int8_t value =
    utilToInt8(utilHexByteToUint8(hex.substring(4, 6)));

  if (axis == 0) {
    lastDpadX = value;
  } else if (axis == 1) {
    lastDpadY = value;
  }

  Serial.print("DPAD <- ");
  Serial.print(axis == 0 ? "X" : "Y");
  Serial.print(" value=");
  Serial.println(value);

  chassisCtrlSetFromJoy(
    0,
    0,
    0,
    lastDpadX,
    lastDpadY
  );

  String reply = "DPAD ";
  reply += axis == 0 ? "X " : "Y ";
  reply += String(value);
  im920CommSendText(reply);
}

void handleJoyPacket(const String& hex) {
  if (hex.length() < 18) {
    return;
  }

  const int8_t lx =
    utilToInt8(utilHexByteToUint8(hex.substring(2, 4)));

  const int8_t ly =
    utilToInt8(utilHexByteToUint8(hex.substring(4, 6)));

  const int8_t rx =
    utilToInt8(utilHexByteToUint8(hex.substring(6, 8)));

  const int8_t ry =
    utilToInt8(utilHexByteToUint8(hex.substring(8, 10)));

  const uint8_t l2 =
    utilHexByteToUint8(hex.substring(10, 12));

  const uint8_t r2 =
    utilHexByteToUint8(hex.substring(12, 14));

  const int8_t dpadX =
    utilToInt8(utilHexByteToUint8(hex.substring(14, 16)));

  const int8_t dpadY =
    utilToInt8(utilHexByteToUint8(hex.substring(16, 18)));

  lastDpadX = dpadX;
  lastDpadY = dpadY;

  if (SHOW_JOY) {
    Serial.print("JOY <- LX=");
    Serial.print(lx);
    Serial.print(" LY=");
    Serial.print(ly);
    Serial.print(" RX=");
    Serial.print(rx);
    Serial.print(" RY=");
    Serial.print(ry);
    Serial.print(" L2=");
    Serial.print(l2);
    Serial.print(" R2=");
    Serial.print(r2);
    Serial.print(" DPX=");
    Serial.print(dpadX);
    Serial.print(" DPY=");
    Serial.println(dpadY);
  }

  chassisCtrlSetFromJoy(lx, ly, rx, dpadX, dpadY);

  ++joyPacketCount;

  if (joyPacketCount % JOY_ACK_INTERVAL == 0) {
    String reply = "JOY OK PWR=";
    reply += String(chassisCtrlGetPowerPercent());
    im920CommSendText(reply);
  }
}

void handlePayloadHex(const String& hex) {
  if (hex.length() < 2) {
    return;
  }

  const uint8_t type =
    utilHexByteToUint8(hex.substring(0, 2));

  if (!isKnownPacketType(type)) {
    return;
  }

  lastRxMs = millis();
  pulseLed();

  if (type == 0x42) {
    handleButtonPacket(hex);
  } else if (type == 0x44) {
    handleDpadPacket(hex);
  } else if (type == 0x4A) {
    handleJoyPacket(hex);
  } else if (type == 0x53) {
    servoCtrlHandlePacket(hex);
  }
}

void handleIm920Line(const String& rawLine) {
  const String line = utilSanitizeAsciiLine(rawLine);

  if (line.length() == 0) {
    return;
  }

  if (SHOW_RAW) {
    Serial.print("IM920 RAW <- ");
    Serial.println(line);
  }

  if (
    line == "OK" ||
    line == "NG" ||
    line.startsWith("IM920")
  ) {
    return;
  }

  const String hex = extractPayloadHex(line);

  if (hex.length() > 0) {
    handlePayloadHex(hex);
  }
}

void readIm920Serial() {
  while (IM920.available()) {
    const char c = static_cast<char>(IM920.read());

    if (c == '\r' || c == '\n') {
      imLine.trim();

      if (imLine.length() > 0) {
        handleIm920Line(imLine);
        imLine = "";
      }

      continue;
    }

    imLine += c;

    if (imLine.length() > 180) {
      imLine = "";
    }
  }
}

void readPcSerialCommand() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      pcLine.trim();

      if (pcLine.length() > 0) {
        sendCommandToIm920(pcLine);
        pcLine = "";
      }

      continue;
    }

    pcLine += c;

    if (pcLine.length() > 120) {
      pcLine = "";
    }
  }
}
}

void im920CommBegin() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  IM920.begin(19200, SERIAL_8N1, IM920_RX, IM920_TX);
  delay(1000);

  sendCommandToIm920("RDID");
  delay(100);
  sendCommandToIm920("RDNN");
  delay(100);
  sendCommandToIm920("RDGN");
  delay(100);
  sendCommandToIm920("RDCH");

  lastRxMs = millis();

  Serial.println("IM920 UART ready: RX=16 TX=17 19200bps");
}

void im920CommUpdate() {
  updateLed();
  readIm920Serial();
  readPcSerialCommand();
}

void im920CommCheckTimeout() {
  if (millis() - lastRxMs <= COMM_TIMEOUT_MS) {
    return;
  }

  if (chassisCtrlIsActive()) {
    Serial.println("COMM TIMEOUT -> STOP");
    chassisCtrlStop();
  }

  lastRxMs = millis();
}

void im920CommSendPeriodicStatus() {
  if (!ENABLE_REPLY_TO_PI) {
    return;
  }

  const uint32_t now = millis();

  if (now - lastStatusTxMs < STATUS_TX_INTERVAL_MS) {
    return;
  }

  lastStatusTxMs = now;

  String message =
    chassisCtrlIsActive() ? "STAT RUN " : "STAT STOP ";

  message += "PWR=";
  message += String(chassisCtrlGetPowerPercent());
  message += canCommIsReady() ? " CAN=1" : " CAN=0";
  message += " FB=";
  message += String(canCommGetFeedbackMask(), HEX);

  im920CommSendText(message);
}

void im920CommSendText(const String& text) {
  if (!ENABLE_REPLY_TO_PI || text.length() == 0) {
    return;
  }

  const String command = "TXDA " + utilTextToHex(text);
  sendCommandToIm920(command);
}
