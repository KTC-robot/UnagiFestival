#include "im920_comm.h"

#include "can_comm/can_comm.h"
#include "chassis_ctrl/chassis_ctrl.h"
#include "servo_ctrl/servo_ctrl.h"
#include "util/util.h"
#include "im920_comm/constants.h"

using namespace CanConfig_im920_comm;

namespace {
HardwareSerial IM920(2);

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
    type == 0x43 ||
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

void flushIm920Input() {
  while (IM920.available()) {
    IM920.read();
  }
}

String readIm920LineBlocking(uint32_t timeoutMs) {
  String line;
  const uint32_t startedMs = millis();

  while (millis() - startedMs < timeoutMs) {
    while (IM920.available()) {
      const char c = static_cast<char>(IM920.read());

      if (c == '\r' || c == '\n') {
        line.trim();

        if (line.length() > 0) {
          return utilSanitizeAsciiLine(line);
        }

        line = "";
        continue;
      }

      line += c;

      if (line.length() > 180) {
        line = "";
      }
    }

    delay(1);
  }

  return "";
}

String queryIm920Setting(const char* command) {
  flushIm920Input();
  sendCommandToIm920(command);

  const String response = readIm920LineBlocking(IM920_QUERY_TIMEOUT_MS);

  Serial.print("IM920 CFG ");
  Serial.print(command);
  Serial.print(" <- ");
  Serial.println(response.length() > 0 ? response : "<NO RESPONSE>");

  return response;
}

void printIm920Configuration() {
  Serial.println("--- IM920 configuration check ---");

  const String version = queryIm920Setting("RDVR");
  const String id = queryIm920Setting("RDID");
  const String node = queryIm920Setting("RDNN");
  const String group = queryIm920Setting("RDGN");
  const String channel = queryIm920Setting("RDCH");

  (void)version;
  (void)id;
  (void)node;

  if (group.length() > 0 && group != EXPECTED_IM920_GROUP) {
    Serial.print("WARNING: IM920 group mismatch. expected=");
    Serial.print(EXPECTED_IM920_GROUP);
    Serial.print(" actual=");
    Serial.println(group);
  }

  if (channel.length() > 0 && channel != EXPECTED_IM920_CHANNEL) {
    Serial.print("WARNING: IM920 channel mismatch. expected=");
    Serial.print(EXPECTED_IM920_CHANNEL);
    Serial.print(" actual=");
    Serial.println(channel);
  }

  Serial.println("--- IM920 configuration check end ---");
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

void handleControlPacket(const String& hex) {
  if (hex.length() < 6) {
    return;
  }

  const uint8_t commandId =
    utilHexByteToUint8(hex.substring(2, 4));

  const int8_t value =
    utilToInt8(utilHexByteToUint8(hex.substring(4, 6)));

  Serial.print("CONTROL <- ID=");
  Serial.print(commandId);
  Serial.print(" value=");
  Serial.println(value);

  switch (commandId) {
    case 1:  // STOP
      chassisCtrlStop();
      lastDpadX = 0;
      lastDpadY = 0;
      im920CommSendText("CTRL STOP");
      break;

    case 2:  // EMERGENCY_STOP
      Serial.println("EMERGENCY STOP by CONTROL command");
      chassisCtrlStop();
      lastDpadX = 0;
      lastDpadY = 0;
      im920CommSendText("CTRL ESTOP");
      break;

    case 3:  // CHANGE_POWER
      chassisCtrlChangePower(value);
      {
        String reply = "CTRL PWR=";
        reply += String(chassisCtrlGetPowerPercent());
        im920CommSendText(reply);
      }
      break;

    default:
      Serial.println("CONTROL unknown command");
      break;
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
  } else if (type == 0x43) {
    handleControlPacket(hex);
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

  printIm920Configuration();

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
