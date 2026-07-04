#include <Arduino.h>

HardwareSerial IM920(2);

const int IM920_RX = 16;
const int IM920_TX = 17;

const int LED_PIN = 2;
const unsigned long BLINK_MS = 100;

String imInput = "";

bool ledOn = false;
unsigned long ledOffAt = 0;

// 16進数1文字かどうかを判定する
bool isHexChar(char c) {
  return isxdigit((unsigned char)c);
}

// 文字列全体が偶数桁のHEXか判定する
// 例: 4C45445F4F4E -> true
// 例: HELLO -> false
bool isPureEvenHexString(const String& text) {
  if (text.length() == 0 || text.length() % 2 != 0) {
    return false;
  }

  for (int i = 0; i < text.length(); i++) {
    if (!isHexChar(text[i])) {
      return false;
    }
  }

  return true;
}

// IM920からの行頭に混ざる制御文字を除去する
String sanitizeAsciiLine(const String& line) {
  String cleaned = "";

  for (int i = 0; i < line.length(); i++) {
    char c = line[i];

    if (c >= 0x20 && c <= 0x7E) {
      cleaned += c;
    }
  }

  cleaned.trim();
  return cleaned;
}

// 例: 4C,45,44,5F,4F,4E -> LED_ON
String decodeCommaSeparatedHex(String payload) {
  String text = "";
  int start = 0;

  payload.trim();

  while (start < payload.length()) {
    int comma = payload.indexOf(',', start);

    String token = (comma == -1)
      ? payload.substring(start)
      : payload.substring(start, comma);

    token.trim();

    if (!isPureEvenHexString(token)) {
      return payload;
    }

    char c = (char)strtol(token.c_str(), nullptr, 16);

    if (c != '\0') {
      text += c;
    }

    if (comma == -1) {
      break;
    }

    start = comma + 1;
  }

  return text;
}

// 例: 4C45445F4F4E -> LED_ON
String decodeContinuousHex(String payload) {
  payload.trim();

  if (!isPureEvenHexString(payload)) {
    return "";
  }

  String text = "";

  for (int i = 0; i < payload.length(); i += 2) {
    String byteHex = payload.substring(i, i + 2);
    char c = (char)strtol(byteHex.c_str(), nullptr, 16);

    if (c != '\0') {
      text += c;
    }
  }

  return text;
}

// IM920のデータ部を文字列に変換する
String decodePayload(String payload) {
  payload.trim();

  if (payload.length() == 0) {
    return "";
  }

  // DCIO受信例: 4C,45,44,5F,4F,4E
  if (payload.indexOf(',') >= 0) {
    return decodeCommaSeparatedHex(payload);
  }

  // DCIO受信例: 4C45445F4F4E
  String decoded = decodeContinuousHex(payload);

  if (decoded.length() > 0) {
    return decoded;
  }

  // ECIO受信例: LED_ON
  return payload;
}

// 受信行から ":" より後ろのデータ部を取り出す
String extractPayloadText(String line) {
  line.trim();

  int colonIndex = line.indexOf(':');

  if (colonIndex >= 0) {
    return decodePayload(line.substring(colonIndex + 1));
  }

  return decodePayload(line);
}

void blinkLed() {
  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOffAt = millis() + BLINK_MS;
}

void updateLed() {
  if (ledOn && (long)(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}

void handleIm920Line(String rawLine) {
  String line = sanitizeAsciiLine(rawLine);

  if (line.length() == 0) {
    return;
  }

  Serial.print("IM920 RX RAW: ");
  Serial.println(line);

  // 起動時確認コマンドなどの応答は処理しない
  if (line == "OK" || line == "NG" || line.startsWith("IM920")) {
    return;
  }

  String payloadText = extractPayloadText(line);

  Serial.print("PAYLOAD: ");
  Serial.println(payloadText);

  if (payloadText == "LED_ON") {
    blinkLed();
  }
}

void readIm920Serial() {
  while (IM920.available()) {
    char c = IM920.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      handleIm920Line(imInput);
      imInput = "";
      continue;
    }

    imInput += c;
  }
}

void sendCommandToIm920(const String& cmd) {
  IM920.print(cmd);
  IM920.print("\r\n");

  Serial.print("CMD -> ");
  Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  IM920.begin(19200, SERIAL_8N1, IM920_RX, IM920_TX);

  Serial.println("ESP32 + IM920sL receive-only LED test");
  Serial.println("Expected payload: LED_ON");
  Serial.println();

  delay(500);

  // 起動確認だけ行う。不要ならこの2行も消してよい
  sendCommandToIm920("RDVR");
  delay(200);
  sendCommandToIm920("RDID");
}

void loop() {
  updateLed();
  readIm920Serial();
}