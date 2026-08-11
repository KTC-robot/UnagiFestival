#include "util.h"

#include <ctype.h>

#include "util/constants.h"

using namespace CanConfig_util;

bool utilIsHexChar(char c) {
  return isxdigit(static_cast<unsigned char>(c));
}

int utilHexCharToInt(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }

  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }

  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }

  return -1;
}

uint8_t utilHexByteToUint8(const String& text) {
  if (text.length() < 2) {
    return 0;
  }

  const int high = utilHexCharToInt(text[0]);
  const int low = utilHexCharToInt(text[1]);

  if (high < 0 || low < 0) {
    return 0;
  }

  return static_cast<uint8_t>((high << 4) | low);
}

int8_t utilToInt8(uint8_t value) {
  return static_cast<int8_t>(value);
}

String utilSanitizeAsciiLine(const String& line) {
  String cleaned;

  for (int index = 0; index < line.length(); ++index) {
    const char c = line[index];

    if (c >= 0x20 && c <= 0x7E) {
      cleaned += c;
    }
  }

  cleaned.trim();
  return cleaned;
}

String utilCollectHexChars(const String& text) {
  String hex;

  for (int index = 0; index < text.length(); ++index) {
    if (utilIsHexChar(text[index])) {
      hex += text[index];
    }
  }

  return hex;
}

String utilTextToHex(const String& text) {
  String hex;

  for (int index = 0; index < text.length(); ++index) {
    char buffer[3];
    snprintf(
      buffer,
      sizeof(buffer),
      "%02X",
      static_cast<uint8_t>(text[index])
    );
    hex += buffer;
  }

  return hex;
}

String utilButtonName(uint8_t id) {
  switch (id) {
    case 0: return "CROSS";
    case 1: return "CIRCLE";
    case 2: return "TRIANGLE";
    case 3: return "SQUARE";
    case 4: return "L1";
    case 5: return "R1";
    case 6: return "L2_BTN";
    case 7: return "R2_BTN";
    case 8: return "SHARE";
    case 9: return "OPTIONS";
    case 10: return "PS";
    case 11: return "L3";
    case 12: return "R3";
    case 13: return "TOUCHPAD";
    default: return "UNKNOWN";
  }
}

float utilJoyToFloat(int value) {
  if (abs(value) < JOY_DEAD) {
    value = 0;
  }

  value = constrain(value, -JOY_MAX, JOY_MAX);
  return static_cast<float>(value) / static_cast<float>(JOY_MAX);
}
