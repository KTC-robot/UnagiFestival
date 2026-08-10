#pragma once

#include <Arduino.h>

bool utilIsHexChar(char c);
int utilHexCharToInt(char c);
uint8_t utilHexByteToUint8(const String& text);
int8_t utilToInt8(uint8_t value);

String utilSanitizeAsciiLine(const String& line);
String utilCollectHexChars(const String& text);
String utilTextToHex(const String& text);
String utilButtonName(uint8_t id);

float utilJoyToFloat(int value);

