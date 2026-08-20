#include "device/ws2812_driver.hpp"

#include <Adafruit_NeoPixel.h>

namespace {
constexpr int WS2812_DATA_PIN = 13;
constexpr uint16_t WS2812_LED_COUNT = 1;

Adafruit_NeoPixel pixels(
  WS2812_LED_COUNT,
  WS2812_DATA_PIN,
  NEO_GRB + NEO_KHZ800
);
}  // namespace

void ws2812DriverBegin()
{
  pixels.begin();
  pixels.clear();
  pixels.show();
}

void ws2812DriverSetRgb(uint8_t red, uint8_t green, uint8_t blue)
{
  const uint32_t color = pixels.Color(red, green, blue);
  for (uint16_t index = 0; index < WS2812_LED_COUNT; ++index)
  {
    pixels.setPixelColor(index, color);
  }
  pixels.show();
}
