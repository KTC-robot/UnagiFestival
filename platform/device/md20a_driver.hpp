#pragma once

#include <stdint.h>

enum class Md20aState : uint8_t {
  STOPPED = 0,
  FORWARD = 1,
  REVERSE = 2,
};

void md20aDriverBegin();
void md20aDriverSetState(Md20aState state);
Md20aState md20aDriverGetState();
