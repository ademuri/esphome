#pragma once

#include <cstdint>

namespace esphome {

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void yield();

void set_millis(uint32_t t);

}  // namespace esphome
