#include "test_helpers.h"
#include "esphome/core/application.h"

namespace esphome {

static uint32_t mock_millis_value = 0;

uint32_t millis() { return mock_millis_value; }
uint32_t micros() { return mock_millis_value * 1000; }
void delay(uint32_t ms) { mock_millis_value += ms; }
void delayMicroseconds(uint32_t us) {}
void yield() {}

void set_millis(uint32_t t) {
  mock_millis_value = t;
#ifdef ESPHOME_TEST
  App.set_loop_component_start_time(t);
#endif
}

}  // namespace esphome
