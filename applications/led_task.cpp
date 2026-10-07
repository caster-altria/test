#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task()
{
  led.start();

  while (true) {
    for (uint8_t g = 0; g < 10; g++) {
      led.set(0, g * 0.01f, 0);
      osDelay(100);
    }

    for (uint8_t g = 10; g > 0; g--) {
      led.set(0, g * 0.01f, 0);
      osDelay(100);
    }

    for (uint8_t r = 0; r < 10; r++) {
      led.set(r * 0.1f, 0, 0);
      osDelay(100);
    }

    for (uint8_t r = 10; r > 0; r--) {
      led.set(r * 0.1f, 0, 0);
      osDelay(100);
    }

    for (uint8_t y = 0; y < 10; y++) {
      led.set(0, 0, y * 0.1f);
      osDelay(100);
    }

    for (uint8_t y = 10; y > 0; y--) {
      led.set(0, 0, y * 0.1f);
      osDelay(100);
    }
  }
}