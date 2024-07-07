#include "led.h"          

void at32_led_toggle()
{
  GPIOB-> odt ^= GPIO_PINS_0;
}

