#include "stm32f4xx_hal.h"
#include "main.h"

#ifndef PIN_HPP
#define PIN_HPP

struct Pin {
  uint16_t pin;
  GPIO_TypeDef *port;
};

#endif