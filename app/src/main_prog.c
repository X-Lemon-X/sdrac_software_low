#include "main.h"


int main_app(void)
{
  // Start the main loop
  while (1){
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
    HAL_Delay(300);
  }
}