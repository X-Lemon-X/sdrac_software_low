#include "main.h"
#include "encoder.hpp"
#include "main_prog.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "logger.hpp" 
#include <string.h>

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

LOGGER::Logger log(LOGGER::LOG_LEVEL::LOG_LEVEL_DEBUG);

int main_app(void)
{
  log.debug("Start main_app\n");
  HAL_I2C_MspInit(&hi2c1);

  encoder_encoder encoder;
  encoder_init_struct(&encoder);
  encoder.hi2c = &hi2c1;
  encoder.address = 0b0000110 << 1; //ENCODER_MT6701_I2C_ADDRESS;
  encoder.resolution = ENCODER_MT6702_RESOLUTION;

  //send string to the  usb
  
  encoder_set_offset(&encoder, 0, 0);
  encoder_read_raw_angle(&encoder);
  
  // printf("angle: %f\n", angle);
  // printf("angle: %f\n", angle);
  // sprintf(log_buffer, "angle: %f\n", angle);
  
  char *msg = "World\n";
  char *msg2 = "Hello\n";
  // Start the main loop
  while (1){
    // log_debug("main loop\n");
    // USBD_UsrLog("main loop");

    int angle = encoder_read_raw_angle(&encoder);
    // CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(10);
    log.debug("angle: \n");
    log.debug(std::to_string(angle));
    // USBD_UsrLog("angle: %f", angle);
    // CDC_Transmit_FS((uint8_t*)msg2, strlen(msg2));
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    HAL_Delay(300);
  }
}