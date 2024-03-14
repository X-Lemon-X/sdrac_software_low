#include "main.h"
#include "encoder.h"
#include "main_prog.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "logger.h" 

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

int main_app(void)
{
  log_level = LOG_LEVEL_DEBUG;
  encoder_encoder encoder;
  encoder_init_struct(&encoder);
  encoder.hi2c = &hi2c1;
  encoder.address = ENCODER_MT6701_I2C_ADDRESS;
  encoder.resolution = ENCODER_MT6702_RESOLUTION;

  //send string to the  usb
  char str[] = "Hello World\n";
  
  
  encoder_set_offset(&encoder, 0, 0);
  encoder_read_raw_angle(&encoder);
  float angle = encoder_read_angle(&encoder);
  log_debug("angle: \n");

  // Start the main loop
  while (1){
    log_debug("main loop\n");
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
    HAL_Delay(300);
  }
}