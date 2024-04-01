#include "main.h"
#include "encoder.hpp"
#include "main_prog.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "logger.hpp" 
#include <string>
#include <charconv>

// extern ADC_HandleTypeDef hadc1;
// extern CAN_HandleTypeDef hcan1;
// extern I2C_HandleTypeDef hi2c1;
// extern TIM_HandleTypeDef htim2;
// extern TIM_HandleTypeDef htim3;

LOGGER::Logger loger(LOGGER::LOG_LEVEL::LOG_LEVEL_DEBUG);
TIMING::Ticker ticker;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10){
    ticker.irq_update_ticker();
  }
  
  // if (htim->Instance == TIM3)
}


int main_prog(void)
{
  loger.debug("Start main_app\n");
  // HAL_I2C_MspInit(&hi2c1);

  TIMING::Ticker ticker;
  TIMING::Timing timing(ticker);
  timing.set_behaviour(500000, true);
  // TIMING::Timing timing2(ticker);

  ENCODER::Encoder encoder(&hi2c1);
  encoder.address = ENCODER_MT6701_I2C_ADDRESS;
  encoder.resolution = ENCODER_MT6702_RESOLUTION;
  encoder.angle_register = ENCODER_MEM_ADDR_ANNGLE;
  encoder.set_offset(0, false);


  loger.debug("Encoder status True if ready:"+ std::to_string(encoder.ping_encoder()));

  //send string to the  usb
  
  // encoder_set_offset(&encoder, 0, 0);
  // encoder_read_raw_angle(&encoder);
  
  // printf("angle: %f\n", angle);
  // printf("angle: %f\n", angle);
  // sprintf(log_buffer, "angle: %f\n", angle);

  // Start the main loop
  while (1){
    // log_debug("main loop\n");
    // USBD_UsrLog("main loop");

    if(timing.triggered()){
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
      
      float angle_f = encoder.read_angle();
      float velocity = encoder.get_velocity();
      // CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
      // auto st= std::string(std::to_chars(velocity));
      // loger.debug("raw angle:" + std::to_string(angle));
      loger.debug("angle:");
      loger.debug("velocity 2:" + std::to_string(velocity));
      // loger.debug("triggered");
      // USBD_UsrLog("triggered");
    }
    
    // USBD_UsrLog("angle: %f", angle);
    // CDC_Transmit_FS((uint8_t*)msg2, strlen(msg2));
    int angle = encoder.read_raw_angle();
    loger.debug("raw angle:" + std::to_string(angle));
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    // HAL_Delay(30);
  }
}