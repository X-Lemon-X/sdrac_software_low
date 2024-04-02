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
  log_debug("Start main_prog\n");

  TIMING::Timing tim_blink(ticker);
  TIMING::Timing tim_encoder(ticker);
  tim_blink.set_behaviour(100000, true);
  tim_encoder.set_behaviour(100000, true);
  
  ENCODER::Encoder encoder(hi2c1,ticker);
  encoder.address = ENCODER_MT6701_I2C_ADDRESS;
  encoder.resolution = ENCODER_MT6702_RESOLUTION;
  encoder.angle_register = ENCODER_MEM_ADDR_ANNGLE;
  encoder.offset = 0;
  encoder.reverse = false;
  encoder.enable_filter = true;
  encoder.enable_velocity= true;
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

    if(tim_blink.triggered()){
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
    }

    if(tim_encoder.triggered()){
      float angle = encoder.read_angle();
      float velocity = encoder.get_velocity();
      log_debug("angle:" + std::to_string(angle) + " velocity:" + std::to_string(velocity));
    }


  }
}