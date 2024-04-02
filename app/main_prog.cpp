#include "main.h"
#include "encoder.hpp"
#include "main_prog.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "logger.hpp"
#include "usb_programer.hpp"
#include "steper_motor.hpp"


#include <string>
#include <charconv>

// extern ADC_HandleTypeDef hadc1;
// extern CAN_HandleTypeDef hcan1;
// extern I2C_HandleTypeDef hi2c1;
// extern TIM_HandleTypeDef htim2;
// extern TIM_HandleTypeDef htim3;
void main_prog_setings();

LOGGER::Logger loger(LOGGER::LOG_LEVEL::LOG_LEVEL_DEBUG);
TIMING::Ticker ticker;

uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];

Pin pin_user_led_1 = {GPIO_PIN_6, GPIOC};
Pin pin_user_led_2 = {GPIO_PIN_7, GPIOC};
Pin pin_user_btn_1 = {GPIO_PIN_9, GPIOC};
Pin pin_tx_led = {GPIO_PIN_12, GPIOB};
Pin pin_rx_led = {GPIO_PIN_13, GPIOB};
Pin pin_encoder = {GPIO_PIN_3, GPIOB};
Pin pin_poz_zero_sensor = {GPIO_PIN_4, GPIOA};
Pin pin_inout_ca1 = {GPIO_PIN_5, GPIOA};
Pin pin_inout_ca2 = {GPIO_PIN_7, GPIOA};
Pin pin_inout_crx = {GPIO_PIN_10, GPIOB};
Pin pin_inout_ctx = {GPIO_PIN_4, GPIOC};
Pin pin_sync_puls = {GPIO_PIN_8, GPIOA};
Pin pin_sync_dir = {GPIO_PIN_9, GPIOA};
Pin pin_temp_steper_board = {GPIO_PIN_0, GPIOA};
Pin pin_temp_board = {GPIO_PIN_1, GPIOA};
Pin pin_temp_motor = {GPIO_PIN_2, GPIOA};
Pin pin_vsense = {GPIO_PIN_3, GPIOA};
Pin pin_steper_direction = {GPIO_PIN_0, GPIOB};
Pin pin_steper_enable = {GPIO_PIN_1, GPIOB};
Pin pin_steper_step = {GPIO_PIN_6, GPIOA};
Pin pin_boot_device = {GPIO_PIN_8, GPIOC};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10){
    ticker.irq_update_ticker();
  }
  
  // if (htim->Instance == TIM3)
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // log_debug("ADC1: " + std::to_string(adc_dma_buffer[0]));
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
}


int main_prog(void)
{
  main_prog_setings();

  log_debug("Start main_prog\n");

  TIMING::Timing tim_blink(ticker);
  TIMING::Timing tim_encoder(ticker);
  TIMING::Timing tim_usb(ticker);
  TIMING::Timing tim_eng(ticker);
  tim_blink.set_behaviour(100000, true);
  tim_encoder.set_behaviour(10000, true);
  tim_usb.set_behaviour(200000, true);
  tim_eng.set_behaviour(500000, true);

  ENCODER::Encoder encoder(hi2c1,ticker);
  encoder.address = ENCODER_MT6701_I2C_ADDRESS;
  encoder.resolution = ENCODER_MT6702_RESOLUTION;
  encoder.angle_register = ENCODER_MEM_ADDR_ANNGLE;
  encoder.offset = 0;
  encoder.reverse = false;
  encoder.enable_filter = false;
  encoder.enable_velocity= true;

  STEPER_MOTOR::SteperMotor stp_motor(htim3, TIM_CHANNEL_1, pin_steper_direction, pin_steper_enable);
  stp_motor.steps_per_revolution = 400;
  stp_motor.gear_ratio = 75;
  stp_motor.max_velocity = 10;
  stp_motor.reverse = false;
  stp_motor.init();
  stp_motor.set_enable(true);
  stp_motor.set_speed(PI/8.0f);

  USB_PROGRAMER::UsbProgramer usb_programer(pin_boot_device);
  // Start the main loop
  float angle;
  float velocity;
  while (1){
    // log_debug("main loop\n");
    // USBD_UsrLog("main loop");

    if(tim_blink.triggered()){
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
    }

    if(tim_encoder.triggered()){
      angle = encoder.read_angle();
      velocity = encoder.get_velocity();
    }

    if(tim_usb.triggered()){
      log_debug("angle:" + std::to_string(angle) + " velocity:" + std::to_string(velocity));
    }

    if (tim_eng.triggered()){
      // stp_motor.set_speed(PI_m2);
    }
  }
}


void main_prog_setings(){

  // dma adc1 settings
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 3);

  // timer 10 settings
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn,6,0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_TIM_Base_Start_IT(&htim10);

  // timer 3 settings
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}