#include "main.h"
#include "encoder.hpp"
#include "main_prog.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "logger.hpp"
#include "usb_programer.hpp"
#include "steper_motor.hpp"
#include "filter.hpp"
#include "filter_alfa_beta.hpp"
#include "can_control.hpp"
#include "board_id.hpp"

#include <string>
#include <charconv>
#include <vector>

const Pin pin_user_led_1 = {GPIO_PIN_6, GPIOC};
const Pin pin_user_led_2 = {GPIO_PIN_7, GPIOC};
const Pin pin_user_btn_1 = {GPIO_PIN_9, GPIOC};
const Pin pin_tx_led = {GPIO_PIN_12, GPIOB};
const Pin pin_rx_led = {GPIO_PIN_13, GPIOB};
const Pin pin_encoder = {GPIO_PIN_3, GPIOB};
const Pin pin_poz_zero_sensor = {GPIO_PIN_4, GPIOA};
const Pin pin_inout_ca1 = {GPIO_PIN_5, GPIOA};
const Pin pin_inout_ca2 = {GPIO_PIN_7, GPIOA};
const Pin pin_inout_crx = {GPIO_PIN_10, GPIOB};
const Pin pin_inout_ctx = {GPIO_PIN_4, GPIOC};
const Pin pin_sync_puls = {GPIO_PIN_8, GPIOA};
const Pin pin_sync_dir = {GPIO_PIN_9, GPIOA};
const Pin pin_temp_steper_board = {GPIO_PIN_0, GPIOA};
const Pin pin_temp_board = {GPIO_PIN_1, GPIOA};
const Pin pin_temp_motor = {GPIO_PIN_2, GPIOA};
const Pin pin_vsense = {GPIO_PIN_3, GPIOA};
const Pin pin_steper_direction = {GPIO_PIN_0, GPIOB};
const Pin pin_steper_enable = {GPIO_PIN_1, GPIOB};
const Pin pin_steper_step = {GPIO_PIN_6, GPIOA};
const Pin pin_boot_device = {GPIO_PIN_8, GPIOC};

// to do
const Pin pin_cid_0 = {GPIO_PIN_0, GPIOC};
const Pin pin_cid_1 = {GPIO_PIN_1, GPIOC};
const Pin pin_cid_2 = {GPIO_PIN_2, GPIOC};

uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];


//-----------------------------------------------------------------------------------------------

LOGGER::Logger loger(LOGGER::LOG_LEVEL::LOG_LEVEL_DEBUG,false);
TIMING::Ticker ticker;
CAN_CONTROL::CanControl can_controler;
BOARD_ID::Board_id board_id(pin_cid_0, pin_cid_1, pin_cid_2);
uint32_t CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID;
uint32_t CAN_KONARM_X_STATUS_FRAME_ID;
uint32_t CAN_KONARM_X_SET_POS_FRAME_ID;
uint32_t CAN_KONARM_X_GET_POS_FRAME_ID;


void prefiferal_config();
void handle_can_rx();
void main_loop();
void id_config();


void main_prog()
{
  id_config();
  prefiferal_config();
  main_loop();
}

void handle_can_rx(){
  CAN_CONTROL::CAN_MSG msg = {0};
  if(can_controler.get_message(&msg)) return;

}

void id_config(){

  switch (board_id.get_id())
  {
  case SDRAC_ID_1:
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_1_CLEAR_ERRORS_FRAME_ID;
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_1_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_1_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_1_GET_POS_FRAME_ID;
    break;
  case SDRAC_ID_2:
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_2_CLEAR_ERRORS_FRAME_ID;
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_2_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_2_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_2_GET_POS_FRAME_ID;
    break;
  case SDRAC_ID_3:
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_3_CLEAR_ERRORS_FRAME_ID;
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_3_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_3_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_3_GET_POS_FRAME_ID;
    break;
  case SDRAC_ID_4:
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_4_CLEAR_ERRORS_FRAME_ID;
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_4_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_4_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_4_GET_POS_FRAME_ID;
    break;
  case SDRAC_ID_5:
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_5_CLEAR_ERRORS_FRAME_ID;
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_5_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_5_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_5_GET_POS_FRAME_ID;
    break;
  // case SDRAC_ID_6:
  //   CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_6_CLEAR_ERRORS_FRAME_ID;
  //   CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_6_STATUS_FRAME_ID;
  //   CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_6_SET_POS_FRAME_ID;
  //   CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_6_GET_POS_FRAME_ID;
  //   break;
  default:
    break;
  }

}

void prefiferal_config(){

  // dma adc1 settings
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 3);

  // timer 10 settings
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn,6,0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_TIM_Base_Start_IT(&htim10);

  // timer 3 settings
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Can1 settings
  CAN_FilterTypeDef can_filter;
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  can_controler.init(hcan1, CAN_FILTER_FIFO0, ticker, pin_tx_led, pin_rx_led);
  HAL_CAN_Start(&hcan1);
}

void main_loop(){
  log_debug("Start main_prog\n");

  TIMING::Timing tim_blink(ticker);
  TIMING::Timing tim_encoder(ticker);
  TIMING::Timing tim_usb(ticker);
  TIMING::Timing tim_eng(ticker);
  tim_blink.set_behaviour(100000, true);
  tim_encoder.set_behaviour(10000, true);
  tim_usb.set_behaviour(20000000, true);
  tim_eng.set_behaviour(500000, true);

  FILTERS::FilterAlfaBeta filter(ticker);
  filter.alfa = 0.5;
  filter.beta = 0.1;
  FILTERS::FilterBase filter_base(ticker);  

  ENCODER::Encoder encoder(hi2c1,ticker,filter_base,filter_base);
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
  stp_motor.set_speed(PI/12.0f);

  USB_PROGRAMER::UsbProgramer usb_programer(pin_boot_device);
  // Start the main loop
  float angle;
  float velocity;
  uint16_t raw_angle;

  while (1){
    // log_debug("main loop\n");
    // USBD_UsrLog("main loop");

    can_controler.handle_led_blink();

    if(tim_blink.triggered()){
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
    }

    if(tim_encoder.triggered()){
      angle = encoder.read_angle();
      raw_angle = encoder.read_raw_angle();
      velocity = ticker.get_seconds();
      // velocity = encoder.get_velocity();
      log_debug(std::to_string(raw_angle) + ";" + std::to_string(velocity));
    }

    if(tim_usb.triggered()){
      // log_debug("angle:" + std::to_string(angle) + " velocity:" + std::to_string(velocity));
    }

    if (tim_eng.triggered()){
      // stp_motor.set_speed(PI_m2);
    }
  }
}