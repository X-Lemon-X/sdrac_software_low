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
#include "CanDB.h"
#include "movement_controler.hpp"

#include <string>
#include <charconv>
#include <vector>

//**************************************************************************************************
// Gpio assigments
GPIO_PIN pin_user_led_1 = {GPIO_PIN_6, GPIOC};
GPIO_PIN pin_user_led_2 = {GPIO_PIN_7, GPIOC};
GPIO_PIN pin_user_btn_1 = {GPIO_PIN_9, GPIOC};
GPIO_PIN pin_tx_led = {GPIO_PIN_12, GPIOB};
GPIO_PIN pin_rx_led = {GPIO_PIN_13, GPIOB};
GPIO_PIN pin_encoder = {GPIO_PIN_3, GPIOB};
GPIO_PIN pin_poz_zero_sensor = {GPIO_PIN_4, GPIOA};
GPIO_PIN pin_inout_ca1 = {GPIO_PIN_5, GPIOA};
GPIO_PIN pin_inout_ca2 = {GPIO_PIN_7, GPIOA};
GPIO_PIN pin_inout_crx = {GPIO_PIN_10, GPIOB};
GPIO_PIN pin_inout_ctx = {GPIO_PIN_4, GPIOC};
GPIO_PIN pin_sync_puls = {GPIO_PIN_8, GPIOA};
GPIO_PIN pin_sync_dir = {GPIO_PIN_9, GPIOA};
GPIO_PIN pin_temp_steper_board = {GPIO_PIN_0, GPIOA};
GPIO_PIN pin_temp_board = {GPIO_PIN_1, GPIOA};
GPIO_PIN pin_temp_motor = {GPIO_PIN_2, GPIOA};
GPIO_PIN pin_vsense = {GPIO_PIN_3, GPIOA};
GPIO_PIN pin_steper_direction = {GPIO_PIN_0, GPIOB};
GPIO_PIN pin_steper_enable = {GPIO_PIN_1, GPIOB};
GPIO_PIN pin_steper_step = {GPIO_PIN_6, GPIOA};
GPIO_PIN pin_boot_device = {GPIO_PIN_8, GPIOC};

// to do
 GPIO_PIN pin_cid_0 = {GPIO_PIN_0, GPIOC};
 GPIO_PIN pin_cid_1 = {GPIO_PIN_1, GPIOC};
 GPIO_PIN pin_cid_2 = {GPIO_PIN_2, GPIOC};

//**************************************************************************************************
// Global stuff

uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];

TIMING::Ticker main_clock;
LOGGER::Logger loger(LOGGER::LOG_LEVEL::LOG_LEVEL_DEBUG,false);
BOARD_ID::Board_id board_id(pin_cid_0, pin_cid_1, pin_cid_2);
STEPER_MOTOR::SteperMotor stp_motor(htim3, TIM_CHANNEL_1, pin_steper_direction, pin_steper_enable);
CAN_CONTROL::CanControl can_controler;
MOVEMENT_CONTROLER::MovementControler movement_controler;
ENCODER::Encoder encoder;
USB_PROGRAMER::UsbProgramer usb_programer(pin_boot_device);

//**************************************************************************************************
// Id dependable configuration 
uint32_t CAN_X_FILTER_MASK_LOW;
uint32_t CAN_X_FILTER_MASK_HIGH;
uint32_t CAN_X_FILTER_ID_LOW;
uint32_t CAN_X_FILTER_ID_HIGH;
uint32_t CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID;
uint32_t CAN_KONARM_X_STATUS_FRAME_ID;
uint32_t CAN_KONARM_X_SET_POS_FRAME_ID;
uint32_t CAN_KONARM_X_GET_POS_FRAME_ID;




/// @brief This function is used to configure the periferals
/// mostly stff that have to be configurated after CumeMX generation
void periferal_config();

/// @brief This function is used to handle the can controll
void handle_can_rx();

/// @brief This function is used to handle the main loop, never returns
void main_loop();

/// @brief This function is used to configure the board base on it's hardware id
void id_config();

/// @brief This function is used to init the interfaces
void init_controls();

//**************************************************************************************************
void main_prog()
{
  log_debug("Start main_prog\n");
  id_config();
  periferal_config();
  init_controls();
  main_loop();
}

void id_config(){
  log_debug("Start id_config\n");
  log_debug("Board id: " + std::to_string(board_id.get_id()));
  switch (board_id.get_id())
  {
  case SDRAC_ID_1:
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_1_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_1_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_1_GET_POS_FRAME_ID;
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_1_CLEAR_ERRORS_FRAME_ID;
    
    // ids 11bit 0b110 0001 0000  and 18 bit 0b00 0000 0000 0000 0000
    //mask 11bit 0b111 1111 0000  and 18 bit 0b00 0000 0000 0000 0000
    CAN_X_FILTER_ID_HIGH = 0x610<<5;   // why 5 bits shift because we want tu push the 11 bit id to the 16 bit regiser strting from 5th bit
    CAN_X_FILTER_ID_LOW = 0x000;
    CAN_X_FILTER_MASK_HIGH = 0xff0<<5;
    CAN_X_FILTER_MASK_LOW = 0x000;

    //-------------------STEPER MOTOR CONFIGURATION-------------------
    stp_motor.set_steps_per_revolution(400);
    stp_motor.set_gear_ratio(75);
    stp_motor.set_max_velocity(10);
    stp_motor.set_min_velocity(0.1);
    stp_motor.set_reverse(false);

    //-------------------ENCODER CONFIGURATION-------------------
    encoder.set_offset(0.0);
    encoder.set_reverse(false);
    encoder.set_enable_filter(false);
    encoder.set_enable_velocity(true);

    //-------------------MOVEMENT CONTROLER CONFIGURATION-------------------
    movement_controler.set_limit_position(-PI/2, PI/2);
    movement_controler.set_max_velocity(PI);

    break;
  case SDRAC_ID_2:
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_2_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_2_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_2_GET_POS_FRAME_ID;
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_2_CLEAR_ERRORS_FRAME_ID;

    CAN_X_FILTER_ID_HIGH = 0x620<<5;
    CAN_X_FILTER_ID_LOW = 0x000;
    CAN_X_FILTER_MASK_HIGH = 0xff0<<5;
    CAN_X_FILTER_MASK_LOW = 0x000;
    break;
  case SDRAC_ID_3:
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_3_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_3_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_3_GET_POS_FRAME_ID;
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_3_CLEAR_ERRORS_FRAME_ID;

    CAN_X_FILTER_ID_HIGH = 0x630<<5;
    CAN_X_FILTER_ID_LOW = 0x000;
    CAN_X_FILTER_MASK_HIGH = 0xff0<<5;
    CAN_X_FILTER_MASK_LOW = 0x000;
    break;
  case SDRAC_ID_4:
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_4_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_4_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_4_GET_POS_FRAME_ID;
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_4_CLEAR_ERRORS_FRAME_ID;

    CAN_X_FILTER_ID_HIGH = 0x640<<5;
    CAN_X_FILTER_ID_LOW = 0x000;
    CAN_X_FILTER_MASK_HIGH = 0xff0<<5;
    CAN_X_FILTER_MASK_LOW = 0x000;
    break;
  case SDRAC_ID_5:
    CAN_KONARM_X_STATUS_FRAME_ID = CAN_KONARM_5_STATUS_FRAME_ID;
    CAN_KONARM_X_SET_POS_FRAME_ID = CAN_KONARM_5_SET_POS_FRAME_ID;
    CAN_KONARM_X_GET_POS_FRAME_ID = CAN_KONARM_5_GET_POS_FRAME_ID;
    CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = CAN_KONARM_5_CLEAR_ERRORS_FRAME_ID;

    CAN_X_FILTER_ID_HIGH = 0x650<<5;
    CAN_X_FILTER_ID_LOW = 0x000;
    CAN_X_FILTER_MASK_HIGH = 0xff0<<5;
    CAN_X_FILTER_MASK_LOW = 0x000;
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

void periferal_config(){
  log_debug("Start periferal_config\n");
  // dma adc1 settings
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 3);

  // timer 10 settings
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn,6,0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_TIM_Base_Start_IT(&htim10);

  // timer 3 settings
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_CAN_DeInit(&hcan1);
  HAL_CAN_Init(&hcan1);

  // Can1 settings
  CAN_FilterTypeDef can_filter;
  can_filter.FilterBank = 1;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.FilterIdHigh = CAN_X_FILTER_ID_HIGH;
  can_filter.FilterIdLow = CAN_X_FILTER_ID_LOW;
  can_filter.FilterMaskIdHigh = CAN_X_FILTER_MASK_HIGH;
  can_filter.FilterMaskIdLow = CAN_X_FILTER_MASK_LOW;
  can_filter.SlaveStartFilterBank = 2;
  HAL_StatusTypeDef status =  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  can_controler.init(hcan1, CAN_FILTER_FIFO0, main_clock, pin_tx_led, pin_rx_led);
  status =  HAL_CAN_Start(&hcan1);
  status =  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

  
}

void handle_can_rx(){
  CAN_CONTROL::CAN_MSG recived_msg = {0};
  if(can_controler.get_message(&recived_msg)) return;

  log_debug("Recived message: " + std::to_string(recived_msg.frame_id));
  
  if(recived_msg.frame_id == CAN_KONARM_X_SET_POS_FRAME_ID){
    can_konarm_1_set_pos_t signals;
    can_konarm_1_set_pos_unpack(&signals, recived_msg.data, recived_msg.data_size);
    float targetPosition = can_konarm_1_set_pos_position_decode(signals.position);
    float targetVelocity = can_konarm_1_set_pos_velocity_decode(signals.velocity);
    movement_controler.set_velocity(targetVelocity);
    movement_controler.set_position(targetPosition);
  }
  else if (recived_msg.frame_id == CAN_KONARM_X_GET_POS_FRAME_ID && recived_msg.remote_request){
    CAN_CONTROL::CAN_MSG send_msg = {0};
    can_konarm_1_get_pos_t src_p;
    send_msg.frame_id = CAN_KONARM_X_GET_POS_FRAME_ID;
    src_p.position = can_konarm_1_get_pos_position_encode(movement_controler.get_current_position());
    src_p.velocity = can_konarm_1_get_pos_velocity_encode(movement_controler.get_current_velocity());
    send_msg.data_size = CAN_KONARM_1_GET_POS_LENGTH;
    can_konarm_1_get_pos_pack(send_msg.data, &src_p, send_msg.data_size);
    can_controler.send_message(send_msg);
  }
  else if (recived_msg.frame_id == CAN_KONARM_X_STATUS_FRAME_ID && recived_msg.remote_request){
    CAN_CONTROL::CAN_MSG send_msg = {0};
    can_konarm_1_status_t src_p;
    send_msg.frame_id = CAN_KONARM_X_STATUS_FRAME_ID;
    src_p.status = can_konarm_1_status_status_encode(CAN_KONARM_1_STATUS_STATUS_OK_CHOICE);
    send_msg.data_size = CAN_KONARM_1_STATUS_LENGTH;
    can_konarm_1_status_pack(send_msg.data, &src_p, send_msg.data_size);
    can_controler.send_message(send_msg);
  }
  else if (recived_msg.frame_id == CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID){
  }
  
}

void init_controls(){
  log_debug("Start init_interfaces\n");

  FILTERS::FilterBase fb(main_clock);  
  encoder.set_address(ENCODER_MT6701_I2C_ADDRESS);
  encoder.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder.set_angle_register(ENCODER_MEM_ADDR_ANNGLE);
  encoder.init(hi2c1,main_clock,fb,fb); 
  stp_motor.init();
  stp_motor.set_enable(false);
  movement_controler.init(main_clock, stp_motor, encoder);
  movement_controler.set_position(encoder.get_angle());
  movement_controler.set_velocity(0);
  movement_controler.set_enable(false);
}

void main_loop(){
  log_debug("Start main_loop\n");
  TIMING::Timing tim_blink(main_clock);
  TIMING::Timing tim_encoder(main_clock);
  TIMING::Timing tim_usb(main_clock);
  TIMING::Timing tim_movement(main_clock);
  tim_blink.set_behaviour(500000, true);
  tim_encoder.set_behaviour(1000, true);
  tim_usb.set_behaviour(500000, true);
  tim_movement.set_behaviour(1000, true);

  // Start the main loop
  while (1){
    handle_can_rx();
    can_controler.handle_led_blink();
    
    if(tim_encoder.triggered()){
      encoder.handle();
    }

    movement_controler.handle();

    // if (tim_movement.triggered()){
    //   movement_controler.handle();
    // }

    if(tim_usb.triggered()){
      usb_programer.handler();
    }

    if(tim_blink.triggered()){
      TOGGLE_GPIO(pin_user_led_1);
    }

  }
}