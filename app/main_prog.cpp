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
#include "filter_moving_avarage.hpp"
#include "can_control.hpp"
#include "board_id.hpp"
#include "CanDB.h"
#include "movement_controler.hpp"
#include "pd_controler.hpp"
#include "pin.hpp"
#include "ntc_termistors.hpp"
#include "version.hpp"
#include <cfloat>
#include "config_struct.hpp"
#include "id_config.hpp"
#include "MCP9700AT.hpp"

#include <string>
#include <charconv>
#include <vector>

//**************************************************************************************************
// Gpio assigments
GPIO_PIN pin_user_led_1 = {GPIO_PIN_6, GPIOC};
GPIO_PIN pin_user_led_2 = {GPIO_PIN_7, GPIOC};
GPIO_PIN pin_user_btn_1 = {GPIO_PIN_9, GPIOA}; // GPIO_PIN_9, GPIOC for rev1
GPIO_PIN pin_tx_led = {GPIO_PIN_12, GPIOB}; 
GPIO_PIN pin_rx_led = {GPIO_PIN_13, GPIOB};
GPIO_PIN pin_encoder = {GPIO_PIN_3, GPIOB};  
GPIO_PIN pin_poz_zero_sensor = {GPIO_PIN_4, GPIOA}; 
GPIO_PIN pin_inout_ca1 = {GPIO_PIN_5, GPIOA}; 
GPIO_PIN pin_inout_ca2 = {GPIO_PIN_7, GPIOA};
GPIO_PIN pin_inout_crx = {GPIO_PIN_4, GPIOC};
GPIO_PIN pin_inout_ctx = {GPIO_PIN_10, GPIOB};
GPIO_PIN pin_sync_sda = {GPIO_PIN_9, GPIOC}; 
GPIO_PIN pin_sync_scl = {GPIO_PIN_8, GPIOA};

GPIO_PIN pin_temp_steper_board = {GPIO_PIN_0, GPIOA};
GPIO_PIN pin_temp_board = {GPIO_PIN_1, GPIOA};
GPIO_PIN pin_temp_motor = {GPIO_PIN_2, GPIOA};
GPIO_PIN pin_vsense = {GPIO_PIN_3, GPIOA};

GPIO_PIN pin_steper_direction = {GPIO_PIN_0, GPIOB};
GPIO_PIN pin_steper_enable = {GPIO_PIN_1, GPIOB};
GPIO_PIN pin_steper_step = {GPIO_PIN_6, GPIOA};
GPIO_PIN pin_boot_device = {GPIO_PIN_8, GPIOC};

// to do
 GPIO_PIN pin_cid_0 = {GPIO_PIN_10, GPIOC};
 GPIO_PIN pin_cid_1 = {GPIO_PIN_11, GPIOC};
 GPIO_PIN pin_cid_2 = {GPIO_PIN_12, GPIOC};

//**************************************************************************************************
// Global stuff

uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE+1];
ID_CONFIG config;
TIMING::Ticker main_clock;
LOGGER::Logger loger(LOGGER::LOG_LEVEL::LOG_LEVEL_DEBUG,false);
BOARD_ID::Board_id board_id(pin_cid_0, pin_cid_1, pin_cid_2);
STEPER_MOTOR::SteperMotor stp_motor(htim3, TIM_CHANNEL_1, pin_steper_direction, pin_steper_enable);
CAN_CONTROL::CanControl can_controler;
MOVEMENT_CONTROLER::MovementControler movement_controler;
PDCONTROLER::PdControler pid_pos(main_clock);

FILTERS::Filter_moving_avarage encoder_arm_moving_avarage(main_clock);
ENCODER::Encoder encoder_arm;
ENCODER::Encoder encoder_motor;

USB_PROGRAMER::UsbProgramer usb_programer(pin_boot_device);
TIMING::Timing tim_can_disconnected(main_clock);
float temoperature_board = 0;
float temoperature_steper_driver = 0;
float temoperature_steper_motor = 0; 
float voltage_vcc = 0;

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

//**************************************************************************************************



//**************************************************************************************************
void main_prog(){
  log_debug("Start main_prog\n");
  pre_periferal_config();
  periferal_config();
  id_config();
  post_id_config();
  init_controls();
  main_loop();
}

void pre_periferal_config(){
  NTCTERMISTORS::termistor_supply_voltage = 3.3;
  NTCTERMISTORS::termistor_divider_resisitor = 100000;
  NTCTERMISTORS::termistor_default_resistance = 100000;
  encoder_arm.set_address(ENCODER_MT6701_I2C_ADDRESS);
  encoder_arm.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder_arm.set_angle_register(ENCODER_MEM_ADDR_ANNGLE);

  encoder_motor.set_address(ENCODER_AS5600_I2C_ADDRESS);
  encoder_motor.set_resolution(ENCODER_AS5600_RESOLUTION);
  encoder_motor.set_angle_register(ENCODER_AS5600_ANGLE_REG);
}


void id_config(){
  log_debug("Start id_config\n");
  log_debug("Board id: " + std::to_string(board_id.get_id()));

  std::string info = "SDRACboard\n";
  info += "Software version:" + std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) + "\n";
  info += "Board id: " + std::to_string(board_id.get_id()) + "\n";
  info += "Description: SDRACboard from SDRAC project https://nihilia.xyz  https://konar.pwr.edu.pl\n";
  usb_programer.set_info(info);

  switch (board_id.get_id()){
  case BOARD_ID_1: config = config_id_1; break;
  case BOARD_ID_2: config = config_id_2; break;
  case BOARD_ID_3: config = config_id_3; break;
  case BOARD_ID_4: config = config_id_4; break;
  case BOARD_ID_5: config = config_id_5; break;
  case BOARD_ID_6: config = config_id_6; break;
  default: config = config_id_default; break;
  }

  CAN_KONARM_X_STATUS_FRAME_ID = config.can_konarm_status_frame_id;
  CAN_KONARM_X_SET_POS_FRAME_ID = config.can_konarm_set_pos_frame_id;
  CAN_KONARM_X_GET_POS_FRAME_ID = config.can_konarm_get_pos_frame_id;
  CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID = config.can_konarm_clear_errors_frame_id;
  
  // ids 11bit 0b110 0001 0000  and 18 bit 0b00 0000 0000 0000 0000
  //mask 11bit 0b111 1111 0000  and 18 bit 0b00 0000 0000 0000 0000
  // for some reason filter mask is not working properly so we have to do it in software
  CAN_X_FILTER_ID_HIGH = config.can_filter_id_high;   // why 5 bits shift because we want tu push the 11 bit id to the 16 bit regiser strting from 5th bit
  CAN_X_FILTER_ID_LOW = config.can_filter_id_low;
  CAN_X_FILTER_MASK_HIGH = config.can_filter_mask_high;
  CAN_X_FILTER_MASK_LOW = config.can_filter_mask_low;


  //-------------------STEPER MOTOR CONFIGURATION-------------------
  stp_motor.set_steps_per_revolution(config.stepper_motor_steps_per_rev);
  stp_motor.set_gear_ratio(config.stepper_motor_gear_ratio);
  stp_motor.set_max_velocity(config.stepper_motor_max_velocity);
  stp_motor.set_min_velocity(config.stepper_motor_min_velocity);
  stp_motor.set_reverse(config.stepper_motor_reverse);
  stp_motor.init();
  stp_motor.set_enable(false);


  //-------------------ENCODER STEPER MOTOR POSITION CONFIGURATION-------------------
  // to do
  encoder_motor.set_function_to_read_angle(ENCODER::translate_reg_to_angle_AS5600);
  encoder_motor.set_offset(config.encoder_motor_offset);
  encoder_motor.set_reverse(config.encoder_motor_reverse);
  encoder_motor.set_enable_pos_filter(false);
  encoder_motor.set_enable_velocity(true);
  encoder_motor.set_enable_velocity_filter(false);
  encoder_motor.set_velocity_sample_amount(config.encoder_motor_velocity_sample_amount);
  encoder_motor.set_dead_zone_correction_angle(config.encoder_motor_dead_zone_correction_angle);
  encoder_motor.init(hi2c1,main_clock,nullptr,nullptr);



  //-------------------ENCODER ARM POSITION CONFIGURATION-------------------
  
  encoder_arm_moving_avarage.set_size(60); // 15 for smooth movement but delay with sampling to 50
  encoder_arm.set_function_to_read_angle(ENCODER::translate_reg_to_angle_MT6701);
  encoder_arm.set_offset(config.encoder_arm_offset);
  encoder_arm.set_reverse(config.encoder_arm_reverse);
  encoder_arm.set_enable_pos_filter(false);
  encoder_arm.set_enable_velocity(true);
  encoder_arm.set_enable_velocity_filter(true);
  encoder_arm.set_velocity_sample_amount(config.encoder_motor_velocity_sample_amount);
  encoder_arm.set_dead_zone_correction_angle(config.encoder_arm_dead_zone_correction_angle);
  encoder_arm.init(hi2c1,main_clock,nullptr,&encoder_arm_moving_avarage);
  

  //-------------------MOVEMENT CONTROLER CONFIGURATION-------------------
  pid_pos.set_Kp(config.pid_p);
  pid_pos.set_Ki(config.pid_i);
  pid_pos.set_Kd(config.pid_d);
  movement_controler.set_limit_position(config.movement_limit_lower, config.movement_limit_upper);
  movement_controler.set_max_velocity(config.movement_max_velocity);
  movement_controler.init(main_clock, stp_motor, encoder_arm, encoder_motor, pid_pos);

}

void periferal_config(){
  log_debug("Start periferal_config\n");
  // dma adc1 settings
  HAL_ADC_Start_DMA(&hadc1, adc_dma_buffer, ADC_DMA_BUFFER_SIZE);

  // timer 10 settings
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn,6,0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_TIM_Base_Start_IT(&htim10);

  // timer 3 settings
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_CAN_DeInit(&hcan1);
  HAL_CAN_Init(&hcan1);


}

void post_id_config(){
    // Can1 settings
  CAN_FilterTypeDef can_filter;
  can_filter.FilterBank = 1;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh = CAN_X_FILTER_ID_HIGH;
  can_filter.FilterIdLow = CAN_X_FILTER_ID_LOW;
  can_filter.FilterMaskIdHigh = CAN_X_FILTER_MASK_HIGH;
  can_filter.FilterMaskIdLow = CAN_X_FILTER_MASK_LOW;
  can_filter.SlaveStartFilterBank = 0;
  HAL_StatusTypeDef status =  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  can_controler.set_filter(CAN_X_FILTER_ID_HIGH, CAN_X_FILTER_MASK_HIGH);
  can_controler.init(hcan1, CAN_FILTER_FIFO0, main_clock, pin_tx_led, pin_rx_led);
  status =  HAL_CAN_Start(&hcan1);
  status =  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING ); //| CAN_IT_RX_FIFO1_MSG_PENDING); 
}

void handle_can_rx(){
  __disable_irq();
  CAN_CONTROL::CAN_MSG *recived_msg = can_controler.get_message();
  __enable_irq();
  if(recived_msg == nullptr) return;
  tim_can_disconnected.reset();

  if(recived_msg->frame_id == CAN_KONARM_X_SET_POS_FRAME_ID){
    can_konarm_1_set_pos_t signals;
    can_konarm_1_set_pos_unpack(&signals, recived_msg->data, recived_msg->data_size);
    float targetPosition = can_konarm_1_set_pos_position_decode(signals.position);
    float targetVelocity = can_konarm_1_set_pos_velocity_decode(signals.velocity);
    movement_controler.set_velocity(targetVelocity);
    movement_controler.set_position(targetPosition);
    movement_controler.set_enable(true);
  }
  else if (recived_msg->frame_id == CAN_KONARM_X_GET_POS_FRAME_ID && recived_msg->remote_request){
    CAN_CONTROL::CAN_MSG *send_msg = (CAN_CONTROL::CAN_MSG*)malloc(sizeof(CAN_CONTROL::CAN_MSG));
    can_konarm_1_get_pos_t src_p;
    send_msg->frame_id = CAN_KONARM_X_GET_POS_FRAME_ID;
    src_p.position = can_konarm_1_get_pos_position_encode(movement_controler.get_current_position());
    src_p.velocity = can_konarm_1_get_pos_velocity_encode(movement_controler.get_current_velocity());
    send_msg->data_size = CAN_KONARM_1_GET_POS_LENGTH;
    can_konarm_1_get_pos_pack(send_msg->data, &src_p, send_msg->data_size);
    can_controler.send_msg_to_queue(send_msg);
  }
  else if (recived_msg->frame_id == CAN_KONARM_X_STATUS_FRAME_ID && recived_msg->remote_request){
    CAN_CONTROL::CAN_MSG *send_msg = (CAN_CONTROL::CAN_MSG*)malloc(sizeof(CAN_CONTROL::CAN_MSG));
    can_konarm_1_status_t src_p;
    send_msg->frame_id = CAN_KONARM_X_STATUS_FRAME_ID;
    src_p.status = can_konarm_1_status_status_encode(CAN_KONARM_1_STATUS_STATUS_OK_CHOICE);
    send_msg->data_size = CAN_KONARM_1_STATUS_LENGTH;
    can_konarm_1_status_pack(send_msg->data, &src_p, send_msg->data_size);
    can_controler.send_msg_to_queue(send_msg);
  }
  else if (recived_msg->frame_id == CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID){
  }

  // delete recived_msg;
  free(recived_msg);
}

void analog_values_assigning(){
  temoperature_board = MCP9700AT::get_temperature(VOLTAGE_VALUE(pin_temp_board));
  temoperature_steper_driver = NTCTERMISTORS::get_temperature(VOLTAGE_VALUE(pin_temp_steper_board));
  temoperature_steper_motor = NTCTERMISTORS::get_temperature(VOLTAGE_VALUE(pin_temp_motor));
  voltage_vcc = VOLTAGE_VALUE(pin_vsense) * ADC_VSENSE_MULTIPLIER;
}

void init_controls(){
  log_debug("Start init_interfaces\n");

  // init the movement controler should be done after the encoder and the steper motor are initialized
  movement_controler.set_position(encoder_arm.get_angle());
  movement_controler.set_velocity(0);
  movement_controler.set_enable(false);
  movement_controler.handle();
}

void main_loop(){
  log_debug("Start main_loop\n");
  TIMING::Timing tim_blink(main_clock);
  TIMING::Timing tim_encoder(main_clock);
  TIMING::Timing tim_usb(main_clock);
  TIMING::Timing tim_movement(main_clock);
  TIMING::Timing tim_data_usb_send(main_clock);
  TIMING::Timing tim_caculate_temp(main_clock);
  tim_blink.set_behaviour(TIMING::frequency_to_period(1), true);
  tim_encoder.set_behaviour(TIMING::frequency_to_period(1000), true);
  tim_usb.set_behaviour(TIMING::frequency_to_period(4), true);
  tim_movement.set_behaviour(TIMING::frequency_to_period(1000), true);
  tim_data_usb_send.set_behaviour(TIMING::frequency_to_period(50), true);
  tim_can_disconnected.set_behaviour(1000000, false);
  tim_caculate_temp.set_behaviour(TIMING::frequency_to_period(20), true);

  while (1){
    handle_can_rx();
    can_controler.handle();

    if(tim_encoder.triggered()){
      encoder_arm.handle();
      encoder_motor.handle();
    }

    if(tim_can_disconnected.triggered()){      
      movement_controler.set_enable(false);
      movement_controler.set_velocity(0);
      movement_controler.set_position(movement_controler.get_current_position());
    }
    movement_controler.handle();

    if(tim_data_usb_send.triggered()){
      log_info(
         "ID:" + std::to_string(board_id.get_id()) +
         " V:" + std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) +
         " Vsen:" + std::to_string(voltage_vcc) +
         " Tste:" + std::to_string(temoperature_steper_motor) + 
         " Tbor:" + std::to_string(temoperature_board) + 
         " Tmot:" + std::to_string(temoperature_steper_driver) + 
         " Eang:" + std::to_string(encoder_arm.get_angle()) + 
         " Vvel:" + std::to_string(encoder_arm.get_velocity()) + 
         " Pos:" + std::to_string(movement_controler.get_current_position()) + 
         " Vel:" + std::to_string(movement_controler.get_current_velocity()));
    }

    if(tim_usb.triggered()){
      usb_programer.handler();
    }

    if(tim_blink.triggered()) {
      TOGGLE_GPIO(pin_user_led_1);
    }
    
    if(tim_caculate_temp.triggered()){
      analog_values_assigning();
    }

  }
}