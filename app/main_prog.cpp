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
#include "MCP9700AT.hpp"
#include "Timing.hpp"
#include "config.hpp"
#include <string>
#include <charconv>
#include <vector>



PDCONTROLER::PdControler pid_pos(main_clock);
FILTERS::Filter_moving_avarage encoder_arm_moving_avarage(main_clock);
TIMING::Timing tim_can_disconnected(main_clock);

float temoperature_board = 0;
float temoperature_steper_driver = 0;
float temoperature_steper_motor = 0; 
float voltage_vcc = 0;
bool board_error = false;

//**************************************************************************************************
void main_prog(){
  log_debug("\"state\":\"Start main_prog\"");
  pre_periferal_config();
  periferal_config();
  id_config();
  post_id_config();
  main_loop();
}

void pre_periferal_config(){
  log_debug("\"state\":\"pre_perifial_config\"");
  NTCTERMISTORS::termistor_supply_voltage = UC_SUPPLY_VOLTAGE;
  NTCTERMISTORS::termistor_divider_resisitor = TERMISTOR_RESISTANCE;
  encoder_arm.set_address(ENCODER_MT6701_I2C_ADDRESS);
  encoder_arm.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder_arm.set_angle_register(ENCODER_MT6701_ANGLE_REG);

  encoder_motor.set_address(ENCODER_AS5600_I2C_ADDRESS);
  encoder_motor.set_resolution(ENCODER_AS5600_RESOLUTION);
  encoder_motor.set_angle_register(ENCODER_AS5600_ANGLE_REG);
}

void periferal_config(){
  log_debug("\"state\":\"periferal_config\"");
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

void id_config(){
  log_debug("\"state\":\"id_config\"");

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
  encoder_arm.set_velocity_sample_amount(config.encoder_arm_velocity_sample_amount);
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

void post_id_config(){
  log_debug("\"state\":\"post_id_config\"");
  CAN_FilterTypeDef can_filter;
  can_filter.FilterBank = 1;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh = config.can_filter_id_high;
  can_filter.FilterIdLow = config.can_filter_id_low;
  can_filter.FilterMaskIdHigh = config.can_filter_mask_high;
  can_filter.FilterMaskIdLow = config.can_filter_mask_low;
  can_filter.SlaveStartFilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  can_controler.set_filter(config.can_filter_id_high, config.can_filter_mask_high);
  can_controler.init(hcan1, CAN_FILTER_FIFO0, main_clock, pin_tx_led, pin_rx_led);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING ); //| CAN_IT_RX_FIFO1_MSG_PENDING); 


  // init the movement controler should be done after the encoder and the steper motor are initialized
  movement_controler.set_position(encoder_arm.get_angle());
  movement_controler.set_velocity(0.0f);
  movement_controler.set_enable(false);
  movement_controler.handle();
}

void analog_values_assigning(){
  temoperature_board = MCP9700AT::get_temperature(VOLTAGE_VALUE(pin_temp_board));
  temoperature_steper_driver = NTCTERMISTORS::get_temperature(VOLTAGE_VALUE(pin_temp_steper_board));
  temoperature_steper_motor = NTCTERMISTORS::get_temperature(VOLTAGE_VALUE(pin_temp_motor));
  voltage_vcc = VOLTAGE_VALUE(pin_vsense) * ADC_VSENSE_MULTIPLIER;
}

void handle_can_rx(){
  __disable_irq();
  CAN_CONTROL::CAN_MSG *recived_msg = can_controler.get_message();
  __enable_irq();
  if(recived_msg == nullptr) return;
  tim_can_disconnected.reset();

  if(recived_msg->frame_id == config.can_konarm_set_pos_frame_id){
    can_konarm_1_set_pos_t signals;
    can_konarm_1_set_pos_unpack(&signals, recived_msg->data, recived_msg->data_size);
    float targetPosition = can_konarm_1_set_pos_position_decode(signals.position);
    float targetVelocity = can_konarm_1_set_pos_velocity_decode(signals.velocity);
    movement_controler.set_velocity(targetVelocity);
    movement_controler.set_position(targetPosition);
    movement_controler.set_enable(true);
  }
  else if (recived_msg->frame_id == config.can_konarm_get_pos_frame_id && recived_msg->remote_request){
    CAN_CONTROL::CAN_MSG *send_msg = (CAN_CONTROL::CAN_MSG*)malloc(sizeof(CAN_CONTROL::CAN_MSG));
    can_konarm_1_get_pos_t src_p;
    send_msg->frame_id = config.can_konarm_get_pos_frame_id;
    src_p.position = can_konarm_1_get_pos_position_encode(movement_controler.get_current_position());
    src_p.velocity = can_konarm_1_get_pos_velocity_encode(movement_controler.get_current_velocity());
    send_msg->data_size = CAN_KONARM_1_GET_POS_LENGTH;
    can_konarm_1_get_pos_pack(send_msg->data, &src_p, send_msg->data_size);
    can_controler.send_msg_to_queue(send_msg);
  }
  else if (recived_msg->frame_id == config.can_konarm_status_frame_id && recived_msg->remote_request){
    CAN_CONTROL::CAN_MSG *send_msg = (CAN_CONTROL::CAN_MSG*)malloc(sizeof(CAN_CONTROL::CAN_MSG));
    can_konarm_1_status_t src_p;
    send_msg->frame_id = config.can_konarm_status_frame_id;
    src_p.status = can_konarm_1_status_status_encode(CAN_KONARM_1_STATUS_STATUS_OK_CHOICE);
    send_msg->data_size = CAN_KONARM_1_STATUS_LENGTH;
    can_konarm_1_status_pack(send_msg->data, &src_p, send_msg->data_size);
    can_controler.send_msg_to_queue(send_msg);
  }
  else if (recived_msg->frame_id == config.can_konarm_clear_errors_frame_id){
  }

  // delete recived_msg;
  free(recived_msg);
}

void main_loop(){
  log_debug("Start main_loop\n");
  TIMING::Timing tim_blink(main_clock);
  TIMING::Timing tim_blink_error(main_clock);
  TIMING::Timing tim_encoder(main_clock);
  TIMING::Timing tim_usb(main_clock);
  TIMING::Timing tim_data_usb_send(main_clock);
  TIMING::Timing tim_caculate_temp(main_clock);
  tim_blink_error.set_behaviour(TIMING::frequency_to_period(TIMING_LED_ERROR_BLINK_FQ),true);
  tim_blink.set_behaviour(TIMING::frequency_to_period(TIMING_LED_BLINK_FQ), true);
  tim_encoder.set_behaviour(TIMING::frequency_to_period(TIMING_ENCODER_UPDATE_FQ), true);
  tim_usb.set_behaviour(TIMING::frequency_to_period(TIMING_USB_RECIVED_DATA_FQ), true);
  tim_data_usb_send.set_behaviour(TIMING::frequency_to_period(TIMING_USB_SEND_DATA_FQ), true);
  tim_can_disconnected.set_behaviour(TIMING_CAN_DISCONNECTED_PERIOD, false);
  tim_caculate_temp.set_behaviour(TIMING::frequency_to_period(TIMING_READ_TEMPERATURE_FQ), true);

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
         "\"Vsen\":\"" + std::to_string(voltage_vcc) +
         "\",\"Tste\":\"" + std::to_string(temoperature_steper_motor) + 
         "\",\"Tbor\":\"" + std::to_string(temoperature_board) + 
         "\",\"Tmot\":\"" + std::to_string(temoperature_steper_driver) + 
         "\",\"Eang\":\"" + std::to_string(encoder_arm.get_angle()) + 
         "\",\"Vvel\":\"" + std::to_string(encoder_arm.get_velocity()) + 
         "\",\"Pos\":\"" + std::to_string(movement_controler.get_current_position()) + 
         "\",\"Vel\":\"" + std::to_string(movement_controler.get_current_velocity()));
    }

    if(tim_usb.triggered()){
      usb_programer.handler();
    }

    if(tim_blink.triggered()) {
      TOGGLE_GPIO(pin_user_led_1);
    }

    if(tim_blink_error.triggered()){
      if(board_error) TOGGLE_GPIO(pin_user_led_2); 
      else WRITE_GPIO(pin_user_led_2,GPIO_PIN_RESET);
    }
    
    if(tim_caculate_temp.triggered()){
      analog_values_assigning();
    }

  }
}