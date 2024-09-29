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
#include "can.h"
#include "movement_controler.hpp"
#include "controler_pid.hpp"
#include "controler_linear.hpp"
#include "controler_pass_through.hpp"
#include "pin.hpp"
#include "ntc_termistors.hpp"
#include "version.hpp"
#include <cfloat>
#include "MCP9700AT.hpp"
#include "Timing.hpp"
#include "config.hpp"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <charconv>
#include <vector>
#include <cmath>



CONTROLER::PIDControler pid_pos(main_clock);
CONTROLER::BasicControler bacis_controler(main_clock);
CONTROLER::PassThroughControler pass_through_controler(main_clock);
FILTERS::Filter_moving_avarage encoder_motor_moving_avarage(main_clock);
TIMING::Timing tim_can_disconnecteded(main_clock);

std::shared_ptr<TIMING::Timing> task_blink_timer;
std::shared_ptr<TIMING::Timing> task_blink_error_timer;
std::shared_ptr<TIMING::Timing> task_read_analog_values_timer;
std::shared_ptr<TIMING::Timing> task_encoder_timer;
std::shared_ptr<TIMING::Timing> task_usb_timer;
std::shared_ptr<TIMING::Timing> task_data_usb_send_timer;
std::shared_ptr<TIMING::Timing> task_caculate_temp_timer;
std::shared_ptr<TIMING::Timing> task_nodelay_timer;
std::shared_ptr<TIMING::Timing> task_can_disconnected_timer;



float temoperature_board = 0;
float temoperature_steper_driver = 0;
float temoperature_steper_motor = 0; 
float voltage_vcc = 0;

//**************************************************************************************************
void run_main_prog(){
  log_debug(loger.parse_to_json_format("state","start"));

  pre_periferal_config();
  periferal_config();
  id_config();
  post_id_config();
  config_tasks();
  main_loop();
}

void pre_periferal_config(){
  log_debug(loger.parse_to_json_format("state","pre_perifial_config"));
}

void periferal_config(){
  log_debug(loger.parse_to_json_format("state","periferal_config"));
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
  log_debug(loger.parse_to_json_format("state","id_config"));

  std::string info = "SDRACboard\n";
  info += "Software version:" + version_string + "\n";
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
  stp_motor.set_enable_reversed(config.stepper_motor_enable_reversed);
  stp_motor.set_prescaler(config.stepper_motor_timer_prescaler);
  stp_motor.init();
  stp_motor.set_enable(false);  


  //-------------------ENCODER ARM POSITION CONFIGURATION-------------------
  encoder_arm.set_function_to_read_angle(ENCODER::translate_reg_to_angle_MT6701);
  encoder_arm.set_offset(config.encoder_arm_offset);
  encoder_arm.set_reverse(config.encoder_arm_reverse);
  encoder_arm.set_enable_position_filter(false);
  encoder_arm.set_enable_velocity(false);
  encoder_arm.set_enable_velocity_filter(false);
  encoder_arm.set_velocity_sample_amount(config.encoder_arm_velocity_sample_amount);
  encoder_arm.set_dead_zone_correction_angle(config.encoder_arm_dead_zone_correction_angle);
  encoder_arm.set_angle_register(ENCODER_MT6701_ANGLE_REG);
  encoder_arm.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder_arm.set_address(ENCODER_MT6701_I2C_ADDRESS);
  encoder_arm.set_enable_encoder(true);
  encoder_arm.init(hi2c1,main_clock,nullptr,nullptr);
  
  //-------------------ENCODER STEPER MOTOR POSITION CONFIGURATION-------------------
  encoder_motor.set_function_to_read_angle(ENCODER::translate_reg_to_angle_MT6701);
  encoder_motor.set_offset(config.encoder_motor_offset);
  encoder_motor.set_reverse(config.encoder_motor_reverse);
  encoder_motor.set_enable_position_filter(false);
  encoder_motor.set_enable_velocity(true);
  encoder_motor.set_enable_velocity_filter(false);
  encoder_motor.set_velocity_sample_amount(config.encoder_motor_velocity_sample_amount);
  encoder_motor.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder_motor.set_angle_register(ENCODER_MT6701_ANGLE_REG);
  encoder_motor.set_address(ENCODER_MT6701_I2C_ADDRESS_2); 
  encoder_motor.set_dead_zone_correction_angle(config.encoder_motor_dead_zone_correction_angle);
  encoder_motor.set_ratio(1.0f / stp_motor.get_gear_ratio());
  encoder_motor.set_enable_encoder(config.encoder_motor_enable);
  encoder_motor_moving_avarage.set_size(50); // 15 for smooth movement but delay with sampling to 50
  encoder_motor.init(hi2c1,main_clock,nullptr,&encoder_motor_moving_avarage);
  

  //-------------------MOVEMENT CONTROLER CONFIGURATION-------------------
  pid_pos.set_Kp(config.pid_p);
  pid_pos.set_Ki(config.pid_i);
  // pid_pos.set_Kd(config.pid_d);
  bacis_controler.set_max_acceleration(1.0f);
  bacis_controler.set_target_pos_max_error(0.001f);


  movement_controler.set_limit_position(config.movement_limit_lower, config.movement_limit_upper);
  movement_controler.set_max_velocity(config.movement_max_velocity);
  movement_controler.set_position(config.movement_limit_upper);
  if(config.encoder_motor_enable){
    movement_controler.init(main_clock, stp_motor, encoder_arm, pass_through_controler,&encoder_motor);
  }else{
    movement_controler.init(main_clock, stp_motor, encoder_arm, pass_through_controler);
  }
}

void post_id_config(){
  log_debug(loger.parse_to_json_format("state","post_id_config"));
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

void error_checks(){
  error_data.temp_board_overheating = !std::isnan(temoperature_board) && temoperature_board > ERRORS_MAX_TEMP_BOARD? true : false;
  error_data.temp_driver_overheating = !std::isnan(temoperature_steper_driver) && temoperature_steper_driver > ERRORS_MAX_TEMP_DRIVER? true : false;
  error_data.temp_engine_overheating = !std::isnan(temoperature_steper_motor) && temoperature_steper_motor > ERRORS_MAX_TEMP_ENGINE? true : false;
  error_data.temp_board_sensor_disconnect = std::isnan(temoperature_board);
  error_data.temp_driver_sensor_disconnect = std::isnan(temoperature_steper_driver);
  error_data.temp_engine_sensor_disconnect = std::isnan(temoperature_steper_motor);

  error_data.encoder_arm_disconnect = !encoder_arm.is_connected();
  error_data.encoder_motor_disconnect = !encoder_motor.is_connected();

  error_data.baord_overvoltage = voltage_vcc > ERRORS_MAX_VCC_VOLTAGE? true : false;
  error_data.baord_undervoltage = voltage_vcc < ERRORS_MIN_VCC_VOLTAGE? true : false;

  // can errors are handled in the handle_can_rx function

  error_data.controler_motor_limit_position = movement_controler.get_limit_position_achieved();
}

void handle_can_rx(){
  __disable_irq();
  CAN_CONTROL::CAN_MSG recived_msg;
  uint8_t status =  can_controler.get_message(&recived_msg);
  __enable_irq();
  if(status != 0) return;
  task_can_disconnected_timer->reset();
  error_data.can_disconnected = false;

  if(recived_msg.frame_id == config.can_konarm_set_pos_frame_id){
    can_konarm_1_set_pos_t signals;
    can_konarm_1_set_pos_unpack(&signals, recived_msg.data, recived_msg.data_size);
    float targetPosition = can_konarm_1_set_pos_position_decode(signals.position);
    float targetVelocity = can_konarm_1_set_pos_velocity_decode(signals.velocity);
    movement_controler.set_velocity(targetVelocity);
    movement_controler.set_position(targetPosition);
    movement_controler.set_enable(true);
  }
  else if (recived_msg.frame_id == config.can_konarm_get_pos_frame_id && recived_msg.remote_request){
    CAN_CONTROL::CAN_MSG send_msg;
    can_konarm_1_get_pos_t src_p;
    send_msg.frame_id = config.can_konarm_get_pos_frame_id;
    src_p.position = can_konarm_1_get_pos_position_encode(movement_controler.get_current_position());
    src_p.velocity = can_konarm_1_get_pos_velocity_encode(movement_controler.get_current_velocity());
    send_msg.data_size = CAN_KONARM_1_GET_POS_LENGTH;
    can_konarm_1_get_pos_pack(send_msg.data, &src_p, send_msg.data_size);
    can_controler.send_msg_to_queue(send_msg);
  }
  else if (recived_msg.frame_id == config.can_konarm_status_frame_id && recived_msg.remote_request){
    CAN_CONTROL::CAN_MSG send_msg;
    can_konarm_1_status_t src_p;
    send_msg.frame_id = config.can_konarm_status_frame_id;
    src_p.status = can_konarm_1_status_status_encode(CAN_KONARM_1_STATUS_STATUS_OK_CHOICE);
    send_msg.data_size = CAN_KONARM_1_STATUS_LENGTH;
    can_konarm_1_status_pack(send_msg.data, &src_p, send_msg.data_size);
    can_controler.send_msg_to_queue(send_msg);
  }
  else if (recived_msg.frame_id == config.can_konarm_clear_errors_frame_id){
  }
  else if (recived_msg.frame_id == config.can_konarm_get_errors_frame_id) {
    CAN_CONTROL::CAN_MSG send_msg;
    can_konarm_1_get_errors_t src_p;
    send_msg.frame_id = config.can_konarm_get_errors_frame_id;
    src_p.temp_engine_overheating = can_konarm_1_get_errors_temp_engine_overheating_encode(error_data.temp_engine_overheating);
    src_p.temp_driver_overheating = can_konarm_1_get_errors_temp_driver_overheating_encode(error_data.temp_driver_overheating);
    src_p.temp_board_overheating = can_konarm_1_get_errors_temp_board_overheating_encode(error_data.temp_board_overheating);
    src_p.temp_engine_sensor_disconnect = can_konarm_1_get_errors_temp_engine_sensor_disconnect_encode(error_data.temp_engine_sensor_disconnect);
    src_p.temp_driver_sensor_disconnect = can_konarm_1_get_errors_temp_driver_sensor_disconnect_encode(error_data.temp_driver_sensor_disconnect);
    src_p.temp_board_sensor_disconnect = can_konarm_1_get_errors_temp_board_sensor_disconnect_encode(error_data.temp_board_sensor_disconnect);
    src_p.encoder_arm_disconnect = can_konarm_1_get_errors_encoder_arm_disconnect_encode(error_data.encoder_arm_disconnect);
    src_p.encoder_motor_disconnect = can_konarm_1_get_errors_encoder_motor_disconnect_encode(error_data.encoder_motor_disconnect);
    src_p.board_overvoltage = can_konarm_1_get_errors_board_overvoltage_encode(error_data.baord_overvoltage);
    src_p.board_undervoltage = can_konarm_1_get_errors_board_undervoltage_encode(error_data.baord_undervoltage);
    src_p.can_disconnected = can_konarm_1_get_errors_can_disconnected_encode(error_data.can_disconnected);
    src_p.can_error = can_konarm_1_get_errors_can_error_encode(error_data.can_error);
    src_p.controler_motor_limit_position = can_konarm_1_get_errors_controler_motor_limit_position_encode(error_data.controler_motor_limit_position);
    send_msg.data_size = CAN_KONARM_1_GET_ERRORS_LENGTH;
    can_konarm_1_get_errors_pack(send_msg.data, &src_p, send_msg.data_size);
    can_controler.send_msg_to_queue(send_msg);
    // we reset can errors after sending the frame wth erro so we can se if its repraing itself
    error_data.can_error = false;
  }
  else{
    // trigeerd when the frame is not recognized
    error_data.can_error = true;
  }
}

void task_encoders(TIMING::Timing& task_timer){
  encoder_arm.handle();
  if(config.encoder_motor_enable)
    encoder_motor.handle();
}

// uint32_t last_time = main_clock.get_micros();
// int32_t counter = 0;

void task_nodelay(TIMING::Timing& task_timer){
  movement_controler.handle();
  handle_can_rx();
  can_controler.handle();
  error_checks();

  
  // counter++;
  // if(counter > 100000){
  //   uint32_t current_time = main_clock.get_micros();
  //   uint32_t diff = current_time - last_time;
  //   last_time = current_time;
  //   float freq = 100000000000 / (float)diff;
  //   log_error("main loop freq " + std::to_string(freq));
  //   counter = 0;
  // }
}

void task_can_disconnect(TIMING::Timing& task_timer){
  movement_controler.set_enable(false);
  movement_controler.set_velocity(0);
  movement_controler.set_position(movement_controler.get_current_position());
  error_data.can_disconnected = true;
}

void task_usb_handler(TIMING::Timing& task_timer){
  usb_programer.handler();
}

void task_usb_data_loging(TIMING::Timing& task_timer){
  log_info(
        loger.parse_to_json_format("ID",std::to_string(board_id.get_id()))+
        loger.parse_to_json_format("Vsen",std::to_string(voltage_vcc))+
        loger.parse_to_json_format("Tste",std::to_string(temoperature_steper_motor))+
        loger.parse_to_json_format("Tbor",std::to_string(temoperature_board))+
        loger.parse_to_json_format("Tmot",std::to_string(temoperature_steper_driver))+
        loger.parse_to_json_format("Eang",std::to_string(encoder_arm.get_angle()))+
        loger.parse_to_json_format("Pos",std::to_string(movement_controler.get_current_position()))+
        loger.parse_to_json_format("Vel",std::to_string(movement_controler.get_current_velocity()))+
        loger.parse_to_json_format("EPos",std::to_string(encoder_motor.get_absoulute_angle()))+
        loger.parse_to_json_format("Err",std::to_string(error_data.get_amount_of_errors()))+
        loger.parse_to_json_format("Errs",
          loger.parse_to_json_format("teng",BOOL_TO_STRING(error_data.temp_engine_overheating))+
          loger.parse_to_json_format("tdri",BOOL_TO_STRING(error_data.temp_driver_overheating))+
          loger.parse_to_json_format("tboa",BOOL_TO_STRING(error_data.temp_board_overheating))+
          loger.parse_to_json_format("tengdis",BOOL_TO_STRING(error_data.temp_engine_sensor_disconnect))+
          loger.parse_to_json_format("tdrivdis",BOOL_TO_STRING(error_data.temp_driver_sensor_disconnect))+
          loger.parse_to_json_format("tborddis",BOOL_TO_STRING(error_data.temp_board_sensor_disconnect))+
          loger.parse_to_json_format("encarmmdis",BOOL_TO_STRING(error_data.encoder_arm_disconnect))+
          loger.parse_to_json_format("encmotdis",BOOL_TO_STRING(error_data.encoder_motor_disconnect))+
          loger.parse_to_json_format("bovolt",BOOL_TO_STRING(error_data.baord_overvoltage))+
          loger.parse_to_json_format("buvolt",BOOL_TO_STRING(error_data.baord_undervoltage))+
          loger.parse_to_json_format("candis",BOOL_TO_STRING(error_data.can_disconnected))+
          loger.parse_to_json_format("canerr",BOOL_TO_STRING(error_data.can_error))+
          loger.parse_to_json_format("motlimit",BOOL_TO_STRING(error_data.controler_motor_limit_position),false)
        ,false,true)
      );
}

void task_blink(TIMING::Timing& task_timer){
  TOGGLE_GPIO(pin_user_led_1);
}

void task_read_analog_values(TIMING::Timing& task_timer){
  temoperature_board = MCP9700AT::get_temperature(VOLTAGE_VALUE(pin_temp_board));
  temoperature_steper_driver = temp_steper_driver.get_temperature(VOLTAGE_VALUE(pin_temp_steper_board)); 
  temoperature_steper_motor = temp_steper_motor.get_temperature(VOLTAGE_VALUE(pin_temp_motor));
  voltage_vcc = VOLTAGE_VALUE(pin_vsense) * ADC_VSENSE_MULTIPLIER;
}

void task_blink_error(TIMING::Timing& task_timer){
  // error_checks();
  auto errors_count = error_data.get_amount_of_errors();
  task_timer.set_behaviour(TIMING::frequency_to_period((float)TIMING_LED_ERROR_BLINK_FQ*errors_count), true);
  
  if(errors_count) TOGGLE_GPIO(pin_user_led_2); 
  else WRITE_GPIO(pin_user_led_2,GPIO_PIN_RESET);
}

void config_tasks(){
   
  task_blink_timer = TIMING::Timing::Make(
    main_clock,
    TIMING::frequency_to_period(TIMING_LED_BLINK_FQ),
    true,
    task_blink 
    );

  task_blink_error_timer = TIMING::Timing::Make(
    main_clock,
    TIMING::frequency_to_period(TIMING_LED_ERROR_BLINK_FQ),
    true,
    task_blink_error
    );
  task_encoder_timer = TIMING::Timing::Make(
    main_clock,
    TIMING::frequency_to_period(TIMING_ENCODER_UPDATE_FQ),
    true,
    task_encoders
    );

  task_usb_timer = TIMING::Timing::Make(
    main_clock,
    TIMING::frequency_to_period(TIMING_USB_RECIVED_DATA_FQ),
    true,
    task_usb_handler
    );

  task_data_usb_send_timer = TIMING::Timing::Make(
    main_clock,
    TIMING::frequency_to_period(TIMING_USB_SEND_DATA_FQ),
    true,
    task_usb_data_loging
    );

  task_can_disconnected_timer = TIMING::Timing::Make(
    main_clock,
    TIMING_CAN_DISCONNECTED_PERIOD,
    false,
    task_can_disconnect
    );
  task_read_analog_values_timer = TIMING::Timing::Make(
    main_clock,
    TIMING::frequency_to_period(TIMING_READ_TEMPERATURE_FQ),
    true,
    task_read_analog_values
    );
  task_nodelay_timer = TIMING::Timing::Make(
    main_clock,
    0,
    true,
    task_nodelay
    );
  task_timer_scheduler.add_timer(task_blink_timer);
  task_timer_scheduler.add_timer(task_blink_error_timer);
  task_timer_scheduler.add_timer(task_encoder_timer);
  task_timer_scheduler.add_timer(task_usb_timer);
  task_timer_scheduler.add_timer(task_data_usb_send_timer);
  task_timer_scheduler.add_timer(task_can_disconnected_timer);
  task_timer_scheduler.add_timer(task_read_analog_values_timer);
  task_timer_scheduler.add_timer(task_nodelay_timer);
}

void main_loop(){
  log_debug("Start main_loop\n");
  task_timer_scheduler.schedules_handle_blocking();

  /*
  // #define DABA
  // #ifdef DABA
  // TIMING::Timing tim_blink(main_clock);
  // TIMING::Timing tim_blink_error(main_clock);
  // TIMING::Timing tim_encoder(main_clock);
  // TIMING::Timing tim_usb(main_clock);
  // TIMING::Timing tim_data_usb_send(main_clock);
  // TIMING::Timing tim_caculate_temp(main_clock);
 

  // tim_blink_error.set_behaviour(TIMING::frequency_to_period(TIMING_LED_ERROR_BLINK_FQ),true);
  // tim_blink.set_behaviour(TIMING::frequency_to_period(TIMING_LED_BLINK_FQ), true);
  // tim_encoder.set_behaviour(TIMING::frequency_to_period(TIMING_ENCODER_UPDATE_FQ), true);
  // tim_usb.set_behaviour(TIMING::frequency_to_period(TIMING_USB_RECIVED_DATA_FQ), true);
  // tim_data_usb_send.set_behaviour(TIMING::frequency_to_period(TIMING_USB_SEND_DATA_FQ), true);
  // tim_can_disconnecteded.set_behaviour(TIMING_CAN_DISCONNECTED_PERIOD, false);
  // tim_caculate_temp.set_behaviour(TIMING::frequency_to_period(TIMING_READ_TEMPERATURE_FQ), true);
  // uint32_t last_time = main_clock.get_micros();
  // int32_t counter = 0;
  // while (true){

  //   counter++;
  //   if(counter > 100000){
  //     uint32_t current_time = main_clock.get_micros();
  //     uint32_t diff = current_time - last_time;
  //     last_time = current_time;
  //     float freq = 100000000000 / (float)diff;
  //     log_error("main loop freq " + std::to_string(freq));
  //     counter = 0;
  //   }
  //   handle_can_rx();
  //   can_controler.handle();
    
  //   if(tim_encoder.triggered()){
  //     encoder_arm.handle();
  //     if(config.encoder_motor_enable)
  //       encoder_motor.handle();
  //   }

  //   if(tim_can_disconnecteded.triggered()){      
  //     movement_controler.set_enable(false);
  //     movement_controler.set_velocity(0);
  //     movement_controler.set_position(movement_controler.get_current_position());
  //     error_data.can_disconnected = true;
  //   }else{
  //     // error_data.can_disconnected = false;
  //   }
    
  //   movement_controler.handle();

  //   if(tim_data_usb_send.triggered()){
  //     log_info(
  //       loger.parse_to_json_format("ID",std::to_string(board_id.get_id()))+
  //       loger.parse_to_json_format("Vsen",std::to_string(voltage_vcc))+
  //       loger.parse_to_json_format("Tste",std::to_string(temoperature_steper_motor))+
  //       loger.parse_to_json_format("Tbor",std::to_string(temoperature_board))+
  //       loger.parse_to_json_format("Tmot",std::to_string(temoperature_steper_driver))+
  //       loger.parse_to_json_format("Eang",std::to_string(encoder_arm.get_angle()))+
  //       loger.parse_to_json_format("Pos",std::to_string(movement_controler.get_current_position()))+
  //       loger.parse_to_json_format("Vel",std::to_string(movement_controler.get_current_velocity()))+
  //       loger.parse_to_json_format("EPos",std::to_string(encoder_motor.get_absoulute_angle()))+
  //       loger.parse_to_json_format("Err",std::to_string(error_data.get_amount_of_errors()))+
  //       loger.parse_to_json_format("Errs",
  //         loger.parse_to_json_format("teng",BOOL_TO_STRING(error_data.temp_engine_overheating))+
  //         loger.parse_to_json_format("tdri",BOOL_TO_STRING(error_data.temp_driver_overheating))+
  //         loger.parse_to_json_format("tboa",BOOL_TO_STRING(error_data.temp_board_overheating))+
  //         loger.parse_to_json_format("tengdis",BOOL_TO_STRING(error_data.temp_engine_sensor_disconnect))+
  //         loger.parse_to_json_format("tdrivdis",BOOL_TO_STRING(error_data.temp_driver_sensor_disconnect))+
  //         loger.parse_to_json_format("tborddis",BOOL_TO_STRING(error_data.temp_board_sensor_disconnect))+
  //         loger.parse_to_json_format("encarmmdis",BOOL_TO_STRING(error_data.encoder_arm_disconnect))+
  //         loger.parse_to_json_format("encmotdis",BOOL_TO_STRING(error_data.encoder_motor_disconnect))+
  //         loger.parse_to_json_format("bovolt",BOOL_TO_STRING(error_data.baord_overvoltage))+
  //         loger.parse_to_json_format("buvolt",BOOL_TO_STRING(error_data.baord_undervoltage))+
  //         loger.parse_to_json_format("candis",BOOL_TO_STRING(error_data.can_disconnected))+
  //         loger.parse_to_json_format("canerr",BOOL_TO_STRING(error_data.can_error))+
  //         loger.parse_to_json_format("motlimit",BOOL_TO_STRING(error_data.controler_motor_limit_position),false)
  //       ,false,true)
  //     );
  //   }

  //   if(tim_usb.triggered()){
  //     usb_programer.handler();
  //   }

  //   if(tim_blink.triggered()) {
  //     TOGGLE_GPIO(pin_user_led_1);
  //   }

    
  //   if(tim_caculate_temp.triggered()){
  //     task_read_analog_values();
  //   }

  //   error_checks();
  //   auto errors_count = error_data.get_amount_of_errors();
  //   tim_blink_error.set_behaviour(TIMING::frequency_to_period((float)TIMING_LED_ERROR_BLINK_FQ*errors_count), true);
  //   if(tim_blink_error.triggered()){
  //     if(errors_count) TOGGLE_GPIO(pin_user_led_2); 
  //     else WRITE_GPIO(pin_user_led_2,GPIO_PIN_RESET);
  //   }
  // }
  // #endif
  */
}