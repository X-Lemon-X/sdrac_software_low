#include "main_prog.hpp"
#include "config.hpp"
#include "stm32f4xx_hal_cortex.h"
#include "stmepic.hpp"

//**************************************************************************************************
// SCARY GLOBAL VARIABLES

stmepic::PIDControler pid_pos(main_clock);
stmepic::BasicLinearPosControler bacis_controler(main_clock);
stmepic::PassThroughControler pass_through_controler(main_clock);
stmepic::filters::FilterMovingAvarage encoder_motor_moving_avarage;
stmepic::filters::FilterSampleSkip encoder_arm_filter_velocity;
stmepic::Timing tim_can_disconnecteded(main_clock);

std::shared_ptr<stmepic::Timing> task_blink_timer;
std::shared_ptr<stmepic::Timing> task_blink_error_timer;
std::shared_ptr<stmepic::Timing> task_read_analog_values_timer;
std::shared_ptr<stmepic::Timing> task_encoder_timer;
std::shared_ptr<stmepic::Timing> task_usb_timer;
std::shared_ptr<stmepic::Timing> task_data_usb_send_timer;
std::shared_ptr<stmepic::Timing> task_caculate_temp_timer;
std::shared_ptr<stmepic::Timing> task_nodelay_timer;
std::shared_ptr<stmepic::Timing> task_can_disconnected_timer;
float temoperature_board=0;
float temoperature_steper_driver=0;
float temoperature_steper_motor=0; 
float voltage_vcc=0;



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
  switch (board_id.get_id()){
  case BOARD_ID_1: config = config_id_1; break;
  case BOARD_ID_2: config = config_id_2; break;
  case BOARD_ID_3: config = config_id_3; break;
  case BOARD_ID_4: config = config_id_4; break;
  case BOARD_ID_5: config = config_id_5; break;
  case BOARD_ID_6: config = config_id_6; break;
  default: config = config_id_default; break;
  }
}

void can_disconnect_timeout_reset(){
  task_can_disconnected_timer->reset();
  error_data.can_disconnected = false;
}

void init_and_set_movement_controler_mode(uint8_t mode){
  movement_controler.set_position(movement_controler.get_current_position());
  movement_controler.set_velocity(0.0f);
  movement_controler.set_enable(false);
  stmepic::encoders::EncoderAbsoluteMagnetic *engine_encoder = nullptr;
  if(config.encoder_motor_enable){
    engine_encoder = &encoder_vel_motor;
  }
  switch (mode){
    case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE:
      // we switch control mode to position control however since the stepr motor don't have the position control yet implemented
      // we will use the velocity control with BasicLinearPosControler that will achive the position control
      movement_controler.init(main_clock, motor,stmepic::MovementControlMode::VELOCITY, bacis_controler);
      break;
    case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE:
      movement_controler.init(main_clock, motor, stmepic::MovementControlMode::VELOCITY,pass_through_controler);
      break;
    case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_TORQUE_CONTROL_CHOICE:
      // torque control is not implemented yet so we will use the velocity control
      movement_controler.init(main_clock, motor,stmepic::MovementControlMode::TORQUE,pass_through_controler);
      break;
    default:
      // if the mode is not supported we will use the velocity control
      movement_controler.init(main_clock, motor,stmepic::MovementControlMode::VELOCITY,pass_through_controler);
      break;
  }
}

void post_id_config(){
  std::string info = "SDRACboard\n";
  info += "Software version:" + version_string + "\n";
  info += "Board id: " + std::to_string(board_id.get_id()) + "\n";
  info += "Description: SDRACboard from SDRAC project https://nihilia.xyz  https://konar.pwr.edu.pl\n";
  usb_programer.set_info(info);

  //-------------------STEPER MOTOR CONFIGURATION-------------------
  stp_motor.set_steps_per_revolution(config.stepper_motor_steps_per_rev);
  stp_motor.set_gear_ratio(config.stepper_motor_gear_ratio);
  stp_motor.set_max_velocity(config.stepper_motor_max_velocity);
  stp_motor.set_min_velocity(config.stepper_motor_min_velocity);
  stp_motor.set_reverse(config.stepper_motor_reverse);
  stp_motor.set_reversed_enable_pin(config.stepper_motor_enable_reversed);
  stp_motor.set_prescaler(config.stepper_motor_timer_prescaler);
  stp_motor.init();
  stp_motor.set_enable(false);  


  //-------------------ENCODER ARM POSITION CONFIGURATION-------------------
  encoder_arm.set_offset(config.encoder_arm_offset);
  encoder_arm.set_reverse(config.encoder_arm_reverse);
  encoder_arm.set_dead_zone_correction_angle(config.encoder_arm_dead_zone_correction_angle);
  encoder_arm.set_angle_register(ENCODER_MT6701_ANGLE_REG);
  encoder_arm.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder_arm.set_address(ENCODER_MT6701_I2C_ADDRESS);
  encoder_arm.set_enable_encoder(true);
  encoder_arm_filter_velocity.set_samples_to_skip(config.encoder_arm_velocity_sample_amount);
  encoder_arm_filter_velocity.set_init_value(0);
  encoder_arm.init(hi2c1,main_clock,stmepic::encoders::translate_reg_to_angle_MT6701,nullptr,&encoder_arm_filter_velocity);
  

  //-------------------ENCODER STEPER MOTOR POSITION CONFIGURATION-------------------
  encoder_vel_motor.set_offset(config.encoder_motor_offset);
  encoder_vel_motor.set_reverse(config.encoder_motor_reverse);
  encoder_vel_motor.set_dead_zone_correction_angle(config.encoder_motor_dead_zone_correction_angle);
  encoder_vel_motor.set_angle_register(ENCODER_MT6701_ANGLE_REG);
  encoder_vel_motor.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder_vel_motor.set_address(ENCODER_MT6701_I2C_ADDRESS_2); 
  encoder_vel_motor.set_ratio(1.0f / stp_motor.get_gear_ratio());
  encoder_vel_motor.set_enable_encoder(config.encoder_motor_enable);
  encoder_motor_moving_avarage.set_size(25); // 15 for smooth movement but delay with sampling to 50
  encoder_motor_moving_avarage.set_samples_to_skip(config.encoder_motor_velocity_sample_amount);
  encoder_vel_motor.init(hi2c1,main_clock,stmepic::encoders::translate_reg_to_angle_MT6701,nullptr,&encoder_motor_moving_avarage);
  

  //-------------------MOVEMENT CONTROLER CONFIGURATION-------------------
  
  // unly used if the pid controler is used
  pid_pos.set_Kp(config.pid_p);
  pid_pos.set_Ki(config.pid_i);
  pid_pos.set_Kd(config.pid_d);

  // used for the position control
  bacis_controler.set_max_acceleration(config.movement_max_acceleration);
  bacis_controler.set_target_pos_max_error(0.01f);

  // for velocity control we use the pass through controler whitch doesn't do anything

  movement_controler.set_limit_position(config.movement_limit_lower, config.movement_limit_upper);
  movement_controler.set_max_velocity(config.movement_max_velocity);
  movement_controler.set_position(config.movement_limit_upper);
  init_and_set_movement_controler_mode(config.movement_control_mode);


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
  can_controler.init(hcan1, CAN_FILTER_FIFO0, main_clock, &pin_tx_led, &pin_rx_led);
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
  error_data.encoder_motor_disconnect = !encoder_vel_motor.is_connected();

  error_data.baord_overvoltage = voltage_vcc > ERRORS_MAX_VCC_VOLTAGE? true : false;
  error_data.baord_undervoltage = voltage_vcc < ERRORS_MIN_VCC_VOLTAGE? true : false;

  // can errors are handled in the handle_can_rx function

  error_data.controler_motor_limit_position = movement_controler.get_limit_position_achieved();
}


void config_tasks(){

  can_controler.add_callback(config.can_konarm_clear_errors_frame_id, can_callback_clear_errors);
  can_controler.add_callback(config.can_konarm_set_control_mode_frame_id, can_callback_set_control_mode);
  can_controler.add_callback(config.can_konarm_get_errors_frame_id, can_callback_get_errors);
  can_controler.add_callback(config.can_konarm_status_frame_id, can_callback_status);
  can_controler.add_callback(config.can_konarm_set_pos_frame_id, can_callback_set_pos);
  can_controler.add_callback(config.can_konarm_get_pos_frame_id, can_callback_get_pos);
  can_controler.add_callback(stmepic::CAN_DEFAULT_FRAME_ID, can_callback_default);

   
  task_blink_timer = stmepic::Timing::Make(
    main_clock,
    stmepic::frequency_to_period_us(TIMING_LED_BLINK_FQ),
    true,
    task_blink 
    ).valueOrDie();

  task_blink_error_timer = stmepic::Timing::Make(
    main_clock,
    stmepic::frequency_to_period_us(TIMING_LED_ERROR_BLINK_FQ),
    true,
    task_blink_error
    ).valueOrDie();
  task_encoder_timer = stmepic::Timing::Make(
    main_clock,
    stmepic::frequency_to_period_us(TIMING_ENCODER_UPDATE_FQ),
    true,
    task_encoders
    ).valueOrDie();

  task_usb_timer = stmepic::Timing::Make(
    main_clock,
    stmepic::frequency_to_period_us(TIMING_USB_RECIVED_DATA_FQ),
    true,
    task_usb_handler
    ).valueOrDie();

  task_data_usb_send_timer = stmepic::Timing::Make(
    main_clock,
    stmepic::frequency_to_period_us(TIMING_USB_SEND_DATA_FQ),
    true,
    task_usb_data_loging
    ).valueOrDie();

  task_can_disconnected_timer = stmepic::Timing::Make(
    main_clock,
    TIMING_CAN_DISCONNECTED_PERIOD,
    false,
    task_can_disconnect
    ).valueOrDie();
  task_read_analog_values_timer = stmepic::Timing::Make(
    main_clock,
    stmepic::frequency_to_period_us(TIMING_READ_TEMPERATURE_FQ),
    true,
    task_read_analog_values
    ).valueOrDie();
  task_nodelay_timer = stmepic::Timing::Make(
    main_clock,
    0,
    true,
    task_nodelay
    ).valueOrDie();

  auto status = task_timer_scheduler.add_timer(task_blink_timer);
  if(!status.ok()){
    log_error("Error adding task_blink_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }

  status = task_timer_scheduler.add_timer(task_blink_error_timer);
  if(!status.ok()){
    log_error("Error adding task_blink_error_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }


  status = task_timer_scheduler.add_timer(task_encoder_timer);
  if (!status.ok()){
    log_error("Error adding task_encoder_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }

  status = task_timer_scheduler.add_timer(task_usb_timer);
  if (!status.ok()){
    log_error("Error adding task_usb_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }

  status = task_timer_scheduler.add_timer(task_data_usb_send_timer);
  if (!status.ok()){
    log_error("Error adding task_data_usb_send_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }

  status = task_timer_scheduler.add_timer(task_can_disconnected_timer);
  if (!status.ok()){
    log_error("Error adding task_can_disconnected_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }

  status = task_timer_scheduler.add_timer(task_read_analog_values_timer);
  if (!status.ok()){
    log_error("Error adding task_read_analog_values_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }

  status = task_timer_scheduler.add_timer(task_nodelay_timer);
  if (!status.ok()){
    log_error("Error adding task_nodelay_timer to the scheduler");
    HAL_NVIC_SystemReset();
  }
}

void main_loop(){
  log_debug("Start main_loop\n");
  task_timer_scheduler.schedules_handle_blocking();
}