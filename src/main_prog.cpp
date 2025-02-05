#include "main_prog.hpp"
#include "can.hpp"
#include "config.hpp"
#include "fram_i2c.hpp"
#include "logger.hpp"
#include "memory_fram.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_i2c.h"
#include "stmepic.hpp"
#include <cstddef>
#include <cstdint>


//**************************************************************************************************
// SCARY GLOBAL VARIABLES

stmepic::movement::PIDControler pid_pos;
stmepic::movement::BasicLinearPosControler bacis_controler;
stmepic::movement::PassThroughControler pass_through_controler;
stmepic::filters::FilterMovingAvarage encoder_motor_moving_avarage;
stmepic::filters::FilterSampleSkip encoder_arm_filter_velocity;
stmepic::Timing tim_can_disconnecteded(stmepic::Ticker::get_instance());

stmepic::SimpleTask task_blink_timer;
stmepic::SimpleTask task_blink_error_timer;
stmepic::SimpleTask task_read_analog_values_timer;
stmepic::SimpleTask task_encoder_timer;
stmepic::SimpleTask task_usb_timer;
stmepic::SimpleTask task_data_usb_send_timer;
stmepic::SimpleTask task_caculate_temp_timer;
stmepic::SimpleTask task_error_timer;

// std::shared_ptr<stmepic::Timing> task_blink_timer;
// std::shared_ptr<stmepic::Timing> task_blink_error_timer;
// std::shared_ptr<stmepic::Timing> task_read_analog_values_timer;
// std::shared_ptr<stmepic::Timing> task_encoder_timer;
// std::shared_ptr<stmepic::Timing> task_usb_timer;
// std::shared_ptr<stmepic::Timing> task_data_usb_send_timer;
// std::shared_ptr<stmepic::Timing> task_caculate_temp_timer;
// std::shared_ptr<stmepic::Timing> task_nodelay_timer;
std::shared_ptr<stmepic::Timing> task_can_disconnected_timer;
float temoperature_board         = 0;
float temoperature_steper_driver = 0;
float temoperature_steper_motor  = 0;
float voltage_vcc                = 0;

//**************************************************************************************************
void run_main_prog() {
  pre_periferal_config();
  periferal_config();
  id_config();
  post_id_config();
  config_tasks();
  main_loop();
}

void pre_periferal_config() {
  log_debug(stmepic::Logger::parse_to_json_format("state", "pre_perifial_config"));
  stmepic::Ticker::get_instance().init(&htim10);
  stmepic::Logger::get_instance().init(LOG_LOGER_LEVEL, LOG_SHOW_TIMESTAMP,
                                       CDC_Transmit_FS, version_string);
}

void periferal_config() {
  log_debug(stmepic::Logger::parse_to_json_format("state", "periferal_config"));
  // dma adc1 settings
  HAL_ADC_Start_DMA(&hadc1, adc_dma_buffer, ADC_DMA_BUFFER_SIZE);

  // HAL_I2C_DeInit(&hi2c1);
  // HAL_I2C_Init(&hi2c1);

  // timer 10 settings
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_TIM_Base_Start_IT(&htim10);


  // timer 3 settings
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  auto mayby_i2c1 =
  stmepic::I2C::Make(hi2c1, pin_i2c1_sda, pin_i2c1_scl, stmepic::HardwareType::DMA);
  if(!mayby_i2c1.ok()) {
    log_error("I2C1 error: " + mayby_i2c1.status().to_string());
    HAL_NVIC_SystemReset();
  }

  i2c1 = mayby_i2c1.valueOrDie();

  auto mayby_i2c3 =
  stmepic::I2C::Make(hi2c3, pin_i2c3_sda, pin_i2c3_scl, stmepic::HardwareType::DMA);
  if(!mayby_i2c3.ok()) {
    log_error("I2C3 error: " + mayby_i2c3.status().to_string());
    HAL_NVIC_SystemReset();
  }
  i2c3 = mayby_i2c3.valueOrDie();

  CAN_FilterTypeDef can_filter;
  can_filter.FilterBank           = 1;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation     = CAN_FILTER_ENABLE;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh         = config.can_filter_id_high;
  can_filter.FilterIdLow          = config.can_filter_id_low;
  can_filter.FilterMaskIdHigh     = config.can_filter_mask_high;
  can_filter.FilterMaskIdLow      = config.can_filter_mask_low;
  can_filter.SlaveStartFilterBank = 0;
  auto mayby_can1 = stmepic::CAN::Make(hcan1, can_filter, &pin_tx_led, &pin_rx_led);
  if(!mayby_can1.ok()) {
    log_error("CAN1 error: " + mayby_can1.status().to_string());
    HAL_NVIC_SystemReset();
  }
  can1 = mayby_can1.valueOrDie();

  i2c1->hardware_start();
  i2c3->hardware_start();
  can1->hardware_start();
}

uint8_t get_board_id() {
  uint8_t id = 0;
  id |= pin_cid_0.read();
  id |= pin_cid_1.read() << 1;
  id |= pin_cid_2.read() << 2;
  return id;
}

void id_config() {
  log_debug(stmepic::Logger::parse_to_json_format("state", "id_config"));
  switch(get_board_id()) {
  case BOARD_ID_1: config = config_id_1; break;
  case BOARD_ID_2: config = config_id_2; break;
  case BOARD_ID_3: config = config_id_3; break;
  case BOARD_ID_4: config = config_id_4; break;
  case BOARD_ID_5: config = config_id_5; break;
  case BOARD_ID_6: config = config_id_6; break;
  default: config = config_id_default; break;
  }
}

void can_disconnect_timeout_reset() {
  task_can_disconnected_timer->reset();
  error_data.can_disconnected = false;
}

void init_and_set_movement_controler_mode(uint8_t mode) {
  movement_controler.set_position(encoder_arm.get_absoulute_angle());
  movement_controler.set_velocity(0.0f);
  movement_controler.set_enable(false);
  stmepic::encoders::EncoderAbsoluteMagnetic* engine_encoder = nullptr;
  if(config.encoder_motor_enable) {
    engine_encoder = &encoder_vel_motor;
  }
  if(motor != nullptr) {
    delete motor;
  }
  motor = new stmepic::motor::MotorClosedLoop(stp_motor, &encoder_arm, engine_encoder, nullptr);

  switch(mode) {
  case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE:
    // we switch control mode to position control however since the stepr
    // motor don't have the position control yet implemented we will use
    // the velocity control with BasicLinearPosControler that will achive
    // the position control
    movement_controler.init(*motor, stmepic::movement::MovementControlMode::VELOCITY, bacis_controler);
    break;
  case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE:
    movement_controler.init(*motor, stmepic::movement::MovementControlMode::VELOCITY,
                            pass_through_controler);
    break;
  case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_TORQUE_CONTROL_CHOICE:
    // torque control is not implemented yet so we will use the velocity control
    movement_controler.init(*motor, stmepic::movement::MovementControlMode::TORQUE, pass_through_controler);
    break;
  default:
    // if the mode is not supported we will use the velocity control
    movement_controler.init(*motor, stmepic::movement::MovementControlMode::VELOCITY,
                            pass_through_controler);
    break;
  }
}

void post_id_config() {
  std::string info = "SDRACboard\n";
  info += "Software version:" + version_string + "\n";
  info += "Board id: " + std::to_string(get_board_id()) + "\n";
  info += "Description: SDRACboard from SDRAC project https://nihilia.xyz  "
          "https://konar.pwr.edu.pl\n";
  usb_programer.set_info(info);

  //-------------------FRAM CONFIGURATION-------------------
  fram = std::shared_ptr<stmepic::memory::FramI2CFM24CLxx>(
  new stmepic::memory::FramI2CFM24CLxx(i2c1, FRAM_BEGIN_ADDRESS, FRAM_SIZE));

  stmepic::DeviceThrededSettingsBase enc_device_settings;
  enc_device_settings.period       = 20;
  enc_device_settings.uxPriority   = 3;
  enc_device_settings.uxStackDepth = 254;

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
  // encoder_arm.set_enable_encoder(true);
  encoder_arm_filter_velocity.set_samples_to_skip(config.encoder_arm_velocity_sample_amount);
  encoder_arm_filter_velocity.set_init_value(0);
  encoder_arm.init(i2c1, stmepic::encoders::translate_reg_to_angle_MT6701, nullptr,
                   &encoder_arm_filter_velocity);
  encoder_arm.device_task_set_settings(enc_device_settings);
  encoder_arm.device_task_start();

  //-------------------ENCODER STEPER MOTOR POSITION
  // CONFIGURATION-------------------
  encoder_vel_motor.set_offset(config.encoder_motor_offset);
  encoder_vel_motor.set_reverse(config.encoder_motor_reverse);
  encoder_vel_motor.set_dead_zone_correction_angle(config.encoder_motor_dead_zone_correction_angle);
  encoder_vel_motor.set_angle_register(ENCODER_MT6701_ANGLE_REG);
  encoder_vel_motor.set_resolution(ENCODER_MT6702_RESOLUTION);
  encoder_vel_motor.set_address(ENCODER_MT6701_I2C_ADDRESS_2);
  encoder_vel_motor.set_ratio(1.0f / stp_motor.get_gear_ratio());
  encoder_motor_moving_avarage.set_size(25); // 15 for smooth movement but delay with sampling to 50
  encoder_motor_moving_avarage.set_samples_to_skip(config.encoder_motor_velocity_sample_amount);
  encoder_vel_motor.init(i2c1, stmepic::encoders::translate_reg_to_angle_MT6701, nullptr,
                         &encoder_motor_moving_avarage);
  if(config.encoder_motor_enable) {
    encoder_vel_motor.device_task_set_settings(enc_device_settings);
    encoder_vel_motor.device_task_start();
  }
  //-------------------MOVEMENT CONTROLER CONFIGURATION-------------------

  // unly used if the pid controler is used
  pid_pos.set_Kp(config.pid_p);
  pid_pos.set_Ki(config.pid_i);
  pid_pos.set_Kd(config.pid_d);

  // used for the position control
  bacis_controler.set_max_acceleration(config.movement_max_acceleration);
  bacis_controler.set_target_pos_max_error(0.01f);

  // for velocity control we use the pass through controler whitch doesn't do
  // anything

  movement_controler.set_limit_position(config.movement_limit_lower, config.movement_limit_upper);
  movement_controler.set_max_velocity(config.movement_max_velocity);
  movement_controler.set_position(config.movement_limit_upper);
  movement_controler.set_position(encoder_arm.get_angle());
  movement_controler.set_velocity(0.0f);
  movement_controler.set_enable(false);
  init_and_set_movement_controler_mode(config.movement_control_mode);
}


void config_tasks() {

  can1->add_callback(config.can_konarm_clear_errors_frame_id, can_callback_clear_errors);
  can1->add_callback(config.can_konarm_set_control_mode_frame_id, can_callback_set_control_mode);
  can1->add_callback(config.can_konarm_get_errors_frame_id, can_callback_get_errors);
  can1->add_callback(config.can_konarm_status_frame_id, can_callback_status);
  can1->add_callback(config.can_konarm_set_pos_frame_id, can_callback_set_pos);
  can1->add_callback(config.can_konarm_get_pos_frame_id, can_callback_get_pos);
  can1->add_callback(0, can_callback_default);

  task_blink_timer.task_init(task_blink, (void*)&pin_user_led_1,
                             FREQUENCY_TO_PERIOD_MS(TIMING_LED_BLINK_FQ));
  task_blink_error_timer.task_init(task_blink_error, nullptr,
                                   FREQUENCY_TO_PERIOD_MS(TIMING_LED_ERROR_BLINK_FQ));
  task_data_usb_send_timer.task_init(task_usb_data_loging, nullptr,
                                     FREQUENCY_TO_PERIOD_MS(TIMING_USB_SEND_DATA_FQ), 3048);

  task_usb_timer.task_init(task_usb_handler, nullptr,
                           FREQUENCY_TO_PERIOD_MS(TIMING_USB_RECIVED_DATA_FQ), 1050);

  task_read_analog_values_timer.task_init(task_read_analog_values, nullptr,
                                          FREQUENCY_TO_PERIOD_MS(TIMING_READ_TEMPERATURE_FQ));

  task_error_timer.task_init(task_error_check, nullptr, FREQUENCY_TO_PERIOD_MS(1000));

  auto mayby_timer = stmepic::Timing::Make(TIMING_CAN_DISCONNECTED_PERIOD, false);
  if(!mayby_timer.ok()) {
    log_error("Can't create can disconnect timer");
    HAL_NVIC_SystemReset();
  }
  task_can_disconnected_timer = mayby_timer.valueOrDie();
  task_can_disconnected_timer->reset();
  error_data.can_disconnected = true;

  task_blink_timer.task_run();
  task_blink_error_timer.task_run();
  task_data_usb_send_timer.task_run();
  task_usb_timer.task_run();
  task_read_analog_values_timer.task_run();
  task_error_timer.task_run();
}

void main_loop() {
  log_debug("Start scheduler\n");
  osKernelStart();
}