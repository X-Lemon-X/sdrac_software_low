#include "main_prog.hpp"
#include "stmepic.hpp"
#include "can.hpp"
#include "config.hpp"
#include "encoder_magnetic.hpp"
#include "fram_i2c.hpp"
#include "logger.hpp"
#include "memory_fram.hpp"
#include "fram_i2c.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_i2c.h"
#include <cstddef>
#include <cstdint>


//**************************************************************************************************
// SCARY GLOBAL VARIABLES

se::movement::PIDController pid_pos;
std::shared_ptr<se::movement::BasicLinearPosControler> basic_controller;
std::shared_ptr<se::movement::PassThroughControler> pass_through_controller;
se::Timer tim_can_disconnecteded(se::Ticker::get_instance());

se::SimpleTask task_default_task;
se::SimpleTask task_blink_error_task;
se::SimpleTask task_read_analog_values_task;
se::SimpleTask task_encoder_timer;
se::SimpleTask task_usb_task;
se::SimpleTask task_data_usb_send_task;
se::SimpleTask task_caculate_temp_timer;
se::SimpleTask task_error_task;

std::shared_ptr<se::Timer> task_can_disconnected_timer;
float temoperature_board         = 0;
float temoperature_steper_driver = 0;
float temoperature_steper_motor  = 0;
float voltage_vcc                = 0;

//**************************************************************************************************
void run_main_prog() {

  //**************************************************************************************************
  // START BY CONFIGURING THE LOGGER
  log_debug(se::Logger::parse_to_json_format("state", "pre_perifial_config"));
  se::Ticker::get_instance().init(&htim10);
  se::Logger::get_instance().init(LOG_LOGER_LEVEL, LOG_SHOW_TIMESTAMP, CDC_Transmit_FS, false, version_string);

  //**************************************************************************************************
  // CONFIGURING THE PERIPHERALS

  log_debug(se::Logger::parse_to_json_format("state", "periferal_config"));
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

  auto mayby_i2c1 = se::I2C::Make(hi2c1, pin_i2c1_sda, pin_i2c1_scl, se::HardwareType::DMA);
  if(!mayby_i2c1.ok()) {
    log_error("I2C1 error: " + mayby_i2c1.status().to_string());
    HAL_NVIC_SystemReset();
  }

  i2c1 = mayby_i2c1.valueOrDie();

  auto mayby_i2c3 = se::I2C::Make(hi2c3, pin_i2c3_sda, pin_i2c3_scl, se::HardwareType::DMA);
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

  STMEPIC_ASSING_TO_OR_HRESET(can1, se::CAN::Make(hcan1, can_filter, &pin_tx_led, &pin_rx_led));


  STMEPIC_NONE_OR_HRESET(i2c1->hardware_reset());
  STMEPIC_NONE_OR_HRESET(i2c3->hardware_reset());
  STMEPIC_NONE_OR_HRESET(can1->hardware_start());


  //**************************************************************************************************
  // START DEFAULT TASK
  STMEPIC_NONE_OR_HRESET(task_default_task.task_init(task_blink, (void *)&pin_user_led_1,
                                                     FREQUENCY_TO_PERIOD_MS(TIMING_LED_BLINK_FQ), startup_robot, 8000));
  STMEPIC_NONE_OR_HRESET(task_default_task.task_run());
  osKernelStart();
}

uint8_t get_board_id() {
  uint8_t id = 0;
  id |= pin_cid_0.read();
  id |= pin_cid_1.read() << 1;
  id |= pin_cid_2.read() << 2;
  return id;
}

se::Status id_config() {
  log_debug(se::Logger::parse_to_json_format("state", "id_config"));

  //-------------------FRAM CONFIGURATION-------------------
  // fram = std::make_shared<se::memory::FramI2CFM24CLxx>(i2c1, FRAM_BEGIN_ADDRESS, FRAM_SIZE);
  STMEPIC_ASSING_TO_OR_RETURN(fram, se::memory::FramI2CFM24CLxx::Make(i2c1, FRAM_BEGIN_ADDRESS, FRAM_SIZE));
  fram->device_start();

  // probably here load data from FRAM
  auto mayby_config = fram->readStruct<ModuleConfig>(FRAM_CONFIG_ADDRESS);
  if(mayby_config.ok()) {
    module_config = mayby_config.valueOrDie();
    log_debug("Config loaded from FRAM");
  } else {
    log_debug("Config not loaded from FRAM");
  }

  switch(get_board_id()) {
  case BOARD_ID_1:
    config        = config_id_1;
    module_config = config_1;
    break;
  case BOARD_ID_2:
    config        = config_id_2;
    module_config = config_2;
    break;
  case BOARD_ID_3:
    config        = config_id_3;
    module_config = config_3;
    break;
  case BOARD_ID_4:
    config        = config_id_4;
    module_config = config_4;
    break;
  case BOARD_ID_5:
    config        = config_id_5;
    module_config = config_5;
    break;
  case BOARD_ID_6:
    config        = config_id_6;
    module_config = config_6;
    break;
  default:
    config        = config_id_default;
    module_config = config_default;
    break;
  }

  auto s = fram->writeStruct(FRAM_CONFIG_ADDRESS, module_config);
  i2c1->hardware_reset();
  return se::Status::OK();
}

void can_disconnect_timeout_reset() {
  task_can_disconnected_timer->timer_reset();
  error_data.can_disconnected = false;
}

void init_and_set_movement_controler_mode(uint8_t mode) {
  movement_controler.set_position(encoder_arm->get_absoulute_angle());
  movement_controler.set_velocity(0.0f);
  movement_controler.set_enable(false);
  std::shared_ptr<se::encoders::EncoderAbsoluteMagnetic> engine_encoder = nullptr;
  if(module_config.encoder_motor_enable) {
    engine_encoder = encoder_vel_motor;
  }

  motor = std::make_shared<se::motor::MotorClosedLoop>(stp_motor, encoder_arm, engine_encoder, nullptr);
  switch(mode) {
  case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE:
    // we switch control mode to position control however since the stepr
    // motor don't have the position control yet implemented we will use
    // the velocity control with BasicLinearPosControler that will achive
    // the position control
    movement_controler.init(motor, se::movement::MovementControlMode::VELOCITY, basic_controller);
    break;
  case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE:
    movement_controler.init(motor, se::movement::MovementControlMode::VELOCITY, pass_through_controller);
    break;
  case CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_TORQUE_CONTROL_CHOICE:
    // torque control is not implemented yet so we will use the velocity control
    movement_controler.init(motor, se::movement::MovementControlMode::TORQUE, pass_through_controller);
    break;
  default:
    // if the mode is not supported we will use the velocity control
    movement_controler.init(motor, se::movement::MovementControlMode::VELOCITY, pass_through_controller);
    break;
  }
}

se::Status post_id_config() {
  std::string info = "SDRACboard\n";
  info += "Software version:" + version_string + "\n";
  info += "Board id: " + std::to_string(get_board_id()) + "\n";
  info += "Description: SDRACboard from SDRAC project https://nihilia.xyz  "
          "https://konar.pwr.edu.pl\n";
  usb_programer.set_info(info);


  // se::DeviceThreadedSettings enc_device_settings;
  // enc_device_settings.period       = 20;
  // enc_device_settings.uxPriority   = 3;
  // enc_device_settings.uxStackDepth = 1054;

  // new se::motor::MotorClosedLoop(stp_motor, &encoder_arm, &encoder_vel_motor, nullptr);

  //-------------------STEPER MOTOR CONFIGURATION-------------------
  stp_motor.set_steps_per_revolution(module_config.stepper_motor_steps_per_rev);
  stp_motor.set_gear_ratio(module_config.stepper_motor_gear_ratio);
  stp_motor.set_max_velocity(module_config.stepper_motor_max_velocity);
  stp_motor.set_min_velocity(module_config.stepper_motor_min_velocity);
  stp_motor.set_reverse(module_config.stepper_motor_reverse);
  stp_motor.set_reversed_enable_pin(module_config.stepper_motor_enable_reversed);
  stp_motor.set_prescaler(module_config.stepper_motor_timer_prescaler);
  STMEPIC_RETURN_ON_ERROR(stp_motor.device_start());
  stp_motor.set_enable(false);


  //-------------------SERVO CONFIGURATION-------------------
  servo_motor = std::make_shared<se::motor::ServoMotorPWM>(htim14, TIM_CHANNEL_1);
  se::motor::ServoMotorPWMSettings servo_settings;
  servo_settings.max_angle_rad      = PI;
  servo_settings.min_pulse_width_us = 500.0f;
  servo_settings.max_pulse_width_us = 2500.0f;
  servo_settings.pwm_frequency      = 330.0f;
  servo_settings.min_angle_rad      = 0.0f;
  servo_settings.max_angle_rad      = 3.14f; // 180 degrees in radians
  servo_settings.n_multiplayer      = 4;     // Default multiplier for resolution
  STMEPIC_RETURN_ON_ERROR(servo_motor->device_set_settings(servo_settings));
  STMEPIC_RETURN_ON_ERROR(servo_motor->device_start());
  // servo_motor->set_enable(true);
  // servo_motor->set_position(PI_d2);


  //-------------------ENCODER ARM POSITION CONFIGURATION-------------------
  auto encoder_arm_filter_velocity = std::make_shared<se::filters::FilterSampleSkip>();
  encoder_arm_filter_velocity->set_samples_to_skip(module_config.encoder_arm_velocity_sample_amount);
  auto mayby_encoder_arm =
  se::encoders::EncoderAbsoluteMagneticMT6701::Make(i2c1, se::encoders::encoder_MT6701_addresses::MT6701_I2C_ADDRESS_1,
                                                    nullptr, encoder_arm_filter_velocity);
  encoder_arm = mayby_encoder_arm.valueOrDie();
  encoder_arm->set_offset(module_config.encoder_arm_offset);
  encoder_arm->set_reverse(module_config.encoder_arm_reverse);
  encoder_arm->set_dead_zone_correction_angle(module_config.encoder_arm_dead_zone_correction_angle);
  // STMEPIC_RETURN_ON_ERROR(encoder_arm->device_task_set_settings(enc_device_settings));
  STMEPIC_RETURN_ON_ERROR(encoder_arm->device_start());

  //-------------------ENCODER STEPER MOTOR POSITION CONFIGURATION-------------------
  // module_config.encoder_motor_enable
  if(module_config.encoder_motor_enable) {
    auto encoder_motor_moving_avarage = std::make_shared<se::filters::FilterMovingAvarage>();
    encoder_motor_moving_avarage->set_size(25); // 15 for smooth movement but delay with sampling to 50
    encoder_motor_moving_avarage->set_samples_to_skip(module_config.encoder_motor_velocity_sample_amount);
    STMEPIC_ASSING_TO_OR_RETURN(encoder_vel_motor, se::encoders::EncoderAbsoluteMagneticMT6701::Make(
                                                   i2c1, se::encoders::encoder_MT6701_addresses::MT6701_I2C_ADDRESS_2,
                                                   nullptr, encoder_motor_moving_avarage));
    encoder_vel_motor->set_offset(module_config.encoder_motor_offset);
    encoder_vel_motor->set_reverse(module_config.encoder_motor_reverse);
    encoder_vel_motor->set_dead_zone_correction_angle(module_config.encoder_motor_dead_zone_correction_angle);
    encoder_vel_motor->set_ratio(1.0f / stp_motor.get_gear_ratio());

    // STMEPIC_RETURN_ON_ERROR(encoder_vel_motor->device_task_set_settings(enc_device_settings));
    STMEPIC_RETURN_ON_ERROR(encoder_vel_motor->device_start());
  } else {
    encoder_vel_motor = encoder_arm;
  }
  //-------------------MOVEMENT CONTROLER CONFIGURATION-------------------

  // unly used if the pid controler is used
  // pid_pos.set_Kp(module_config.pid_p);
  // pid_pos.set_Ki(module_config.pid_i);
  // pid_pos.set_Kd(module_config.pid_d);

  // pass through controler is used for the velocity control
  pass_through_controller = std::make_shared<se::movement::PassThroughControler>();

  // used for the position control
  basic_controller = std::make_shared<se::movement::BasicLinearPosControler>();
  basic_controller->set_max_acceleration(module_config.movement_max_acceleration);
  basic_controller->set_target_pos_max_error(0.01f);

  // for velocity control we use the pass through controler whitch doesn't do
  // anything

  movement_controler.set_limit_position(module_config.movement_limit_lower, module_config.movement_limit_upper);
  movement_controler.set_max_velocity(module_config.movement_max_velocity);
  movement_controler.set_position(module_config.movement_limit_upper);
  movement_controler.set_position(encoder_arm->get_angle());
  movement_controler.set_velocity(0.0f);
  movement_controler.set_enable(false);
  init_and_set_movement_controler_mode(module_config.movement_control_mode);
  return se::Status::OK();
}

se::Status startup_robot(stmepic::SimpleTask &task, void *args) {
  (void)task;
  (void)args;
  STMEPIC_RETURN_ON_ERROR(id_config());
  STMEPIC_RETURN_ON_ERROR(post_id_config());


  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_clear_errors_frame_id, can_callback_clear_errors));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_set_control_mode_frame_id, can_callback_set_control_mode));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_get_errors_frame_id, can_callback_get_errors));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_status_frame_id, can_callback_status));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_set_pos_frame_id, can_callback_set_pos));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_get_pos_frame_id, can_callback_get_pos));
  STMEPIC_RETURN_ON_ERROR(
  can1->add_callback(config.can_konarm_set_effector_position_frame_id, can_callback_set_effector_position));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(0, can_callback_default));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_get_torque_frame_id, can_callback_get_torque));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_get_config_frame_id, can_callback_get_config));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_send_config_frame_id, can_callback_send_config));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_set_and_reset_frame_id, can_callback_set_and_reset));
  STMEPIC_RETURN_ON_ERROR(can1->add_callback(config.can_konarm_set_torque_frame_id, can_callback_set_torque));


  STMEPIC_RETURN_ON_ERROR(task_blink_error_task.task_init(task_blink_error, nullptr,
                                                          FREQUENCY_TO_PERIOD_MS(TIMING_LED_ERROR_BLINK_FQ)));
  STMEPIC_RETURN_ON_ERROR(task_data_usb_send_task.task_init(task_usb_data_loging, nullptr,
                                                            FREQUENCY_TO_PERIOD_MS(TIMING_USB_SEND_DATA_FQ),
                                                            nullptr, 15048));

  STMEPIC_RETURN_ON_ERROR(task_usb_task.task_init(task_usb_handler, nullptr,
                                                  FREQUENCY_TO_PERIOD_MS(TIMING_USB_RECIVED_DATA_FQ), nullptr, 1050));

  STMEPIC_RETURN_ON_ERROR(task_read_analog_values_task.task_init(task_read_analog_values, nullptr,
                                                                 FREQUENCY_TO_PERIOD_MS(TIMING_READ_TEMPERATURE_FQ)));

  STMEPIC_RETURN_ON_ERROR(task_error_task.task_init(task_error_check, nullptr, FREQUENCY_TO_PERIOD_MS(1000)));

  STMEPIC_ASSING_TO_OR_RETURN(task_can_disconnected_timer, se::Timer::Make(TIMING_CAN_DISCONNECTED_PERIOD, false));
  task_can_disconnected_timer->timer_reset();

  error_data.can_disconnected = true;

  STMEPIC_RETURN_ON_ERROR(task_blink_error_task.task_run());
  STMEPIC_RETURN_ON_ERROR(task_data_usb_send_task.task_run());
  STMEPIC_RETURN_ON_ERROR(task_usb_task.task_run());
  STMEPIC_RETURN_ON_ERROR(task_read_analog_values_task.task_run());
  STMEPIC_RETURN_ON_ERROR(task_error_task.task_run());
  return se::Status::OK();
}
