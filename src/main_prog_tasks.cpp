#include "MCP9700AT.hpp"
#include "config.hpp"
#include "logger.hpp"
#include "main_prog.hpp"
#include "stmepic.hpp"
#include "simple_task.hpp"


// void task_encoders(stmepic::Timing& task_timer) {
//   encoder_arm.handle();
//   if(config.encoder_motor_enable)
//     encoder_vel_motor.handle();
// }

// void task_nodelay(stmepic::Timing& task_timer) {
//   movement_controler.handle();
//   can_controler.handle();
//   error_checks();
// }

stmepic::Status task_error_check(stmepic::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  if(task_can_disconnected_timer->triggered()) {
    movement_controler.set_enable(false);
    movement_controler.set_velocity(0);
    movement_controler.set_position(movement_controler.get_current_position());
    error_data.can_disconnected = true;
  }

  error_data.temp_board_overheating =
  !std::isnan(temoperature_board) && temoperature_board > ERRORS_MAX_TEMP_BOARD ? true : false;
  error_data.temp_driver_overheating =
  !std::isnan(temoperature_steper_driver) && temoperature_steper_driver > ERRORS_MAX_TEMP_DRIVER ? true : false;
  error_data.temp_engine_overheating =
  !std::isnan(temoperature_steper_motor) && temoperature_steper_motor > ERRORS_MAX_TEMP_ENGINE ? true : false;
  error_data.temp_board_sensor_disconnect  = std::isnan(temoperature_board);
  error_data.temp_driver_sensor_disconnect = std::isnan(temoperature_steper_driver);
  error_data.temp_engine_sensor_disconnect = std::isnan(temoperature_steper_motor);

  error_data.encoder_arm_disconnect = !encoder_arm->device_is_connected().valueOrDie();
  if(config.encoder_motor_enable) {
    error_data.encoder_motor_disconnect = !encoder_vel_motor->device_is_connected().valueOrDie();
  } else {
    error_data.encoder_motor_disconnect = false;
  }

  error_data.baord_overvoltage  = voltage_vcc > ERRORS_MAX_VCC_VOLTAGE ? true : false;
  error_data.baord_undervoltage = voltage_vcc < ERRORS_MIN_VCC_VOLTAGE ? true : false;

  // can errors are handled in the handle_can_rx function

  error_data.controler_motor_limit_position = movement_controler.get_limit_position_achieved();
  return stmepic::Status::OK();
}

stmepic::Status task_usb_handler(stmepic::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  usb_programer.handler();
  return stmepic::Status::OK();
}

stmepic::Status task_usb_data_loging(stmepic::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  log_info(
  stmepic::Logger::parse_to_json_format("ID", get_board_id()) + stmepic::Logger::parse_to_json_format("Vsen", voltage_vcc) +
  stmepic::Logger::parse_to_json_format("Tste", temoperature_steper_motor) +
  stmepic::Logger::parse_to_json_format("Tbor", temoperature_board) +
  stmepic::Logger::parse_to_json_format("Tmot", temoperature_steper_driver) +
  stmepic::Logger::parse_to_json_format("EAang", encoder_arm->get_angle()) +
  stmepic::Logger::parse_to_json_format("EMang", encoder_vel_motor->get_angle()) +
  stmepic::Logger::parse_to_json_format("Pos", movement_controler.get_current_position()) +
  stmepic::Logger::parse_to_json_format("Vel", movement_controler.get_current_velocity()) +
  stmepic::Logger::parse_to_json_format("EPos", encoder_vel_motor->get_absoulute_angle()) +
  stmepic::Logger::parse_to_json_format("Err", error_data.get_amount_of_errors()) +
  stmepic::Logger::parse_to_json_format(
  "Errs",
  stmepic::Logger::parse_to_json_format("teng", BOOL_TO_STRING(error_data.temp_engine_overheating)) +
  stmepic::Logger::parse_to_json_format("tdri", BOOL_TO_STRING(error_data.temp_driver_overheating)) +
  stmepic::Logger::parse_to_json_format("tboa", BOOL_TO_STRING(error_data.temp_board_overheating)) +
  stmepic::Logger::parse_to_json_format("tengdis", BOOL_TO_STRING(error_data.temp_engine_sensor_disconnect)) +
  stmepic::Logger::parse_to_json_format("tdrivdis", BOOL_TO_STRING(error_data.temp_driver_sensor_disconnect)) +
  stmepic::Logger::parse_to_json_format("tborddis", BOOL_TO_STRING(error_data.temp_board_sensor_disconnect)) +
  stmepic::Logger::parse_to_json_format("encarmmdis", BOOL_TO_STRING(error_data.encoder_arm_disconnect)) +
  stmepic::Logger::parse_to_json_format("encmotdis", BOOL_TO_STRING(error_data.encoder_motor_disconnect)) +
  stmepic::Logger::parse_to_json_format("bovolt", BOOL_TO_STRING(error_data.baord_overvoltage)) +
  stmepic::Logger::parse_to_json_format("buvolt", BOOL_TO_STRING(error_data.baord_undervoltage)) +
  stmepic::Logger::parse_to_json_format("candis", BOOL_TO_STRING(error_data.can_disconnected)) +
  stmepic::Logger::parse_to_json_format("canerr", BOOL_TO_STRING(error_data.can_error)) +
  stmepic::Logger::parse_to_json_format("motlimit", BOOL_TO_STRING(error_data.controler_motor_limit_position), false),
  false, true));
  return stmepic::Status::OK();
}

stmepic::Status task_blink(stmepic::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  pin_user_led_1.toggle();
  return stmepic::Status::OK();
}

stmepic::Status task_read_analog_values(stmepic::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  temoperature_board = stmepic::sensors::temperature::MCP9700AT::get_temperature(pin_temp_board.get_voltage());
  temoperature_steper_driver = temp_steper_driver.get_temperature(pin_temp_steper_board.get_voltage());
  temoperature_steper_motor  = temp_steper_motor.get_temperature(pin_temp_motor.get_voltage());
  voltage_vcc                = pin_vsense.get_voltage() * ADC_VSENSE_MULTIPLIER;
  return stmepic::Status::OK();
}

stmepic::Status task_blink_error(stmepic::SimpleTask &task_handler, void *args) {
  (void)args;
  // error_checks();
  auto errors_count = error_data.get_amount_of_errors();
  task_handler.task_set_period(FREQUENCY_TO_PERIOD_MS((float)TIMING_LED_ERROR_BLINK_FQ * errors_count));

  if(errors_count)
    pin_user_led_2.toggle();
  else
    pin_user_led_2.write(GPIO_PIN_RESET);
  return stmepic::Status::OK();
}