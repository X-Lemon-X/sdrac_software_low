#include "MCP9700AT.hpp"
#include "config.hpp"
#include "logger.hpp"
#include "main_prog.hpp"
#include "stmepic.hpp"
#include "simple_task.hpp"


se::Status task_error_check(se::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  if(task_can_disconnected_timer->triggered()) {
    movement_controler.set_enable(false);
    movement_controler.set_velocity(0);
    movement_controler.set_position(movement_controler.get_current_position());

    // servo_motor->set_enable(false);
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
  return se::Status::OK();
}

se::Status task_usb_handler(se::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  usb_programer.handler();
  return se::Status::OK();
}

se::Status task_usb_data_loging(se::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  log_info(
  se::Logger::parse_to_json_format("ID", get_board_id()) + se::Logger::parse_to_json_format("Vsen", voltage_vcc) +
  se::Logger::parse_to_json_format("Tste", temoperature_steper_motor) +
  se::Logger::parse_to_json_format("Tbor", temoperature_board) +
  se::Logger::parse_to_json_format("Tmot", temoperature_steper_driver) +
  se::Logger::parse_to_json_format("EAang", encoder_arm->get_angle()) +
  se::Logger::parse_to_json_format("EMang", encoder_vel_motor->get_angle()) +
  se::Logger::parse_to_json_format("Pos", movement_controler.get_current_position()) +
  se::Logger::parse_to_json_format("Vel", movement_controler.get_current_velocity()) +
  se::Logger::parse_to_json_format("EPos", encoder_vel_motor->get_absoulute_angle()) +
  se::Logger::parse_to_json_format("Err", error_data.get_amount_of_errors()) +
  se::Logger::parse_to_json_format(
  "Errs",
  se::Logger::parse_to_json_format("teng", BOOL_TO_STRING(error_data.temp_engine_overheating)) +
  se::Logger::parse_to_json_format("tdri", BOOL_TO_STRING(error_data.temp_driver_overheating)) +
  se::Logger::parse_to_json_format("tboa", BOOL_TO_STRING(error_data.temp_board_overheating)) +
  se::Logger::parse_to_json_format("tengdis", BOOL_TO_STRING(error_data.temp_engine_sensor_disconnect)) +
  se::Logger::parse_to_json_format("tdrivdis", BOOL_TO_STRING(error_data.temp_driver_sensor_disconnect)) +
  se::Logger::parse_to_json_format("tborddis", BOOL_TO_STRING(error_data.temp_board_sensor_disconnect)) +
  se::Logger::parse_to_json_format("encarmmdis", BOOL_TO_STRING(error_data.encoder_arm_disconnect)) +
  se::Logger::parse_to_json_format("encmotdis", BOOL_TO_STRING(error_data.encoder_motor_disconnect)) +
  se::Logger::parse_to_json_format("bovolt", BOOL_TO_STRING(error_data.baord_overvoltage)) +
  se::Logger::parse_to_json_format("buvolt", BOOL_TO_STRING(error_data.baord_undervoltage)) +
  se::Logger::parse_to_json_format("candis", BOOL_TO_STRING(error_data.can_disconnected)) +
  se::Logger::parse_to_json_format("canerr", BOOL_TO_STRING(error_data.can_error)) +
  se::Logger::parse_to_json_format("motlimit", BOOL_TO_STRING(error_data.controler_motor_limit_position), false),
  false, true));
  return se::Status::OK();
}

se::Status task_blink(se::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  STMEPIC_NONE_OR_HRESET(task_handler.task_get_status());
  pin_user_led_1.toggle();

  auto st = encoder_arm->device_get_status();
  if(!st.ok())
    i2c1->hardware_reset();

  return se::Status::OK();
}

se::Status task_read_analog_values(se::SimpleTask &task_handler, void *args) {
  (void)task_handler;
  (void)args;
  temoperature_board = se::sensors::temperature::MCP9700AT::get_temperature(pin_temp_board.get_voltage());
  temoperature_steper_driver = temp_steper_driver.get_temperature(pin_temp_steper_board.get_voltage());
  temoperature_steper_motor  = temp_steper_motor.get_temperature(pin_temp_motor.get_voltage());
  voltage_vcc                = pin_vsense.get_voltage() * ADC_VSENSE_MULTIPLIER;
  return se::Status::OK();
}

se::Status task_blink_error(se::SimpleTask &task_handler, void *args) {
  (void)args;
  // error_checks();
  auto errors_count = error_data.get_amount_of_errors();
  task_handler.task_set_period(FREQUENCY_TO_PERIOD_MS((float)TIMING_LED_ERROR_BLINK_FQ * errors_count));

  if(errors_count)
    pin_user_led_2.toggle();
  else
    pin_user_led_2.write(GPIO_PIN_RESET);
  return se::Status::OK();
}