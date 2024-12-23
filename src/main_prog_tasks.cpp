#include "config.hpp"
#include "logger.hpp"
#include "main_prog.hpp"



void task_encoders(stmepic::Timing& task_timer){
  encoder_arm.handle();
  if(config.encoder_motor_enable)
    encoder_vel_motor.handle();
}

void task_nodelay(stmepic::Timing& task_timer){
  movement_controler.handle();
  can_controler.handle();
  error_checks();
}

void task_can_disconnect(stmepic::Timing& task_timer){
  movement_controler.set_enable(false);
  movement_controler.set_velocity(0);
  movement_controler.set_position(movement_controler.get_current_position());
  error_data.can_disconnected = true;
}

void task_usb_handler(stmepic::Timing& task_timer){
  usb_programer.handler();
}

void task_usb_data_loging(stmepic::Timing& task_timer){
  log_info(
        stmepic::Logger::parse_to_json_format("ID",get_board_id())+
        stmepic::Logger::parse_to_json_format("Vsen",voltage_vcc)+
        stmepic::Logger::parse_to_json_format("Tste",temoperature_steper_motor)+
        stmepic::Logger::parse_to_json_format("Tbor",temoperature_board)+
        stmepic::Logger::parse_to_json_format("Tmot",temoperature_steper_driver)+
        stmepic::Logger::parse_to_json_format("EEang",encoder_arm.get_angle())+
        stmepic::Logger::parse_to_json_format("EAang",encoder_vel_motor.get_angle())+
        stmepic::Logger::parse_to_json_format("Pos",movement_controler.get_current_position())+
        stmepic::Logger::parse_to_json_format("Vel",movement_controler.get_current_velocity())+
        stmepic::Logger::parse_to_json_format("EPos",encoder_vel_motor.get_absoulute_angle())+
        stmepic::Logger::parse_to_json_format("Err",error_data.get_amount_of_errors())+
        stmepic::Logger::parse_to_json_format("Errs",
          stmepic::Logger::parse_to_json_format("teng",BOOL_TO_STRING(error_data.temp_engine_overheating))+
          stmepic::Logger::parse_to_json_format("tdri",BOOL_TO_STRING(error_data.temp_driver_overheating))+
          stmepic::Logger::parse_to_json_format("tboa",BOOL_TO_STRING(error_data.temp_board_overheating))+
          stmepic::Logger::parse_to_json_format("tengdis",BOOL_TO_STRING(error_data.temp_engine_sensor_disconnect))+
          stmepic::Logger::parse_to_json_format("tdrivdis",BOOL_TO_STRING(error_data.temp_driver_sensor_disconnect))+
          stmepic::Logger::parse_to_json_format("tborddis",BOOL_TO_STRING(error_data.temp_board_sensor_disconnect))+
          stmepic::Logger::parse_to_json_format("encarmmdis",BOOL_TO_STRING(error_data.encoder_arm_disconnect))+
          stmepic::Logger::parse_to_json_format("encmotdis",BOOL_TO_STRING(error_data.encoder_motor_disconnect))+
          stmepic::Logger::parse_to_json_format("bovolt",BOOL_TO_STRING(error_data.baord_overvoltage))+
          stmepic::Logger::parse_to_json_format("buvolt",BOOL_TO_STRING(error_data.baord_undervoltage))+
          stmepic::Logger::parse_to_json_format("candis",BOOL_TO_STRING(error_data.can_disconnected))+
          stmepic::Logger::parse_to_json_format("canerr",BOOL_TO_STRING(error_data.can_error))+
          stmepic::Logger::parse_to_json_format("motlimit",BOOL_TO_STRING(error_data.controler_motor_limit_position),false)
        ,false,true)
      );
}

void task_blink(stmepic::Timing& task_timer){
  TOGGLE_GPIO(pin_user_led_1);
}

void task_read_analog_values(stmepic::Timing& task_timer){
  temoperature_board = stmepic::sensors::MCP9700AT::get_temperature(VOLTAGE_VALUE(pin_temp_board));
  temoperature_steper_driver = temp_steper_driver.get_temperature(VOLTAGE_VALUE(pin_temp_steper_board)); 
  temoperature_steper_motor = temp_steper_motor.get_temperature(VOLTAGE_VALUE(pin_temp_motor));
  voltage_vcc = VOLTAGE_VALUE(pin_vsense) * ADC_VSENSE_MULTIPLIER;
}

void task_blink_error(stmepic::Timing& task_timer){
  // error_checks();
  auto errors_count = error_data.get_amount_of_errors();
  task_timer.set_behaviour(stmepic::frequency_to_period_us((float)TIMING_LED_ERROR_BLINK_FQ*errors_count), true);
  
  if(errors_count) TOGGLE_GPIO(pin_user_led_2); 
  else WRITE_GPIO(pin_user_led_2,GPIO_PIN_RESET);
}