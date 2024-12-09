#include "config.hpp"
#include "motor.hpp"
#include "steper_motor.hpp"
#include "movement_controler.hpp"
#include "dfu_usb_programer.hpp"
#include "controler_pid.hpp"
#include "filter.hpp"
#include "filter_moving_avarage.hpp"
#include "filter_alfa_beta.hpp"
#include "ntc_termistors.hpp"
#include "pin.hpp"
#include <limits>
#include <string>
#include "can.h"

//**************************************************************************************************
// Gpio assigments
stmepic::GpioPin pin_user_led_1 = {GPIO_PIN_6, GPIOC,0};
stmepic::GpioPin pin_user_led_2 = {GPIO_PIN_7, GPIOC,0};
stmepic::GpioPin pin_user_btn_1 = {GPIO_PIN_9, GPIOA,0}; // GPIO_PIN_9, GPIOC for rev 1 of the board
stmepic::GpioPin pin_tx_led = {GPIO_PIN_12, GPIOB,0}; 
stmepic::GpioPin pin_rx_led = {GPIO_PIN_13, GPIOB,0};
stmepic::GpioPin pin_encoder = {GPIO_PIN_3, GPIOB,0};  
stmepic::GpioPin pin_poz_zero_sensor = {GPIO_PIN_4, GPIOA,0}; 
stmepic::GpioPin pin_inout_ca1 = {GPIO_PIN_5, GPIOA,0}; 
stmepic::GpioPin pin_inout_ca2 = {GPIO_PIN_7, GPIOA,0};
stmepic::GpioPin pin_inout_crx = {GPIO_PIN_4, GPIOC,0};
stmepic::GpioPin pin_inout_ctx = {GPIO_PIN_10, GPIOB,0};
stmepic::GpioPin pin_sync_sda = {GPIO_PIN_9, GPIOC,0}; 
stmepic::GpioPin pin_sync_scl = {GPIO_PIN_8, GPIOA,0};
stmepic::GpioPin pin_temp_steper_board = {GPIO_PIN_0, GPIOA,0};
stmepic::GpioPin pin_temp_board = {GPIO_PIN_1, GPIOA,0};
stmepic::GpioPin pin_temp_motor = {GPIO_PIN_2, GPIOA,0};
stmepic::GpioPin pin_vsense = {GPIO_PIN_3, GPIOA,0};
stmepic::GpioPin pin_steper_direction = {GPIO_PIN_0, GPIOB,0};
stmepic::GpioPin pin_steper_enable = {GPIO_PIN_1, GPIOB,0};
stmepic::GpioPin pin_steper_step = {GPIO_PIN_6, GPIOA,0};
stmepic::GpioPin pin_boot_device = {GPIO_PIN_8, GPIOC,0};
stmepic::GpioPin pin_cid_0 = {GPIO_PIN_10, GPIOC,0};
stmepic::GpioPin pin_cid_1 = {GPIO_PIN_11, GPIOC,0};
stmepic::GpioPin pin_cid_2 = {GPIO_PIN_12, GPIOC,0};


//**************************************************************************************************
// CONFIG GLOBAL
// ids 11bit 0b110 0001 0000  and 18 bit 0b00 0000 0000 0000 0000
//mask 11bit 0b111 1111 0000  and 18 bit 0b00 0000 0000 0000 0000
// for some reason filter mask is not working properly so we have to do it in software
// why 5 bits shift because we want tu push the 11 bit id to the 16 bit regiser strting from 5th bit


const IdConfig config_id_default ={
  0xff0,
  0x000,
  0x600,
  0x000,
  0x601,
  0x602,
  0x603,
  0x604,
  0x605,
  0x606,

  400.0f,
  40.0f,
  0.0f,
  0.0f,
  false,
  false,
  960,

  0.0f,
  false,
  0.0f,
  0,

  0.0f,
  false,
  0.0f,
  0,
  false,

  0.0f,
  0.0f,
  0.0f,

  0.0f,
  0.0f,
  0.0f,
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  0.7f
};

const IdConfig config_id_1 ={
  0xff0,
  0x000,
  0x610,
  0x000,

  CAN_KONARM_1_STATUS_FRAME_ID,
  CAN_KONARM_1_SET_POS_FRAME_ID,
  CAN_KONARM_1_GET_POS_FRAME_ID,
  CAN_KONARM_1_CLEAR_ERRORS_FRAME_ID,
  CAN_KONARM_1_GET_ERRORS_FRAME_ID,
  CAN_KONARM_1_SET_CONTROL_MODE_FRAME_ID,

  400.0f,
  40.0f,
  PI_m2,
  0.05f,
  false,
  false,
  960,

  -2.9145636558532715f,
  true,
  PI,
  0,

  0.0f,
  false,
  0.0f,
  40,
  true,
  
  0.9f,
  0.0f,
  0.1f,

  1.5f,
  -PI_m2,
  PI_m2,
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_2 = {
  0xff0,
  0x000,
  0x620,
  0x000,

  CAN_KONARM_2_STATUS_FRAME_ID,
  CAN_KONARM_2_SET_POS_FRAME_ID,
  CAN_KONARM_2_GET_POS_FRAME_ID,
  CAN_KONARM_2_CLEAR_ERRORS_FRAME_ID,
  CAN_KONARM_2_GET_ERRORS_FRAME_ID,
  CAN_KONARM_2_SET_CONTROL_MODE_FRAME_ID,

  400.0f,
  40.0f,
  PI_m2,
  0.05f,
  false,
  false,
  960,

  1.1063838005065918f,
  false,
  PI,
  40,

  0.0f,
  false,
  0.0f,
  0,
  true,
  
  0.9f,
  0.0f,
  0.1f,

  1.5f,
  -PI_d2,
  PI_d2,
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_3 = {
  0xff0,
  0x000,
  0x630,
  0x000,

  CAN_KONARM_3_STATUS_FRAME_ID,
  CAN_KONARM_3_SET_POS_FRAME_ID,
  CAN_KONARM_3_GET_POS_FRAME_ID,
  CAN_KONARM_3_CLEAR_ERRORS_FRAME_ID,
  CAN_KONARM_3_GET_ERRORS_FRAME_ID,
  CAN_KONARM_3_SET_CONTROL_MODE_FRAME_ID,

  400.0f,
  40.0f,
  PI_m2,
  0.05f,
  true,
  false,
  960,

  -1.6793255805969238f,
  true,
  PI,
  40,

  0.0f,
  false,
  -PI_d2,
  0,
  true,
  
  0.9f,
  0.0f,
  0.1f,

  1.5f,
  -1.1f,
  4.28f,
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_4 = {
  0xff0,
  0x000,
  0x640,
  0x000,

  CAN_KONARM_4_STATUS_FRAME_ID,
  CAN_KONARM_4_SET_POS_FRAME_ID,
  CAN_KONARM_4_GET_POS_FRAME_ID,
  CAN_KONARM_4_CLEAR_ERRORS_FRAME_ID,
  CAN_KONARM_4_GET_ERRORS_FRAME_ID,
  CAN_KONARM_4_SET_CONTROL_MODE_FRAME_ID,

  6400.0f,
  71.9f,
  PI,
  0.05f,
  false,
  true,
  32,

  0.645806f,
  true,
  0,
  0,

  0.0f,
  false,
  0.0f,
  10,
  false,

  0.9f,
  0.0f,
  0.1f,

  1.0f,
  -std::numeric_limits<float>::max(),
  std::numeric_limits<float>::max(),
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_5 = {
  0xff0,
  0x000,
  0x650,
  0x000,

  CAN_KONARM_5_STATUS_FRAME_ID,
  CAN_KONARM_5_SET_POS_FRAME_ID,
  CAN_KONARM_5_GET_POS_FRAME_ID,
  CAN_KONARM_5_CLEAR_ERRORS_FRAME_ID,
  CAN_KONARM_5_GET_ERRORS_FRAME_ID,
  CAN_KONARM_5_SET_CONTROL_MODE_FRAME_ID,

  6400.0f,
  71.9f,
  PI,
  0.05f,
  false,
  true,
  32,

  2.0513157844543457f,
  true,
  0,
  0,

  0.0f,
  false,
  0.0f,
  10,
  false,

  0.9f,
  0.0f,
  0.1f,

  1.0,
  -std::numeric_limits<float>::max(),
  std::numeric_limits<float>::max(),
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_6 = {
  0xff0,
  0x000,
  0x660,
  0x000,

  CAN_KONARM_6_STATUS_FRAME_ID,
  CAN_KONARM_6_SET_POS_FRAME_ID,
  CAN_KONARM_6_GET_POS_FRAME_ID,
  CAN_KONARM_6_CLEAR_ERRORS_FRAME_ID,
  CAN_KONARM_6_GET_ERRORS_FRAME_ID,
  CAN_KONARM_6_SET_CONTROL_MODE_FRAME_ID,

  6400.0f,
  71.9f,
  PI,
  0.05f,
  false,
  true,
  32,

  -2.034059f,
  true,
  0,
  0,

  0.0f,
  false,
  0.0f,
  10,
  false,

  0.9f,
  0.0f,
  0.1f,

  1.0f,
  -std::numeric_limits<float>::max(),
  std::numeric_limits<float>::max(),
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  1.5f
};



//**************************************************************************************************
// Global stuff

std::string version_string = std::to_string(VERSION_MAJOR)+"."+std::to_string(VERSION_MINOR) + "." + std::to_string(VERSION_BUILD);
uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE+1];
IdConfig config;
stmepic::Ticker main_clock;
stmepic::TimeScheduler task_timer_scheduler(main_clock);
stmepic::Logger loger(LOG_LOGER_LEVEL,LOG_SHOW_TIMESTAMP, version_string);
stmepic::Board_id board_id(pin_cid_0, pin_cid_1, pin_cid_2);

stmepic::SteperMotorStepDir stp_motor(htim3, TIM_CHANNEL_1, pin_steper_direction, pin_steper_enable);
stmepic::encoders::EncoderAbsoluteMagnetic encoder_arm;
stmepic::encoders::EncoderAbsoluteMagnetic encoder_vel_motor;
stmepic::MotorClosedLoop motor(stp_motor, &encoder_arm, &encoder_vel_motor);
stmepic::CanControl<> can_controler;
stmepic::MovementControler movement_controler;
stmepic::UsbProgramer usb_programer(pin_boot_device,loger);
stmepic::sensors::NTCTERMISTORS::NtcTermistors temp_steper_driver(UC_SUPPLY_VOLTAGE,TERMISTOR_RESISTANCE);
stmepic::sensors::NTCTERMISTORS::NtcTermistors temp_steper_motor(UC_SUPPLY_VOLTAGE,TERMISTOR_RESISTANCE);
ErrorData error_data;


unsigned int ErrorData::get_amount_of_errors() const {
  return (uint8_t)temp_engine_overheating + 
         (uint8_t)temp_driver_overheating + 
         (uint8_t)temp_board_overheating + 
         (uint8_t)temp_engine_sensor_disconnect + 
         (uint8_t)temp_driver_sensor_disconnect + 
         (uint8_t)temp_board_sensor_disconnect + 
         (uint8_t)encoder_arm_disconnect + 
         (uint8_t)encoder_motor_disconnect + 
         (uint8_t)baord_overvoltage +
         (uint8_t)baord_undervoltage + 
         (uint8_t)can_disconnected + 
         (uint8_t)can_error + 
         (uint8_t)controler_motor_limit_position;
}