#include "config.hpp"
#include "steper_motor.hpp"
#include "movement_controler.hpp"
#include "usb_programer.hpp"
#include "pid_controler.hpp"
#include "filter.hpp"
#include "filter_moving_avarage.hpp"
#include "filter_alfa_beta.hpp"
#include "ntc_termistors.hpp"
#include "pin.hpp"
#include <limits>

//**************************************************************************************************
// Gpio assigments
GpioPin pin_user_led_1 = {GPIO_PIN_6, GPIOC,0};
GpioPin pin_user_led_2 = {GPIO_PIN_7, GPIOC,0};
GpioPin pin_user_btn_1 = {GPIO_PIN_9, GPIOA,0}; // GPIO_PIN_9, GPIOC for rev 1 of the board
GpioPin pin_tx_led = {GPIO_PIN_12, GPIOB,0}; 
GpioPin pin_rx_led = {GPIO_PIN_13, GPIOB,0};
GpioPin pin_encoder = {GPIO_PIN_3, GPIOB,0};  
GpioPin pin_poz_zero_sensor = {GPIO_PIN_4, GPIOA,0}; 
GpioPin pin_inout_ca1 = {GPIO_PIN_5, GPIOA,0}; 
GpioPin pin_inout_ca2 = {GPIO_PIN_7, GPIOA,0};
GpioPin pin_inout_crx = {GPIO_PIN_4, GPIOC,0};
GpioPin pin_inout_ctx = {GPIO_PIN_10, GPIOB,0};
GpioPin pin_sync_sda = {GPIO_PIN_9, GPIOC,0}; 
GpioPin pin_sync_scl = {GPIO_PIN_8, GPIOA,0};
GpioPin pin_temp_steper_board = {GPIO_PIN_0, GPIOA,0};
GpioPin pin_temp_board = {GPIO_PIN_1, GPIOA,0};
GpioPin pin_temp_motor = {GPIO_PIN_2, GPIOA,0};
GpioPin pin_vsense = {GPIO_PIN_3, GPIOA,0};
GpioPin pin_steper_direction = {GPIO_PIN_0, GPIOB,0};
GpioPin pin_steper_enable = {GPIO_PIN_1, GPIOB,0};
GpioPin pin_steper_step = {GPIO_PIN_6, GPIOA,0};
GpioPin pin_boot_device = {GPIO_PIN_8, GPIOC,0};
GpioPin pin_cid_0 = {GPIO_PIN_10, GPIOC,0};
GpioPin pin_cid_1 = {GPIO_PIN_11, GPIOC,0};
GpioPin pin_cid_2 = {GPIO_PIN_12, GPIOC,0};


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

  400.0f,
  40.0f,
  0.0f,
  0.0f,
  false,

  0.0f,
  false,
  0.0f,
  0,

  0.0f,
  false,
  0.0f,
  0,

  0.0f,
  0.0f,
  0.0f,

  0.0f,
  0.0f,
  0.0f,
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

  400.0f,
  40.0f,
  PI,
  0.01f,
  false,

  -5.23547649f,
  true,
  0.0f,
  0,

  0.0f,
  false,
  0.0f,
  10,

  0.9f,
  0.0f,
  0.1f,

  PI,
  -PI,
  PI,
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

  400.0f,
  40.0f,
  2.0f,
  0.01f,
  true,

  -2.617355f,
  false,
  PI,
  10,

  0.0f,
  false,
  0.0f,
  10,

  0.9f,
  0.0f,
  0.1f,

  PI,
  -std::numeric_limits<float>::max(),
  std::numeric_limits<float>::max(),
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

  400.0f,
  40.0f,
  2.0f,
  0.01f,
  false,

  -2.617355f,
  false,
  PI,
  10,

  0.0f,
  false,
  0.0f,
  10,

  0.9f,
  0.0f,
  0.1f,

  PI,
  -1.089126f,
  4.236856f,
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

  400.0f,
  71.9f,
  PI,
  0.01f,
  false,

  0.0f,
  true,
  PI_m3d2,
  0,

  0.0f,
  false,
  0.0f,
  10,

  0.9f,
  0.0f,
  0.1f,

  PI,
  -std::numeric_limits<float>::max(),
  std::numeric_limits<float>::max(),
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

  400.0f,
  71.9f,
  PI,
  0.01f,
  false,

  0.0f,
  true,
  PI_m3d2,
  0,

  0.0f,
  false,
  0.0f,
  10,

  0.9f,
  0.0f,
  0.1f,

  PI,
  -std::numeric_limits<float>::max(),
  std::numeric_limits<float>::max(),
};

const IdConfig config_id_6 = {
  0xff0,
  0x000,
  0x660,
  0x000,

  CAN_KONARM_5_STATUS_FRAME_ID,
  CAN_KONARM_5_SET_POS_FRAME_ID,
  CAN_KONARM_5_GET_POS_FRAME_ID,
  CAN_KONARM_5_CLEAR_ERRORS_FRAME_ID,

  400.0f,
  71.9f,
  PI,
  0.01f,
  false,

  0.0f,
  true,
  PI_m3d2,
  0,

  0.0f,
  false,
  0.0f,
  10,

  0.9f,
  0.0f,
  0.1f,

  PI,
  -std::numeric_limits<float>::max(),
  std::numeric_limits<float>::max(),
};



//**************************************************************************************************
// Global stuff

uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE+1];
IdConfig config;
TIMING::Ticker main_clock;
LOGGER::Logger loger(LOG_LOGER_LEVEL,LOG_SHOW_TIMESTAMP);
BOARD_ID::Board_id board_id(pin_cid_0, pin_cid_1, pin_cid_2);

STEPER_MOTOR::SteperMotor stp_motor(htim3, TIM_CHANNEL_1, pin_steper_direction, pin_steper_enable);
CAN_CONTROL::CanControl can_controler;
MOVEMENT_CONTROLER::MovementControler movement_controler;
ENCODER::Encoder encoder_arm;
ENCODER::Encoder encoder_motor;
USB_PROGRAMER::UsbProgramer usb_programer(pin_boot_device);
NTCTERMISTORS::NtcTermistors temp_steper_driver(UC_SUPPLY_VOLTAGE,TERMISTOR_RESISTANCE);
NTCTERMISTORS::NtcTermistors temp_steper_motor(UC_SUPPLY_VOLTAGE,TERMISTOR_RESISTANCE);
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
         (uint8_t)can_disconnect + 
         (uint8_t)can_error + 
         (uint8_t)controler_motor_limit_position;
}