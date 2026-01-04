#include "config.hpp"
#include "can_messages.h"
#include "controler_pid.hpp"
#include "dfu_usb_programer.hpp"
#include "encoder_magnetic.hpp"
#include "filter.hpp"
#include "filter_alfa_beta.hpp"
#include "filter_moving_avarage.hpp"
#include "fram_i2c.hpp"
#include "gpio.hpp"
#include "memory_fram.hpp"
#include "motor.hpp"
#include "movement_controler.hpp"
#include "ntc_termistor.hpp"
#include "steper_motor.hpp"
#include "usbd_cdc_if.h"
#include <limits>
#include <string>
#include "sdrac_shared_types.hpp"

//**************************************************************************************************
// Gpio assigments
se::GpioPin pin_user_led_1(*GPIOC, GPIO_PIN_6);
se::GpioPin pin_user_led_2(*GPIOC, GPIO_PIN_7);
se::GpioPin pin_user_btn_1(*GPIOA, GPIO_PIN_9); // GPIOC, GPIO_PIN_9 for rev 1 of the board
se::GpioPin pin_tx_led(*GPIOB, GPIO_PIN_12);
se::GpioPin pin_rx_led(*GPIOB, GPIO_PIN_13);
se::GpioPin pin_encoder(*GPIOB, GPIO_PIN_3);
se::GpioPin pin_poz_zero_sensor(*GPIOA, GPIO_PIN_4);
se::GpioPin pin_inout_ca1(*GPIOA, GPIO_PIN_5);
se::GpioPin pin_inout_ca2(*GPIOA, GPIO_PIN_7);
se::GpioPin pin_inout_crx(*GPIOC, GPIO_PIN_4);
se::GpioPin pin_inout_ctx(*GPIOB, GPIO_PIN_10);
se::GpioPin pin_i2c1_sda(*GPIOB, GPIO_PIN_7);
se::GpioPin pin_i2c1_scl(*GPIOB, GPIO_PIN_6);
se::GpioPin pin_i2c3_sda(*GPIOC, GPIO_PIN_9);
se::GpioPin pin_i2c3_scl(*GPIOA, GPIO_PIN_8);
se::GpioAnalog pin_temp_steper_board(*GPIOA, GPIO_PIN_0);
se::GpioAnalog pin_temp_board(*GPIOA, GPIO_PIN_1);
se::GpioAnalog pin_temp_motor(*GPIOA, GPIO_PIN_2);
se::GpioAnalog pin_vsense(*GPIOA, GPIO_PIN_3);
se::GpioPin pin_steper_direction(*GPIOB, GPIO_PIN_0);
se::GpioPin pin_steper_enable(*GPIOB, GPIO_PIN_1);
se::GpioPin pin_steper_step(*GPIOA, GPIO_PIN_6);
se::GpioPin pin_boot_device(*GPIOC, GPIO_PIN_8);
se::GpioPin pin_cid_0(*GPIOC, GPIO_PIN_10);
se::GpioPin pin_cid_1(*GPIOC, GPIO_PIN_11);
se::GpioPin pin_cid_2(*GPIOC, GPIO_PIN_12);


//**************************************************************************************************
// CONFIG GLOBAL
// ids 11bit 0b110 0001 0000  and 18 bit 0b00 0000 0000 0000 0000
// mask 11bit 0b111 1111 0000  and 18 bit 0b00 0000 0000 0000 0000
// for some reason filter mask is not working properly so we have to do it in
// software why 5 bits shift because we want tu push the 11 bit id to the 16 bit
// regiser strting from 5th bit

const IdConfig config_id_default = { 0xff0, 0x000, 0x600, 0x000, 0x601, 0x602,
                                     0x603, 0x604, 0x605, 0x606, 0x60a };

const ModuleConfig config_default = {

  400.0f, 40.0f, 0.0f, 0.0f,
  false,  false, 960,

  0.0f,   false, 0.0f, 0,

  0.0f,   false, 0.0f, 0,
  false,

  0.0f,   0.0f,  0.0f,

  0.0f,   0.0f,  0.0f, CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  0.7f
};

const IdConfig config_id_1  = { 0xff0,
                                0x000,
                                0x610,
                                0x000,

                                CAN_KONARM_1_STATUS_FRAME_ID,
                                CAN_KONARM_1_SET_POS_FRAME_ID,
                                CAN_KONARM_1_GET_POS_FRAME_ID,
                                CAN_KONARM_1_CLEAR_ERRORS_FRAME_ID,
                                CAN_KONARM_1_GET_ERRORS_FRAME_ID,
                                CAN_KONARM_1_SET_CONTROL_MODE_FRAME_ID,
                                CAN_KONARM_1_SET_EFFECTOR_POSITION_FRAME_ID };
const ModuleConfig config_1 = { 400.0f,
                                40.0f,
                                PI_m2,
                                0.05f,
                                false,
                                false,
                                960,

                                -1.6655197143554688f,
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
                                CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE,
                                1.5f };

const IdConfig config_id_2 = { 0xff0,
                               0x000,
                               0x620,
                               0x000,

                               CAN_KONARM_2_STATUS_FRAME_ID,
                               CAN_KONARM_2_SET_POS_FRAME_ID,
                               CAN_KONARM_2_GET_POS_FRAME_ID,
                               CAN_KONARM_2_CLEAR_ERRORS_FRAME_ID,
                               CAN_KONARM_2_GET_ERRORS_FRAME_ID,
                               CAN_KONARM_2_SET_CONTROL_MODE_FRAME_ID,
                               CAN_KONARM_2_SET_EFFECTOR_POSITION_FRAME_ID };

const ModuleConfig config_2 = {

  400.0f,    68.0f,  PI_m2, 0.05f,
  false,     false,  960,

  2.738156f, // 1.617966f
  false,     PI,     40,

  0.0f,      false,  0.0f,  0,
  true,

  0.9f,      0.0f,   0.1f,

  1.5f,      -PI_d2, PI_d2, CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_3  = { 0xff0,
                                0x000,
                                0x630,
                                0x000,

                                CAN_KONARM_3_STATUS_FRAME_ID,
                                CAN_KONARM_3_SET_POS_FRAME_ID,
                                CAN_KONARM_3_GET_POS_FRAME_ID,
                                CAN_KONARM_3_CLEAR_ERRORS_FRAME_ID,
                                CAN_KONARM_3_GET_ERRORS_FRAME_ID,
                                CAN_KONARM_3_SET_CONTROL_MODE_FRAME_ID,
                                CAN_KONARM_3_SET_EFFECTOR_POSITION_FRAME_ID };
const ModuleConfig config_3 = {

  400.0f,
  40.0f,
  PI_m2,
  0.05f,
  true,
  false,
  960,

  -1.711155414581298f,
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
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_4  = { 0xff0,
                                0x000,
                                0x640,
                                0x000,

                                CAN_KONARM_4_STATUS_FRAME_ID,
                                CAN_KONARM_4_SET_POS_FRAME_ID,
                                CAN_KONARM_4_GET_POS_FRAME_ID,
                                CAN_KONARM_4_CLEAR_ERRORS_FRAME_ID,
                                CAN_KONARM_4_GET_ERRORS_FRAME_ID,
                                CAN_KONARM_4_SET_CONTROL_MODE_FRAME_ID,
                                CAN_KONARM_4_SET_EFFECTOR_POSITION_FRAME_ID };
const ModuleConfig config_4 = {

  6400.0f,
  71.9f,
  PI,
  0.05f,
  false,
  true,
  32,

  0.709466f,
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
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_5 = { 0xff0,
                               0x000,
                               0x650,
                               0x000,

                               CAN_KONARM_5_STATUS_FRAME_ID,
                               CAN_KONARM_5_SET_POS_FRAME_ID,
                               CAN_KONARM_5_GET_POS_FRAME_ID,
                               CAN_KONARM_5_CLEAR_ERRORS_FRAME_ID,
                               CAN_KONARM_5_GET_ERRORS_FRAME_ID,
                               CAN_KONARM_5_SET_CONTROL_MODE_FRAME_ID,
                               CAN_KONARM_5_SET_EFFECTOR_POSITION_FRAME_ID };

const ModuleConfig config_5 = {

  6400.0f,
  71.9f,
  PI,
  0.05f,
  true,
  true,
  32,

  2.026005f,
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
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE,
  1.5f
};

const IdConfig config_id_6  = { 0xff0,
                                0x000,
                                0x660,
                                0x000,

                                CAN_KONARM_6_STATUS_FRAME_ID,
                                CAN_KONARM_6_SET_POS_FRAME_ID,
                                CAN_KONARM_6_GET_POS_FRAME_ID,
                                CAN_KONARM_6_CLEAR_ERRORS_FRAME_ID,
                                CAN_KONARM_6_GET_ERRORS_FRAME_ID,
                                CAN_KONARM_6_SET_CONTROL_MODE_FRAME_ID,
                                CAN_KONARM_6_SET_EFFECTOR_POSITION_FRAME_ID };
const ModuleConfig config_6 = {

  6400.0f,
  71.9f,
  PI,
  0.05f,
  true,
  true,
  32,

  2.883117f,
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
  CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE,
  1.5f
};

//**************************************************************************************************
// Global stuff


std::shared_ptr<se::I2C> i2c1;
std::shared_ptr<se::I2C> i2c3;
std::shared_ptr<se::CAN> can1;

std::string version_string =
std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) + "." + std::to_string(VERSION_BUILD);
uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE + 1];
IdConfig config;
ModuleConfig module_config;
// se::Ticker &main_clock = se::GlobalTicker::get_instance();
// se::Board_id board_id(pin_cid_0, pin_cid_1, pin_cid_2);

se::motor::SteperMotorStepDir stp_motor(htim3, TIM_CHANNEL_1, pin_steper_direction, pin_steper_enable);
std::shared_ptr<se::memory::FRAM> fram;
std::shared_ptr<se::encoders::EncoderAbsoluteMagneticMT6701> encoder_arm;
std::shared_ptr<se::encoders::EncoderAbsoluteMagneticMT6701> encoder_vel_motor;
std::shared_ptr<se::motor::MotorClosedLoop> motor;
std::shared_ptr<se::motor::ServoMotorPWM> servo_motor;
se::movement::MovementControler movement_controler;
se::dfu::UsbProgramer usb_programer(pin_boot_device);
se::sensors::temperature::NtcTermistor temp_steper_driver(UC_SUPPLY_VOLTAGE, TERMISTOR_RESISTANCE);
se::sensors::temperature::NtcTermistor temp_steper_motor(UC_SUPPLY_VOLTAGE, TERMISTOR_RESISTANCE);
ErrorData error_data;
