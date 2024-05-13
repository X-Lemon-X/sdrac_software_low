#include "logger.hpp"
#include "Timing.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "pin.hpp"
#include "can_control.hpp"
#include "board_id.hpp"

#ifndef MAIN_PROG_H
#define MAIN_PROG_H

//**************************************************************************************************
// log levels 
#define LOG_DEBUG
// #define LOG_INFO
// #define LOG_WARN
// #define LOG_ERROR


//**************************************************************************************************
// CONSTANTS
#define PI 3.14159265358979323846f
#define PI_d2 1.57079632679489661923f
#define PI_d4 0.78539816339744830962f
#define PI_m2 6.28318530717958647692f

//**************************************************************************************************
// I2C CONSTANTS
#define ENCODER_MEM_ADDR_ANNGLE 0x03
#define ENCODER_MT6701_I2C_ADDRESS 0xC //  0b0001100
#define ENCODER_MT6702_RESOLUTION 16384

//**************************************************************************************************
// CAN CONSTANTS
#define CAN_STRIP_BITS_FOR_MASK(id) ((id & 0xFF0) << 5)
extern uint32_t CAN_X_FILTER_MASK_LOW;
extern uint32_t CAN_X_FILTER_MASK_HIGH;
extern uint32_t CAN_X_FILTER_ID_LOW;
extern uint32_t CAN_X_FILTER_ID_HIGH;

extern uint32_t CAN_KONARM_X_CLEAR_ERRORS_FRAME_ID;
extern uint32_t CAN_KONARM_X_STATUS_FRAME_ID;
extern uint32_t CAN_KONARM_X_SET_POS_FRAME_ID;
extern uint32_t CAN_KONARM_X_GET_POS_FRAME_ID;
// extern uint32_t CAN_KONARM_X_GET_TEMP_FRAME_ID;

//**************************************************************************************************
// ID CONSTANTS

#define SDRAC_ID_0 0x00
#define SDRAC_ID_1 0x01
#define SDRAC_ID_2 0x02
#define SDRAC_ID_3 0x03
#define SDRAC_ID_4 0x04
#define SDRAC_ID_5 0x05
#define SDRAC_ID_6 0x06
#define SDRAC_ID_7 0x07

//**************************************************************************************************
// PINOUT CONSTANTS

extern GPIO_PIN pin_user_led_1;
extern GPIO_PIN pin_user_led_2;
extern GPIO_PIN pin_user_btn_1;
extern GPIO_PIN pin_tx_led;
extern GPIO_PIN pin_rx_led;
extern GPIO_PIN pin_encoder;
extern GPIO_PIN pin_poz_zero_sensor;
extern GPIO_PIN pin_inout_ca1;
extern GPIO_PIN pin_inout_ca2;
extern GPIO_PIN pin_inout_crx;
extern GPIO_PIN pin_inout_ctx;
extern GPIO_PIN pin_sync_puls;
extern GPIO_PIN pin_sync_dir;
extern GPIO_PIN pin_temp_steper_board;
extern GPIO_PIN pin_temp_board;
extern GPIO_PIN pin_temp_motor;
extern GPIO_PIN pin_vsense;
extern GPIO_PIN pin_steper_direction;
extern GPIO_PIN pin_steper_enable ;
extern GPIO_PIN pin_steper_step ;
extern GPIO_PIN pin_boot_device;

//to do
extern GPIO_PIN pin_cid_0;
extern GPIO_PIN pin_cid_1;
extern GPIO_PIN pin_cid_2;


//**************************************************************************************************
// all the global variables, peripherals, and buffors are declared here

#define ADC_DMA_BUFFER_SIZE 16
extern uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart3;
extern USBD_HandleTypeDef hUsbDeviceFS;

//-----------------------------------

extern LOGGER::Logger loger;
extern TIMING::Ticker main_clock;
extern CAN_CONTROL::CanControl can_controler;
extern BOARD_ID::Board_id board_id;


//**************************************************************************************************
/// @brief main program, this function is called from main and never returns
void main_prog();


//**************************************************************************************************
// debug loging options

#ifdef LOG_DEBUG
  #define _LOG_LVL 0
#endif // LOG_DEBUG

#ifdef LOG_INFO
  #define _LOG_LVL 1
#endif // LOG_INFO

#ifdef LOG_WARN
  #define _LOG_LVL 2
#endif // LOG_WARN

#ifdef LOG_ERROR
  #define _LOG_LVL 3
#endif // LOG_ERROR


#if _LOG_LVL <= 0
  #define log_debug(...) loger.debug(__VA_ARGS__)
#else
  #define log_debug(...)
#endif // LOG_DEBUG

#if _LOG_LVL <= 1
  #define log_info(...) loger.info(__VA_ARGS__)
#else
  #define log_info(...)
#endif // LOG_INFO

#if _LOG_LVL <= 2
  #define log_warn(...) loger.warn(__VA_ARGS__)
#else
  #define log_warn(...)
#endif // LOG_WARN

#if _LOG_LVL <= 3
  #define log_error(...) loger.error(__VA_ARGS__)
#else
  #define log_error(...)
#endif // LOG_ERROR



#endif // MAIN_PROG_H