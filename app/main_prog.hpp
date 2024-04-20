#include "logger.hpp"
#include "Timing.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

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
#define PI 3.14159265358979323846
#define PI_d2 1.57079632679489661923
#define PI_d4 0.78539816339744830962
#define PI_m2 6.28318530717958647692


#define ADC_DMA_BUFFER_SIZE 16

//**************************************************************************************************
// I2C CONSTANTS
#define ENCODER_MEM_ADDR_ANNGLE 0x03
#define ENCODER_MT6701_I2C_ADDRESS 0b0000110 << 1
#define ENCODER_MT6702_RESOLUTION 16384

//**************************************************************************************************
// CAN CONSTANTS


//**************************************************************************************************
// PINOUT CONSTANTS

struct Pin {
  uint16_t pin;
  GPIO_TypeDef *port;
};

extern Pin pin_user_led_1;
extern Pin pin_user_led_2;
extern Pin pin_user_btn_1;
extern Pin pin_tx_led;
extern Pin pin_rx_led;
extern Pin pin_encoder;
extern Pin pin_poz_zero_sensor;
extern Pin pin_inout_ca1;
extern Pin pin_inout_ca2;
extern Pin pin_inout_crx;
extern Pin pin_inout_ctx;
extern Pin pin_sync_puls;
extern Pin pin_sync_dir;
extern Pin pin_temp_steper_board;
extern Pin pin_temp_board;
extern Pin pin_temp_motor;
extern Pin pin_vsense;
extern Pin pin_steper_direction;
extern Pin pin_steper_enable ;
extern Pin pin_steper_step ;
extern Pin pin_boot_device;


//**************************************************************************************************
// all the global variables, peripherals, and buffors are declared here

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

extern LOGGER::Logger loger;
extern TIMING::Ticker ticker;



int main_prog();



// struct Pin{
//   uint16_t pin;
//   GPIO_TypeDef *port;
// };




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