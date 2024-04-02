#include "logger.hpp"
#include "Timing.hpp"

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

//**************************************************************************************************
// I2C CONSTANTS
#define ENCODER_MEM_ADDR_ANNGLE 0x03
#define ENCODER_MT6701_I2C_ADDRESS 0b0000110 << 1
#define ENCODER_MT6702_RESOLUTION 16384

//**************************************************************************************************
// CAN CONSTANTS


//**************************************************************************************************
// all the global variables and peripherals are declared here
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart3;
extern LOGGER::Logger loger;
extern TIMING::Ticker ticker;
int main_prog();






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