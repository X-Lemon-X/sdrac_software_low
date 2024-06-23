
#ifndef MAIN_PROG_H
#define MAIN_PROG_H

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
#include "version.hpp"
#include "config_struct.hpp"

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
#define PI_m3d2 4.71238898038468985769f

//**************************************************************************************************
// I2C CONSTANTS
#define ENCODER_MEM_ADDR_ANNGLE 0x03
#define ENCODER_MT6701_I2C_ADDRESS 0xC //  0b0001100
#define ENCODER_MT6702_RESOLUTION 16384

#define ENCODER_AS5600_I2C_ADDRESS 0x36 // 0b0110110
#define ENCODER_AS5600_RESOLUTION 4096
#define ENCODER_AS5600_ANGLE_REG 0x0C

//**************************************************************************************************
// ADC CONSTANTS

#define UC_SUPPLY_VOLTAGE 3.3f
#define ADC_VSENSE_MULTIPLIER 43.830646314f
#define TERMISTOR_RESISTANCE 100000.0f

//**************************************************************************************************
// ID CONFIG
extern ID_CONFIG config;

//**************************************************************************************************
// PINOUT
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
extern GPIO_PIN pin_cid_0;
extern GPIO_PIN pin_cid_1;
extern GPIO_PIN pin_cid_2;

// GPIO_PIN pin_user_led_1 = {GPIO_PIN_6, GPIOC,0};
// GPIO_PIN pin_user_led_2 = {GPIO_PIN_7, GPIOC,0};
// GPIO_PIN pin_user_btn_1 = {GPIO_PIN_9, GPIOA,0}; // GPIO_PIN_9, GPIOC for rev 1 of the board
// GPIO_PIN pin_tx_led = {GPIO_PIN_12, GPIOB,0}; 
// GPIO_PIN pin_rx_led = {GPIO_PIN_13, GPIOB,0};
// GPIO_PIN pin_encoder = {GPIO_PIN_3, GPIOB,0};  
// GPIO_PIN pin_poz_zero_sensor = {GPIO_PIN_4, GPIOA,0}; 
// GPIO_PIN pin_inout_ca1 = {GPIO_PIN_5, GPIOA,0}; 
// GPIO_PIN pin_inout_ca2 = {GPIO_PIN_7, GPIOA,0};
// GPIO_PIN pin_inout_crx = {GPIO_PIN_4, GPIOC,0};
// GPIO_PIN pin_inout_ctx = {GPIO_PIN_10, GPIOB,0};
// GPIO_PIN pin_sync_sda = {GPIO_PIN_9, GPIOC,0}; 
// GPIO_PIN pin_sync_scl = {GPIO_PIN_8, GPIOA,0};
// GPIO_PIN pin_temp_steper_board = {GPIO_PIN_0, GPIOA,0};
// GPIO_PIN pin_temp_board = {GPIO_PIN_1, GPIOA,0};
// GPIO_PIN pin_temp_motor = {GPIO_PIN_2, GPIOA,0};
// GPIO_PIN pin_vsense = {GPIO_PIN_3, GPIOA,0};
// GPIO_PIN pin_steper_direction = {GPIO_PIN_0, GPIOB,0};
// GPIO_PIN pin_steper_enable = {GPIO_PIN_1, GPIOB,0};
// GPIO_PIN pin_steper_step = {GPIO_PIN_6, GPIOA,0};
// GPIO_PIN pin_boot_device = {GPIO_PIN_8, GPIOC,0};
// GPIO_PIN pin_cid_0 = {GPIO_PIN_10, GPIOC,0};
// GPIO_PIN pin_cid_1 = {GPIO_PIN_11, GPIOC,0};
// GPIO_PIN pin_cid_2 = {GPIO_PIN_12, GPIOC,0};


//**************************************************************************************************
// all the global variables, peripherals, and buffors are declared here

#define ADC_DMA_BUFFER_SIZE 8
extern uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE+1];

/// @brief ADC handler
extern ADC_HandleTypeDef hadc1;

/// @brief  DMA handler
extern DMA_HandleTypeDef hdma_adc1;

/// @brief CAN handler for the can comuncatin between baords
extern CAN_HandleTypeDef hcan1;

/// @brief I2C handler for the encoder1, encoder2
extern I2C_HandleTypeDef hi2c1;

/// @brief I2C handler for the syncronisation between the boards
extern I2C_HandleTypeDef hi2c3;

/// @brief TIM handler for the main clock
extern TIM_HandleTypeDef htim1;

/// @brief TIM handler for the encoder1, encoder2
extern TIM_HandleTypeDef htim2;

/// @brief TIM handler for the stepper motor control
extern TIM_HandleTypeDef htim3;

/// @brief TIM handler for the IO use
extern TIM_HandleTypeDef htim8;

/// @brief TIM handler for the [us] precision clock
extern TIM_HandleTypeDef htim10;

/// @brief UART handler for the IO use
extern UART_HandleTypeDef huart3;

/// @brief USB handler for on board USB-C
extern USBD_HandleTypeDef hUsbDeviceFS;

//-----------------------------------

extern LOGGER::Logger loger;
extern TIMING::Ticker main_clock;
extern CAN_CONTROL::CanControl can_controler;
extern BOARD_ID::Board_id board_id;


//**************************************************************************************************
/// @brief main program, this function is called from main and never returns
void main_prog();


/// @brief This function is used to configure things that have to be configurated before all the periferals
void pre_periferal_config();

/// @brief This function is used to configure the periferals
/// mostly stuff that have to be configurated after CumeMX generation
void periferal_config();

/// @brief This function is used to handle the can controll
void handle_can_rx();

/// @brief This function is used to handle the main loop, never returns
void main_loop();

/// @brief This function is used to configure the board base on it's hardware id
void id_config();

void post_id_config();

/// @brief This function is used to init the interfaces
void init_controls();

//**************************************************************************************************
// debug loging options


#define _LOG_LVL 4

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