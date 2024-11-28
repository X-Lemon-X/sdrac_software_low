
#ifndef CONFIG_PROG_H
#define CONFIG_PROG_H

// #include <cstddef>
// #include <cstdint>
// #include <cstdint>
#include <limits>
#include <random>
#include "logger.hpp"
#include "Timing.hpp"
#include "can.h"
#include "main.h"
#include "motor.hpp"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "pin.hpp"
#include "can_control.hpp"
#include "board_id.hpp"
#include "version.hpp"
#include "steper_motor.hpp"
#include "movement_controler.hpp"
#include "dfu_usb_programer.hpp"
#include "filter.hpp"
#include "filter_moving_avarage.hpp"
#include "filter_alfa_beta.hpp"
#include "ntc_termistors.hpp"

//**************************************************************************************************
// log levels 
#define LOG_DEBUG
// #define LOG_INFO
// #define LOG_WARN
// #define LOG_ERROR
#define LOG_SHOW_TIMESTAMP true

//**************************************************************************************************
// BOARD IDs  else if()

#define BOARD_ID_0 0x0
#define BOARD_ID_1 0x1
#define BOARD_ID_2 0x2
#define BOARD_ID_3 0x3
#define BOARD_ID_4 0x4
#define BOARD_ID_5 0x5
#define BOARD_ID_6 0x6
#define BOARD_ID_7 0x7

//**************************************************************************************************
// CONSTANTS
#define PI 3.14159265358979323846f
#define PI_d2 1.57079632679489661923f
#define PI_d4 0.78539816339744830962f
#define PI_m2 6.28318530717958647692f
#define PI_m3d2 4.71238898038468985769f

//**************************************************************************************************
// I2C CONSTANTS
#define ENCODER_MT6701_ANGLE_REG 0x03
#define ENCODER_MT6701_I2C_ADDRESS 0xC // 0x06 ds:b0000110  << 1 =  0b00001100 = 0xC
#define ENCODER_MT6701_I2C_ADDRESS_2 0x8C // 0x46 ds:b01000110  << 1 =  0b10001100 = 0x8C
#define ENCODER_MT6702_RESOLUTION 16384

#define ENCODER_AS5600_I2C_ADDRESS 0x6C // ds:0x36 << 1 = 0b0110 1100 = 0x6C 
#define ENCODER_AS5600_RESOLUTION 4096
#define ENCODER_AS5600_ANGLE_REG 0x0C

//**************************************************************************************************
// ADC CONSTANTS

#define ADC_DMA_BUFFER_SIZE 8
#define UC_SUPPLY_VOLTAGE 3.3f
#define ADC_VSENSE_MULTIPLIER 43.830646314f
#define TERMISTOR_RESISTANCE 100000.0f
#define ERRORS_MAX_VCC_VOLTAGE 49.0f
#define ERRORS_MIN_VCC_VOLTAGE 10.0f
#define ERRORS_MAX_TEMP_BOARD 45.0f
#define ERRORS_MAX_TEMP_DRIVER 50.0f
#define ERRORS_MAX_TEMP_ENGINE 100.0f

extern uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE+1];

//**************************************************************************************************
// TIMING CONSTANTS
// update frequency of the components

#define TIMING_LED_BLINK_FQ 2
#define TIMING_LED_ERROR_BLINK_FQ 1
#define TIMING_ENCODER_UPDATE_FQ 1000
#define TIMING_USB_RECIVED_DATA_FQ 5
#define TIMING_USB_SEND_DATA_FQ 100
#define TIMING_READ_TEMPERATURE_FQ 20
#define TIMING_CAN_DISCONNECTED_PERIOD 1000000



//**************************************************************************************************
// ID CONFIG

struct IdConfig {
// CAN 
uint32_t can_filter_mask_high;
uint32_t can_filter_mask_low;
uint32_t can_filter_id_high;
uint32_t can_filter_id_low;

uint32_t can_konarm_status_frame_id;
uint32_t can_konarm_set_pos_frame_id;
uint32_t can_konarm_get_pos_frame_id;
uint32_t can_konarm_clear_errors_frame_id;
uint32_t can_konarm_get_errors_frame_id;
uint32_t can_konarm_set_control_mode_frame_id;


// Steper motor config
float stepper_motor_steps_per_rev;
float stepper_motor_gear_ratio;
float stepper_motor_max_velocity;
float stepper_motor_min_velocity;
bool stepper_motor_reverse;
bool stepper_motor_enable_reversed;
uint32_t stepper_motor_timer_prescaler;

// Encoder pos arm
float encoder_arm_offset;
bool  encoder_arm_reverse;
float encoder_arm_dead_zone_correction_angle;
uint16_t encoder_arm_velocity_sample_amount;

// Encoder pos motor
float encoder_motor_offset;
bool  encoder_motor_reverse;
float encoder_motor_dead_zone_correction_angle;
uint16_t encoder_motor_velocity_sample_amount;
bool encoder_motor_enable;

// pid config
float pid_p;
float pid_i;
float pid_d;

//--------------------Movement config

/// @brief Maximum velocity of the arm
float movement_max_velocity;
/// @brief upper limit position of the arm
float movement_limit_lower;
/// @brief lower limit position of the arm
float movement_limit_upper;
uint8_t movement_control_mode;
float movement_max_acceleration;
};

/// @brief struct for the error data that represents the state of the system
/// Max amount of errors is equal to max length of the CAN frame so 8 x 8 = 64 (max)
/// The errors are represented as bool values
/// 0 - no error
/// 1 - error
/// you can retrieve the amount of errors by calling get_amount_of_errors()
class ErrorData
{
public:
  ErrorData(){};
  // temperature errors
  bool temp_engine_overheating = false;
  bool temp_driver_overheating = false;
  bool temp_board_overheating = false;
  bool temp_engine_sensor_disconnect = false;
  bool temp_driver_sensor_disconnect = false;
  bool temp_board_sensor_disconnect = false;

  // encoder errors
  bool encoder_arm_disconnect = false;
  bool encoder_motor_disconnect = false;

  // board errors
  bool baord_overvoltage = false;
  bool baord_undervoltage = false;

  // can errors
  bool can_disconnected = false;
  bool can_error = false;

  // other errors
  bool controler_motor_limit_position = false;

  /// @brief get the amount of errors
  /// @return the amount of errors
  unsigned int get_amount_of_errors() const;
};


extern IdConfig config;
extern const IdConfig config_id_default;
extern const IdConfig config_id_1;
extern const IdConfig config_id_2;
extern const IdConfig config_id_3;
extern const IdConfig config_id_4;
extern const IdConfig config_id_5;
extern const IdConfig config_id_6;


//**************************************************************************************************
// PINOUT
extern stmepic::GpioPin pin_user_led_1;
extern stmepic::GpioPin pin_user_led_2;
extern stmepic::GpioPin pin_user_btn_1;
extern stmepic::GpioPin pin_tx_led;
extern stmepic::GpioPin pin_rx_led;
extern stmepic::GpioPin pin_encoder;
extern stmepic::GpioPin pin_poz_zero_sensor;
extern stmepic::GpioPin pin_inout_ca1;
extern stmepic::GpioPin pin_inout_ca2;
extern stmepic::GpioPin pin_inout_crx;
extern stmepic::GpioPin pin_inout_ctx;
extern stmepic::GpioPin pin_sync_puls;
extern stmepic::GpioPin pin_sync_dir;
extern stmepic::GpioPin pin_temp_steper_board;
extern stmepic::GpioPin pin_temp_board;
extern stmepic::GpioPin pin_temp_motor;
extern stmepic::GpioPin pin_vsense;
extern stmepic::GpioPin pin_steper_direction;
extern stmepic::GpioPin pin_steper_enable ;
extern stmepic::GpioPin pin_steper_step ;
extern stmepic::GpioPin pin_boot_device;
extern stmepic::GpioPin pin_cid_0;
extern stmepic::GpioPin pin_cid_1;
extern stmepic::GpioPin pin_cid_2;

//**************************************************************************************************
// all the global variables, peripherals, and buffors are declared here


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

extern UART_HandleTypeDef huart3;

/// @brief USB handler for on board USB-C
extern USBD_HandleTypeDef hUsbDeviceFS;

//**************************************************************************************************
// GLOBAL OBEJCTS

extern std::string version_string;
extern stmepic::Logger loger;
extern stmepic::Ticker main_clock;
extern stmepic::TimeScheduler task_timer_scheduler;
extern stmepic::Board_id board_id;
extern stmepic::Encoder encoder_arm;
extern stmepic::Encoder encoder_motor;
extern stmepic::SteperMotorStepDir stp_motor;
extern stmepic::MotorBaseClosedLoop motor;
extern stmepic::CanControl<> can_controler;
extern stmepic::UsbProgramer usb_programer;
extern stmepic::MovementControler movement_controler;
extern stmepic::sensors::NTCTERMISTORS::NtcTermistors temp_steper_driver;
extern stmepic::sensors::NTCTERMISTORS::NtcTermistors temp_steper_motor;
extern ErrorData error_data;

//**************************************************************************************************
// debug loging options


// #define _LOG_LVL 4

#ifdef LOG_DEBUG
  #define _LOG_LVL 0
  #define LOG_LOGER_LEVEL stmepic::LOG_LEVEL::LOG_LEVEL_DEBUG
#endif // LOG_DEBUG


#ifdef LOG_INFO
  #define _LOG_LVL 1
  #define LOG_LOGER_LEVEL stmepic::LOG_LEVEL::LOG_LEVEL_INFO
#endif // LOG_INFO

#ifdef LOG_WARN
  #define _LOG_LVL 2
  #define LOG_LOGER_LEVEL stmepic::LOG_LEVEL::LOG_LEVEL_WARNING
#endif // LOG_WARN

#ifdef LOG_ERROR
  #define _LOG_LVL 3
  #define LOG_LOGER_LEVEL stmepic::LOG_LEVEL::LOG_LEVEL_ERROR
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