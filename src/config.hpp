
#ifndef CONFIG_PROG_H
#define CONFIG_PROG_H

// #include <cstddef>
// #include <cstdint>
// #include <cstdint>
#include "Timing.hpp"
#include "can_messages.h"
#include "can.hpp"
// #include "can_control.hpp"
#include "dfu_usb_programer.hpp"
#include "encoder_magnetic.hpp"
#include "filter.hpp"
#include "filter_alfa_beta.hpp"
#include "filter_moving_avarage.hpp"
#include "memory_fram.hpp"
#include "fram_i2c.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "main.h"
#include "motor.hpp"
#include "movement_controler.hpp"
#include "servo_motor.hpp"
#include "ntc_termistor.hpp"
#include "steper_motor.hpp"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "version.hpp"
#include "can2.0.hpp"
#include "MT6701.hpp"
#include <limits>
#include <memory>
#include <random>
namespace se = stmepic;

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

// Memory Fram constants
#define FRAM_SIZE 16000
#define FRAM_BEGIN_ADDRESS 0x10
#define FRAM_CONFIG_ADDRESS 0x01

#define FRAM_CONFIG_ADDRESS 0x01

//**************************************************************************************************
// I2C CONSTANTS
// #define ENCODER_MT6701_ANGLE_REG 0x03
// #define ENCODER_MT6701_I2C_ADDRESS 0x06   // 0x06 ds:b0000110  << 1 =  0b00001100 = 0xC
// #define ENCODER_MT6701_I2C_ADDRESS_2 0x46 // 0x46 ds:b01000110  << 1 =  0b10001100 =
// 0x8C #define ENCODER_MT6702_RESOLUTION 16384

// #define ENCODER_AS5600_I2C_ADDRESS 0x36 // ds:0x36 << 1 = 0b0110 1100 = 0x6C
// #define ENCODER_AS5600_RESOLUTION 4096
// #define ENCODER_AS5600_ANGLE_REG 0x0C

//**************************************************************************************************
// ADC CONSTANTS

#define ADC_DMA_BUFFER_SIZE 6
#define UC_SUPPLY_VOLTAGE 3.3f
#define ADC_VSENSE_MULTIPLIER 43.830646314f
#define TERMISTOR_RESISTANCE 100000.0f
#define ERRORS_MAX_VCC_VOLTAGE 49.0f
#define ERRORS_MIN_VCC_VOLTAGE 10.0f
#define ERRORS_MAX_TEMP_BOARD 45.0f
#define ERRORS_MAX_TEMP_DRIVER 50.0f
#define ERRORS_MAX_TEMP_ENGINE 100.0f

extern uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE + 1];

//**************************************************************************************************
// TIMING CONSTANTS
// update frequency of the components

#define TIMING_LED_BLINK_FQ 2
#define TIMING_LED_ERROR_BLINK_FQ 1
#define TIMING_USB_RECIVED_DATA_FQ 5
#define TIMING_USB_SEND_DATA_FQ 10
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
  uint32_t can_konarm_set_effector_position_frame_id;


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
  bool encoder_arm_reverse;
  float encoder_arm_dead_zone_correction_angle;
  uint16_t encoder_arm_velocity_sample_amount;

  // Encoder pos motor
  float encoder_motor_offset;
  bool encoder_motor_reverse;
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
class ErrorData {
public:
  ErrorData(){};
  // temperature errors
  bool temp_engine_overheating       = false;
  bool temp_driver_overheating       = false;
  bool temp_board_overheating        = false;
  bool temp_engine_sensor_disconnect = false;
  bool temp_driver_sensor_disconnect = false;
  bool temp_board_sensor_disconnect  = false;

  // encoder errors
  bool encoder_arm_disconnect   = false;
  bool encoder_motor_disconnect = false;

  // board errors
  bool baord_overvoltage  = false;
  bool baord_undervoltage = false;

  // can errors
  bool can_disconnected = false;
  bool can_error        = false;

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
extern se::GpioPin pin_user_led_1;
extern se::GpioPin pin_user_led_2;
extern se::GpioPin pin_user_btn_1;
extern se::GpioPin pin_tx_led;
extern se::GpioPin pin_rx_led;
extern se::GpioPin pin_encoder;
extern se::GpioPin pin_poz_zero_sensor;
extern se::GpioPin pin_inout_ca1;
extern se::GpioPin pin_inout_ca2;
extern se::GpioPin pin_inout_crx;
extern se::GpioPin pin_inout_ctx;
extern se::GpioPin pin_i2c3_sda;
extern se::GpioPin pin_i2c3_scl;
extern se::GpioAnalog pin_temp_steper_board;
extern se::GpioAnalog pin_temp_board;
extern se::GpioAnalog pin_temp_motor;
extern se::GpioAnalog pin_vsense;
extern se::GpioPin pin_steper_direction;
extern se::GpioPin pin_steper_enable;
extern se::GpioPin pin_steper_step;
extern se::GpioPin pin_boot_device;
extern se::GpioPin pin_cid_0;
extern se::GpioPin pin_cid_1;
extern se::GpioPin pin_cid_2;

extern se::GpioPin pin_i2c1_sda;
extern se::GpioPin pin_i2c1_scl;

//**************************************************************************************************
// all the global variables, peripherals, and buffors are declared here


/// @brief ADC handler
extern ADC_HandleTypeDef hadc1;

/// @brief  DMA handler
extern DMA_HandleTypeDef hdma_adc1;

/// @brief CAN handler for the can comuncatin between baords
extern CAN_HandleTypeDef hcan1;

/// @brief I2C handler for the encoder1, encoder2, fram
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

/// @brief TIM handler for the IO use
extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart3;


/// @brief USB handler for on board USB-C
extern USBD_HandleTypeDef hUsbDeviceFS;

//**************************************************************************************************
// GLOBAL OBEJCTS

extern std::string version_string;
// extern se::Ticker &main_clock;
// extern se::TimeScheduler task_timer_scheduler;
// extern se::Board_id board_id;
extern std::shared_ptr<se::encoders::EncoderAbsoluteMagneticMT6701> encoder_arm;
extern std::shared_ptr<se::encoders::EncoderAbsoluteMagneticMT6701> encoder_vel_motor;
extern se::motor::SteperMotorStepDir stp_motor;
extern std::shared_ptr<se::memory::FRAM> fram;
extern std::shared_ptr<se::motor::MotorClosedLoop> motor;
extern std::shared_ptr<se::motor::ServoMotorPWM> servo_motor;
// extern se::CanControl<> can_controler;
extern se::dfu::UsbProgramer usb_programer;
extern se::movement::MovementControler movement_controler;
extern se::sensors::temperature::NtcTermistor temp_steper_driver;
extern se::sensors::temperature::NtcTermistor temp_steper_motor;
extern ErrorData error_data;

extern std::shared_ptr<se::I2C> i2c1;
extern std::shared_ptr<se::I2C> i2c3;
extern std::shared_ptr<se::CAN> can1;
//**************************************************************************************************
// debug loging options


// #define _LOG_LVL 4

#ifdef LOG_DEBUG
#define _LOG_LVL 0
#define LOG_LOGER_LEVEL se::LOG_LEVEL::LOG_LEVEL_DEBUG
#endif // LOG_DEBUG


// #ifdef LOG_INFO
// #define _LOG_LVL 1
// #define LOG_LOGER_LEVEL se::LOG_LEVEL::LOG_LEVEL_INFO
// #endif // LOG_INFO

// #ifdef LOG_WARN
// #define _LOG_LVL 2
// #define LOG_LOGER_LEVEL se::LOG_LEVEL::LOG_LEVEL_WARNING
// #endif // LOG_WARN

// #ifdef LOG_ERROR
// #define _LOG_LVL 3
// #define LOG_LOGER_LEVEL se::LOG_LEVEL::LOG_LEVEL_ERROR
// #endif // LOG_ERROR

// #if _LOG_LVL <= 0
// #define log_debug(...) se::Logger::get_instance().debug(__VA_ARGS__)
// #else
// #define log_debug(...)
// #endif // LOG_DEBUG

// #if _LOG_LVL <= 1
// #define log_info(...) se::Logger::get_instance().info(__VA_ARGS__)
// #else
// #define log_info(...)
// #endif // LOG_INFO

// #if _LOG_LVL <= 2
// #define log_warn(...) se::Logger::get_instance().warn(__VA_ARGS__)
// #else
// #define log_warn(...)
// #endif // LOG_WARN

// #if _LOG_LVL <= 3
// #define log_error(...) se::Logger::get_instance().error(__VA_ARGS__)
// #else
// #define log_error(...)
// #endif // LOG_ERROR


#endif // MAIN_PROG_H  mc->current_state.position = mc->motor->get_absolute_position();
