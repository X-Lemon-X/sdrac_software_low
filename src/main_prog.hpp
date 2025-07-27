
#pragma once

#include "cmsis_os.h"


#include "can_messages.h"
#include "encoder.hpp"

#include "MCP9700AT.hpp"
#include "Timing.hpp"
// #include "can_control.hpp"
#include "config.hpp"
#include "controler_linear.hpp"
#include "controler_pass_through.hpp"
#include "controler_pid.hpp"
#include "dfu_usb_programer.hpp"
#include "filter.hpp"
#include "filter_alfa_beta.hpp"
#include "filter_moving_avarage.hpp"
#include "gpio.hpp"
#include "logger.hpp"
#include "movement_controler.hpp"
#include "ntc_termistor.hpp"
#include "steper_motor.hpp"
#include "servo_motor.hpp"

#include <cfloat>
#include <cmath>
#include <memory>
#include <string>


#ifndef MAIN_PROG_H
#define MAIN_PROG_H


extern se::movement::PIDControler pid_pos;
extern std::shared_ptr<se::movement::BasicLinearPosControler> bacis_controler;
extern std::shared_ptr<se::movement::PassThroughControler> pass_through_controler;
extern se::filters::FilterMovingAvarage encoder_motor_moving_avarage;
extern se::filters::FilterSampleSkip encoder_arm_filter_velocity;
extern se::Timer tim_can_disconnecteded;
extern se::SimpleTask task_default_task;
extern se::SimpleTask task_blink_error_task;
extern se::SimpleTask task_read_analog_values_task;
// extern se::SimpleTask task_encoder_timer;
extern se::SimpleTask task_usb_task;
extern se::SimpleTask task_data_usb_send_task;
// extern se::SimpleTask task_caculate_temp_timer;
extern se::SimpleTask task_error_task;

// // extern std::shared_ptr<se::Timing> task_blink_timer;
// extern std::shared_ptr<se::Timing> task_blink_error_timer;
// extern std::shared_ptr<se::Timing> task_read_analog_values_timer;
// extern std::shared_ptr<se::Timing> task_encoder_timer;
// extern std::shared_ptr<se::Timing> task_usb_timer;
// extern std::shared_ptr<se::Timing> task_data_usb_send_timer;
// extern std::shared_ptr<se::Timing> task_caculate_temp_timer;
// extern std::shared_ptr<se::Timing> task_nodelay_timer;
extern std::shared_ptr<se::Timer> task_can_disconnected_timer;
extern float temoperature_board;
extern float temoperature_steper_driver;
extern float temoperature_steper_motor;
extern float voltage_vcc;

//**************************************************************************************************
/// CONFIGURATION FUNCTIONS

/// @brief main program, this function is called from main and never returns
void run_main_prog();

/// @brief This function is used to handle the can controll
void handle_can_rx();

/// @brief get board if from hex encoded
uint8_t get_board_id();

/// @brief This function is used to configure the board base on it's hardware id
se::Status id_config();

/// @brief  Initites stuff after id have been configured
se::Status post_id_config();

/// @brief This function is used to configure the tasks
se::Status config_tasks();

se::Status startup_robot(se::SimpleTask &task, void *args);

/// @brief initites the movement controler and sets the mode initiation
/// is also required to chnage control mode since wewant to have known state of the robot when
// changing control algorithm
void init_and_set_movement_controler_mode(uint8_t mode);

/// @brief can disconnect timeout reset
void can_disconnect_timeout_reset();

/// @brief error checks
void error_checks();

///**************************************************************************************************
/// TASKS
se::Status task_error_check(se::SimpleTask &task_handler, void *args);
se::Status task_blink(se::SimpleTask &task_handler, void *arg);
se::Status task_blink_error(se::SimpleTask &task_handler, void *arg);
se::Status task_encoders(se::SimpleTask &task_handler, void *arg);
se::Status task_usb_handler(se::SimpleTask &task_handler, void *arg);
se::Status task_usb_data_loging(se::SimpleTask &task_handler, void *arg);
se::Status task_can_disconnect(se::SimpleTask &task_handler, void *arg);
se::Status task_read_analog_values(se::SimpleTask &task_handler, void *arg);
se::Status task_nodelay(se::SimpleTask &task_handler, void *arg);

///**************************************************************************************************
/// CAN CALLBACKS

void can_callback_default(se::CanBase &can, se::CanDataFrame &received_msg, void *args);
void can_callback_get_errors(se::CanBase &can, se::CanDataFrame &received_msg, void *args);
void can_callback_clear_errors(se::CanBase &can, se::CanDataFrame &received_msg, void *args);
void can_callback_status(se::CanBase &can, se::CanDataFrame &received_msg, void *args);
void can_callback_get_pos(se::CanBase &can, se::CanDataFrame &received_msg, void *args);
void can_callback_set_pos(se::CanBase &can, se::CanDataFrame &received_msg, void *args);
void can_callback_set_control_mode(se::CanBase &can, se::CanDataFrame &received_msg, void *args);
void can_callback_set_effector_position(se::CanBase &can, se::CanDataFrame &received_msg, void *args);


#endif // MAIN_PROG_H