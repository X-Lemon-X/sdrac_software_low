
#pragma once
#include "can_control.hpp"

#include "can.h"
#include "encoder.hpp"

#include "logger.hpp"
#include "dfu_usb_programer.hpp"
#include "steper_motor.hpp"
#include "filter.hpp"
#include "filter_alfa_beta.hpp"
#include "filter_moving_avarage.hpp"
#include "can_control.hpp"
#include "board_id.hpp"
#include "movement_controler.hpp"
#include "controler_pid.hpp"
#include "controler_linear.hpp"
#include "controler_pass_through.hpp"
#include "pin.hpp"
#include "ntc_termistors.hpp"
#include <cfloat>
#include "MCP9700AT.hpp"
#include "Timing.hpp"
#include "config.hpp"
#include <memory>
#include <string>
#include <cmath>


#ifndef MAIN_PROG_H
#define MAIN_PROG_H


extern stmepic::PIDControler pid_pos;
extern stmepic::BasicLinearPosControler bacis_controler;
extern stmepic::PassThroughControler pass_through_controler;
extern stmepic::filters::FilterMovingAvarage encoder_motor_moving_avarage;
extern stmepic::filters::FilterSampleSkip encoder_arm_filter_velocity;
extern stmepic::Timing tim_can_disconnecteded;
extern std::shared_ptr<stmepic::Timing> task_blink_timer;
extern std::shared_ptr<stmepic::Timing> task_blink_error_timer;
extern std::shared_ptr<stmepic::Timing> task_read_analog_values_timer;
extern std::shared_ptr<stmepic::Timing> task_encoder_timer;
extern std::shared_ptr<stmepic::Timing> task_usb_timer;
extern std::shared_ptr<stmepic::Timing> task_data_usb_send_timer;
extern std::shared_ptr<stmepic::Timing> task_caculate_temp_timer;
extern std::shared_ptr<stmepic::Timing> task_nodelay_timer;
extern std::shared_ptr<stmepic::Timing> task_can_disconnected_timer;
extern float temoperature_board;
extern float temoperature_steper_driver;
extern float temoperature_steper_motor; 
extern float voltage_vcc;

//**************************************************************************************************
/// CONFIGURATION FUNCTIONS

/// @brief main program, this function is called from main and never returns
void run_main_prog();

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

/// @brief  Initites stuff after id have been configured
void post_id_config();

/// @brief This function is used to configure the tasks
void config_tasks();

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
void task_blink(stmepic::Timing& task_timer);
void task_blink_error(stmepic::Timing& task_timer);
void task_encoders(stmepic::Timing& task_timer);
void task_usb_handler(stmepic::Timing& task_timer);
void task_usb_data_loging(stmepic::Timing& task_timer);
void task_can_disconnect(stmepic::Timing& task_timer);
void task_read_analog_values(stmepic::Timing& task_timer);
void task_nodelay(stmepic::Timing& task_timer);

///**************************************************************************************************
/// CAN CALLBACKS

void can_callback_default(stmepic::can_msg &recived_msg);
void can_callback_get_errors(stmepic::can_msg &recived_msg);
void can_callback_clear_errors(stmepic::can_msg &recived_msg);
void can_callback_status(stmepic::can_msg &recived_msg);
void can_callback_get_pos(stmepic::can_msg &recived_msg);
void can_callback_set_pos(stmepic::can_msg &recived_msg);
void can_callback_set_control_mode(stmepic::can_msg &recived_msg);


#endif // MAIN_PROG_H