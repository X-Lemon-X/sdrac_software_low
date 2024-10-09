
#ifndef MAIN_PROG_H
#define MAIN_PROG_H

//**************************************************************************************************
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

#endif // MAIN_PROG_H