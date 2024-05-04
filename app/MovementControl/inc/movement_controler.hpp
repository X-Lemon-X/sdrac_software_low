#include "main.h"
#include "stm32f4xx_hal.h"
#include "Timing.hpp" 
#include "steper_motor.hpp"
#include "encoder.hpp"

#ifndef MOVE_CONTROLER_HPP
#define MOVE_CONTROLER_HPP

namespace MOVEMENT_CONTROLER
{

class MovementControler{ 
  protected:
  TIMING::Ticker *ticker;
  STEPER_MOTOR::SteperMotor *steper_motor;
  ENCODER::Encoder *encoder;
  bool initialized;

  float max_velocity;
  float min_position;
  float max_position;


  float target_position;
  float current_position;
  float target_velocity;
  float current_velocity;
  bool enable;
  public:

  /// @brief Simple movement controler for an engine with simple PD algorithm
  MovementControler();
  
  /// @brief Initialize the controler
  /// @param ticker Ticker object for main system time wity microsecond resolution
  /// @param steper_motor Steper motor object
  /// @param encoder Encoder object
  void init(TIMING::Ticker &ticker, STEPER_MOTOR::SteperMotor &steper_motor, ENCODER::Encoder &encoder);
  
  /// @brief Handler for the controler this function should be called in the main loop as often as possible
  void handler();

  /// @brief Set the target velocity for the engine
  /// @param velocity Target velocity in rad/s
  void set_velocity(float velocity);

  /// @brief Enable or disable the engine
  /// @param enable True to turn on the endine-brake, false to turn off the engine-brake
  void set_enable(bool enable);

  /// @brief Set the target position for the engine
  /// @param position Target position in rad can be positive or negative
  void set_position(float position);

  /// @brief Set the limit position for arm
  /// @param min_position Minimum position in rad can be positive or negative
  /// @param max_position Maximum position in rad can be positive or negative have to be greater than min_position
  void set_limit_position(float min_position, float max_position);

  /// @brief Set the max velocity for the arm
  /// @param max_velocity Maximum velocity in rad/s
  void set_max_velocity(float max_velocity);

  /// @brief Get the current position of the arm
  /// @return Current position in rad
  float get_current_position();

  /// @brief Get the current velocity of the arm
  /// @return Current velocity in rad/s
  float get_current_velocity();

};





} // namespace ENGINE_CONTROLER
#endif