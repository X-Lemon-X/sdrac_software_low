

#ifndef STEPER_MOTOR_HPP
#define STEPER_MOTOR_HPP

#include "stm32f4xx_hal.h"
#include "main_prog.hpp"
#include "Timing.hpp"

namespace STEPER_MOTOR
{
class SteperMotor{
private:
  float radians_to_frequency;
  float frequency;
  float angle;
  TIM_HandleTypeDef &htim;
public:
  float steps_per_revolution;
  float gear_ratio;
  // float max_acceleration;
  float max_velocity;
  float angle_limit_max;
  float angle_limit_min;
  bool reverse;
  uint8_t direction_pin;
  uint8_t enable_pin;

  SteperMotor(TIM_HandleTypeDef &htim, uint8_t direction_pin, uint8_t enable_pin);
  
  /// @brief Initialize the SteperMotor, calcualtes all necessary stuff to avoid calcualting it over again
  /// after the initialization
  void init();
  void set_curent_position(float angle);
  void set_speed(float speed);
  void disable();
  void enable();

}
}


#endif // STEPER_MOTOR_HPP