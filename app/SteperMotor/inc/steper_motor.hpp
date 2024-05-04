

#ifndef STEPER_MOTOR_HPP
#define STEPER_MOTOR_HPP

#include "stm32f4xx_hal.h"
#include "main_prog.hpp"
#include "Timing.hpp"


#define CPU_FREQ_MHZ 96.0f

namespace STEPER_MOTOR
{
class SteperMotor{
private:
  float radians_to_frequency;
  float frequency;
  float angle;
  TIM_HandleTypeDef &htim;
  unsigned int timer_channel;
  const GPIO_PIN &direction_pin;
  const GPIO_PIN &enable_pin;
  GPIO_PinState direction_positive;
  GPIO_PinState direction_negative;

public:
  float steps_per_revolution;
  float gear_ratio;
  // float max_acceleration;
  float max_velocity;
  float min_velocity;
  bool reverse;


  SteperMotor(TIM_HandleTypeDef &htim,unsigned int timer_channel,const GPIO_PIN &direction_pin, const GPIO_PIN &enable_pin);
  
  /// @brief Initialize the SteperMotor, calcualtes all necessary stuff to avoid calcualting it over again
  /// after the initialization
  void init();

  /// @brief Set the current speed of the SteperMotor
  /// @param speed The speed in radians per second, can be negative or positive to change the direction
  void set_speed(float speed);

  /// @brief enable or disable the SteperMotor, can be used as a break
  /// @param enable True to enable the SteperMotor, false to disable it
  void set_enable(bool enable);
};
}


#endif // STEPER_MOTOR_HPP