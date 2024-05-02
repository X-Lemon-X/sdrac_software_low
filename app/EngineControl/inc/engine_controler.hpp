#include "main.h"
#include "stm32f4xx_hal.h"
#include "Timing.hpp" 
#include "steper_motor.hpp"
#include "encoder.hpp"

#ifndef ENGINE_CONTROLER_HPP
#define ENGINE_CONTROLER_HPP

namespace ENGINE_CONTROLER
{

class EngineControler{ 
  protected:
  TIMING::Ticker &ticker;
  STEPER_MOTOR::SteperMotor &steper_motor;
  ENCODER::Encoder &encoder;

  public:
  EngineControler(TIMING::Ticker &ticker, STEPER_MOTOR::SteperMotor &steper_motor, ENCODER::Encoder &encoder);
  
  void init();
  void handler();
  void set_velocity(float speed);
  void set_enable(bool enable);
  void set_position(float position);
};





} // namespace ENGINE_CONTROLER
#endif