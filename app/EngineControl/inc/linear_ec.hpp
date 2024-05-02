
#include "engine_controler.hpp"
#include "steper_motor.hpp"
#include "encoder.hpp"
#include "Timing.hpp"

#ifndef ENGINE_CONTROLER_LINEAR_HPP
#define ENGINE_CONTROLER_LINEAR_HPP

namespace EC_LINEAR
{

class lenear_ec : public ENGINE_CONTROLER::EngineControler
{
private:
  float velocity;
  float target_position;
public:
  lenear_ec(TIMING::Ticker &ticker, STEPER_MOTOR::SteperMotor &steper_motor, ENCODER::Encoder &encoder): 
    ENGINE_CONTROLER::EngineControler(ticker, steper_motor, encoder){};

  float max_acceleration;
  float max_velocity;
  float positive_limit_position;
  float negative_limit_position;

  void init();
  void handler();
  void set_velocity(float speed);
  void set_enable(bool enable);
  void set_position(float position);
};



} // namespace ENGINE_CONTROLER

#endif