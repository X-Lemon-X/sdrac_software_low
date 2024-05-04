
#include "main.h"
#include "stm32f4xx_hal.h"
#include "movement_controler.hpp"


using namespace MOVEMENT_CONTROLER;


MovementControler::MovementControler(){
  initialized = false;
  target_position = 0;
  current_position = 0;
  target_velocity = 0;
  current_velocity = 0;
}

void MovementControler::init(TIMING::Ticker &_ticker, STEPER_MOTOR::SteperMotor &_steper_motor, ENCODER::Encoder &_encoder){
  ticker = &_ticker;
  steper_motor = &_steper_motor;
  encoder = &_encoder;
  initialized = true;
}

void MovementControler::handle(){
  if (!initialized)
    return;

  steper_motor->set_speed(target_velocity);
}

void MovementControler::set_velocity(float velocity){
  target_velocity = velocity;
}

void MovementControler::set_enable(bool enable){
  this->enable = enable;
}

void MovementControler::set_position(float position){
  target_position = position;
}

void MovementControler::set_limit_position(float min_position, float max_position){
  this->min_position = min_position;
  this->max_position = max_position;
}

void MovementControler::set_max_velocity(float max_velocity){
  this->max_velocity = abs(max_velocity);
}


float MovementControler::get_current_position(){
  return current_position;
}

float MovementControler::get_current_velocity(){
  return current_velocity;
}
