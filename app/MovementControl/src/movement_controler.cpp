
#include "main.h"
#include "stm32f4xx_hal.h"
#include "movement_controler.hpp"


using namespace MOVEMENT_CONTROLER;


MovementEquation::MovementEquation(TIMING::Ticker &_ticker): ticker(_ticker){}

MovementControler::MovementControler(){
  initialized = false;
  target_position = 0;
  current_position = 0;
  target_velocity = 0;
  current_velocity = 0;
}

MovementControler::~MovementControler(){
  if(initialized){
    steper_motor->set_velocity(0.0);
    steper_motor->set_enable(false);
  }
}

void MovementControler::init(TIMING::Ticker &_ticker, STEPER_MOTOR::SteperMotor &_steper_motor, ENCODER::Encoder &_encoder, MovementEquation &_movement_equation){
  ticker = &_ticker;
  steper_motor = &_steper_motor;
  encoder = &_encoder;
  movement_equation = &_movement_equation;
  initialized = true;
  movement_equation->begin_state(encoder->read_angle(), encoder->get_velocity(), ticker->get_seconds());
}

void MovementControler::handle(){
  if (!initialized) return;
  current_position = encoder->get_angle();
  current_velocity = encoder->get_velocity();
  float new_velocity = movement_equation->calculate(current_position, target_position, current_velocity, target_velocity);
  
  if (abs(new_velocity) > max_velocity)
    new_velocity = max_velocity;

  if(enable) {
    steper_motor->set_enable(true);
  } else {
    steper_motor->set_enable(false);
    new_velocity = 0.0;
  }

  if (current_position < min_position || current_position > max_position)
    new_velocity = 0.0;

  steper_motor->set_velocity(new_velocity);
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


float MovementControler::get_current_position()const{
  return current_position;
}

float MovementControler::get_current_velocity()const{
  return current_velocity;
}
