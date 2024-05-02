#include "linear_ec.hpp"  

#include "engine_controler.hpp"
#include "steper_motor.hpp"
#include "encoder.hpp"
#include "Timing.hpp"

using namespace EC_LINEAR;

void lenear_ec::init(){
  steper_motor.init();
}


void lenear_ec::handler(){
  float current_position = encoder.read_angle();
  float delta_position = target_position - current_position;
  
}

void lenear_ec::set_velocity(float velocity){
  this->velocity=velocity;
}

void lenear_ec::set_enable(bool enable){
  steper_motor.set_enable(enable);
}

void lenear_ec::set_position(float position){
  this->target_position = position;
}



