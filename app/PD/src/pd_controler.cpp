
#include "pd_controler.hpp"
#include "math.h"

using namespace PDCONTROLER;

PdControler::PdControler(TIMING::Ticker &ticker): MOVEMENT_CONTROLER::MovementEquation(ticker){
  Kp = 0;
  Kd = 0;
  Ki = 0;
  previous_velocity = 0;
  previous_position = 0;
  previous_time = 0;
}

float PdControler::calculate(float current_position, float target_position, float current_velocity, float target_velocity){  
  const float current_time = ticker.get_seconds();
  // const float state_velocity = (previous_position-current_position) / (current_time - previous_time);
  float dt = current_time - previous_time;
  previous_time = current_time;
  // return Kp * (target_position - current_position) + Kd * (target_velocity - state_velocity);
  
  
  // float error = target_velocity - current_velocity;
  // add inertia to the system
  float velocity =  (Kp * previous_velocity) + (Kd * target_velocity);
  previous_velocity = velocity;
  return velocity;
}


void PdControler::begin_state(float current_position, float current_velocity, float current_time){
  previous_position = current_position;
  previous_time = current_time;
  current_velocity = current_velocity;
}