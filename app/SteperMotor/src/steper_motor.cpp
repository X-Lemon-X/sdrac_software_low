
#include "steper_motor.hpp"
#include "main_prog.hpp"

using namespace STEPER_MOTOR;

SteperMotor::SteperMotor(TIM_HandleTypeDef &_htim, uint8_t _direction_pin, uint8_t _enable_pin):
htim(_htim),
direction_pin(_direction_pin),
enable_pin(_enable_pin){
  this->radians_to_frequency = 0;
  this->steps_per_revolution = 400;
  this->gear_ratio = 1;
  this->max_velocity = 0;
  this->angle_limit_max = 0;
  this->angle_limit_min = 0;
  this->reverse = false;
}


void SteperMotor::init(){
  // Calculate the frequency to radians
  this->radians_to_frequency = (this->steps_per_revolution * this->gear_ratio) / (2 * PI);
}

void SteperMotor::set_curent_position(float angle){
  this->angle = angle;
}

void SteperMotor::set_speed(float speed){
  this->frequency = speed * this->radians_to_frequency;
  __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, this->frequency);
}




