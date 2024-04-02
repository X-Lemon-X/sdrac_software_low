
#include "steper_motor.hpp"
#include "main_prog.hpp"

using namespace STEPER_MOTOR;

SteperMotor::SteperMotor(TIM_HandleTypeDef &_htim,unsigned int _timer_channel, Pin _direction_pin, Pin _enable_pin):
htim(_htim),
direction_pin(_direction_pin),
timer_channel(_timer_channel),
enable_pin(_enable_pin){
  this->radians_to_frequency = 0;
  this->steps_per_revolution = 400;
  this->gear_ratio = 1;
  this->max_velocity = 0;
  this->reverse = false;
  this->init();
}


void SteperMotor::init(){
  uint32_t core_freq = HAL_RCC_GetHCLKFreq();
  uint32_t prescaler = htim.Instance->PSC;
  this->radians_to_frequency = core_freq / prescaler / ((this->steps_per_revolution * this->gear_ratio) / PI_m2);

  if (this->reverse){
    this->direction_positive = GPIO_PIN_RESET;
    this->direction_negative = GPIO_PIN_SET;
  }else{
    this->direction_positive = GPIO_PIN_SET;
    this->direction_negative = GPIO_PIN_RESET;
  }

}

void SteperMotor::set_speed(float speed){

  if (speed > 0)
    HAL_GPIO_WritePin(direction_pin.port, direction_pin.pin,direction_positive);
  else {
    HAL_GPIO_WritePin(direction_pin.port, direction_pin.pin,direction_negative);
    speed = -speed;
  }

  uint16_t counter = this->radians_to_frequency / speed;
  htim.Instance->ARR = counter;
  htim.Instance->CCR1 = counter/2;
}

void SteperMotor::set_enable(bool enable){
  if (enable){
    HAL_GPIO_WritePin(enable_pin.port, enable_pin.pin,GPIO_PIN_SET);
  }else{
    HAL_GPIO_WritePin(enable_pin.port, enable_pin.pin,GPIO_PIN_RESET);
    htim.Instance->CCR1 = 0;
  }
}




