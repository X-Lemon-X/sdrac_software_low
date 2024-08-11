
#include "steper_motor.hpp"
#include <cmath>

using namespace STEPER_MOTOR;

#define PIM2 6.28318530717958647692f


SteperMotor::SteperMotor(TIM_HandleTypeDef &_htim,unsigned int _timer_channel,const GpioPin &_direction_pin,const GpioPin &_enable_pin):
htim(_htim),
direction_pin(_direction_pin),
timer_channel(_timer_channel),
enable_pin(_enable_pin){
  this->radians_to_frequency = 0;
  this->steps_per_revolution = 400;
  this->gear_ratio = 1;
  this->max_velocity = 0;
  this->min_velocity = 0;
  this->reverse = false;
  this->init();
}


void SteperMotor::init(){
  uint32_t core_freq = HAL_RCC_GetHCLKFreq();
  uint32_t prescaler = htim.Instance->PSC;
  this->radians_to_frequency = core_freq / prescaler / ((this->steps_per_revolution * this->gear_ratio) / PIM2  );

  if (this->reverse){
    this->direction_positive = GPIO_PIN_RESET;
    this->direction_negative = GPIO_PIN_SET;
  }else{
    this->direction_positive = GPIO_PIN_SET;
    this->direction_negative = GPIO_PIN_RESET;
  }

}

void SteperMotor::set_velocity(float velocity){

  if (std::abs(velocity) > this->max_velocity)
    velocity = this->max_velocity;
  else if (std::abs(velocity) < this->min_velocity){
    htim.Instance->CCR1 = 0;
    return;
  }

  if (velocity > 0)
    WRITE_GPIO(direction_pin,direction_positive);
  else {
    WRITE_GPIO(direction_pin,direction_negative);
    velocity = -velocity;
  }

  if(velocity == 0){
    htim.Instance->CCR1 = 0;
    return;
  }
  uint16_t counter = (uint16_t)(this->radians_to_frequency / velocity);
  htim.Instance->ARR = counter;
  htim.Instance->CCR1 = counter/2;
}


void SteperMotor::set_enable(bool enable){
  WRITE_GPIO(enable_pin, enable? GPIO_PIN_SET : GPIO_PIN_RESET);
}




