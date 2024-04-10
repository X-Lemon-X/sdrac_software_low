
#include "main.h"
#include "encoder.hpp"
#include <stdexcept>
// #include "main_prog.hpp"

using namespace ENCODER;

Encoder::Encoder(I2C_HandleTypeDef &_hi2c,TIMING::Ticker &_ticker,FILTERS::FilterBase &_filter,FILTERS::FilterBase &_filter_velocity): 
hi2c(_hi2c),
ticker(_ticker),
filter_angle(_filter).
filter_velocity(_filter_velocity) {

this->resolution = 4096;
this->address = 0x40;
this->reverse = false;
this->offset = 0;
this->raw_angle = 0;
this->data[0] = 0;
this->data[1] = 0;
this->enable_filter = false;
this->enable_velocity = false;
this->enable_velocity_filter = false;

}

bool Encoder::ping_encoder(){
  return HAL_I2C_IsDeviceReady(&hi2c, address, 1, 100) == HAL_OK;
}

uint16_t Encoder::read_raw_angle(){
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c, address, angle_register, 1, data, 2, 20);
  if(status != HAL_OK) return -2;
  uint16_t reg1 = (uint16_t)this->data[0];
  reg1 = reg1 << 6;
  uint16_t reg2 = (this->data[1] & 0xfc) >> 2;
  this->raw_angle = reg1 + reg2;
  return this->raw_angle;
}

float Encoder::calculate_velocity(float angle){
  float current_tiem = ticker.get_seconds();
  float current_velocity = (angle-prev_angle) / (current_tiem - last_time);
  last_time = current_tiem;
  prev_angle = angle;
  if(this->enable_velocity_filter)
    current_velocity = filter_angle.calculate(current_velocity);
  return current_velocity;
}

float Encoder::read_angle(){
  float angle = (float)read_raw_angle()*6.28318530f / (float)this->resolution;
  if(this->reverse) angle = -angle;
  angle += this->offset;

  angle = filter_angle.calculate(angle);

  if(this->enable_velocity)
    this->velocity = calculate_velocity(angle);

  return angle;
}

float Encoder::get_velocity(){
  return this->velocity;
}
