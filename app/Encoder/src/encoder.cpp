
#include "main.h"
#include "encoder.hpp"
#include <stdexcept>
// #include "main_prog.hpp"

using namespace ENCODER;

Encoder::Encoder(){
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

void Encoder::init(I2C_HandleTypeDef &hi2c, TIMING::Ticker &ticker, FILTERS::FilterBase &filter_angle, FILTERS::FilterBase &filter_velocity){
  this->hi2c = &hi2c;
  this->ticker = &ticker;
  this->filter_angle = &filter_angle;
  this->filter_velocity = &filter_velocity;
  this->last_time = ticker.get_seconds();
  this->prev_angle = 0;
  this->current_velocity = 0;
  this->current_angle = 0;
}

bool Encoder::ping_encoder(){
  return HAL_I2C_IsDeviceReady(hi2c, address, 1, 100) == HAL_OK;
}

uint16_t Encoder::read_raw_angle(){
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, address, angle_register, 1, data, 2, 20);
  if(status != HAL_OK) return -2;
  uint16_t reg1 = (uint16_t)this->data[0];
  reg1 = reg1 << 6;
  uint16_t reg2 = (this->data[1] & 0xfc) >> 2;
  this->raw_angle = reg1 + reg2;
  return this->raw_angle;
}

float Encoder::calculate_velocity(float angle){
  float current_tiem = ticker->get_seconds();
  float current_velocity = (angle-prev_angle) / (current_tiem - last_time);
  last_time = current_tiem;
  prev_angle = angle;
  if(this->enable_velocity_filter)
    current_velocity = filter_angle->calculate(current_velocity);
  return current_velocity;
}

float Encoder::read_angle(){
  float angle = (float)read_raw_angle()*6.28318530f / (float)this->resolution;
  if(this->reverse) angle = -angle;
  angle += this->offset;

  angle = filter_angle->calculate(angle);

  if(this->enable_velocity)
    this->current_velocity = calculate_velocity(angle);
  
  current_angle = angle;
  return angle;
}

void Encoder::handle(){
  read_angle();
}


float Encoder::get_velocity(){
  return this->current_velocity;
}

float Encoder::get_angle(){
  return this->current_angle;
}
