
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
  this->prev_angle = read_angle();
  this->prev_angle_velocity = this->absoulte_angle;
  this->current_velocity = 0;
  this->over_drive_angle = 0;

}

bool Encoder::ping_encoder(){
  return HAL_I2C_IsDeviceReady(hi2c, address, 1, 100) == HAL_OK;
}

uint16_t Encoder::read_raw_angle(){
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, address, angle_register, 1, this->data, 2, 5);
  if(status != HAL_OK) return -2;
  uint16_t reg = (uint16_t)this->data[0] << 6;
  reg |= (uint16_t)(this->data[1] & 0xfc) >> 2;
  return reg;
}

float Encoder::calculate_velocity(float angle){
  float current_tiem = ticker->get_seconds();
  const float dt = current_tiem - last_time;
  float current_velocity = (angle-prev_angle_velocity) / dt;
  last_time = current_tiem;
  if(this->enable_velocity_filter)
    current_velocity = filter_angle->calculate(current_velocity);

  log_debug(std::to_string(angle) + ";" + std::to_string(current_tiem));
  prev_angle_velocity = angle;
  return current_velocity;
}

float Encoder::read_angle(){
  float angle = (float)read_raw_angle()*PI_m2 / (float)this->resolution;
  if(this->reverse) angle = PI_m2 - angle;
  angle += this->offset;
  if (angle > PI_m2) angle -= PI_m2;
  if (angle < 0) angle += PI_m2;

  angle = filter_angle->calculate(angle);

  if(prev_angle - angle > ANGLE_MAX_DEFFERENCE)
    over_drive_angle += PI_m2;
  else if(angle - prev_angle > ANGLE_MAX_DEFFERENCE)
    over_drive_angle -= PI_m2;
  prev_angle = angle;
  absoulte_angle = angle + over_drive_angle;

  if(this->enable_velocity && ++velocity_sample_count >= velocity_samples_amount){
    this->current_velocity = calculate_velocity(absoulte_angle);
    velocity_sample_count = 0;
  }
  return angle;
}

void Encoder::handle(){
  read_angle();
}


float Encoder::get_velocity(){
  return this->current_velocity;
}

float Encoder::get_angle(){
  return this->absoulte_angle;
}

void Encoder::set_resolution(uint16_t resolution){
  this->resolution = resolution;
}
  
void Encoder::set_offset(float offset){
  this->offset = offset;
}

void Encoder::set_reverse(bool reverse){
  this->reverse = reverse;
}
  
void Encoder::set_address(uint8_t address){
  this->address = address;
}
  
void Encoder::set_angle_register(uint8_t angle_register){
  this->angle_register = angle_register;
}
  
void Encoder::set_magnes_detection_register(uint8_t magnes_detection_register){
  this->magnes_detection_register = magnes_detection_register;
}
  
void Encoder::set_enable_filter(bool enable_filter){
  this->enable_filter = enable_filter;
}

void Encoder::set_enable_velocity(bool enable_velocity){
  this->enable_velocity = enable_velocity;
}
  
void Encoder::set_enable_velocity_filter(bool enable_velocity_filter){
  this->enable_velocity_filter = enable_velocity_filter;
}

void Encoder::set_velocity_sample_amount(uint16_t velocity_samples_amount){
  this->velocity_samples_amount = velocity_samples_amount;
}
