
#include "main.h"
#include "encoder.hpp"
#include <stdexcept>
// #include "main_prog.hpp"

using namespace ENCODER;

Encoder::Encoder(I2C_HandleTypeDef &_hi2c,TIMING::Ticker &_ticker,float *_velocity_filter_weight): 
hi2c(_hi2c),
ticker(_ticker),
velocity_filter_weight(_velocity_filter_weight) {

this->resolution = 4096;
this->address = 0x40;
this->reverse = false;
this->offset = 0;
this->raw_angle = 0;
this->data[0] = 0;
this->data[1] = 0;
this->enable_filter = false;
this->enable_velocity = false;

  for(int i = 0; i < VELOCITY_FILTER_SIZE; i++){
    this->velocity_previous.push_back(0.0f);
  }
}

bool Encoder::ping_encoder(){
  return HAL_I2C_IsDeviceReady(&hi2c, address, 1, 100) == HAL_OK;
}

uint16_t Encoder::read_raw_angle(){
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c, address, angle_register, 1, data, 2, 20);
  if(status != HAL_OK) return -2;
  
  // loger.debug("data[0]: " + std::to_string(this->data[0]));
  // loger.debug("data[1]: " + std::to_string(this->data[1]));

  uint16_t reg1 = (uint16_t)this->data[0] << 6;
  uint16_t reg2 = this->data[1] & 0xfe;
  this->raw_angle = reg1 + reg2;
  return this->raw_angle;
}

float Encoder::calculate_velocity(float angle){
  uint64_t current_tiem = ticker.get_micros();
  
  float current_velocity = (angle-prev_angle) / ((float)(current_tiem - last_time)*0.000001f);
  if(!this->enable_filter) return current_velocity;

  velocity_previous.push_back(current_velocity);
  velocity_previous.pop_front();
  float sum = 0;
  for (int j = 0 ; j < VELOCITY_FILTER_SIZE; j++)
    sum += velocity_previous[j] * velocity_filter_weight[j];
  this->last_time = current_tiem;
  this->prev_angle = angle;
  return sum;
}

float Encoder::read_angle(){
  float angle = (float)read_raw_angle()*6.28318530f / (float)this->resolution;
  if(this->reverse) angle = -angle;
  angle += this->offset;

  if(this->enable_velocity)
    this->velocity = calculate_velocity(angle);

  return angle;
}

float Encoder::get_velocity(){
  return this->velocity;
}
