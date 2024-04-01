
#include "main.h"
#include "encoder.hpp"
#include <stdexcept>
// #include "main_prog.hpp"

using namespace ENCODER;


#define PIx2 6.28318530717958647692

Encoder::Encoder(I2C_HandleTypeDef *hi2c,float *_velocity_filter_weight): velocity_filter_weight(_velocity_filter_weight) {
  if(hi2c == NULL) return;
  this->hi2c = hi2c;
  this->resolution = 4096;
  this->address = 0x40;
  this->reverse = false;
  this->offset = 0;
  this->raw_angle = 0;
  this->data[0] = 0;
  this->data[1] = 0;

  for(int i = 0; i < VELOCITY_FILTER_SIZE; i++){
    this->velocity_previous.push_back(0.0f);
  }
}

bool Encoder::ping_encoder(){
  return HAL_I2C_IsDeviceReady(this->hi2c, this->address, 1, 100) == HAL_OK;
}

void Encoder::set_offset(float offset, bool reverse){
  this->offset = offset;
  this->reverse = reverse;
}

int Encoder::read_raw_angle(){
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(this->hi2c, this->address, this->angle_register, 1, this->data, 2, 20);
  if(status != HAL_OK) return -2;
  
  // loger.debug("data[0]: " + std::to_string(this->data[0]));
  // loger.debug("data[1]: " + std::to_string(this->data[1]));

  uint16_t reg1 = (uint16_t)this->data[0] << 6;
  uint16_t reg2 = this->data[1] & 0xfe;
  this->raw_angle = reg1 + reg2;
  return this->raw_angle;
}

float Encoder::calculate_velocity(float angle, float current_time){
  float current_velocity = (angle-prev_angle) / (current_time - last_time);
  velocity_previous.push_back(current_velocity);
  velocity_previous.pop_front();
  // this->velocity_list.erase(this->velocity_list.begin());

  float sum = 0;
  for (int j = 0 ; j < VELOCITY_FILTER_SIZE; j++)
    sum += velocity_previous[j] * velocity_filter_weight[j];
  // this->velocity_list[VELOCITY_FILTER_SIZE-1] = sum;
  
  this->last_time = current_time;
  this->prev_angle = angle;

  return sum;
}


float Encoder::read_angle(){
  float angle = (float)read_raw_angle()*PIx2 / (float)this->resolution;
  if(this->reverse) angle = -angle;
  angle += this->offset;

  // float current_time = ticker.get_tick();
  float current_time = (float)HAL_GetTick() / 1000.0f;
  this->velocity = calculate_velocity(angle, current_time);
  
  return angle;
}

float Encoder::get_velocity(){
  return this->velocity;
}
