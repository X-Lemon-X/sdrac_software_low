
#include "main.h"
#include "encoder.hpp"
#include <stdexcept>
// #include "main_prog.hpp"

using namespace ENCODER;


#define PIx2 6.28318530717958647692

Encoder::Encoder(I2C_HandleTypeDef *hi2c){
  if(hi2c == NULL) return;
  this->hi2c = hi2c;
  this->resolution = 4096;
  this->address = 0x40;
  this->reverse = false;
  this->offset = 0;
  this->raw_angle = 0;
  this->data[0] = 0;
  this->data[1] = 0;
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
  
  log.debug("data[0]: " + std::to_string(this->data[0]));
  log.debug("data[1]: " + std::to_string(this->data[1]));

  uint16_t reg1 = (uint16_t)this->data[0] << 6;
  uint16_t reg2 = this->data[1] & 0xfe;
  this->raw_angle = reg1 + reg2;
  return this->raw_angle;
}

float Encoder::read_angle(){
  int prev_raw_angle = this->raw_angle;
  int angle = read_raw_angle()*PIx2;
  if(this->reverse) angle = -angle;
  angle += this->offset;
  return angle;
}
