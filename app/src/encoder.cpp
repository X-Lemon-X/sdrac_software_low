
#include "main.h"
#include "encoder.hpp"



int encoder_init_struct(encoder_encoder *encoder){
  if(encoder == NULL) return 1;
  encoder->resolution = 4096;
  encoder->address = 0x40;
  encoder->hi2c = NULL;
  encoder->reverse = 0;
  encoder->offset = 0;
  encoder->raw_angle = 0;
  encoder->data[0] = 0;
  encoder->data[1] = 0;
  return 0;
}


int encoder_init(encoder_encoder *encoder){
  if(encoder == NULL) return 1;
  return HAL_I2C_IsDeviceReady(encoder->hi2c, encoder->address, 1, 100);
}

int encoder_set_offset(encoder_encoder *encoder, float offset, uint8_t reverse){
  if(encoder == NULL) return 1;
  encoder->offset = offset;
  encoder->reverse = reverse;
  return 0;
}

int encoder_read_raw_angle(encoder_encoder *encoder){
  if(encoder == NULL) return -1;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(encoder->hi2c, encoder->address, encoder->angle_register, 1, encoder->data, 2, 20);
  if(status != HAL_OK) return -2;
  
  uint16_t reg1 = (uint16_t)encoder->data[0] << 6;
  uint16_t reg2 = encoder->data[1] & 0xfe;
  encoder->raw_angle = reg1 + reg2;
  return encoder->raw_angle;
}


float encoder_read_angle(encoder_encoder *encoder){
  if(encoder == NULL) return 1;


  return 0;
}
