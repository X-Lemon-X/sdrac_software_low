
#include "main.h"
#include "encoder.h"

#define ENCODER_MEM_ADDR_ANNGLE 0x3FFF


int encoder_init_struct(encoder_encoder *encoder){
  if(encoder == NULL) return 1;
  encoder->resolution = 4096;
  encoder->address = 0x40;
  encoder->hi2c1 = NULL;
  encoder->reverse = 0;
  encoder->offset = 0;
  encoder->raw_angle = 0;
  encoder->data[0] = 0;
  encoder->data[1] = 0;
  return 0;
}


int encoder_init(encoder_encoder *encoder, uint8_t address, I2C_HandleTypeDef *hi2c1){
  if(encoder == NULL) return 1;
  return 0;
}

int encoder_set_offset(encoder_encoder *encoder, float offset, uint8_t reverse){
  if(encoder == NULL) return 1;
  encoder->offset = offset;
  encoder->reverse = reverse;
  return 0;
}

uint16_t encoder_read_raw_angle(encoder_encoder *encoder){
  if(encoder == NULL) return 1;
  return 0;
}


float encoder_read_angle(encoder_encoder *encoder){
  if(encoder == NULL) return 1;


  return 0;
}
