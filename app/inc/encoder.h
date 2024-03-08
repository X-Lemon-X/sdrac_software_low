#include "main.h"
// #include <cstdint>

#ifndef ENCODERS_H
#define ENCODERS_H

#define ENCODER_MT6701_I2C_ADDRESS 0x06
#define ENCODER_MT6702_RESOLUTION 16384


typedef struct {
  uint16_t resolution;
  uint8_t address;
  I2C_HandleTypeDef *hi2c1;

  
  uint8_t reverse;
  float offset;
  uint16_t raw_angle;
  uint8_t data[2];
}encoder_encoder;


/// @brief  initiate the encoder struct with default values
/// @param encoder pointer to the encoder object
/// @return 
int encoder_init_struct(encoder_encoder *encoder);

/// @brief initiate the eoncoder with the appropriate settings
/// @param encoder - pointer to the new encoder object
/// @param address - I2C address of the encoder
/// @return 
int encoder_init(encoder_encoder *encoder, uint8_t address, I2C_HandleTypeDef *hi2c1, uint16_t resolution);

/// @brief set offset for the encoder
/// @param encoder - pointer to the encoder object
/// @param offset - offset to set in radians
/// @return 
int encoder_set_offset(encoder_encoder *encoder, float offset, uint8_t reverse);


/// @brief read the raw angle from the encoder
/// @param encoder - pointer to the encoder object
/// @return angle as integer without modifications
uint16_t encoder_read_raw_angle(encoder_encoder *encoder);

/// @brief  read the angle from the encoder
/// @param encoder pointer to the encoder object
/// @return angle in radians with all modifications
float encoder_read_angle(encoder_encoder *encoder);


#endif // ENCODERS_H