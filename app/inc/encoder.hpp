#include "main.h"
#include "main_prog.hpp"
// #include <cstdint>

#ifndef ENCODERS_H
#define ENCODERS_H

namespace ENCODER {
  
class Encoder {
private:
  I2C_HandleTypeDef *hi2c;
  uint16_t raw_angle;
  uint8_t data[2];
public:
  uint16_t resolution;
  float offset;
  bool reverse;
  uint8_t address;
  uint8_t angle_register;
  uint8_t magnes_detection_register;
  
  Encoder(I2C_HandleTypeDef *hi2c);
  bool ping_encoder();
  void set_offset(float offset, bool reverse);
  int read_raw_angle();
  float read_angle();
};

}


#endif // ENCODERS_H