#include "main.h"
#include "main_prog.hpp"
#include <deque>
#include "Timing.hpp"
// #include <cstdint>

#ifndef ENCODERS_H
#define ENCODERS_H

#define VELOCITY_FILTER_SIZE 6

namespace ENCODER {

const float basic_velocity_filter_weight[VELOCITY_FILTER_SIZE] = {
  0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  
class Encoder {
private:
  I2C_HandleTypeDef *hi2c;
  uint16_t raw_angle;
  uint8_t data[2];
  std::deque<float> velocity_previous;
  float *velocity_filter_weight;
  float last_time;
  float prev_angle;
  float velocity;

  /// @brief Calucaltes velcoicty, and passes it thoroung a filter
  /// @param angle current angle
  /// @param current_time  current time
  /// @return returns the filtered velocity
  float calculate_velocity(float angle, float current_time);

public:
  uint16_t resolution;
  float offset;
  bool reverse;
  uint8_t address;
  uint8_t angle_register;
  uint8_t magnes_detection_register;
  
  Encoder(I2C_HandleTypeDef *hi2c, float *velocity_filter_weight = (float*)basic_velocity_filter_weight);
  bool ping_encoder();
  void set_offset(float offset, bool reverse);
  int read_raw_angle();
  float read_angle();
  float get_velocity();
};

}


#endif // ENCODERS_H