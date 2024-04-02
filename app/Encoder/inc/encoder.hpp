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
  I2C_HandleTypeDef &hi2c;
  TIMING::Ticker &ticker;
  uint16_t raw_angle;
  uint8_t data[2];
  std::deque<float> velocity_previous;
  std::deque<float> angle_previous;
  float *velocity_filter_weight;
  uint64_t last_time;
  float prev_angle;
  float velocity;

  /// @brief Calucaltes velcoicty, and passes it thoroung a filter
  /// @param angle current angle
  /// @param current_time  current time
  /// @return returns the filtered velocity
  float calculate_velocity(float angle);

public:
  uint16_t resolution;
  float offset;
  bool reverse;
  uint8_t address;
  uint8_t angle_register;
  uint8_t magnes_detection_register;
  bool enable_filter;
  bool enable_velocity;
  
  Encoder(I2C_HandleTypeDef &hi2c,TIMING::Ticker &ticker ,float *velocity_filter_weight = (float*)basic_velocity_filter_weight);
  
  /// @brief Pings the encoder to check if it is connected
  /// @return true if the encoder is connected
  bool ping_encoder();

  /// @brief reads the raw angle from the encoder
  /// @return the raw angle in uint16_t
  uint16_t read_raw_angle();

  /// @brief reads the angle from the encoder
  /// @return the angle in radians
  float read_angle();

  /// @brief reads the velocity from the encoder and filters it
  /// @return the velocity in radians per second 
  float get_velocity();
};

}


#endif // ENCODERS_H