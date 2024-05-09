#include "main.h"
#include "main_prog.hpp"
#include <deque>
#include "Timing.hpp"
#include "filter.hpp"
// #include <cstdint>

#ifndef ENCODERS_H
#define ENCODERS_H

#define VELOCITY_FILTER_SIZE 6

namespace ENCODER {
  
class Encoder {
private:
  I2C_HandleTypeDef *hi2c;
  TIMING::Ticker *ticker;
  FILTERS::FilterBase *filter_angle;
  FILTERS::FilterBase *filter_velocity;

  uint16_t raw_angle;
  uint8_t data[2];
  float last_time;
  float prev_angle;
  float current_velocity;
  float current_angle;

  uint16_t resolution;
  float offset;
  bool reverse;
  uint8_t address;
  uint8_t angle_register;
  uint8_t magnes_detection_register;
  bool enable_filter;
  bool enable_velocity;
  bool enable_velocity_filter;
  
  /// @brief Calucaltes velcoicty, and passes it thoroung a filter
  /// @param angle current angle
  /// @param current_time  current time
  /// @return returns the filtered velocity
  float calculate_velocity(float angle);

public:
  
  /// @brief Init fucnion of the encoder
  Encoder();
  
  /// @brief Init fucnion of the encoder
  void init(I2C_HandleTypeDef &hi2c,TIMING::Ticker &ticker , FILTERS::FilterBase &filter_angle,FILTERS::FilterBase &filter_velocity);

  /// @brief Pings the encoder to check if it is connected
  /// @return true if the encoder is connected
  bool ping_encoder();

  /// @brief handles the encoder updates data read from the encoder
  void handle();

  /// @brief reads the raw angle from the encoder
  /// @return the raw angle in uint16_t
  uint16_t read_raw_angle();

  /// @brief reads the angle from the encoder
  /// @return the angle in radians
  float read_angle();

  /// @brief reads the last calculated velocity
  /// @return the velocity in radians per second 
  float get_velocity();

  /// @brief gets the last calcualted angle
  /// @return the angle in radians
  float get_angle();

  /// @brief sets the resolution of the encoder
  /// @param resolution the resolution of the encoder 
  void set_resolution(uint16_t resolution){this->resolution = resolution;};
  
  /// @brief sets the offset of the encoder
  /// @param offset the offset in radians
  void set_offset(float offset){this->offset = offset;};
  
  /// @brief sets the reverse of the encoder
  /// @param reverse true if the encoder is reversed
  void set_reverse(bool reverse){this->reverse = reverse;};
  
  /// @brief sets the address of the encoder 
  /// @param address the address of the encoder
  void set_address(uint8_t address){this->address = address;};
  
  /// @brief sets the angle register of the encoder
  /// @param angle_register the angle register of the encoder
  void set_angle_register(uint8_t angle_register){this->angle_register = angle_register;};
  
  /// @brief sets the magnes detection register of the encoder
  /// @param magnes_detection_register the magnes detection register of the encoder 
  void set_magnes_detection_register(uint8_t magnes_detection_register){this->magnes_detection_register = magnes_detection_register;};
  
  /// @brief sets the enable filter of the encoder 
  /// @param enable_filter true if the filter is enabled
  void set_enable_filter(bool enable_filter){this->enable_filter = enable_filter;};
  
  /// @brief sets the enable velocity of the encoder 
  /// @param enable_velocity true if the you wnat the velocity to be calculated
  void set_enable_velocity(bool enable_velocity){this->enable_velocity = enable_velocity;};
  
  /// @brief sets the enable velocity filter of the encoder 
  /// @param enable_velocity_filter true if the velocity filter is enabled
  void set_enable_velocity_filter(bool enable_velocity_filter){this->enable_velocity_filter = enable_velocity_filter;};

};

}


#endif // ENCODERS_H