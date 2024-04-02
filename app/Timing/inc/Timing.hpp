#include "stm32f4xx_hal.h"

#ifndef TIMING_HPP
#define TIMING_HPP

namespace TIMING
{

class Ticker{
private:
  uint64_t tick_millis;
  uint64_t tick_micros;
public:

  /// @brief Construct a new Ticker object
  Ticker();

  /// @brief Update the current ticker 1++ for each call
  void irq_update_ticker(); 

  // void update_ticker_loop();

  /// @brief Get current time in microseconds
  /// @return  current time in microseconds [us]
  uint64_t get_micros();

  /// @brief get time in milliseconds
  /// @return current time in milliseconds [ms]
  uint64_t get_millis() const;

  /// @brief  get time in seconds with microsecond resolution
  /// @return  current time in seconds [s]
  float get_seconds();
};
  
class Timing
{
private:
  Ticker &ticker;
  uint64_t period;
  uint64_t last_time;
  bool repeat;
public:
  /// @brief Construct a new Timing object
  /// @param ticker reference to the ticker object with us resolution
  Timing(Ticker &ticker);

  /// @brief Set the behaviour of the timer
  /// @param period period of the timer in microseconds [us]
  /// @param repeat if the timer should repeat
  void set_behaviour(uint64_t period, bool repeat=true);

  /// @brief Check if the timer has triggered
  /// @return true if the timer has triggered
  bool triggered();
};

} // namespace TIMING


#endif // TIMING_HPP