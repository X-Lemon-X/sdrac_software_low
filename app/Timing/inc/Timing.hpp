#include "stm32f4xx_hal.h"

#ifndef TIMING_HPP
#define TIMING_HPP

namespace TIMING
{

class Ticker{
private:
  uint64_t last_time;
public:
  Ticker();

  /// @brief Update the current ticker 1++ for each call
  void irq_update_ticker(); 

  /// @brief Get current tick of this iteration
  /// @return  current tick in microseconds [us]
  uint64_t get_tick() const;
};
  
class Timing
{
private:
  Ticker &ticker;
  uint64_t period;
  uint64_t last_time;
  bool repeat;
public:
  Timing(Ticker &ticker);
  void set_behaviour(uint64_t period, bool repeat=true);
  bool triggered();
};

} // namespace TIMING


#endif // TIMING_HPP