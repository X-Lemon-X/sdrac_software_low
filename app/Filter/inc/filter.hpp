
#include "Timing.hpp"

#ifndef FILTER_HPP
#define FILTER_HPP

namespace FILTERS{

class FilterAlphaBeta{
private:
  TIMING::Ticker &ticker;
  float ypri;
  float ypost;
  float vpri;
  float vpost;
  float prev_time;
public:
  float alfa;
  float beta;
FilterAlphaBeta(TIMING::Ticker &ticker);
void calculate(float x);
};


  
} // namespace FILTERS


#endif