
#include "Timing.hpp"

#ifndef FILTER_HPP
#define FILTER_HPP

namespace FILTERS{

class FilterBase{
protected:
  TIMING::Ticker &ticker;
public:
  FilterBase(TIMING::Ticker &ticker);
  float calculate(float calculate);
};

class FilterAlphaBeta: private FilterBase{
private:
  float ypri;
  float ypost;
  float vpri;
  float vpost;
  float prev_time;
public:
  float alfa;
  float beta;
FilterAlphaBeta(TIMING::Ticker &ticker);
 float calculate(float x) override;
};


  
} // namespace FILTERS


#endif