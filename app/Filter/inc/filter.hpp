
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


  
} // namespace FILTERS


#endif