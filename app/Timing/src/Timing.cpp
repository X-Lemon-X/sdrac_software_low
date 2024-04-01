#include "Timing.hpp"
#include "main.h"


using namespace TIMING;

Ticker::Ticker(){
  irq_update_ticker();
}

void Ticker::irq_update_ticker(){
  last_time++;
}

void Timing::set_behaviour(uint64_t _period, bool _repeat){
  period = _period;
  repeat = _repeat;
}

uint64_t Ticker::get_tick() const{
  return last_time;
}

Timing::Timing(Ticker &_ticker): ticker(_ticker){
  period = 0;
  last_time = ticker.get_tick();
  repeat = true;
}

bool Timing::triggered(){
  if (ticker.get_tick() - last_time > period){
    if (repeat) last_time = ticker.get_tick();
    return true;
  }
  return false;
}