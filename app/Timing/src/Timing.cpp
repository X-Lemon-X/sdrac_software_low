#include "Timing.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"


using namespace TIMING;

Ticker::Ticker(){
  tick_millis = 0;
  tick_micros = 0;
}

void Ticker::irq_update_ticker(){
  tick_millis++;
}

void Timing::set_behaviour(uint64_t _period, bool _repeat){
  period = _period;
  repeat = _repeat;
}

uint64_t Ticker::get_micros() {
  __disable_irq();
  tick_micros = (uint64_t)TIM10->CNT + ((uint64_t)tick_millis)*1000;
  __enable_irq();
  return tick_micros;
}

uint64_t Ticker::get_millis() const {
  return tick_millis;
}

float Ticker::get_seconds() {
  return (float)get_micros() / 1000000.0f;
}

Timing::Timing(Ticker &_ticker): ticker(_ticker){
  period = 0;
  last_time = ticker.get_micros();
  repeat = true;
}

bool Timing::triggered(){
  uint64_t current_time = ticker.get_micros();
  if (current_time - last_time > period){
    if (repeat) last_time = current_time;
    return true;
  }
  return false;
}