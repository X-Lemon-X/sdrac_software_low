#include "Timing.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"


using namespace TIMING;

uint32_t TIMING::frequency_to_period(uint32_t frequency){
  return 1000000/frequency;
}

Ticker::Ticker(){
  tick_millis = 0;
  tick_micros = 0;
}

void Ticker::irq_update_ticker(){
  tick_millis++;
  tick_micros = tick_millis*1000;
}

void Timing::set_behaviour(uint32_t _period, bool _repeat){
  period = _period;
  repeat = _repeat;
}

uint32_t Ticker::get_micros() {
  __disable_irq();
  uint32_t mic =  (uint32_t)TIM10->CNT + tick_micros;
  __enable_irq();
  return mic;
}

uint32_t Ticker::get_millis() const {
  return tick_millis;
}

float Ticker::get_seconds() {
  return (float)get_micros() * 0.000001f;
}

Timing::Timing(Ticker &_ticker): ticker(_ticker){
  period = 0;
  last_time = ticker.get_micros();
  repeat = true;
}

void Timing::reset(){
  last_time = ticker.get_micros();
}

bool Timing::triggered(){
  uint32_t current_time = ticker.get_micros();
  if (current_time - last_time > period){
    if (repeat) last_time = current_time;
    return true;
  }
  return false;
}