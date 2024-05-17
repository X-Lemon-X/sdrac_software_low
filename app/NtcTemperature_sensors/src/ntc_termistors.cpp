
#include "ntc_termistors.hpp"
#include "main.h"
#include <cmath>

using namespace NTCTERMISTORS;

float NTCTERMISTORS::get_temperature(uint16_t adc_value) {
  // float voltage = (float)(adc_value / 4095) * termistor_supply_voltage;
  float ntc_resistance = termistor_divider_resisitor * adc_value / ( 4095 - adc_value);
  float logR = log(ntc_resistance);
  float T = 1.0 / (NTC_TERMISTOR_C1 +  NTC_TERMISTOR_C2*logR + NTC_TERMISTOR_C3*logR*logR*logR);
  return T - 273.15;
}