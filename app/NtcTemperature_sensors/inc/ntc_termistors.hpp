#include "main.h"

#ifndef NTC_TERMISTORS_HPP
#define NTC_TERMISTORS_HPP

#define  NTC_TERMISTOR_C1 1.009249522e-03
#define  NTC_TERMISTOR_C2 2.378405444e-04
#define  NTC_TERMISTOR_C3 2.019202697e-07

namespace NTCTERMISTORS{

static float termistor_supply_voltage = 3.3;
static float termistor_divider_resisitor = 100000;
static float termistor_default_resistance = 100000;

  
  float get_temperature(uint16_t adc_value);

} // namespace NTCTERMISOTRS


#endif