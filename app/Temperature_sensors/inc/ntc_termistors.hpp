#include "main.h"

#ifndef __NTC_TERMISTORS_HPP
#define __NTC_TERMISTORS_HPP

#define NTC_TERMISTOR_C1 1.009249522e-03f
#define NTC_TERMISTOR_C2 2.378405444e-04f
#define NTC_TERMISTOR_C3 2.019202697e-07f
#define NTC_TERMISTOR_MIN_TEMPERATURE -40.0f
#define NTC_TERMISTOR_MAX_TEMPERATURE 150.0f

namespace NTCTERMISTORS{

static float termistor_supply_voltage = 3.3f;
static float termistor_divider_resisitor = 100000.0f;

  
float get_temperature(float voltage_value);

} // namespace NTCTERMISOTRS


#endif