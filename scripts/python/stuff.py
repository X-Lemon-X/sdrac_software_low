

core_frequency = 96e6
prescaler = 32
timer_frequency = core_frequency / prescaler

PI = 3.14159265359


velocity = 0.03
velocity2 = 2
steps_per_revolution = 200*32
gear_ratio = 1

frequency =  steps_per_revolution * gear_ratio  / (2*PI)   * velocity
period = 1 / frequency
single_tick_period = 1 / timer_frequency
ticks_per_period = period / single_tick_period

# optimaized version of the algorytm to avodi recalculating the same values
ticks_per_period = timer_frequency / frequency

const_value = ( core_frequency / prescaler )/( steps_per_revolution * gear_ratio  / (2*PI))
ticks_per_period2 = lambda x: const_value / x


print(f"Frequency: {frequency}")
print(f"Period: {period}")
print(f"Single tick period: {single_tick_period}")
print(f"Ticks per period: {ticks_per_period}")
print(f"Ticks for vel:{str(velocity).ljust(10)} ticks:{ticks_per_period2(velocity)}")
print(f"Ticks for vel:{str(velocity2).ljust(10)} ticks:{ticks_per_period2(velocity2)}")
print(f"Tick time: {ticks_per_period * single_tick_period}")



# calcualet timer prescaler to get 1ms tick and counte max value should be equal to 1000
# 1ms = 1e-3
core_frequency = 96e6
counte_max_value = 1000
prescaler = core_frequency / counte_max_value / 1000
print(f"Prescaler: {prescaler}") 






