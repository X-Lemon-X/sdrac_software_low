#include "logger.hpp"

#ifndef MAIN_PROG_H
#define MAIN_PROG_H

#define DEBUG
#define PI 3.14159265358979323846
#define PI_d2 1.57079632679489661923
#define PI_d4 0.78539816339744830962
#define PI_m2 6.28318530717958647692

#define true 1
#define false 0


#define ENCODER_MEM_ADDR_ANNGLE 0x03
#define ENCODER_MT6701_I2C_ADDRESS 0b0000110 << 1
#define ENCODER_MT6702_RESOLUTION 16384

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart3;
extern LOGGER::Logger loger;

int main_prog();

#endif // MAIN_PROG_H