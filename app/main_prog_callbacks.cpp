#include "main.h"
#include "stm32f4xx_hal.h"
#include "config.hpp"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10){
    main_clock.irq_update_ticker();
  }
  
  // if (htim->Instance == TIM3)
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance == CAN1){
    can_controler.irq_handle_rx();
  }
  
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance == CAN1){
    can_controler.irq_handle_rx();
  }
}

// void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
// {
//   if(hcan->Instance == CAN1){
//     can_controler.irq_handle_tx();
//   }
// }

// void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
// {
//   if(hcan->Instance == CAN1){
//     can_controler.irq_handle_tx();
//   }
// }

// void HAL_ADC_HalfConvCpltCallback(ADC_HandleTypeDef* hadc)
// {
//   if(hadc->Instance == ADC1){
//     pin_temp_steper_board.analog_value = adc_dma_buffer[0];
//     pin_temp_board.analog_value = adc_dma_buffer[1];
//     pin_temp_motor.analog_value = adc_dma_buffer[2];
//     pin_vsense.analog_value = adc_dma_buffer[3];
//     pin_poz_zero_sensor.analog_value = adc_dma_buffer[4];
//     pin_inout_ca1.analog_value = adc_dma_buffer[5];
//     pin_inout_ca2.analog_value = adc_dma_buffer[6];
//     pin_inout_crx.analog_value = adc_dma_buffer[7];
//   }
//   // log_debug("ADC1: " + std::to_string(adc_dma_buffer[0]));
// }

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1){
    pin_temp_steper_board.analog_value = adc_dma_buffer[0];
    pin_temp_board.analog_value = adc_dma_buffer[1];
    pin_temp_motor.analog_value = adc_dma_buffer[2];
    pin_vsense.analog_value = adc_dma_buffer[3];
    // pin_poz_zero_sensor.analog_value = adc_dma_buffer[4];
    // pin_inout_ca1.analog_value = adc_dma_buffer[5];
    // pin_inout_ca2.analog_value = adc_dma_buffer[6];
    // pin_inout_crx.analog_value = adc_dma_buffer[7];
  }
  // log_debug("ADC1: " + std::to_string(adc_dma_buffer[0]));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1){
    // pin_temp_steper_board.analog_value = adc_dma_buffer[0];
    // pin_temp_board.analog_value = adc_dma_buffer[1];
    // pin_temp_motor.analog_value = adc_dma_buffer[2];
    // pin_vsense.analog_value = adc_dma_buffer[3];
    pin_poz_zero_sensor.analog_value = adc_dma_buffer[4];
    pin_inout_ca1.analog_value = adc_dma_buffer[5];
    pin_inout_ca2.analog_value = adc_dma_buffer[6];
    pin_inout_crx.analog_value = adc_dma_buffer[7];
  }
  // log_debug("ADC1: " + std::to_string(adc_dma_buffer[0]));
}
