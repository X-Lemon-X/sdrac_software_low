#include "main_prog.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"


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


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // log_debug("ADC1: " + std::to_string(adc_dma_buffer[0]));
}
