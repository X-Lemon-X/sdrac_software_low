#include "can.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "config.hpp"
#include "can_control.hpp"
#include "main_prog.hpp"

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

void can_callback_set_pos(stmepic::can_msg &recived_msg){
  can_disconnect_timeout_reset();
  can_konarm_1_set_pos_t signals;
  (void)can_konarm_1_set_pos_unpack(&signals, recived_msg.data, recived_msg.data_size);
  // float targetPosition = can_konarm_1_set_pos_position_decode(signals.position);
  // float targetVelocity = can_konarm_1_set_pos_velocity_decode(signals.velocity);
  movement_controler.set_velocity(signals.position);
  movement_controler.set_position(signals.velocity);
  movement_controler.set_enable(true); 
}

void can_callback_get_pos(stmepic::can_msg &recived_msg){
  can_disconnect_timeout_reset();
  stmepic::can_msg send_msg;
  can_konarm_1_get_pos_t src_p;
  send_msg.frame_id = config.can_konarm_get_pos_frame_id;
  src_p.position = movement_controler.get_current_position();
  src_p.velocity = movement_controler.get_current_velocity();
  send_msg.data_size = CAN_KONARM_1_GET_POS_LENGTH;
  (void)can_konarm_1_get_pos_pack(send_msg.data, &src_p, send_msg.data_size);
  can_controler.send_can_msg_to_queue(send_msg);
}

void can_callback_status(stmepic::can_msg &recived_msg){
  can_disconnect_timeout_reset();
  stmepic::can_msg send_msg;
  can_konarm_1_status_t src_p;
  send_msg.frame_id = config.can_konarm_status_frame_id;
  src_p.status = can_konarm_1_status_status_encode(CAN_KONARM_1_STATUS_STATUS_OK_CHOICE);
  send_msg.data_size = CAN_KONARM_1_STATUS_LENGTH;
  can_konarm_1_status_pack(send_msg.data, &src_p, send_msg.data_size);
  can_controler.send_can_msg_to_queue(send_msg);
}

void can_callback_clear_errors(stmepic::can_msg &recived_msg){
  can_disconnect_timeout_reset();
}

void can_callback_get_errors(stmepic::can_msg &recived_msg){
  can_disconnect_timeout_reset();
  stmepic::can_msg send_msg;
  can_konarm_1_get_errors_t src_p;
  send_msg.frame_id = config.can_konarm_get_errors_frame_id;
  src_p.temp_engine_overheating = error_data.temp_engine_overheating;
  src_p.temp_driver_overheating = error_data.temp_driver_overheating;
  src_p.temp_board_overheating = error_data.temp_board_overheating;
  src_p.temp_engine_sensor_disconnect = error_data.temp_engine_sensor_disconnect;
  src_p.temp_driver_sensor_disconnect = error_data.temp_driver_sensor_disconnect;
  src_p.temp_board_sensor_disconnect = error_data.temp_board_sensor_disconnect;
  src_p.encoder_arm_disconnect = error_data.encoder_arm_disconnect;
  src_p.encoder_motor_disconnect = error_data.encoder_motor_disconnect;
  src_p.board_overvoltage = error_data.baord_overvoltage;
  src_p.board_undervoltage = error_data.baord_undervoltage;
  src_p.can_disconnected = error_data.can_disconnected;
  src_p.can_error = error_data.can_error;
  src_p.controler_motor_limit_position = error_data.controler_motor_limit_position;
  send_msg.data_size = CAN_KONARM_1_GET_ERRORS_LENGTH;
  can_konarm_1_get_errors_pack(send_msg.data, &src_p, send_msg.data_size);
  can_controler.send_can_msg_to_queue(send_msg);
  error_data.can_error = false;
}

void can_callback_default(stmepic::can_msg &recived_msg){
  can_disconnect_timeout_reset();
  error_data.can_error = true;
}

void can_callback_set_control_mode(stmepic::can_msg &recived_msg){
  can_disconnect_timeout_reset();
  can_konarm_1_set_control_mode_t signals;
  can_konarm_1_set_control_mode_unpack(&signals, recived_msg.data, recived_msg.data_size);
  if(!can_konarm_1_set_control_mode_control_mode_is_in_range(signals.control_mode)){
    error_data.can_error = true;
    log_error("Control mode is out of range");
    return;
  }
  init_and_set_movement_controler_mode(signals.control_mode);
}