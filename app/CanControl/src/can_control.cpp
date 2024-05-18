
#include "can_control.hpp"
#include "CanDB.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "main_prog.hpp"
#include <string.h>
#include "stdlib.h"
#include "list.hpp"


using namespace CAN_CONTROL;

CanControl::CanControl(){}

CanControl::~CanControl(){
  delete timing_led_rx;
  delete timing_led_tx;
}

void CanControl::init(CAN_HandleTypeDef &_can_interface, uint32_t _can_fifo,TIMING::Ticker &_ticker,const GPIO_PIN &_pin_tx_led,const GPIO_PIN &_pin_rx_led){
  can_interface = &_can_interface;
  can_fifo = _can_fifo;
  ticker = &_ticker;
  pin_tx_led = &_pin_tx_led;
  pin_rx_led = &_pin_rx_led;
  timing_led_rx = new TIMING::Timing(*ticker);
  timing_led_tx = new TIMING::Timing(*ticker);
  
  // rx_msg_buffer.resize(CAN_QUEUE_SIZE+1);
  timing_led_rx->set_behaviour(CAN_LED_BLINK_PERIOD_US,false);
  timing_led_tx->set_behaviour(CAN_LED_BLINK_PERIOD_US,false);
}

void CanControl::push_to_queue(CAN_MSG *msg){
  if(rx_msg_buffer.size() == CAN_QUEUE_SIZE){
    delete msg;
    return;
  }
  rx_msg_buffer.push_back(msg);

  // if(rx_msg_buffer_2.size() == CAN_QUEUE_SIZE){
  //   free(msg);
  //   return;
  // }
  // rx_msg_buffer_2.push_back(msg);

}

void CanControl::blink_tx_led(){
  WRITE_GPIO((*pin_tx_led),GPIO_PIN_SET);
  timing_led_tx->reset();
}

void CanControl::blink_rx_led(){
  WRITE_GPIO((*pin_rx_led),GPIO_PIN_SET);
  timing_led_rx->reset();
}

void CanControl::irq_handle_rx(){
  blink_rx_led();
  if (HAL_CAN_GetRxMessage(can_interface, can_fifo, &header, data) != HAL_OK)
    return;
  // CAN_MSG *msg = (CAN_MSG*)malloc(sizeof(CAN_MSG));
  CAN_MSG *msg = new CAN_MSG;
  if(msg == nullptr)
    return;
  msg->frame_id = header.StdId;
  msg->remote_request = header.RTR == CAN_RTR_REMOTE;
  msg->data_size = header.DLC;
  memcpy(msg->data,data,msg->data_size);
  // free(data);
  push_to_queue(msg);
}

void CanControl::irq_handle_tx(){
  __NOP();
}

void CanControl::handle(){
  if(this->timing_led_rx->triggered())
    WRITE_GPIO((*pin_rx_led),GPIO_PIN_RESET);
  
  if(this->timing_led_tx->triggered())
    WRITE_GPIO((*pin_tx_led),GPIO_PIN_RESET);
}

void CanControl::send_message(CAN_MSG &msg){

  while(HAL_CAN_GetTxMailboxesFreeLevel(can_interface)==0){__NOP();}

  CAN_TxHeaderTypeDef tx_header;
  tx_header.StdId = msg.frame_id;
  tx_header.DLC = msg.data_size;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.TransmitGlobalTime = DISABLE;
  HAL_StatusTypeDef status =HAL_CAN_AddTxMessage(can_interface,&tx_header,msg.data,&last_tx_mailbox);
  if(status != HAL_OK)
    log_debug("CAN TX error");
  blink_tx_led();
}

int CanControl::get_message(CAN_MSG **msg){
  // if(rx_msg_buffer_2.size() == 0)
  //   return 0;
  // CAN_MSG *ms = rx_msg_buffer_2.get_front();
  // *msg = ms;
  // rx_msg_buffer_2.pop_front();
  // return rx_msg_buffer_2.size();
  
  if(rx_msg_buffer.empty())
    return 1;
  if(msg == nullptr)
    return 2;
  CAN_MSG *ms = rx_msg_buffer.front();
  // *msg = rx_msg_buffer.front();
  *msg = ms;
  rx_msg_buffer.pop_front();
  return 0;
}