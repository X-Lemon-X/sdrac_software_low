#include "stm32f4xx_hal.h"
#include "pin.hpp"
#include "main.h"
#include "CanDB.h"  
#include "Timing.hpp"
#include <list>

#ifndef CAN_CONTROL_HPP
#define CAN_CONTROL_HPP

#define CAN_DATA_FRAME_MAX_SIZE 8
#define CAN_QUEUE_SIZE 128
#define CAN_LED_BLINK_PERIOD_US 500

namespace CAN_CONTROL {

struct CAN_MSG{
  uint32_t frame_id;
  bool remote_request;
  uint8_t data[CAN_DATA_FRAME_MAX_SIZE];
  uint8_t data_size;
};

class CanControl{
private:
  CAN_HandleTypeDef *can_interface;
  uint32_t can_fifo;
  TIMING::Ticker *ticker;
  TIMING::Timing *timing_led_rx;
  TIMING::Timing *timing_led_tx;
  uint8_t data[CAN_DATA_FRAME_MAX_SIZE];
  CAN_RxHeaderTypeDef header = {0};
  std::list<CAN_MSG> rx_msg_buffer;
  const Pin *pin_tx_led;
  const Pin *pin_rx_led;

  /// @brief  turn on the TX led for a short period
 void blink_tx_led();

  /// @brief  turn on the RX led for a short period
  void blink_rx_led();

  /// @brief  push a message to the RX buffer
  /// @param msg  CAN_MSG to be pushed to the buffer
  void push_to_queue(CAN_MSG &msg);
public:
  
  /// @brief  Construct a new Can Control object
  CanControl();

  /// @brief  init the CanControl object
  /// @param can_interface  CAN interface
  /// @param can_fifo  CAN fifo number
  /// @param ticker  main system ticker object
  /// @param pin_tx_led  pin object for the TX led
  /// @param pin_rx_led  pin object for the RX led
  void init(
    CAN_HandleTypeDef &can_interface, 
    uint32_t can_fifo,
    TIMING::Ticker &ticker,
    const Pin &pin_tx_led,
    const Pin &pin_rx_led);


  /// @brief  Handle the RX interrupt
  void irq_handle_rx();

  /// @brief  Handle the TX interrupt
  void irq_handle_tx();

  /// @brief  Handle the LED blink this function should be called in the main loop
  void handle_led_blink();

  /// @brief  Send a message over CAN bus
  void send_message(CAN_MSG &&msg);
  
  /// @brief  Get the message from the RX buffer
  /// @param msg  pointer to the CAN_MSG object
  /// @return 0 if the message was received, 1 if the buffer is empty
  uint8_t get_message(CAN_MSG *msg);
};

} // namespace CAN_CONTROL

#endif // CAN_CONTROL_HPP