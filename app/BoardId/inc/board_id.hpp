#include "pin.hpp"
#include "stm32f4xx_hal.h"
#include "main.h"

#ifndef BOARD_ID_HPP
#define BOARD_ID_HPP


/// @brief if BOARD_ID_OWERWRITE is defined, then BOARD_ID_OWERWRITE_ID will be used as board id  
// #define BOARD_ID_OWERWRITE
// #define BOARD_ID_OWERWRITE_ID 2




namespace BOARD_ID{

#define BOARD_ID_0 0x0
#define BOARD_ID_1 0x1
#define BOARD_ID_2 0x2
#define BOARD_ID_3 0x3
#define BOARD_ID_4 0x4
#define BOARD_ID_5 0x5
#define BOARD_ID_6 0x6
#define BOARD_ID_7 0x7


class Board_id{
private:
  const GpioPin &id_0;
  const GpioPin &id_1;
  const GpioPin &id_2;
  uint8_t id;
  bool id_set;
public:
  Board_id(const GpioPin &id_0,const GpioPin &id_1,const GpioPin &id_2);
  uint8_t get_id();
};
  
} // namespace BOARD_ID


#endif