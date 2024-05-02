#include "pin.hpp"
#include "stm32f4xx_hal.h"
#include "main.h"

#ifndef BOARD_ID_HPP
#define BOARD_ID_HPP

namespace BOARD_ID{

class Board_id{
private:
  const Pin &id_0;
  const Pin &id_1;
  const Pin &id_2;
  uint8_t id;
  bool id_set;
public:
  Board_id(const Pin &id_0,const Pin &id_1,const Pin &id_2);
  uint8_t get_id();
};
  
} // namespace BOARD_ID


#endif