#include "board_id.hpp"

using namespace BOARD_ID;

Board_id::Board_id(const Pin &_id_0,const Pin &_id_1,const Pin &_id_2): id_0(_id_0), id_1(_id_1), id_2(_id_2){
  this->id_set = false;
}

uint8_t Board_id::get_id(){
  return 1;
  
  if (id_set) return id;
  id = 0;
  id |= (uint8_t)HAL_GPIO_ReadPin(id_0.port,id_0.pin);
  id |= (uint8_t)HAL_GPIO_ReadPin(id_1.port,id_1.pin) << 1;
  id |= (uint8_t)HAL_GPIO_ReadPin(id_2.port,id_2.pin) << 2;
  id_set = true;
  return id;
}