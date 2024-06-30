#include "logger.hpp"
#include "main.h"
#include "usbd_cdc_if.h"
#include "main_prog.hpp"

using namespace LOGGER;


Logger::Logger(LOG_LEVEL level, bool _print_info): log_level(level),print_info(_print_info) {}

void Logger::error(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_ERROR) return;
  transmit(msg,"ERROR");
}

void Logger::warning(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_WARNING ) return;
  transmit(msg,"WARNING");
}

void Logger::info(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_INFO) return;
  transmit(msg,"INFO");
}

void Logger::debug(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_DEBUG) return;
  transmit(msg,"DEBUG");
}


void Logger::transmit(std::string msg,std::string prefix){
  if(print_info){
    msg = 
        "{\"time\":\""+std::to_string(HAL_GetTick())+ 
        "\",\"level\":\""+prefix+
        "\",\"ver\":\"" + std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR)+
        "\",\"id\":\"" + std::to_string(board_id.get_id())+
        "\",\"msg\":{" + msg + "\"}}\n";
  }else{
    msg += "\n";
  }
  CDC_Transmit_FS((uint8_t*)msg.c_str(), msg.length());
  // HAL_Delay(1);
}
