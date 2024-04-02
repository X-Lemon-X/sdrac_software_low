#include "logger.hpp"
#include "main.h"
#include "usbd_cdc_if.h"

using namespace LOGGER;

// void log_test (const char *__restrict logbuffer const char *__restrict msg, ...){
//   if (log_level <= LOG_LEVEL_DEBUG) return;
//   va_list args;
//   if (logbuffer == NULL)
//     logbuffer = (char*)malloc(LOGER_MAX_MSG_LEN*sizeof(char));
  
//   va_start(args, msg);
//   vsprintf(logbuffer, msg, args);
//   va_end(args);
//   CDC_Transmit_FS((uint8_t*)logbuffer, LOGER_MAX_MSG_LEN*sizeof(char));
// }

// PUTCHAR_PROTOTYPE
// {
//   /* Place your implementation of fputc here */
//   /* e.g. write a character to the USART1 and Loop until the end of transmission */
//   // HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
//   CDC_Transmit_FS((uint8_t*)&ch, 1);
//   return ch;
// }

Logger::Logger(LOG_LEVEL level){
  log_level = level;
}

void Logger::error(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_ERROR) return;
  transmit("[ERROR]"+msg);
}

void Logger::warning(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_WARNING ) return;
  transmit("[WARNING]"+msg);
}

void Logger::info(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_INFO) return;
  transmit("[INFO]"+msg);
}

void Logger::debug(std::string msg){
  if (log_level > LOG_LEVEL::LOG_LEVEL_DEBUG) return;
  transmit("[DEBUG]"+msg);
}


void Logger::transmit(std::string msg){
  msg += "\n";
  msg = "["+std::to_string(HAL_GetTick())+"]"+ msg;
  CDC_Transmit_FS((uint8_t*)msg.c_str(), msg.length());

  HAL_Delay(1);
}
