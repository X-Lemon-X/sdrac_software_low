#include "logger.h"
#include "main.h"
#include "usbd_cdc_if.h"

log_lvl log_level;
char *logbuffer = NULL; 

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


void log_transmit(const char *msg){
  size_t len = strlen(msg);
  CDC_Transmit_FS((uint8_t*)msg, len);
  // memset(logbuffer, 0, len);
}

void log_init(log_lvl level){
  log_level = level;
  logbuffer = (char*)malloc(LOGER_MAX_MSG_LEN);
  memset(logbuffer, 0, LOGER_MAX_MSG_LEN);
}

void log_deinit(){
  free(logbuffer);
}

void log_debug(const char *msg){
  if (log_level < LOG_LEVEL_DEBUG) return;
  log_transmit(msg);
}

void log_info(const char *msg){
  if (log_level < LOG_LEVEL_INFO) return;
  log_transmit(msg);
}

void log_warning(const char *msg){
  if (log_level < LOG_LEVEL_WARNING) return;
  log_transmit(msg);
}

void log_error(const char *msg){
  if (log_level < LOG_LEVEL_ERROR) return;
  log_transmit(msg);
}
