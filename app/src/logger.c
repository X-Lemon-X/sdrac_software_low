#include "logger.h"
#include "main.h"
#include "usbd_cdc_if.h"

log_lvl log_level;

void log_debug(const char *msg){
  if (log_level <= LOG_LEVEL_DEBUG) return;
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void log_info(const char *msg){
  if (log_level <= LOG_LEVEL_INFO) return;
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void log_warning(const char *msg){
  if (log_level <= LOG_LEVEL_WARNING) return;
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void log_error(const char *msg){
  if (log_level <= LOG_LEVEL_ERROR) return;
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}
