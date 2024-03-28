
#ifndef LOGER_H
#define LOGER_H

#define LOGER_MAX_MSG_LEN 1024

// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

enum LOG_LEVEL {
  LOG_LEVEL_DEBUG=0,
  LOG_LEVEL_INFO=1,
  LOG_LEVEL_WARNING=2,
  LOG_LEVEL_ERROR=3
};

typedef int log_lvl;

/// @brief  initiate the logger
/// @param level - log level
void log_init(log_lvl level);

/// @brief  deinit the logger
void log_deinit();

void log_error(const char *msg);
void log_warning(const char *msg);
void log_info(const char *msg);
void log_debug(const char *msg);


#endif // LOGER_H