
#ifndef LOGER_H
#define LOGER_H

enum LOG_LEVEL {
  LOG_LEVEL_DEBUG=0,
  LOG_LEVEL_INFO=1,
  LOG_LEVEL_WARNING=2,
  LOG_LEVEL_ERROR=3
};

typedef int log_lvl;

log_lvl log_level;
void log_error(const char *msg);
void log_warning(const char *msg);
void log_info(const char *msg);
void log_debug(const char *msg);


#endif // LOGER_H