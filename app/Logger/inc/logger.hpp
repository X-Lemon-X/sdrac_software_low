#include <string>

#ifndef LOGER_H
#define LOGER_H

// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

namespace LOGGER {

enum class LOG_LEVEL {
  LOG_LEVEL_DEBUG=0,
  LOG_LEVEL_INFO=1,
  LOG_LEVEL_WARNING=2,
  LOG_LEVEL_ERROR=3
};

class Logger {
private:
  LOG_LEVEL log_level;
  bool print_info;
  void transmit(std::string msg, std::string prefix);
public:
  /// @brief  initiate the logger
  /// @param level - log level
  Logger(LOG_LEVEL level, bool print_info);

  void error(std::string msg);
  void warning(std::string msg);
  void info(std::string msg);
  void debug(std::string msg);
};

}
#endif // LOGER_H