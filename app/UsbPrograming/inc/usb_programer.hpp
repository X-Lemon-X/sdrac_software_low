#include <string>
#include "main_prog.hpp"

#ifndef USB_PROGRAMER_H
#define USB_PROGRAMER_H

#define USB_PROGRAMER_REBOOT "SB_reboot"
#define USB_PROGRAMER_PROGRAM "SB_program"
#define USB_PROGRAMER_BUFFER_SIZE 2048

namespace USB_PROGRAMER{

class UsbProgramer
{
private:
  std::string reboot_device_phrase;
  std::string program_device_phrase;
  Pin boot_device;
  uint8_t buffer[USB_PROGRAMER_BUFFER_SIZE];
public:
  UsbProgramer(Pin boot_device);
  
  /// @brief should be called in the main loop to handle the usb programing
  void handler();

  /// @brief resets the stm32 uC
  void reset_device();

  /// @brief restart stm32 device and enters DFU  mode for USB programing
  void program_device();
};
}

#endif // USB_PROGRAMER_H