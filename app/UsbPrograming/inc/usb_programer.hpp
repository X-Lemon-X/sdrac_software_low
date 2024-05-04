#include <string>
#include "main_prog.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#ifndef USB_PROGRAMER_H
#define USB_PROGRAMER_H

#define USB_PROGRAMER_REBOOT "SB_reboot"
#define USB_PROGRAMER_PROGRAM "SB_program"
#define USB_PROGRAMER_BUFFER_SIZE APP_RX_DATA_SIZE

namespace USB_PROGRAMER{



class UsbProgramer
{
private:
  std::string reboot_device_phrase;
  std::string enter_dfu_mode_phrase;
  const GPIO_PIN &boot_device;
  uint8_t buffer[USB_PROGRAMER_BUFFER_SIZE];
public:
  UsbProgramer(const GPIO_PIN &boot_device);
  
  /// @brief should be called in the main loop to handle the usb programing
  void handler();

  /// @brief resets the stm32 uC
  void reset_device();

  /// @brief restart stm32 device and enters DFU  mode for USB programing
  void enter_dfu_mode();
};
}

#endif // USB_PROGRAMER_H