
#include "usb_programer.hpp"
#include "main_prog.hpp"
// #include "usb_device.h"
#include "usbd_cdc_if.h"
// #include "usbd_def.h"
#include <string>

using namespace USB_PROGRAMER;

uint8_t usb_programer_buffer[USB_PROGRAMER_BUFFER_SIZE];
uint32_t usb_programer_buffer_len = 0;
uint8_t usb_programer_data_recived = 0;

UsbProgramer::UsbProgramer(const GPIO_PIN &_boot_device): boot_device(_boot_device){
}

void UsbProgramer::reset_device(){
  HAL_NVIC_SystemReset();
}

void UsbProgramer::enter_dfu_mode(){

  WRITE_GPIO(this->boot_device, GPIO_PIN_SET);
  HAL_Delay(50);
  reset_device();
  WRITE_GPIO(this->boot_device,GPIO_PIN_RESET);
  while (true){}
}

void UsbProgramer::handler(){
  uint32_t size =0;
  // add thsi static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);
  // to a file USB_DEVICE/App/usbd_cdc_if.h
  // CDC_Receive_FS(buffer, &size);
  if(usb_programer_data_recived == 0) return;
  usb_programer_data_recived = 0;
  size = usb_programer_buffer_len;
  usb_programer_buffer_len = 0;
  if(strcmp((char*)usb_programer_buffer, USB_PROGRAMER_REBOOT)==0){
    log_debug("UsbProgramer:Rebooting device");
    reset_device();
  }
  else if(strcmp((char*)usb_programer_buffer, USB_PROGRAMER_PROGRAM)==0){
    log_debug("UsbProgramer: Entering USB-DFU mode");
    enter_dfu_mode();
  }
  else{
    log_debug("UsbProgramer:Unknown command");
  }
}
