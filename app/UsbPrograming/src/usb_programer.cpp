
#include "usb_programer.hpp"
#include "main_prog.hpp"
// #include "usb_device.h"
#include "usbd_cdc_if.h"
// #include "usbd_def.h"
#include <string>

using namespace USB_PROGRAMER;

UsbProgramer::UsbProgramer(Pin _boot_device): boot_device(_boot_device){
  this->reboot_device_phrase = USB_PROGRAMER_REBOOT;
  this->program_device_phrase = USB_PROGRAMER_PROGRAM;
}

void UsbProgramer::reset_device(){
  HAL_NVIC_SystemReset();
}

void UsbProgramer::program_device(){

  HAL_GPIO_WritePin(this->boot_device.port, this->boot_device.pin, GPIO_PIN_SET);
  HAL_Delay(50);
  reset_device();
  HAL_GPIO_WritePin(this->boot_device.port, this->boot_device.pin,GPIO_PIN_RESET);
  while (true){}
}



void UsbProgramer::handler(){
  uint32_t size =0;
  // add thsi static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);
  // to a file USB_DEVICE/App/usbd_cdc_if.h
  // CDC_Receive_FS(buffer, &size);
  memset(buffer, 0, sizeof(USB_PROGRAMER_BUFFER_SIZE));
  uint8_t stset=  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, buffer);
  uint8_t strec=   USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  log_debug("USB_Handler:Received size: " + std::to_string(size) + " status: " + std::to_string(stset) + " "  + std::to_string(strec));
  char *ptr = (char*)buffer;
  std::string str(ptr);
  log_debug("Received: " + str);
}
