# SDRAC
This is a repo for software for SDRACboards;


## TO DO:
 - simple arm control alogrithm
 - reading ADC
 - reading/writing data to FRAM
 - IO control from CAN ?
## DONE:
 - USB data reciving/sending
 - distingushing between different baords
 - CAN reciving/sending data
 - steper motor interface
 - ADC interface
 - reading angles from encoder
 - Timing and timers main system clock with microsecond precision

## Starting CAN converter
```bash
ls /dev | grep ttyAC # to check on which port the converter is
sudo slcand -o -c -s8 /dev/ttyACM0 can0 # dont forget to change the port
sudo ip link set dev can0 up type can bitrate 1000000 
```

falg -s sets the speed of the transmission
```bash
  -s0 = 10k
  -s1 = 20k
  -s2 = 50k
  -s3 = 100k
  -s4 = 125k
  -s5 = 250k
  -s6 = 500k
  -s7 = 750k
  -s8 = 1M
```


## Generating files in CubeMX
### Adding code to main
Open file sdrac_cubemx.ioc in CubeMX
Set all the settings you want and generate code.
in main.c add the following code
```c
#include "main_prog.hpp"
```
and in the main function add the following code
```c
  main_prog(); // before the infinite loop
```
then chnage the extension of the main.c to main.cpp


### Adding code to USB_DEVICE
Open file usbd_cdc_if.c "USB_DEVICE/App/usbd_cdc_if.c"
 In function 
 ```c
 int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) 
 ```
 modify the code to look like this
```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  // this is external buffer whitch can be found in usb_programer.hpp
  extern uint8_t usb_programer_buffer[APP_RX_DATA_SIZE];
  extern uint32_t usb_programer_buffer_len;
  extern uint8_t usb_programer_data_recived;
  memset(usb_programer_buffer, 0, APP_RX_DATA_SIZE);
  memcpy(usb_programer_buffer, Buf, *Len);
  usb_programer_buffer_len = *Len;
  usb_programer_data_recived = 1;
  return (USBD_OK);
  /* USER CODE END 6 */
}
```

## Adding code to CMakeLists.txt
Add all included directories and .c files to a CMakeLists.txt




