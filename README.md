## SDRAC
this is a repo for software for SDRACboards;


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


### Adding code to CMakefile
Add all include direcotries and c files to  a Cmakefile.txt



### Starting CAN converter
```bash
ls /dev | grep ttyAC # aby sprawdzić na którym porcie jest podpięty
sudo slcand -o -c -s8 /dev/ttyACM0 can0 # nie zapominje o zmianie portu
sudo ip link set dev can0 up type can bitrate 1000000 
```

parametr -s ustwia predekość transmisji
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


### Problems with MXCube genrations
In main.c 
- DMA for ADC should be initaied after ADC is initated not befor whitch couses th emain code not to be executed

### TO DO:
 - simple arm control
 - CAN reciving data and and sending data
 - reading ADC
 - USB data reciving
 - saveing stuff to SRAM
 - reading data from SRAM
 - IO control
 - making deference between boards
### DONE:
 - steper motor interface
 - ADC interface
 - reading angles from encoder
 - USB data sending
 - Timing and timers 