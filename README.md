# SDRAC
This is a a low level software for SDRACboards from SDRAC project (6DOF Manipulator). It is written in C++ and uses STM32CubeMX HAL library.
The project is based on the STM32F412RGT6 microcontroller.

## TO DO:
 - simple arm control alogrithm ( PID ?  P is working for velocity control)
 - reading ADC (sort of working)
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
flag -s sets the speed of the transmission
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

# Building the project:
Building the project requires GNU Arm Embedded Toolchain to be installed.
1. Download it from this site https://developer.arm.com/downloads/-/gnu-rm 
2. Move the extracted files in some directory for example "$HOME/.local/share/arm-none-aebi", 
3. Add this in your .profile file 
```bash
# Add the arm-none-aebi to the path
if [ -d "$HOME/.local/share/arm-none-aebi/bin" ]; then
    PATH="$HOME/.local/share/arm-none-aebi/bin:$PATH" 
fi
```
4. Restart the pc

# Flashing the board
There are three ways to flash the board:
1. Using the ST-Link
2. Using the USB-flash-script (Preferred unless the board was never flashed before)
3. Using the USB DFU-mode (can be used if the board supports it)
### Using the USB-flash-script 
To flash the board using the USB you just have to run the following command
```bash
./auto-usb-flash.sh
```
if you don't have required programs installed the script will ask you if you want to install them automatically.

### Using the USB
To flash the board using the USB you need to have the dfu-util installed.
You can download it by running the following command
```bash
sudo apt-get install dfu-util -y
```
Then after you connect the board to pc via usb you have to enter the dfu mode by pressing the BOOT0 button and then clicking the RESET button and releasing the BOOT0 button.
Then you can flash the board by command
```bash
dfu-util -a 0 -i 0 -s 0x08000000:leave -D build/executable.bin
```

### Using the ST-Link
To flash the board using the ST-Link you need to have the ST-Link utility installed. 
You can download it by running the following command
```bash
sudo apt-get install stlink-tools -y
```
Then after you connect the board to pc via st-link you can flash the board by running the following command
```bash
st-flash --reset write build/executable.bin 0x08000000
```

# Debugging
To debug the baord you need cortex-debug extension for vscode.
and st-link utility installed.
the in vs code add in .vscode/launch.json the following code
```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Cortex Debug",
      "cwd": "${workspaceFolder}",
      "executable": "./build/executable.elf",
      "request": "launch",
      "type": "cortex-debug",
      "runToEntryPoint": "main",
      "servertype": "stlink"
    }
  ]
}
```
and then you can start the debugging by pressing F5 or by clicking on the debug icon in the debug tab.


## Generating files in CubeMX
### seting up CubeMX
Open file sdrac_cubemx.ioc in CubeMX
In project menager set the Toolchain to Makefile, and the Toolchain/folder to the folder where the project is located.
Set all the settings you want and generate code.

### Adding code to main
"For SDRAC project"
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
"For SDRAC project"
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

## Preparing CMakeLists.txt
After generating the code in CubeMX all necessary files will be generated like Core, Drivers, ....
Including the Makefile, STM32Fxxx.ld, stm32f4xxxx.s files.

From Makefiel you have to copy all the source xx.c files, include directories, and most importantly add stm32f4xxxx.s to source files and set appropriate linker STM32Fxxx.ld in target_link_options in CMakeLists.txt.
After that you shopuld be able to build the project using cmake.




