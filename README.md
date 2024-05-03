## SDRAC
this is a repo for software for SDRACboards;


## Generating files in CubeMX
### Adding code to main
Open file sdrac_cubemx.ioc in CubeMX
Set all the settings you want and generate code.
if fore some reason main_app(); deleted it self from main add #include "main_app.h" in main.c
and add main_app(); in int main(); before the loop after all the initializations.
### Adding code to Makefile
Add all include direcotries and c files to  a mekefile


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