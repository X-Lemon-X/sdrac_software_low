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


