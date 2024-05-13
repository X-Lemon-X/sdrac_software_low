cmake_minimum_required(VERSION 3.16)

# ether use toolchain folder with arm-none-eabi.cmake file
# set(CMAKE_TOOLCHAIN_FILE "toolchain/arm-none-eabi.cmake")

# or just use this below
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_AR                        arm-none-eabi-ar)
set(CMAKE_ASM_COMPILER              arm-none-eabi-gcc)
set(CMAKE_C_COMPILER                arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER              arm-none-eabi-g++)
set(CMAKE_LINKER                    arm-none-eabi-ld)
set(CMAKE_OBJCOPY                   arm-none-eabi-objcopy CACHE INTERNAL "")
set(CMAKE_RANLIB                    arm-none-eabi-ranlib CACHE INTERNAL "")
set(CMAKE_SIZE                      arm-none-eabi-size CACHE INTERNAL "")
set(CMAKE_STRIP                     arm-none-eabi-strip CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

project(executable)

enable_language(CXX C ASM)


set(TARGET executable.elf) #${CMAKE_PROJECT_NAME}

set(CMAKE_CXX_STANDARD 17)

# ważne żeby dodać startup_stm32f412rx.s" do listy source files
# inaczej nie będzie działać


add_executable(${TARGET}
# All source files from Makefile
"Core/Src/stm32f4xx_it.c"
"Core/Src/stm32f4xx_hal_msp.c"
"USB_DEVICE/App/usb_device.c"
"USB_DEVICE/App/usbd_desc.c"
"USB_DEVICE/App/usbd_cdc_if.c"
"USB_DEVICE/Target/usbd_conf.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
"Core/Src/system_stm32f4xx.c"
"Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
"Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
"Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
"Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"

# Change the main.c to main.cpp
"Core/Src/main.cpp"

# Add the startup file the name should be the same as the stm32 you are using
"Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f412rx.s"

# Add all your custom source files
"app/main_prog_callbacks.cpp"
"app/main_prog.cpp"
"app/Encoder/src/encoder.cpp"
"app/Logger/src/logger.cpp"
"app/Timing/src/Timing.cpp"
"app/UsbPrograming/src/usb_programer.cpp"
"app/SteperMotor/src/steper_motor.cpp"
"app/Filter/src/filter.cpp"
"app/Filter/src/filter_alfa_beta.cpp"
"app/Filter/src/filter_moving_avarage.cpp"
"app/Memory/src/fram_menager.cpp"
"app/CanControl/src/can_control.cpp"
"app/MovementControl/src/movement_controler.cpp"
"app/CanControl/src/CanDB.c"
"app/CanControl/src/can_control.cpp"
"app/BoardId/src/board_id.cpp"
"app/PD/src/pd_controler.cpp"

)


# utaiwmay jakie stm32 używamy
target_compile_definitions(${TARGET} PRIVATE
	STM32F412Rx
	USE_HAL
)

# add include directories from Makefile 
# then add all your costom directories
target_include_directories(${TARGET} PRIVATE
#from Makefile
"USB_DEVICE/App"
"USB_DEVICE/Target"
"Core/Inc"
"Drivers/STM32F4xx_HAL_Driver/Inc"
"Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
"Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
"Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
"Drivers/CMSIS/Device/ST/STM32F4xx/Include"
"Drivers/CMSIS/Include"

#custom directories
"app"
"app/Encoder/inc"
"app/Logger/inc"
"app/Memory/inc"
"app/Timing/inc"
"app/UsbPrograming/inc"
"app/SteperMotor/inc"
"app/Filter/inc"
"app/Memory/inc"
"app/CanControl/inc"
"app/MovementControl/inc"
"app/BoardId/inc"
"app/PD/inc"
)


# use   -u_printf_float to enable float to string conversion add about 10kb to binary
 
target_compile_options(${TARGET} PRIVATE
$<$<COMPILE_LANGUAGE:ASM>:
		-O0 -g

		-mthumb
		-mhard-float
		-mcpu=cortex-m4
		-mfpu=fpv4-sp-d16
		-specs=nano.specs

		-x assembler-with-cpp
	>
  $<$<COMPILE_LANGUAGE:C>:
  -O0 -g
  -std=c17
  -fexceptions
  -fdata-sections
  -ffunction-sections
  -Wall
  -Wextra
  -Wpedantic
  -Wdouble-promotion

  -mthumb
  -mhard-float
  -mcpu=cortex-m4
  -mfpu=fpv4-sp-d16
  -specs=nano.specs
  -u_printf_float
>

$<$<COMPILE_LANGUAGE:CXX>:
-O0 -g
-std=c++17
-fexceptions
-Wall
-Wextra
-Wpedantic
-Wdouble-promotion
-Werror=return-type

-fno-threadsafe-statics
-fno-common
-fno-exceptions
-fno-rtti
-u_printf_float  # enable float to string conversion add about 10kb to binary
-fdata-sections  # place data in separate sections to be bale to remove unused data decrease binary size drastically
-ffunction-sections # place data in separate sections to be bale to remove unused data decrease binary size drastically

-mthumb
-mhard-float
-mcpu=cortex-m4
-mfloat-abi=hard
-mfpu=fpv4-sp-d16
-specs=nosys.specs
-specs=nano.specs
>

)


# add linker options from Makefile
target_link_options(${TARGET} PRIVATE
	-T ${CMAKE_SOURCE_DIR}/STM32F412RGTx_FLASH.ld
	-Wl,-Map=${CMAKE_PROJECT_NAME}.map
	-Wl,--gc-sections  # remove unused sections
	-Wl,-cref
	-Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group

	-mthumb
	-mhard-float
	-mfloat-abi=hard
	-mcpu=cortex-m4
	-mfpu=fpv4-sp-d16
	-specs=nano.specs
	-specs=nosys.specs
  -u_printf_float
	-static
  # --data-sections

)

target_link_libraries(${TARGET} m)

add_custom_command(
	TARGET ${TARGET} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${CMAKE_PROJECT_NAME}.bin
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${CMAKE_PROJECT_NAME}.hex
	COMMAND ${CMAKE_SIZE} ${TARGET}
)