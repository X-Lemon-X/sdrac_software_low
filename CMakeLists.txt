cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE "toolchain/arm-none-eabi.cmake")

project(executable)

enable_language(CXX C ASM)


set(TARGET executable.elf) #${CMAKE_PROJECT_NAME}

set(CMAKE_CXX_STANDARD 17)

# ważne żeby dodać startup_stm32f412rx.s" do listy source files
# inaczej nie będzie działać


add_executable(${TARGET}
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

"Core/Src/main.cpp"
"Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f412rx.s"

"app/Encoder/src/encoder.cpp"
"app/Logger/src/logger.cpp"
"app/main_prog.cpp"
"app/Timing/src/Timing.cpp"
"app/UsbPrograming/src/usb_programer.cpp"
"app/SteperMotor/src/steper_motor.cpp"
"app/Filter/src/filter.cpp"
)


# utaiwmay jakie stm32 używamy
target_compile_definitions(${TARGET} PRIVATE
	STM32F412Rx
	USE_HAL
)

target_include_directories(${TARGET} PRIVATE
"USB_DEVICE/App"
"USB_DEVICE/Target"
"Core/Inc"
"Drivers/STM32F4xx_HAL_Driver/Inc"
"Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
"Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
"Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
"Drivers/CMSIS/Device/ST/STM32F4xx/Include"
"Drivers/CMSIS/Include"

"app"
"app/Encoder/inc"
"app/Logger/inc"
"app/Memory/inc"
"app/Timing/inc"
"app/UsbPrograming/inc"
"app/SteperMotor/inc"
"app/Filter/inc"
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
-u_printf_float

-mthumb
-mhard-float
-mcpu=cortex-m4
-mfloat-abi=hard
-mfpu=fpv4-sp-d16
-specs=nosys.specs
-specs=nano.specs
>

)



target_link_options(${TARGET} PRIVATE
	-T ${CMAKE_SOURCE_DIR}/STM32F412RGTx_FLASH.ld
	-Wl,-Map=${CMAKE_PROJECT_NAME}.map
	-Wl,--gc-sections
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

)

target_link_libraries(${TARGET} m)

add_custom_command(
	TARGET ${TARGET} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${CMAKE_PROJECT_NAME}.bin
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${CMAKE_PROJECT_NAME}.hex
	COMMAND ${CMAKE_SIZE} ${TARGET}
)