cmake_minimum_required(VERSION 3.16)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME sdrac_cubemx)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable CMake support for ASM and C languages
enable_language(CXX C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
# target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
#     # Add user defined library search paths
# )

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
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
  "app/Temperature_sensors/src/ntc_termistors.cpp"
  "app/Temperature_sensors/src/MCP9700AT.cpp"
  "app/config.cpp"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
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
  "app/Temperature_sensors/inc"
  "app/list/inc"
  "app/Gpio/inc"
)



# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

add_custom_command(
	TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf firmware.bin
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf firmware.hex
	COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_PROJECT_NAME}.elf firmware.elf
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_SOURCE_DIR}/compile_commands.json
)
