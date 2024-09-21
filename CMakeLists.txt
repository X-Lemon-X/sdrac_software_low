cmake_minimum_required(VERSION 3.15)

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
# set(CMAKE_GENERATOR "Ninja")

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME sdrac_cubemx)
set(PROEJCT_MAJOR_VERSION 3)
option(COPY_COMPILE_COMMANDS "Copy compile_commands.json to project root" OFF)

## enable ccache if available
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()


# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# aditional compiler flags
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -u_printf_float")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}  -u_printf_float")

# aditional linker flags
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS}  -u_printf_float")
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS}  -u_printf_float")


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
  "lib/main_prog_callbacks.cpp"
  "lib/main_prog.cpp"
  "lib/config.cpp"
  "lib/Encoder/src/encoder.cpp"
  "lib/Logger/src/logger.cpp"
  "lib/Timing/src/Timing.cpp"
  "lib/UsbPrograming/src/usb_programer.cpp"
  "lib/SteperMotor/src/steper_motor.cpp"
  "lib/Filter/src/filter.cpp"
  "lib/Filter/src/filter_alfa_beta.cpp"
  "lib/Filter/src/filter_moving_avarage.cpp"
  "lib/Memory/src/fram_menager.cpp"
  "lib/CanControl/src/can_control.cpp"
  "lib/MovementControl/src/movement_controler.cpp"
  
  "lib/CanControl/src/can_control.cpp"
  "lib/BoardId/src/board_id.cpp"
  "lib/Contolers/src/pid_controler.cpp"
  "lib/Contolers/src/basic_controler.cpp"
  "lib/Contolers/src/pass_through_controler.cpp"
  "lib/Temperature_sensors/src/ntc_termistors.cpp"
  "lib/Temperature_sensors/src/MCP9700AT.cpp"
  
  "submodules/ariadna_constants/can_messages/output/can.c"
  )
# "lib/CanControl/src/CanDB.c"


# Add include paths

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  "lib"
  "lib/Encoder/inc"
  "lib/Logger/inc"
  "lib/Memory/inc"
  "lib/Timing/inc"
  "lib/UsbPrograming/inc"
  "lib/SteperMotor/inc"
  "lib/Filter/inc"
  "lib/Memory/inc"
  "lib/CanControl/inc"
  "lib/MovementControl/inc"
  "lib/BoardId/inc"
  "lib/Contolers/inc"
  "lib/Temperature_sensors/inc"
  "lib/containers/inc"
  "lib/Gpio/inc"

  "submodules/ariadna_constants/can_messages/output"
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
  # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_SOURCE_DIR}/compile_commands.json
  # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/update-software-version.sh ${CMAKE_SOURCE_DIR}/.git/hooks/post-commit
  COMMAND ${CMAKE_SOURCE_DIR}/scripts/shell/update-software-version.sh ${CMAKE_SOURCE_DIR} ${PROEJCT_MAJOR_VERSION}
  # Show ccache statistics
  COMMAND ${CCACHE_PROGRAM} -s
 )

if( COPY_COMPILE_COMMANDS )
  message(STATUS "Copying compile_commands.json")
  add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_SOURCE_DIR}/compile_commands.json
  )
else()
  message(STATUS "Not copying compile_commands.json")
endif()