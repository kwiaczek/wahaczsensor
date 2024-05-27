# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/wiaczek/Code/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(wahaczsensor C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

file(GLOB VL53L5CX_SOURCES "${CMAKE_SOURCE_DIR}/ext/Platform/*.cpp" "${CMAKE_SOURCE_DIR}/ext/VL53L5CX_ULD_API/src/*.cpp")

add_executable(wahaczsensor  ${VL53L5CX_SOURCES}
wahaczsensor.cpp
)

pico_set_program_name(wahaczsensor "wahaczsensor")
pico_set_program_version(wahaczsensor "0.1")

pico_enable_stdio_uart(wahaczsensor 0)
pico_enable_stdio_usb(wahaczsensor 1)

# Add the standard library to the build
target_link_libraries(wahaczsensor
        pico_stdlib
        hardware_i2c
)

# Add the standard include files to the build
target_include_directories(wahaczsensor PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  ${CMAKE_SOURCE_DIR}/ext/Platform
  ${CMAKE_SOURCE_DIR}/ext/VL53L5CX_ULD_API/inc
)

pico_add_extra_outputs(wahaczsensor)