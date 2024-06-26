# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
if(NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH variable is required. Please specify the path to the Pico SDK using -DPICO_SDK_PATH=<path>")
endif()

if(NOT DEFINED RP2040_DIR)
    message(FATAL_ERROR "RP2040_DIR variable is required. Please specify the path to the Pico SDK using -DRP2040_DIR=<path>")
endif()

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
file(GLOB SOURCE_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")

message(${SOURCE_FILES})
add_executable(wahaczsensor  ${VL53L5CX_SOURCES}
  ${SOURCE_FILES}
)

pico_set_program_name(wahaczsensor "wahaczsensor")
pico_set_program_version(wahaczsensor "0.1")

pico_enable_stdio_uart(wahaczsensor 0)
pico_enable_stdio_usb(wahaczsensor 1)

# Add the standard library to the build
target_link_libraries(wahaczsensor
        pico_stdlib
        pico_multicore
        pico_i2c_slave
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

find_program(CLANG_FORMAT_BIN clang-format)
if(CLANG_FORMAT_BIN)
  file(GLOB_RECURSE ALL_SOURCE_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp" "${CMAKE_SOURCE_DIR}/src/*.hpp")
  add_custom_target(
      format
      COMMAND ${CLANG_FORMAT_BIN} -i ${ALL_SOURCE_FILES}
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endif()

if(RP2040_DIR)
  add_custom_target(
    copyrp2040
    COMMAND cp ${CMAKE_SOURCE_DIR}/build/wahaczsensor.uf2 ${RP2040_DIR}
    DEPENDS wahaczsensor
  ) 
endif()