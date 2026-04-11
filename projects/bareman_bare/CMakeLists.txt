cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_PLATFORM rp2350-arm-s)
set(PICO_BOARD pico2)

# Locate the avionics root (2 levels up from this file)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

# Pull in Pico SDK (must be before project())
set(PICO_SDK_ROOT ${PICO_SDK_PATH})
if (NOT EXISTS "${PICO_SDK_ROOT}/pico_sdk_init.cmake")
    message(FATAL_ERROR "Pico SDK not found at ${PICO_SDK_ROOT}.")
endif()
include(${PICO_SDK_ROOT}/pico_sdk_init.cmake)

set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "" FORCE)
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(bareman_bare C CXX ASM)

pico_sdk_init()

# RadioLib (local copy)
if (NOT EXISTS "${RADIOLIB_PATH}/src/RadioLib.h")
    message(FATAL_ERROR "RadioLib not found at ${RADIOLIB_PATH}.")
endif()
add_subdirectory(${RADIOLIB_PATH} ${CMAKE_CURRENT_BINARY_DIR}/RadioLib)

add_executable(SX1276_Test SX1276_Test.cpp)

pico_set_program_name(SX1276_Test "SX1276_Test")
pico_set_program_version(SX1276_Test "0.1")

pico_enable_stdio_uart(SX1276_Test 0)
pico_enable_stdio_usb(SX1276_Test 1)

target_include_directories(SX1276_Test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${RADIOLIB_PATH}/src
    ${AVIONICS_ROOT}/projects/libs/packets
)

target_link_libraries(SX1276_Test
    pico_stdlib
    hardware_spi
    hardware_uart
    hardware_gpio
    RadioLib
)

pico_add_extra_outputs(SX1276_Test)
