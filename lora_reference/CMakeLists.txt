# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(LoRa_Driver_V1 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add subdirectory for the LR11XX driver library. This will build the library and make it available for linking.

add_subdirectory(lr11xx_driver)

# Add executable. Default name is the project name, version 0.1

add_executable(LoRa_Driver_V1 LoRa_Driver_V1.cpp 
    lr11xx_hal.c
)

pico_set_program_name(LoRa_Driver_V1 "LoRa_Driver_V1")
pico_set_program_version(LoRa_Driver_V1 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(LoRa_Driver_V1 0)
pico_enable_stdio_usb(LoRa_Driver_V1 1)

# Add the standard library to the build
target_link_libraries(LoRa_Driver_V1
        pico_stdlib
        hardware_spi
        hardware_gpio
        lr11xx_driver)

# Add the standard include files to the build
target_include_directories(LoRa_Driver_V1 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(LoRa_Driver_V1)

