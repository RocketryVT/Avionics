# LoRa / Radio drivers — library CMakeLists
#
# Include via add_subdirectory() from a project after pico_sdk_init().
# Do NOT add cmake_minimum_required / project / pico_sdk_init here.
#
# Targets exposed to consumers:
#
#   radio_hal   — INTERFACE: RadioLib 7.x (local) + PicoHal.h + radio wrappers.
#                 Link to use any RadioLib-backed radio.
#
#   lr11xx      — Semtech LR11XX C driver + Pico SPI HAL (static library).

# -- RadioLib (local copy) -----------------------------------------------------
# Expects RADIOLIB_PATH to be set (done in deps.cmake).
# Fall back to a path relative to the avionics root if not set.
if (NOT DEFINED RADIOLIB_PATH)
    get_filename_component(_lora_avionics_root
        "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
    set(RADIOLIB_PATH "${_lora_avionics_root}/RadioLib-7.7.1")
    unset(_lora_avionics_root)
endif()

if (NOT EXISTS "${RADIOLIB_PATH}/CMakeLists.txt")
    message(FATAL_ERROR
        "RadioLib not found at '${RADIOLIB_PATH}'.\n"
        "Set RADIOLIB_PATH to the RadioLib-7.x source directory, or place "
        "RadioLib-7.7.1/ next to the Avionics repository root.")
endif()

set(RADIOLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RADIOLIB_BUILD_TESTS    OFF CACHE BOOL "" FORCE)

add_subdirectory(${RADIOLIB_PATH} ${CMAKE_CURRENT_BINARY_DIR}/radiolib EXCLUDE_FROM_ALL)

# -- radio_hal INTERFACE target ------------------------------------------------
# Consumers get:
#   • RadioLib headers (via the RadioLib target)
#   • hal/PicoHal.h               — Pico SDK implementation of RadioLibHal
#   • radio/Radio.hpp             — radio::IRadio + radio::Packet
#   • radio/SpiDevice.hpp         — radio::SpiDevice (shared SPI helper)
#   • radio/sx1276/SX1276.hpp     — radio::sx1276::SX1276 (RadioLib wrapper)
#   • radio/rf69/RF69.hpp         — radio::rf69::RF69    (RadioLib wrapper)
add_library(radio_hal INTERFACE)

target_include_directories(radio_hal INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/hal    # PicoHal.h
    ${CMAKE_CURRENT_SOURCE_DIR}/radio  # Radio.hpp, SpiDevice.hpp, sx1276/, rf69/
)

target_link_libraries(radio_hal INTERFACE
    RadioLib
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# -- sx1231 static library (native driver, no RadioLib) ------------------------
# Provides radio::sx1231::SX1231.  Link alongside radio_hal (for Radio.hpp /
# SpiDevice.hpp) or standalone — both header sets are in radio/.
add_library(sx1231 STATIC
    radio/sx1231/SX1231.cpp
)

target_include_directories(sx1231 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/radio  # Radio.hpp, SpiDevice.hpp, sx1231/SX1231.hpp
)

target_link_libraries(sx1231 PUBLIC
    pico_stdlib
    hardware_spi
    hardware_gpio
)

target_compile_features(sx1231 PUBLIC cxx_std_23)

# -- LR11XX (Semtech vendor driver) --------------------------------------------
add_subdirectory(lr11xx_driver)

add_library(lr11xx STATIC lr11xx_hal.c)

target_include_directories(lr11xx PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}            # lr11xx_hal_context.h
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_driver
)

target_link_libraries(lr11xx PUBLIC
    lr11xx_driver
    pico_stdlib
    hardware_spi
    hardware_gpio
    pico_time
)

if (TARGET FreeRTOS-Kernel)
    target_link_libraries(lr11xx PUBLIC FreeRTOS-Kernel)
    # FreeRTOS-Kernel-Core injects freertos_sdk_config.h into every Pico SDK
    # header via PICO_CONFIG_RTOS_ADAPTER_HEADER, which then #includes
    # "FreeRTOSConfig.h".  Propagate the config directory so it is findable
    # when lr11xx is compiled as a standalone static library.
    if (DEFINED FREERTOS_CONFIG_FILE_DIRECTORY)
        target_include_directories(lr11xx PUBLIC ${FREERTOS_CONFIG_FILE_DIRECTORY})
    else()
        message(WARNING
            "lr11xx: FreeRTOS-Kernel is linked but FREERTOS_CONFIG_FILE_DIRECTORY "
            "is not set. Set it to the directory containing FreeRTOSConfig.h before "
            "add_subdirectory(lora).")
    endif()
endif()
