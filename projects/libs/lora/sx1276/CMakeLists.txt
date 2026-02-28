# SX1276 / RadioLib — library CMakeLists
#
# Included by the parent lora/CMakeLists.txt via add_subdirectory().
# Do NOT add cmake_minimum_required / project / pico_sdk_init here.
#
# Targets exposed:
#   sx1276_hal  — INTERFACE: RadioLib library + PicoHal.h include path.
#                 Link this to use any RadioLib-supported radio (SX127x, SX126x, …).
#                 RadioLib is fetched automatically at configure time.

include(FetchContent)

FetchContent_Declare(
    RadioLib
    GIT_REPOSITORY  https://github.com/jgromes/RadioLib.git
    GIT_TAG         6.6.0
    GIT_SHALLOW     TRUE
)

# Suppress RadioLib's own examples / tests — we only want the library.
set(RADIOLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RADIOLIB_BUILD_TESTS    OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(RadioLib)

# INTERFACE target: consumers get RadioLib + PicoHal.h by linking sx1276_hal.
add_library(sx1276_hal INTERFACE)

target_include_directories(sx1276_hal INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}   # exposes PicoHal.h
)

target_link_libraries(sx1276_hal INTERFACE
    RadioLib
    pico_stdlib
    hardware_spi
    hardware_gpio
)
