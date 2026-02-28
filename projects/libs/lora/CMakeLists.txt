# LoRa drivers — library CMakeLists
#
# Include this via add_subdirectory() from your project after pico_sdk_init().
# Do NOT add cmake_minimum_required / project / pico_sdk_init here.
#
# Targets exposed to consumers:
#
#   lr11xx        — Semtech LR11XX C driver + Pico SPI HAL compiled together.
#                   Link this to use the low-level lr11xx_*.h API.
#
#   sx1276_hal    — INTERFACE target: RadioLib (fetched) + PicoHal.h headers.
#                   Link this to use RadioLib's SX127x (and other) radios.

# ── LR11XX ────────────────────────────────────────────────────────────────────
# Semtech's vendor driver is already a proper static library target.
add_subdirectory(lr11xx_driver)

# Wrap the vendor driver + Pico HAL into one convenience target so callers
# only need: target_link_libraries(my_target lr11xx)
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
    pico_time          # make_timeout_time_ms / time_reached used in the HAL
)

# If FreeRTOS-Kernel is available (i.e. the consuming project has already
# called FreeRTOS_Kernel_import.cmake), link it so the HAL can call vTaskDelay
# and use semaphores.  The consuming project must ensure FreeRTOSConfig.h is
# on the compile path for lr11xx (see target_include_directories in the
# project's own src/CMakeLists.txt).
if(TARGET FreeRTOS-Kernel)
    target_link_libraries(lr11xx PUBLIC FreeRTOS-Kernel)
endif()

# ── SX1276 / RadioLib ─────────────────────────────────────────────────────────
add_subdirectory(sx1276)
