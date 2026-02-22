add_library(lora INTERFACE)

target_include_directories(lora INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(lora INTERFACE cxx_std_20)

target_link_libraries(lora INTERFACE
    hardware_spi
    hardware_gpio
    hardware_dma
    pico_time
)
