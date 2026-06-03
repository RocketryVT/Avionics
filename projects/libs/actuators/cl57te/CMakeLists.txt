add_library(cl57te
    src/cl57te.cpp
)
target_include_directories(cl57te PUBLIC include)
target_link_libraries(cl57te PUBLIC
    hardware_gpio
    hardware_timer
    pico_stdlib
)
