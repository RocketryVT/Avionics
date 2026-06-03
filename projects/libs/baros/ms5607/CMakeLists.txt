add_library(ms5607
    src/MS5607.cpp
)
target_include_directories(ms5607 PUBLIC include)
target_link_libraries(ms5607 PUBLIC
    pico_stdlib
)
