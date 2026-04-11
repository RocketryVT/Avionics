add_library(gps
    src/types.cpp
)

target_include_directories(gps PUBLIC include)
target_compile_features(gps PUBLIC cxx_std_23)
