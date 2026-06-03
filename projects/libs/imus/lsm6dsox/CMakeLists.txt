add_library(lsm6dsox
    src/LSM6DSOX.cpp
)

target_include_directories(lsm6dsox PUBLIC include)

target_compile_features(lsm6dsox PUBLIC cxx_std_23)
