if (NOT TARGET lsm6dsox)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../imus/lsm6dsox
                     ${CMAKE_CURRENT_BINARY_DIR}/lsm6dsox)
endif()

if (NOT TARGET lis3mdl)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../mags/lis3mdl
                     ${CMAKE_CURRENT_BINARY_DIR}/lis3mdl)
endif()

add_library(nine_axis_imu
    src/ICM20948/icm20948.cpp
    src/LSM6DSOX_LIS3MDL/LSM6DSOX_LIS3MDL.cpp
)

target_include_directories(nine_axis_imu PUBLIC include)

target_link_libraries(nine_axis_imu PUBLIC
    lsm6dsox
    lis3mdl
)

target_compile_features(nine_axis_imu PUBLIC cxx_std_23)
