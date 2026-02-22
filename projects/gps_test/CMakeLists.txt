cmake_minimum_required(VERSION 3.14)
project(gps_test)

# Check if we are built as part of the main project or standalone
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # Standalone build - pull in libs
    add_subdirectory(../libs libs)
endif()

add_executable(gps_test main.cpp)
target_link_libraries(gps_test PRIVATE gps math_utils)
