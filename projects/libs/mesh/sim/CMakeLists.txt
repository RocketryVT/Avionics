cmake_minimum_required(VERSION 3.20)

project(mesh_sim LANGUAGES C CXX)

add_library(mesh_host
    ../mesh.cpp
)

target_include_directories(mesh_host PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../../packets
)

target_compile_features(mesh_host PUBLIC cxx_std_23)

add_library(fusion_host
    ../../Third_Party/Fusion/Fusion/FusionAhrs.c
    ../../Third_Party/Fusion/Fusion/FusionBias.c
    ../../Third_Party/Fusion/Fusion/FusionCompass.c
)

target_include_directories(fusion_host PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../../Third_Party/Fusion/Fusion
)

add_executable(mesh_sim
    mesh_sim.cpp
)

target_link_libraries(mesh_sim PRIVATE
    mesh_host
    fusion_host
)
