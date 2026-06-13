add_library(mesh
    mesh.cpp
)

target_include_directories(mesh PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${LIBS_ROOT}/packets
)

target_compile_features(mesh PUBLIC cxx_std_23)
