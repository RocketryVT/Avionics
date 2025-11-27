# Centralized dependency paths for Avionics projects.
# Consumers can override LIBS_ROOT (or any derived var) via -D on the CMake
# command line to point at a different checkout of the shared libraries.

# Resolve repository root based on this file's location (../ from cmake/).
get_filename_component(_deps_repo_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
if (NOT DEFINED AVIONICS_ROOT)
    set(AVIONICS_ROOT "${_deps_repo_root}" CACHE PATH "Root of the Avionics repository")
endif()

if (NOT DEFINED PROJECTS_ROOT)
    set(PROJECTS_ROOT "${AVIONICS_ROOT}/projects" CACHE PATH "Root of project sources")
endif()

if (NOT DEFINED LIBS_ROOT)
    set(LIBS_ROOT "${PROJECTS_ROOT}/libs" CACHE PATH "Root of shared third-party libraries")
endif()

# Third-party components used by the Pico examples; override if you keep libs
# in a nonstandard location.
if (NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH "${LIBS_ROOT}/Third_Party/pico-sdk" CACHE PATH "Path to the Pico SDK")
endif()

if (NOT DEFINED PICOTOOL_FETCH_FROM_GIT_PATH)
    set(PICOTOOL_FETCH_FROM_GIT_PATH "${LIBS_ROOT}/Third_Party/picotool/fetch_from_git.cmake" CACHE FILEPATH "Path to the picotool fetch script")
endif()

if (NOT DEFINED FREERTOS_KERNEL_PATH)
    set(FREERTOS_KERNEL_PATH "${LIBS_ROOT}/Third_Party/FreeRTOS-Kernel" CACHE PATH "Path to the FreeRTOS Kernel")
endif()

if (NOT DEFINED FREERTOS_THIRD_PARTY_DIR)
    set(FREERTOS_THIRD_PARTY_DIR "${FREERTOS_KERNEL_PATH}/portable/ThirdParty" CACHE PATH "Path to FreeRTOS community ports")
endif()

# In-house protobuf packet definitions.
if (NOT DEFINED PACKETS_PROTOBUF_PATH)
    set(PACKETS_PROTOBUF_PATH "${LIBS_ROOT}/packets/protobufs" CACHE PATH "Path to shared protobuf packet definitions")
endif()

# Additional Third_Party libraries commonly used across projects. Override any
# of these via -D<NAME>=... if you keep a different checkout/layout.
if (NOT DEFINED CMSIS_DSP_PATH)
    set(CMSIS_DSP_PATH "${LIBS_ROOT}/Third_Party/CMSIS-DSP" CACHE PATH "Path to CMSIS-DSP")
endif()

if (NOT DEFINED FUSION_PATH)
    set(FUSION_PATH "${LIBS_ROOT}/Third_Party/Fusion" CACHE PATH "Path to Fusion")
endif()

if (NOT DEFINED LIBFIXMATH_PATH)
    set(LIBFIXMATH_PATH "${LIBS_ROOT}/Third_Party/libfixmath" CACHE PATH "Path to libfixmath")
endif()

if (NOT DEFINED LIBFIXMATRIX_PATH)
    set(LIBFIXMATRIX_PATH "${LIBS_ROOT}/Third_Party/libfixmatrix" CACHE PATH "Path to libfixmatrix")
endif()

if (NOT DEFINED MADGWICK_AHRS_PATH)
    set(MADGWICK_AHRS_PATH "${LIBS_ROOT}/Third_Party/MadgwickAHRS" CACHE PATH "Path to MadgwickAHRS")
endif()

if (NOT DEFINED MAHONY_AHRS_PATH)
    set(MAHONY_AHRS_PATH "${LIBS_ROOT}/Third_Party/MahonyAHRS" CACHE PATH "Path to MahonyAHRS")
endif()

if (NOT DEFINED NANOPB_PATH)
    set(NANOPB_PATH "${LIBS_ROOT}/Third_Party/nanopb" CACHE PATH "Path to nanopb")
endif()

unset(_deps_repo_root)
