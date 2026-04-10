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

# Rocketry At Virginia Tech libraries

if (NOT DEFINED PICO_LOGGER_PATH)
    set(PICO_LOGGER_PATH "${LIBS_ROOT}/pico-logger" CACHE PATH "Path to pico-logger")
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

if (NOT DEFINED EIGEN_PATH)
    set(EIGEN_PATH "${LIBS_ROOT}/Third_Party/eigen" CACHE PATH "Path to Eigen")
endif()

if (NOT DEFINED LIBFIXKALMAN_PATH)
    set(LIBFIXKALMAN_PATH "${LIBS_ROOT}/Third_Party/libfixkalman" CACHE PATH "Path to libfixkalman")
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

if (NOT DEFINED UBXLIB_PATH)
    set(UBXLIB_PATH "${LIBS_ROOT}/Third_Party/ubxlib" CACHE PATH "Path to ubxlib")
endif()


# LoRa / radio drivers (Semtech LR11XX + RadioLib-backed SX1276 / RF69)
if (NOT DEFINED LORA_ROOT)
    set(LORA_ROOT "${LIBS_ROOT}/lora" CACHE PATH "Path to LoRa drivers")
endif()

# Local RadioLib source tree (7.x).  The lora CMakeLists.txt falls back to
# ${AVIONICS_ROOT}/RadioLib-7.6.0 automatically, but setting it here makes
# the path explicit and lets projects override via -DRADIOLIB_PATH=...
if (NOT DEFINED RADIOLIB_PATH)
    set(RADIOLIB_PATH "${AVIONICS_ROOT}/RadioLib-7.6.0"
        CACHE PATH "Path to local RadioLib 7.x source")
endif()

# GPS NMEA parser
if (NOT DEFINED GPS_ROOT)
    set(GPS_ROOT "${LIBS_ROOT}/gps" CACHE PATH "Path to GPS library")
endif()

# LIS3MDL magnetometer driver
if (NOT DEFINED LIS3MDL_ROOT)
    set(LIS3MDL_ROOT "${LIBS_ROOT}/lis3mdl" CACHE PATH "Path to LIS3MDL driver")
endif()

# ISM330DLC IMU driver
if (NOT DEFINED ISM330DLC_ROOT)
    set(ISM330DLC_ROOT "${LIBS_ROOT}/ism330dlc" CACHE PATH "Path to ISM330DLC driver")
endif()

# Ground-station math (haversine, azimuth, elevation)
if (NOT DEFINED MATH_UTILS_ROOT)
    set(MATH_UTILS_ROOT "${LIBS_ROOT}/math_utils" CACHE PATH "Path to math_utils library")
endif()

unset(_deps_repo_root)
