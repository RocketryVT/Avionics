#pragma once

// boards/board.hpp — one include that gives app/task code everything board-
// related for the active PICO_BOARD:
//
//   * HAS_*   capability flags   (use in  #if HAS_SX1276 ... )
//   * Pins::  connector->GPIO map (from the board's *_pins.hpp)
//   * Board:: per-feature settings (freq/baud/SF), supplied by the project
//
// Capability resolution
// ---------------------
// A capability can be guaranteed two ways:
//   BOARD_HAS_<F>  — soldered/intrinsic to the board   (boards/<board>.h)
//   APP_HAS_<F>    — plugged into a connector by THIS firmware (board_profile.hpp)
// We merge them:   HAS_<F> = BOARD_HAS_<F> || APP_HAS_<F>
// so user code never cares whether a part is hardwired (bareman/ADS) or plugged
// into a carrier (gs_pcb_v1). Carriers leave BOARD_HAS_<F> at 0 and let the
// project's board_profile.hpp assert APP_HAS_<F>.
//
// Each project that uses peripherals provides a board_profile.hpp on its include
// path (APP_HAS_* + the Board:: settings namespaces). It is optional: a project
// with none simply omits it and every HAS_* resolves to 0.

#include "pico.h"               // active board header -> BOARD_HAS_* + detect macro
#include "boards/devices.hpp"   // Board:: model enums, spec_of(), *Instance structs
#include "boards/board_pins.hpp"  // Pins:: for the active board (before the profile,
                                  // so the profile can wire instances to Pins::)

#if defined(__has_include)
#  if __has_include("board_profile.hpp")
#    include "board_profile.hpp"   // per-project APP_HAS_* + Board:: instance arrays
#  endif
#endif

// --- Canonical capability vocabulary -----------------------------------------
// Default every known flag (board + app side) to 0 so HAS_* is always defined
// and usable in both #if and ordinary C++ expressions. Add new features here.
// Vocabulary: WIFI, GPS, RADIO, IMU, MAG, BARO, ACCEL, STEPPERS, SERVO,
// CAMERA, VTX, OSD, ANT_SWITCH. Per-device specificity (which chip, how many, ranges/rates)
// is NOT expressed as macros — it lives in the constexpr Board::*Instance arrays
// in board_profile.hpp + spec_of() in devices.hpp. Keep the macro surface coarse.
#ifndef BOARD_HAS_WIFI
#define BOARD_HAS_WIFI 0
#endif
#ifndef APP_HAS_WIFI
#define APP_HAS_WIFI 0
#endif
#define HAS_WIFI (BOARD_HAS_WIFI || APP_HAS_WIFI)

#ifndef BOARD_HAS_GPS
#define BOARD_HAS_GPS 0
#endif
#ifndef APP_HAS_GPS
#define APP_HAS_GPS 0
#endif
#define HAS_GPS (BOARD_HAS_GPS || APP_HAS_GPS)

#ifndef BOARD_HAS_RADIO
#define BOARD_HAS_RADIO 0
#endif
#ifndef APP_HAS_RADIO
#define APP_HAS_RADIO 0
#endif
#define HAS_RADIO (BOARD_HAS_RADIO || APP_HAS_RADIO)

#ifndef BOARD_HAS_SX1276
#define BOARD_HAS_SX1276 0
#endif
#ifndef APP_HAS_SX1276
#define APP_HAS_SX1276 0
#endif
#define HAS_SX1276 (BOARD_HAS_SX1276 || APP_HAS_SX1276)

#ifndef BOARD_HAS_RFM69
#define BOARD_HAS_RFM69 0
#endif
#ifndef APP_HAS_RFM69
#define APP_HAS_RFM69 0
#endif
#define HAS_RFM69 (BOARD_HAS_RFM69 || APP_HAS_RFM69)

#ifndef BOARD_HAS_IMU
#define BOARD_HAS_IMU 0
#endif
#ifndef APP_HAS_IMU
#define APP_HAS_IMU 0
#endif
#define HAS_IMU (BOARD_HAS_IMU || APP_HAS_IMU)

#ifndef BOARD_HAS_MAG
#define BOARD_HAS_MAG 0
#endif
#ifndef APP_HAS_MAG
#define APP_HAS_MAG 0
#endif
#define HAS_MAG (BOARD_HAS_MAG || APP_HAS_MAG)

#ifndef BOARD_HAS_BARO
#define BOARD_HAS_BARO 0
#endif
#ifndef APP_HAS_BARO
#define APP_HAS_BARO 0
#endif
#define HAS_BARO (BOARD_HAS_BARO || APP_HAS_BARO)

#ifndef BOARD_HAS_ACCEL
#define BOARD_HAS_ACCEL 0
#endif
#ifndef APP_HAS_ACCEL
#define APP_HAS_ACCEL 0
#endif
#define HAS_ACCEL (BOARD_HAS_ACCEL || APP_HAS_ACCEL)   // dedicated high-g accel

#ifndef BOARD_HAS_STEPPERS
#define BOARD_HAS_STEPPERS 0
#endif
#ifndef APP_HAS_STEPPERS
#define APP_HAS_STEPPERS 0
#endif
#define HAS_STEPPERS (BOARD_HAS_STEPPERS || APP_HAS_STEPPERS)

#ifndef BOARD_HAS_SERVO
#define BOARD_HAS_SERVO 0
#endif
#ifndef APP_HAS_SERVO
#define APP_HAS_SERVO 0
#endif
#define HAS_SERVO (BOARD_HAS_SERVO || APP_HAS_SERVO)

#ifndef BOARD_HAS_CAMERA
#define BOARD_HAS_CAMERA 0
#endif
#ifndef APP_HAS_CAMERA
#define APP_HAS_CAMERA 0
#endif
#define HAS_CAMERA (BOARD_HAS_CAMERA || APP_HAS_CAMERA)

#ifndef BOARD_HAS_VTX
#define BOARD_HAS_VTX 0
#endif
#ifndef APP_HAS_VTX
#define APP_HAS_VTX 0
#endif
#define HAS_VTX (BOARD_HAS_VTX || APP_HAS_VTX)

#ifndef BOARD_HAS_OSD
#define BOARD_HAS_OSD 0
#endif
#ifndef APP_HAS_OSD
#define APP_HAS_OSD 0
#endif
#define HAS_OSD (BOARD_HAS_OSD || APP_HAS_OSD)

#ifndef BOARD_HAS_ANT_SWITCH
#define BOARD_HAS_ANT_SWITCH 0
#endif
#ifndef APP_HAS_ANT_SWITCH
#define APP_HAS_ANT_SWITCH 0
#endif
#define HAS_ANT_SWITCH (BOARD_HAS_ANT_SWITCH || APP_HAS_ANT_SWITCH)  // RF/antenna mux
