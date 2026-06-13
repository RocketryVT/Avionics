#pragma once

// boards/board_pins.hpp — board-neutral pin map selector.
//
// Application/example code should include THIS header, not a specific
// <board>_pins.hpp. Switching PICO_BOARD in CMake then automatically swaps the
// pin definitions: the active SDK board header (selected by PICO_BOARD and
// found via PICO_BOARD_HEADER_DIRS) defines a board-detection macro, and we
// dispatch on it here.
//
// Contract: every board's *_pins.hpp must expose the same canonical Pins::
// vocabulary the app code uses, mapped to that board's GPIOs:
//     Pins::LORA0_{EN,DIO0,SCK,MOSI,MISO,NSS,RST}   // 915 MHz LoRa  (SPI0)
//     Pins::LORA1_{EN,DIO0,SCK,MOSI,MISO,NSS,RST}   // 433 MHz GFSK  (SPI1)
//     Pins::GPS_TX, Pins::GPS_RX                     // u-blox GPS    (UART0)
//
// To add a board: create boards/<board>.h (sets PICO_BOARD + a detect macro),
// create boards/<board>_pins.hpp providing the vocabulary above, and add a
// branch below.

#include "pico.h"   // pulls in the active board header and its detect macro

#if defined(GS_PCB_V1)
  #include "boards/gs_pcb_v1_pins.hpp"
#elif defined(BAREMAN_PCB_V2)
  #include "boards/bareman_pcb_v2_pins.hpp"
#else
  #error "board_pins.hpp: unknown/unsupported PICO_BOARD — add a detect-macro \
branch here and a matching boards/<board>_pins.hpp that exposes the canonical \
Pins:: vocabulary (LORA0_*, LORA1_*, GPS_TX/GPS_RX)."
#endif
