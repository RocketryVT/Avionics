// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------
//
// gs_pcb_v1 — Ground Station PCB rev 1 (Pico 2 W carrier board).
//
// This is a CARRIER / breakout board: you drop a stock Pico 2 W onto header
// pins and the PCB fans every GPIO out to labeled JST connectors. Nothing is
// hardwired on the board, so what is "present" depends entirely on what a given
// firmware plugs into each connector. This header therefore describes only:
//   * the Pico 2 W module's intrinsic capabilities (Wi-Fi/flash/SMPS), and
//   * the fixed connector -> GPIO routing (the peripheral mux defaults below;
//     full functional map in gs_pcb_v1_pins.hpp, namespace Pins).
//
// Peripheral *presence* (a radio/IMU/baro plugged into a connector) is NOT a
// board fact — each project declares it in its own board_profile.hpp as APP_HAS_*
// alongside the matching settings. boards/board.hpp merges board intrinsics with
// the project profile into a unified HAS_* for user code.
//
// This layers on the stock `pico2_w` board (CYW43 Wi-Fi, flash, SMPS, bootrom
// inherited) and only overrides the connector mux defaults.
//
// Activate from CMake (before pico_sdk_init):
//     set(PICO_BOARD gs_pcb_v1)
//     set(PICO_BOARD_HEADER_DIRS ${AVIONICS_ROOT}/boards)
//
// KiCad source: projects/ground_station/Hardware/GSPCB
//
// Connector -> GPIO routing (see gs_pcb_v1_pins.hpp for the full table):
//   GPIO 0/1   I2C0 connector  (SDA/SCL)
//   GPIO 2/3   I2C1 connector  (SDA/SCL)
//   GPIO 4/5   STEP1 connector (PUL-/DIR-)
//   GPIO 6/7   STEP2 connector (PUL-/DIR-)
//   GPIO 8-11  SPI1 connector  (MISO/CS/SCK/MOSI) — "LORA1"
//   GPIO 12/13 UART0 connector (TX=12, RX=13, fixed by mux) — GPS
//   GPIO 16-22 SPI0 connector  (EN/G0/SCK/MOSI/MISO/CS/RST) — "LORA0"
//   GPIO 26-28 SPI1 connector continued (RST/G0/EN)

#ifndef _BOARDS_GS_PCB_V1_H
#define _BOARDS_GS_PCB_V1_H

// For board detection in application code.
#define GS_PCB_V1

// --- CMake-time board settings ----------------------------------------------
// The Pico SDK scans THIS file (the one named by PICO_BOARD) for
// pico_board_cmake_set() directives at configure time; it does NOT follow the
// #include of pico2_w.h below, so CYW43 (Wi-Fi) support must be declared here or
// the pico_cyw43_arch library / pico/cyw43_arch.h is never built. These lines
// expand to nothing for the C/C++/ASM compiler.
pico_board_cmake_set(PICO_PLATFORM, rp2350)
pico_board_cmake_set(PICO_CYW43_SUPPORTED, 1)

// --- Board capability flags --------------------------------------------------
// THIS IS A CARRIER BOARD: it only breaks a stock Pico 2 W out to labeled JST
// connectors — no peripherals are soldered on. So the only capabilities the
// board can guarantee are the ones intrinsic to the Pico 2 W module itself.
// Declare them with the BOARD_ prefix (clear of the SDK's PICO_ namespace);
// boards that lack a feature just omit its flag (an undefined macro is 0 in #if).
//
// What is actually PLUGGED INTO the connectors varies per firmware, so peripheral
// presence (SX1276, RFM69, IMU, baro, steppers, ...) is declared by each project
// in its own board_profile.hpp as APP_HAS_*, NOT here. boards/board.hpp merges
// the two into a unified HAS_* for user code. The connector->GPIO routing is the
// board's job and lives in gs_pcb_v1_pins.hpp (namespace Pins).
#define BOARD_HAS_WIFI 1   // CYW43, soldered on the Pico 2 W module



// --- UART (u-blox M10 GPS on UART0) ------------------------------------------
// NOTE: the RP2350 mux fixes UART0 TX to GPIO12 and UART0 RX to GPIO13.
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 12
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 13
#endif

// --- Default I2C (I2C0 — barometer MS5611) -----------------------------------
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 0
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 1
#endif

// --- Default SPI (SPI0 — LoRa0 / BOTTOM omni radio) --------------------------
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 18
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 19   // MOSI
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 20   // MISO
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 21
#endif

// Inherit everything else (CYW43 Wi-Fi, flash, SMPS, bootrom) from the Pico 2 W.
// Our #ifndef-guarded defines above win; pico2_w.h fills in the rest.
#include "boards/pico2_w.h"

#endif // _BOARDS_GS_PCB_V1_H
