// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------
//
// bareman_pcb_v2 — RP2350 Bareman tracker board.
//
// This board carries the tracker peripherals directly. The GPIO map lives in
// bareman_pcb_v2_pins.hpp; this SDK board header only exposes board detection,
// SDK defaults, and coarse guaranteed capabilities.

#ifndef _BOARDS_BAREMAN_PCB_V2_H
#define _BOARDS_BAREMAN_PCB_V2_H

#define BAREMAN_PCB_V2

pico_board_cmake_set(PICO_PLATFORM, rp2350)

// The v2 tracker firmware currently uses the working GPS, barometer, and
// SX1276 path. The IMU/mag footprint is intentionally not advertised here
// until the physical chip path is usable.
#define BOARD_HAS_GPS   1
#define BOARD_HAS_BARO  1
#define BOARD_HAS_RADIO 1
#define BOARD_HAS_SX1276 1

// 16 MB external flash on the Bareman PCB.
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES 16777216
#endif

// GPS: UART0, Pico TX -> GPS RX on GPIO16, GPS TX -> Pico RX on GPIO17.
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 16
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 17
#endif

// MS5607 barometer: I2C0 on GPIO20/21.
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 20
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 21
#endif

// SX1276 LoRa: SPI1 on GPIO26-29.
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 1
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 26
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 27
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 28
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 29
#endif

// Must be kept at the end of the header.
#include "boards/pico2.h"

#endif // _BOARDS_BAREMAN_PCB_V2_H
