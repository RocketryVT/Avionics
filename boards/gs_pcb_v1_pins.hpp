#pragma once

// gs_pcb_v1_pins.hpp — Ground Station PCB rev 1 (Pico 2 W) pin map.
//
// Single source of truth for GSPCB GPIO assignments. Application code should
// include this and use Pins::* instead of hardcoding GPIO numbers. The SDK-level
// peripheral defaults (PICO_DEFAULT_UART/I2C/SPI) live in the companion board
// header gs_pcb_v1.h — keep the two in sync.
//
// KiCad source: projects/ground_station/Hardware/GSPCB
//
// UART/I2C/SPI mux note (RP2350): the GPS UART is UART0, where the function mux
// fixes TX to GPIO12 and RX to GPIO13. GPS_TX below is the Pico's *transmit*
// line (-> GPS RX); GPS_RX is the Pico's *receive* line (<- GPS TX).

namespace Pins {

// -- I2C0 (GPIO 0/1) ----------------------------------------------------------
static constexpr unsigned I2C0_SDA   =  0;  // GPIO 0,  phys  1
static constexpr unsigned I2C0_SCL   =  1;  // GPIO 1,  phys  2

// -- I2C1 (GPIO 2/3) ----------------------------------------------------------
static constexpr unsigned I2C1_SDA   =  2;  // GPIO 2,  phys  4
static constexpr unsigned I2C1_SCL   =  3;  // GPIO 3,  phys  5

// -- Servo 1 / TOP = Azimuth --------------------------------------------------
// CL57TE PUL+ / DIR+ are held high by the PCB; Pico drives the active-low
// opto inputs only.
static constexpr unsigned STEP1_PUL_N = 4;  // GPIO 4, phys 6 — Servo1 PUL-
static constexpr unsigned STEP1_DIR_N = 5;  // GPIO 5, phys 7 — Servo1 DIR-

// -- Servo 2 / BOTTOM = Elevation ---------------------------------------------
static constexpr unsigned STEP2_PUL_N = 6;  // GPIO 6, phys 9 — Servo2 PUL-
static constexpr unsigned STEP2_DIR_N = 7;  // GPIO 7, phys 10 — Servo2 DIR-

// -- LoRa 1 / TOP — 433 MHz GFSK (RFM69HCW), SPI1 ----------------------------
static constexpr unsigned LORA1_MISO =  8;  // GPIO  8, phys 11
static constexpr unsigned LORA1_NSS  =  9;  // GPIO  9, phys 12  (CS)
static constexpr unsigned LORA1_SCK  = 10;  // GPIO 10, phys 14
static constexpr unsigned LORA1_MOSI = 11;  // GPIO 11, phys 15
static constexpr unsigned LORA1_RST  = 26;  // GPIO 26, phys 31
static constexpr unsigned LORA1_DIO0 = 27;  // GPIO 27, phys 32  (G0 / IRQ)
static constexpr unsigned LORA1_EN   = 28;  // GPIO 28, phys 34  (power enable)

// -- GPS (u-blox M10, UART0) --------------------------------------------------
static constexpr unsigned GPS_TX  = 12;  // GPIO 12, phys 16 — UART0 TX → GPS RX
static constexpr unsigned GPS_RX  = 13;  // GPIO 13, phys 17 — UART0 RX ← GPS TX

// -- LoRa 0 / BOTTOM — 915 MHz LoRa (SX1276), SPI0 ---------------------------
static constexpr unsigned LORA0_EN   = 16;  // GPIO 16, phys 21 (power enable)
static constexpr unsigned LORA0_DIO0 = 17;  // GPIO 17, phys 22 (G0 / IRQ)
static constexpr unsigned LORA0_SCK  = 18;  // GPIO 18, phys 24
static constexpr unsigned LORA0_MOSI = 19;  // GPIO 19, phys 25
static constexpr unsigned LORA0_MISO = 20;  // GPIO 20, phys 26
static constexpr unsigned LORA0_NSS  = 21;  // GPIO 21, phys 27 (CS)
static constexpr unsigned LORA0_RST  = 22;  // GPIO 22, phys 29

} // namespace Pins
