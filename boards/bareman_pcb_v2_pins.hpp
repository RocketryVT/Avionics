#pragma once

// bareman_pcb_v2_pins.hpp — Bareman tracker v2 pin map.
//
// Values are taken from the current bareman_tracker firmware pin map. If the
// KiCad project is added to this checkout, this file should be cross-checked
// against the schematic net labels and kept as the single source of truth.

namespace Pins {

// -- SX1276 LoRa, SPI1 --------------------------------------------------------
static constexpr unsigned LR_SCK     = 26;
static constexpr unsigned LR_MOSI    = 27;
static constexpr unsigned LR_MISO    = 28;
static constexpr unsigned LR_NSS     = 29;
static constexpr unsigned LR_DIO0    = 22;  // TxDone / RxDone interrupt
static constexpr unsigned LR_NRESET  = 23;

// Canonical radio vocabulary used by board-neutral code.
static constexpr unsigned LORA0_SCK  = LR_SCK;
static constexpr unsigned LORA0_MOSI = LR_MOSI;
static constexpr unsigned LORA0_MISO = LR_MISO;
static constexpr unsigned LORA0_NSS  = LR_NSS;
static constexpr unsigned LORA0_DIO0 = LR_DIO0;
static constexpr unsigned LORA0_RST  = LR_NRESET;
static constexpr unsigned LORA0_EN   = 0xFFu; // no separate power-enable GPIO

// -- MS5607 barometer / sensor I2C0 ------------------------------------------
static constexpr unsigned BARO_SDA   = 20;
static constexpr unsigned BARO_SCL   = 21;
static constexpr unsigned I2C0_SDA   = BARO_SDA;
static constexpr unsigned I2C0_SCL   = BARO_SCL;

// -- Status LED ---------------------------------------------------------------
static constexpr unsigned STATUS     = 12;

// -- Debug UART ---------------------------------------------------------------
static constexpr unsigned DBG_TX     = 13;
static constexpr unsigned DBG_RX     = 14;

// -- GPS, UART0 ---------------------------------------------------------------
static constexpr unsigned GPS_UART_TX = 16; // RP2350 TX -> GPS RX
static constexpr unsigned GPS_UART_RX = 17; // GPS TX -> RP2350 RX
static constexpr unsigned GPS_TX      = GPS_UART_TX;
static constexpr unsigned GPS_RX      = GPS_UART_RX;

} // namespace Pins
