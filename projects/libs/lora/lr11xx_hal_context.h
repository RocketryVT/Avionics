// lr11xx_hal_context.h
//
// Defines lr11xx_hal_context_t — the board-specific configuration block that
// is passed as the opaque `context` pointer throughout the Semtech LR11XX
// driver API.
//
// Usage (C or C++):
//
//   #include "lr11xx_hal_context.h"
//
//   static lr11xx_hal_context_t g_radio = LR11XX_HAL_CONTEXT_INIT(
//       spi0, 8000000,   // SPI bus, clock Hz
//       6, 4, 7,         // SCK, MOSI, MISO
//       5, 0, 1          // NSS, BUSY, NRESET
//   );
//
//   lr11xx_hal_init( &g_radio );
//   lr11xx_radio_set_pkt_type( &g_radio, LR11XX_RADIO_PKT_TYPE_LORA );
//   ...

#pragma once

#include <stdint.h>
#include "hardware/spi.h"
#include "FreeRTOS.h"
#include "semphr.h"

// ── Context struct ────────────────────────────────────────────────────────────
// Fill in the public fields, then call lr11xx_hal_init().
// The _mutex / _mutex_buf fields are internal — zero-init them (or use
// LR11XX_HAL_CONTEXT_INIT which does this automatically).

typedef struct lr11xx_hal_context_s
{
    // ── User-configurable (set before lr11xx_hal_init) ────────────────────────
    spi_inst_t* spi;        ///< SPI peripheral — spi0 or spi1
    uint32_t    spi_hz;     ///< SPI clock frequency in Hz (max 16 MHz)
    uint        pin_sck;    ///< GPIO: SPI clock
    uint        pin_mosi;   ///< GPIO: SPI MOSI
    uint        pin_miso;   ///< GPIO: SPI MISO
    uint        pin_nss;    ///< GPIO: chip-select (active-low output)
    uint        pin_busy;   ///< GPIO: BUSY signal (active-high input)
    uint        pin_nreset; ///< GPIO: NRESET (active-low output)

    // ── Internal — initialised by lr11xx_hal_init(); do not modify ────────────
    SemaphoreHandle_t _mutex;      ///< SPI bus mutex (dynamic allocation)
} lr11xx_hal_context_t;

// ── Convenience initialiser ───────────────────────────────────────────────────
// Produces a brace-initialiser expression for lr11xx_hal_context_t with the
// internal fields zero-initialised.  Compatible with C99 and C++17.
//
// Example:
//   static lr11xx_hal_context_t g_radio = LR11XX_HAL_CONTEXT_INIT(
//       spi0, 8000000, 6, 4, 7, 5, 0, 1 );
//
#define LR11XX_HAL_CONTEXT_INIT( _spi, _hz, _sck, _mosi, _miso, _nss, _busy, _nreset ) \
    { (_spi), (_hz), (_sck), (_mosi), (_miso), (_nss), (_busy), (_nreset), NULL }
