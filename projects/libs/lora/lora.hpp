#pragma once

// Convenience header — pull in the HAL for the current platform and
// provide a default `Lora` type alias for quick use.
//
// Usage:
//   #include "lora.hpp"
//
//   Rp2040Hal hal({.spi = spi0, .pin_sck = 18, ...});
//   Lora radio(hal, {.pin_reset = 5, .pin_busy = 6, .pin_dio1 = 7});
//   radio.initialize();
//   radio.send({buf, len});

#include "hal/rp2040_hal.hpp"
#include "SX126x/sx126x.hpp"

using Lora = SX126x<Rp2040Hal>;
