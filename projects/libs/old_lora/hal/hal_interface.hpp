#pragma once

#include <cstdint>
#include <cstddef>
#include <concepts>

// Any HAL must provide these primitives.
// Constrain chip driver templates with: template <HalImpl Hal>
template <typename T>
concept HalImpl = requires(T t, uint8_t pin, bool level,
                           const uint8_t* tx, uint8_t* rx, size_t len,
                           uint32_t ms, uint32_t us) {
    // SPI
    { t.spi_transfer(tx, rx, len) } -> std::same_as<void>;
    { t.spi_write(tx, len)        } -> std::same_as<void>;
    { t.spi_read(rx, len)         } -> std::same_as<void>;

    // GPIO
    { t.gpio_set(pin, level) } -> std::same_as<void>;
    { t.gpio_get(pin)        } -> std::convertible_to<bool>;

    // Timing
    { t.delay_ms(ms) } -> std::same_as<void>;
    { t.delay_us(us) } -> std::same_as<void>;
    { t.millis()     } -> std::convertible_to<uint32_t>;
};

// Optional DMA extension for SPI-capable HALs.
template <typename T>
concept HalSpiDmaImpl = requires(T t, int tx_chan, int rx_chan,
                                 const uint8_t* tx, uint8_t* rx, size_t len) {
    { t.spi_dma_init(tx_chan, rx_chan) } -> std::same_as<void>;
    { t.spi_transfer_dma(tx, rx, len)  } -> std::same_as<bool>;
    { t.spi_write_dma(tx, len)         } -> std::same_as<bool>;
    { t.spi_read_dma(rx, len)          } -> std::same_as<bool>;
};
