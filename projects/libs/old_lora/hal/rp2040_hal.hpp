#pragma once

// RP2040 / RP2350 HAL implementation — the ONLY file that includes pico-sdk headers.

#include <cstdint>
#include <cstddef>

#include <hardware/dma.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <pico/time.h>

#if __has_include("FreeRTOS.h")
#include "FreeRTOS.h"
#include "task.h"
#define LORA_FREERTOS_AVAILABLE 1
#else
#define LORA_FREERTOS_AVAILABLE 0
#endif

struct Rp2040Hal
{
    struct Config {
        spi_inst_t* spi;
        uint8_t     pin_sck;
        uint8_t     pin_mosi;
        uint8_t     pin_miso;
        uint8_t     pin_cs;
        uint32_t    spi_freq_hz = 2'000'000;
    };

    explicit Rp2040Hal(Config cfg) : m_cfg{cfg} {
        // Initialize SPI peripheral
        spi_init(m_cfg.spi, m_cfg.spi_freq_hz);

        // Assign SPI function to pins
        gpio_set_function(m_cfg.pin_sck,  GPIO_FUNC_SPI);
        gpio_set_function(m_cfg.pin_mosi, GPIO_FUNC_SPI);
        gpio_set_function(m_cfg.pin_miso, GPIO_FUNC_SPI);

        // CS is manual GPIO (active low)
        gpio_init(m_cfg.pin_cs);
        gpio_set_dir(m_cfg.pin_cs, GPIO_OUT);
        gpio_put(m_cfg.pin_cs, 1);
    }

    // --- SPI -----------------------------------------------------------
    void spi_transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
        gpio_put(m_cfg.pin_cs, 0);
        spi_write_read_blocking(m_cfg.spi, tx, rx, len);
        gpio_put(m_cfg.pin_cs, 1);
    }

    void spi_write(const uint8_t* data, size_t len) {
        gpio_put(m_cfg.pin_cs, 0);
        spi_write_blocking(m_cfg.spi, data, len);
        gpio_put(m_cfg.pin_cs, 1);
    }

    void spi_read(uint8_t* buf, size_t len) {
        gpio_put(m_cfg.pin_cs, 0);
        spi_read_blocking(m_cfg.spi, 0x00, buf, len);
        gpio_put(m_cfg.pin_cs, 1);
    }

    void spi_dma_init(int tx_channel, int rx_channel) {
        m_dma_tx_channel = tx_channel;
        m_dma_rx_channel = rx_channel;
        m_dma_enabled = true;
        dma_channel_claim(tx_channel);
        dma_channel_claim(rx_channel);
    }

    bool spi_transfer_dma(const uint8_t* tx, uint8_t* rx, size_t len) {
        if (!m_dma_enabled || !tx || !rx || len == 0) {
            return false;
        }

        gpio_put(m_cfg.pin_cs, 0);

        dma_channel_config cfg_rx = dma_channel_get_default_config(m_dma_rx_channel);
        channel_config_set_transfer_data_size(&cfg_rx, DMA_SIZE_8);
        channel_config_set_dreq(&cfg_rx, spi_get_dreq(m_cfg.spi, false));
        channel_config_set_read_increment(&cfg_rx, false);
        channel_config_set_write_increment(&cfg_rx, true);
        dma_channel_configure(
            m_dma_rx_channel,
            &cfg_rx,
            rx,
            &spi_get_hw(m_cfg.spi)->dr,
            len,
            false
        );

        dma_channel_config cfg_tx = dma_channel_get_default_config(m_dma_tx_channel);
        channel_config_set_transfer_data_size(&cfg_tx, DMA_SIZE_8);
        channel_config_set_dreq(&cfg_tx, spi_get_dreq(m_cfg.spi, true));
        channel_config_set_read_increment(&cfg_tx, true);
        channel_config_set_write_increment(&cfg_tx, false);
        dma_channel_configure(
            m_dma_tx_channel,
            &cfg_tx,
            &spi_get_hw(m_cfg.spi)->dr,
            tx,
            len,
            false
        );

        dma_start_channel_mask((1u << m_dma_tx_channel) | (1u << m_dma_rx_channel));
        dma_channel_wait_for_finish_blocking(m_dma_tx_channel);
        dma_channel_wait_for_finish_blocking(m_dma_rx_channel);

        gpio_put(m_cfg.pin_cs, 1);
        return true;
    }

    bool spi_write_dma(const uint8_t* data, size_t len) {
        if (!m_dma_enabled || !data || len == 0) {
            return false;
        }

        uint8_t sink = 0;
        gpio_put(m_cfg.pin_cs, 0);

        dma_channel_config cfg_rx = dma_channel_get_default_config(m_dma_rx_channel);
        channel_config_set_transfer_data_size(&cfg_rx, DMA_SIZE_8);
        channel_config_set_dreq(&cfg_rx, spi_get_dreq(m_cfg.spi, false));
        channel_config_set_read_increment(&cfg_rx, false);
        channel_config_set_write_increment(&cfg_rx, false);
        dma_channel_configure(
            m_dma_rx_channel,
            &cfg_rx,
            &sink,
            &spi_get_hw(m_cfg.spi)->dr,
            len,
            false
        );

        dma_channel_config cfg_tx = dma_channel_get_default_config(m_dma_tx_channel);
        channel_config_set_transfer_data_size(&cfg_tx, DMA_SIZE_8);
        channel_config_set_dreq(&cfg_tx, spi_get_dreq(m_cfg.spi, true));
        channel_config_set_read_increment(&cfg_tx, true);
        channel_config_set_write_increment(&cfg_tx, false);
        dma_channel_configure(
            m_dma_tx_channel,
            &cfg_tx,
            &spi_get_hw(m_cfg.spi)->dr,
            data,
            len,
            false
        );

        dma_start_channel_mask((1u << m_dma_tx_channel) | (1u << m_dma_rx_channel));
        dma_channel_wait_for_finish_blocking(m_dma_tx_channel);
        dma_channel_wait_for_finish_blocking(m_dma_rx_channel);

        gpio_put(m_cfg.pin_cs, 1);
        return true;
    }

    bool spi_read_dma(uint8_t* buf, size_t len) {
        if (!m_dma_enabled || !buf || len == 0) {
            return false;
        }

        const uint8_t dummy = 0x00;
        gpio_put(m_cfg.pin_cs, 0);

        dma_channel_config cfg_rx = dma_channel_get_default_config(m_dma_rx_channel);
        channel_config_set_transfer_data_size(&cfg_rx, DMA_SIZE_8);
        channel_config_set_dreq(&cfg_rx, spi_get_dreq(m_cfg.spi, false));
        channel_config_set_read_increment(&cfg_rx, false);
        channel_config_set_write_increment(&cfg_rx, true);
        dma_channel_configure(
            m_dma_rx_channel,
            &cfg_rx,
            buf,
            &spi_get_hw(m_cfg.spi)->dr,
            len,
            false
        );

        dma_channel_config cfg_tx = dma_channel_get_default_config(m_dma_tx_channel);
        channel_config_set_transfer_data_size(&cfg_tx, DMA_SIZE_8);
        channel_config_set_dreq(&cfg_tx, spi_get_dreq(m_cfg.spi, true));
        channel_config_set_read_increment(&cfg_tx, false);
        channel_config_set_write_increment(&cfg_tx, false);
        dma_channel_configure(
            m_dma_tx_channel,
            &cfg_tx,
            &spi_get_hw(m_cfg.spi)->dr,
            &dummy,
            len,
            false
        );

        dma_start_channel_mask((1u << m_dma_tx_channel) | (1u << m_dma_rx_channel));
        dma_channel_wait_for_finish_blocking(m_dma_tx_channel);
        dma_channel_wait_for_finish_blocking(m_dma_rx_channel);

        gpio_put(m_cfg.pin_cs, 1);
        return true;
    }

    // --- GPIO ----------------------------------------------------------
    void gpio_set(uint8_t pin, bool high) {
        gpio_put(pin, high ? 1 : 0);
    }

    bool gpio_get(uint8_t pin) {
        return ::gpio_get(pin);
    }

    // --- Timing --------------------------------------------------------
    void delay_ms(uint32_t ms) {
#if LORA_FREERTOS_AVAILABLE
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            vTaskDelay(pdMS_TO_TICKS(ms));
            return;
        }
#endif
        sleep_ms(ms);
    }
    void delay_us(uint32_t us) { sleep_us(us); }

    uint32_t millis() {
        return static_cast<uint32_t>(time_us_64() / 1000);
    }

private:
    Config m_cfg;
    bool m_dma_enabled = false;
    int m_dma_tx_channel = -1;
    int m_dma_rx_channel = -1;
};
