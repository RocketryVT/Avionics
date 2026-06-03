#pragma once

// SX126x LoRa transceiver driver (SX1261, SX1262, SX1268).
// Platform-independent — all hardware access goes through the Hal template parameter.

#include "../lora_interface.hpp"
#include "../hal/hal_interface.hpp"

#include <cstdint>
#include <cstring>
#include <span>
#include <algorithm>

// ---------------------------------------------------------------------------
// SX126x opcodes (datasheet §13)
// ---------------------------------------------------------------------------
namespace sx126x {

enum Opcode : uint8_t {
    SET_SLEEP              = 0x84,
    SET_STANDBY            = 0x80,
    SET_FS                 = 0xC1,
    SET_TX                 = 0x83,
    SET_RX                 = 0x82,
    SET_RF_FREQUENCY       = 0x86,
    SET_PA_CONFIG          = 0x95,
    SET_TX_PARAMS          = 0x8E,
    SET_PACKET_TYPE        = 0x8A,
    SET_MODULATION_PARAMS  = 0x8B,
    SET_PACKET_PARAMS      = 0x8C,
    SET_BUFFER_BASE_ADDR   = 0x8F,
    SET_DIO_IRQ_PARAMS     = 0x08,
    CLEAR_IRQ_STATUS       = 0x02,
    GET_IRQ_STATUS         = 0x12,
    GET_RX_BUFFER_STATUS   = 0x13,
    READ_BUFFER            = 0x1E,
    WRITE_BUFFER           = 0x0E,
    GET_PACKET_STATUS      = 0x14,
    GET_STATUS             = 0xC0,
    WRITE_REGISTER         = 0x0D,
    READ_REGISTER          = 0x1D,
    SET_DIO2_AS_RF_SWITCH  = 0x9D,
    SET_DIO3_AS_TCXO_CTRL  = 0x97,
    CALIBRATE              = 0x89,
    CALIBRATE_IMAGE        = 0x98,
};

enum StandbyMode : uint8_t {
    STDBY_RC   = 0x00,
    STDBY_XOSC = 0x01,
};

enum PacketType : uint8_t {
    PACKET_TYPE_LORA = 0x01,
};

// IRQ flags
constexpr uint16_t IRQ_TX_DONE        = 0x0001;
constexpr uint16_t IRQ_RX_DONE        = 0x0002;
constexpr uint16_t IRQ_PREAMBLE_DET   = 0x0004;
constexpr uint16_t IRQ_SYNC_WORD_VALID= 0x0008;
constexpr uint16_t IRQ_HEADER_VALID   = 0x0010;
constexpr uint16_t IRQ_HEADER_ERR     = 0x0020;
constexpr uint16_t IRQ_CRC_ERR        = 0x0040;
constexpr uint16_t IRQ_CAD_DONE       = 0x0080;
constexpr uint16_t IRQ_CAD_DETECTED   = 0x0100;
constexpr uint16_t IRQ_TIMEOUT        = 0x0200;
constexpr uint16_t IRQ_ALL            = 0x03FF;

} // namespace sx126x

// ---------------------------------------------------------------------------
// SX126x driver
// ---------------------------------------------------------------------------
template <typename Hal>
    requires HalImpl<Hal>
struct SX126x
{
    struct Config {
        uint8_t  pin_reset;
        uint8_t  pin_busy;
        uint8_t  pin_dio1;
        uint32_t frequency_hz      = 915'000'000;
        uint8_t  bandwidth         = 7;    // 0-9 -> 7.81–500 kHz (LoRa BW index)
        uint8_t  spreading_factor  = 9;    // 5–12
        uint8_t  coding_rate       = 1;    // 1=4/5, 2=4/6, 3=4/7, 4=4/8
        int8_t   tx_power_dbm      = 14;
        bool     use_dio2_rf_switch = true;
        bool     use_tcxo           = false;
    };

    SX126x(Hal& hal, Config cfg)
        : m_hal{hal}, m_cfg{cfg} {}

    // -----------------------------------------------------------------------
    // LoraImpl required methods
    // -----------------------------------------------------------------------

    bool initialize() {
        // Hardware reset
        m_hal.gpio_set(m_cfg.pin_reset, false);
        m_hal.delay_ms(1);
        m_hal.gpio_set(m_cfg.pin_reset, true);
        m_hal.delay_ms(5);
        wait_busy();

        // Set standby RC
        standby();

        // Set packet type to LoRa
        uint8_t pkt_type = sx126x::PACKET_TYPE_LORA;
        write_command(sx126x::SET_PACKET_TYPE, &pkt_type, 1);

        // Use DIO2 as RF switch control if configured
        if (m_cfg.use_dio2_rf_switch) {
            uint8_t enable = 0x01;
            write_command(sx126x::SET_DIO2_AS_RF_SWITCH, &enable, 1);
        }

        // TCXO control via DIO3 if configured
        if (m_cfg.use_tcxo) {
            uint8_t tcxo_params[4] = {
                0x01,   // 1.7V TCXO supply
                0x00, 0x00, 0x64  // 10 ms timeout
            };
            write_command(sx126x::SET_DIO3_AS_TCXO_CTRL, tcxo_params, 4);

            // Calibrate all blocks
            uint8_t calib_param = 0x7F;
            write_command(sx126x::CALIBRATE, &calib_param, 1);
            wait_busy();
        }

        // Calibrate image for the configured frequency band
        calibrate_image();

        // Set frequency
        set_frequency(m_cfg.frequency_hz);

        // Set buffer base addresses (TX at 0, RX at 128)
        uint8_t buf_addrs[2] = {0x00, 0x80};
        write_command(sx126x::SET_BUFFER_BASE_ADDR, buf_addrs, 2);

        // Configure modulation parameters
        set_modulation_params();

        // Configure packet parameters (explicit header, max 255 bytes)
        set_packet_params(255);

        // Set PA config and TX power
        set_pa_config();
        set_tx_power(m_cfg.tx_power_dbm);

        // Enable TX_DONE + RX_DONE + TIMEOUT on DIO1
        set_dio_irq(sx126x::IRQ_TX_DONE | sx126x::IRQ_RX_DONE | sx126x::IRQ_TIMEOUT,
                     sx126x::IRQ_TX_DONE | sx126x::IRQ_RX_DONE | sx126x::IRQ_TIMEOUT);

        return true;
    }

    bool send(std::span<const uint8_t> data) {
        if (data.size() > 255) return false;

        standby();

        // Write payload to TX buffer at offset 0
        write_buffer(0x00, data.data(), data.size());

        // Update packet params with actual payload length
        set_packet_params(static_cast<uint8_t>(data.size()));

        // Clear IRQ flags
        clear_irq(sx126x::IRQ_ALL);

        // Start TX (no timeout — 0x000000)
        uint8_t timeout[3] = {0x00, 0x00, 0x00};
        write_command(sx126x::SET_TX, timeout, 3);

        // Wait for TX_DONE or TIMEOUT
        uint32_t start = m_hal.millis();
        while ((m_hal.millis() - start) < 10'000) {
            uint16_t irq = get_irq_status();
            if (irq & sx126x::IRQ_TX_DONE) {
                clear_irq(sx126x::IRQ_ALL);
                return true;
            }
            if (irq & sx126x::IRQ_TIMEOUT) {
                clear_irq(sx126x::IRQ_ALL);
                return false;
            }
            m_hal.delay_ms(1);
        }

        standby();
        return false;
    }

    int receive(std::span<uint8_t> buf) {
        standby();
        clear_irq(sx126x::IRQ_ALL);

        // Start RX with 5 s timeout (timeout = value * 15.625 µs)
        // 5s = 5000000 / 15.625 = 320000 = 0x04E200
        uint8_t timeout[3] = {0x04, 0xE2, 0x00};
        write_command(sx126x::SET_RX, timeout, 3);

        uint32_t start = m_hal.millis();
        while ((m_hal.millis() - start) < 6'000) {
            uint16_t irq = get_irq_status();
            if (irq & sx126x::IRQ_RX_DONE) {
                clear_irq(sx126x::IRQ_ALL);

                // Get payload length and start offset
                uint8_t status[2] = {};
                read_command(sx126x::GET_RX_BUFFER_STATUS, status, 2);
                uint8_t payload_len = status[0];
                uint8_t rx_offset   = status[1];

                uint8_t len = std::min(payload_len, static_cast<uint8_t>(buf.size()));
                read_buffer(rx_offset, buf.data(), len);

                // Cache RSSI/SNR
                uint8_t pkt_status[3] = {};
                read_command(sx126x::GET_PACKET_STATUS, pkt_status, 3);
                m_last_rssi = static_cast<int16_t>(pkt_status[0]) / -2;
                m_last_snr  = static_cast<int8_t>(pkt_status[1]) / 4;

                return len;
            }
            if (irq & (sx126x::IRQ_TIMEOUT | sx126x::IRQ_CRC_ERR)) {
                clear_irq(sx126x::IRQ_ALL);
                return -1;
            }
            m_hal.delay_ms(1);
        }

        standby();
        return -1;
    }

    void set_frequency(uint32_t freq_hz) {
        m_cfg.frequency_hz = freq_hz;
        // freq_reg = freq_hz * 2^25 / 32 MHz
        uint32_t freq_reg = static_cast<uint32_t>(
            (static_cast<uint64_t>(freq_hz) << 25) / 32'000'000ULL
        );
        uint8_t params[4] = {
            static_cast<uint8_t>((freq_reg >> 24) & 0xFF),
            static_cast<uint8_t>((freq_reg >> 16) & 0xFF),
            static_cast<uint8_t>((freq_reg >>  8) & 0xFF),
            static_cast<uint8_t>((freq_reg >>  0) & 0xFF),
        };
        write_command(sx126x::SET_RF_FREQUENCY, params, 4);
    }

    void set_bandwidth(uint8_t bw) {
        m_cfg.bandwidth = bw;
        set_modulation_params();
    }

    void set_spreading_factor(uint8_t sf) {
        m_cfg.spreading_factor = sf;
        set_modulation_params();
    }

    void set_coding_rate(uint8_t cr) {
        m_cfg.coding_rate = cr;
        set_modulation_params();
    }

    void set_tx_power(int8_t dbm) {
        m_cfg.tx_power_dbm = dbm;
        // Clamp to SX1262 range: -9 to +22 dBm
        if (dbm > 22) dbm = 22;
        if (dbm < -9) dbm = -9;

        uint8_t params[2] = {
            static_cast<uint8_t>(dbm),
            0x04  // ramp time: 200 µs
        };
        write_command(sx126x::SET_TX_PARAMS, params, 2);
    }

    int16_t get_rssi() { return m_last_rssi; }
    int8_t  get_snr()  { return m_last_snr; }

    void sleep() {
        uint8_t cfg = 0x00;  // cold start (no config retention)
        write_command(sx126x::SET_SLEEP, &cfg, 1);
    }

    void standby() {
        uint8_t mode = sx126x::STDBY_RC;
        write_command(sx126x::SET_STANDBY, &mode, 1);
        wait_busy();
    }

private:
    Hal&   m_hal;
    Config m_cfg;

    int16_t m_last_rssi = 0;
    int8_t  m_last_snr  = 0;

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    void wait_busy() {
        while (m_hal.gpio_get(m_cfg.pin_busy)) {
            m_hal.delay_us(100);
        }
    }

    void write_command(uint8_t opcode, const uint8_t* data, size_t len) {
        wait_busy();
        uint8_t cmd = opcode;
        // Send opcode byte, then data, in one CS-low transaction.
        // We do this by manually controlling CS since the HAL wraps it.
        // Use spi_transfer with a combined buffer.
        uint8_t tx_buf[16] = {};
        tx_buf[0] = cmd;
        if (data && len > 0 && (len + 1) <= sizeof(tx_buf)) {
            std::memcpy(&tx_buf[1], data, len);
        }
        uint8_t rx_buf[16] = {};
        m_hal.spi_transfer(tx_buf, rx_buf, len + 1);
    }

    void read_command(uint8_t opcode, uint8_t* data, size_t len) {
        wait_busy();
        // Opcode + NOP byte + data bytes
        uint8_t tx_buf[16] = {};
        uint8_t rx_buf[16] = {};
        tx_buf[0] = opcode;
        // tx_buf[1] = 0x00 (NOP, already zeroed)
        m_hal.spi_transfer(tx_buf, rx_buf, len + 2);
        if (data) {
            std::memcpy(data, &rx_buf[2], len);
        }
    }

    void write_buffer(uint8_t offset, const uint8_t* data, size_t len) {
        wait_busy();
        // WRITE_BUFFER: opcode, offset, then payload
        // For payloads > 14 bytes we need a larger buffer; use two SPI calls.
        uint8_t header[2] = {sx126x::WRITE_BUFFER, offset};
        // Manual CS: write header then payload in one transaction isn't possible
        // with the generic HAL unless we copy. For simplicity, allocate on stack
        // (max 257 bytes for LoRa).
        uint8_t tx_buf[258];
        tx_buf[0] = sx126x::WRITE_BUFFER;
        tx_buf[1] = offset;
        std::memcpy(&tx_buf[2], data, len);
        m_hal.spi_write(tx_buf, len + 2);
    }

    void read_buffer(uint8_t offset, uint8_t* data, size_t len) {
        wait_busy();
        // READ_BUFFER: opcode, offset, NOP, then read data
        uint8_t tx_buf[258] = {};
        uint8_t rx_buf[258] = {};
        tx_buf[0] = sx126x::READ_BUFFER;
        tx_buf[1] = offset;
        // tx_buf[2] = NOP (already 0)
        m_hal.spi_transfer(tx_buf, rx_buf, len + 3);
        std::memcpy(data, &rx_buf[3], len);
    }

    void set_modulation_params() {
        uint8_t params[4] = {
            m_cfg.spreading_factor,
            m_cfg.bandwidth,
            m_cfg.coding_rate,
            0x01  // low data rate optimize: auto
        };
        write_command(sx126x::SET_MODULATION_PARAMS, params, 4);
    }

    void set_packet_params(uint8_t payload_len) {
        uint8_t params[6] = {
            0x00, 0x08,    // preamble length: 8 symbols
            0x00,          // explicit header
            payload_len,
            0x01,          // CRC on
            0x00,          // standard IQ
        };
        write_command(sx126x::SET_PACKET_PARAMS, params, 6);
    }

    void set_pa_config() {
        // SX1262 high-power PA: paDutyCycle=0x04, hpMax=0x07, deviceSel=0x00, paLut=0x01
        uint8_t params[4] = {0x04, 0x07, 0x00, 0x01};
        write_command(sx126x::SET_PA_CONFIG, params, 4);
    }

    void set_dio_irq(uint16_t irq_mask, uint16_t dio1_mask) {
        uint8_t params[8] = {
            static_cast<uint8_t>((irq_mask  >> 8) & 0xFF),
            static_cast<uint8_t>((irq_mask      ) & 0xFF),
            static_cast<uint8_t>((dio1_mask >> 8) & 0xFF),
            static_cast<uint8_t>((dio1_mask     ) & 0xFF),
            0x00, 0x00,  // DIO2 mask
            0x00, 0x00,  // DIO3 mask
        };
        write_command(sx126x::SET_DIO_IRQ_PARAMS, params, 8);
    }

    uint16_t get_irq_status() {
        uint8_t status[2] = {};
        read_command(sx126x::GET_IRQ_STATUS, status, 2);
        return (static_cast<uint16_t>(status[0]) << 8) | status[1];
    }

    void clear_irq(uint16_t mask) {
        uint8_t params[2] = {
            static_cast<uint8_t>((mask >> 8) & 0xFF),
            static_cast<uint8_t>((mask     ) & 0xFF),
        };
        write_command(sx126x::CLEAR_IRQ_STATUS, params, 2);
    }

    void calibrate_image() {
        uint8_t params[2];
        if (m_cfg.frequency_hz >= 902'000'000) {
            params[0] = 0xE1; params[1] = 0xE9;  // 902-928 MHz
        } else if (m_cfg.frequency_hz >= 863'000'000) {
            params[0] = 0xD7; params[1] = 0xDB;  // 863-870 MHz
        } else if (m_cfg.frequency_hz >= 779'000'000) {
            params[0] = 0xC1; params[1] = 0xC5;  // 779-787 MHz
        } else if (m_cfg.frequency_hz >= 470'000'000) {
            params[0] = 0x75; params[1] = 0x81;  // 470-510 MHz
        } else {
            params[0] = 0x6B; params[1] = 0x6F;  // 430-440 MHz
        }
        write_command(sx126x::CALIBRATE_IMAGE, params, 2);
        wait_busy();
    }
};
