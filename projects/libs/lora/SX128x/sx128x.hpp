#pragma once

// SX128x LoRa transceiver driver (SX1280, SX1281, SX1282).
// LoRa-only, blocking and non-blocking TX/RX.

#include "../lora_interface.hpp"
#include "../hal/hal_interface.hpp"

#include <array>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <span>

namespace sx128x {

enum Opcode : uint8_t {
    SET_SLEEP            = 0x84,
    SET_STANDBY          = 0x80,
    SET_TX               = 0x83,
    SET_RX               = 0x82,
    SET_RF_FREQUENCY     = 0x86,
    SET_TX_PARAMS        = 0x8E,
    SET_PACKET_TYPE      = 0x8A,
    SET_MODULATION_PARAMS= 0x8B,
    SET_PACKET_PARAMS    = 0x8C,
    SET_BUFFER_BASE_ADDR = 0x8F,
    SET_DIO_IRQ_PARAMS   = 0x8D,
    GET_IRQ_STATUS       = 0x15,
    CLEAR_IRQ_STATUS     = 0x97,
    GET_RX_BUFFER_STATUS = 0x17,
    READ_BUFFER          = 0x1B,
    WRITE_BUFFER         = 0x1A,
    GET_PACKET_STATUS    = 0x1D,
    WRITE_REGISTER       = 0x18,
    READ_REGISTER        = 0x19,
};

inline constexpr uint8_t PACKET_TYPE_LORA = 0x01;
inline constexpr uint8_t STDBY_RC         = 0x00;

// IRQ flags
inline constexpr uint16_t IRQ_TX_DONE   = 0x0001;
inline constexpr uint16_t IRQ_RX_DONE   = 0x0002;
inline constexpr uint16_t IRQ_CRC_ERR   = 0x0040;
inline constexpr uint16_t IRQ_TIMEOUT   = 0x4000;
inline constexpr uint16_t IRQ_ALL       = 0xFFFF;

// LoRa params
inline constexpr uint8_t LORA_HEADER_EXPLICIT = 0x00;
inline constexpr uint8_t LORA_HEADER_IMPLICIT = 0x80;
inline constexpr uint8_t LORA_CRC_ON          = 0x20;
inline constexpr uint8_t LORA_IQ_STANDARD     = 0x40;

// LoRa BW codes (kHz)
inline constexpr std::array<uint8_t, 4> kBwReg = {
    0x34, 0x26, 0x18, 0x0A
};
inline constexpr std::array<uint32_t, 4> kBwHz = {
    203'125, 406'250, 812'500, 1'625'000
};

inline constexpr std::array<uint8_t, 8> kSfReg = {
    0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0
};

inline constexpr std::array<uint8_t, 4> kCrReg = {
    0x01, 0x02, 0x03, 0x04
};

inline constexpr uint16_t REG_LORA_SF_CONFIG         = 0x0925;
inline constexpr uint16_t REG_FREQ_ERROR_CORRECTION  = 0x093C;

}  // namespace sx128x

template <typename Hal>
    requires HalImpl<Hal>
struct SX128x
{
    struct Config {
        uint8_t  pin_reset;
        uint8_t  pin_busy;
        uint8_t  pin_dio1;
        uint32_t frequency_hz        = 2'400'000'000;
        uint8_t  bandwidth           = 1;   // index into kBwReg (0..3)
        uint8_t  spreading_factor    = 9;   // 5..12
        uint8_t  coding_rate         = 1;   // 1..4 (4/5..4/8)
        int8_t   tx_power_dbm        = 13;  // -18..13
        bool     implicit_header     = false;
        uint8_t  implicit_header_len = 0;
    };

    SX128x(Hal& hal, Config cfg) : m_hal{hal}, m_cfg{cfg} {}

    bool initialize() {
        reset();
        standby();

        uint8_t pkt_type = sx128x::PACKET_TYPE_LORA;
        write_command(sx128x::SET_PACKET_TYPE, &pkt_type, 1);

        // Buffer base: TX=0, RX=128
        uint8_t base[2] = {0x00, 0x80};
        write_command(sx128x::SET_BUFFER_BASE_ADDR, base, 2);

        set_frequency(m_cfg.frequency_hz);
        apply_modem_config();
        set_tx_power(m_cfg.tx_power_dbm);

        // Enable TX_DONE/RX_DONE/TIMEOUT/CRC on DIO1
        set_dio_irq(sx128x::IRQ_TX_DONE | sx128x::IRQ_RX_DONE | sx128x::IRQ_TIMEOUT | sx128x::IRQ_CRC_ERR,
                    sx128x::IRQ_TX_DONE | sx128x::IRQ_RX_DONE | sx128x::IRQ_TIMEOUT | sx128x::IRQ_CRC_ERR);

        clear_irq(sx128x::IRQ_ALL);
        return true;
    }

    bool send(std::span<const uint8_t> data) {
        if (!start_send(data)) {
            return false;
        }
        while (true) {
            const int result = poll_send();
            if (result > 0) {
                return true;
            }
            if (result < 0) {
                return false;
            }
            m_hal.delay_ms(1);
        }
    }

    int receive(std::span<uint8_t> buf) {
        if (!start_receive(buf)) {
            return -1;
        }
        while (true) {
            const int result = poll_receive();
            if (result != 0) {
                return result;
            }
            m_hal.delay_ms(1);
        }
    }

    void set_frequency(uint32_t hz) {
        m_cfg.frequency_hz = hz;
        const uint32_t frf = static_cast<uint32_t>(
            (static_cast<uint64_t>(hz) << 18) / 52'000'000ULL
        );
        uint8_t params[3] = {
            static_cast<uint8_t>((frf >> 16) & 0xFF),
            static_cast<uint8_t>((frf >> 8) & 0xFF),
            static_cast<uint8_t>(frf & 0xFF)
        };
        write_command(sx128x::SET_RF_FREQUENCY, params, 3);
    }

    void set_bandwidth(uint8_t bw) {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(bw, 3));
        apply_modem_config();
    }

    void set_spreading_factor(uint8_t sf) {
        m_cfg.spreading_factor = static_cast<uint8_t>(std::clamp<int>(sf, 5, 12));
        apply_modem_config();
    }

    void set_coding_rate(uint8_t cr) {
        m_cfg.coding_rate = static_cast<uint8_t>(std::clamp<int>(cr, 1, 4));
        apply_modem_config();
    }

    void set_tx_power(int8_t dbm) {
        dbm = static_cast<int8_t>(std::clamp<int>(dbm, -18, 13));
        m_cfg.tx_power_dbm = dbm;
        const uint8_t power = static_cast<uint8_t>(dbm + 18);
        uint8_t params[2] = {power, 0xE0};
        write_command(sx128x::SET_TX_PARAMS, params, 2);
    }

    int16_t get_rssi() { return m_last_rssi; }
    int8_t  get_snr()  { return m_last_snr; }

    void sleep() {
        uint8_t cfg = 0x00;
        write_command(sx128x::SET_SLEEP, &cfg, 1);
    }

    void standby() {
        uint8_t mode = sx128x::STDBY_RC;
        write_command(sx128x::SET_STANDBY, &mode, 1);
        wait_busy();
    }

    void set_implicit_header(uint8_t length) {
        m_cfg.implicit_header = true;
        m_cfg.implicit_header_len = length;
        apply_packet_params(length);
    }

    void set_explicit_header() {
        m_cfg.implicit_header = false;
        apply_packet_params(0);
    }

    bool start_send(std::span<const uint8_t> data) {
        if (m_pending != Pending::None) {
            return false;
        }
        if (data.size() > kMaxPayload) {
            return false;
        }

        standby();
        clear_irq(sx128x::IRQ_ALL);

        write_buffer(0x00, data);
        apply_packet_params(static_cast<uint8_t>(data.size()));

        uint8_t timeout[3] = {0x00, 0x00, 0x00};
        write_command(sx128x::SET_TX, timeout, 3);

        m_pending = Pending::Tx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_send() {
        if (m_pending != Pending::Tx) {
            return 0;
        }

        if ((m_hal.millis() - m_op_start_ms) > kTxTimeoutMs) {
            clear_irq(sx128x::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return -1;
        }

        const uint16_t irq = get_irq_status();
        if (irq & sx128x::IRQ_TX_DONE) {
            clear_irq(sx128x::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return 1;
        }
        if (irq & sx128x::IRQ_TIMEOUT) {
            clear_irq(sx128x::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return -1;
        }

        return 0;
    }

    bool start_receive(std::span<uint8_t> buf, bool continuous = false) {
        if (m_pending != Pending::None) {
            return false;
        }
        if (buf.empty()) {
            return false;
        }

        standby();
        clear_irq(sx128x::IRQ_ALL);

        const uint8_t implicit_len = select_implicit_len(buf.size());
        if (use_implicit_header()) {
            apply_packet_params(implicit_len);
        }

        m_rx_buf = buf.data();
        m_rx_buf_len = buf.size();
        m_rx_expected = implicit_len;
        m_rx_continuous = continuous;

        uint8_t timeout[3];
        if (continuous) {
            timeout[0] = 0x00;
            timeout[1] = 0xFF;
            timeout[2] = 0xFF;
        } else {
            timeout[0] = 0x02;  // 1 ms
            timeout[1] = static_cast<uint8_t>((kRxTimeoutMs >> 8) & 0xFF);
            timeout[2] = static_cast<uint8_t>(kRxTimeoutMs & 0xFF);
        }
        write_command(sx128x::SET_RX, timeout, 3);

        m_pending = Pending::Rx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_receive() {
        if (m_pending != Pending::Rx) {
            return 0;
        }

        if (!m_rx_continuous && (m_hal.millis() - m_op_start_ms) > kRxTimeoutMs) {
            clear_irq(sx128x::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return -1;
        }

        const uint16_t irq = get_irq_status();
        if (irq & sx128x::IRQ_RX_DONE) {
            if (irq & sx128x::IRQ_CRC_ERR) {
                clear_irq(sx128x::IRQ_ALL);
                if (!m_rx_continuous) {
                    standby();
                }
                m_pending = Pending::None;
                return -1;
            }

            uint8_t status[2] = {};
            read_command(sx128x::GET_RX_BUFFER_STATUS, status, 2);
            uint8_t payload_len = status[0];
            uint8_t offset = status[1];
            if (payload_len == 0 && m_rx_expected != 0) {
                payload_len = m_rx_expected;
            }

            const uint8_t copy_len = static_cast<uint8_t>(
                std::min<size_t>(payload_len, m_rx_buf_len)
            );
            read_buffer(offset, {m_rx_buf, copy_len});

            uint8_t pkt_status[5] = {};
            read_command(sx128x::GET_PACKET_STATUS, pkt_status, 5);
            const uint8_t rssi_sync = pkt_status[0];
            const int8_t snr_raw = static_cast<int8_t>(pkt_status[1]);
            m_last_snr = static_cast<int8_t>(snr_raw / 4);
            int16_t rssi = static_cast<int16_t>(-static_cast<int16_t>(rssi_sync) / 2);
            if (snr_raw < 0) {
                rssi = static_cast<int16_t>(rssi + m_last_snr);
            }
            m_last_rssi = rssi;

            clear_irq(sx128x::IRQ_ALL);
            if (!m_rx_continuous) {
                standby();
            }
            m_pending = Pending::None;
            return copy_len;
        }

        if (irq & (sx128x::IRQ_TIMEOUT | sx128x::IRQ_CRC_ERR)) {
            clear_irq(sx128x::IRQ_ALL);
            if (!m_rx_continuous) {
                standby();
            }
            m_pending = Pending::None;
            return -1;
        }

        return 0;
    }

private:
    static constexpr size_t kMaxPayload = 255;
    static constexpr uint32_t kTxTimeoutMs = 5'000;
    static constexpr uint32_t kRxTimeoutMs = 6'000;

    enum class Pending : uint8_t {
        None,
        Tx,
        Rx
    };

    Hal&   m_hal;
    Config m_cfg;

    int16_t m_last_rssi = 0;
    int8_t  m_last_snr  = 0;
    Pending m_pending = Pending::None;
    uint32_t m_op_start_ms = 0;
    uint8_t* m_rx_buf = nullptr;
    size_t m_rx_buf_len = 0;
    uint8_t m_rx_expected = 0;
    bool m_rx_continuous = false;

    void reset() {
        m_hal.gpio_set(m_cfg.pin_reset, false);
        m_hal.delay_ms(1);
        m_hal.gpio_set(m_cfg.pin_reset, true);
        m_hal.delay_ms(5);
        wait_busy();
    }

    void wait_busy() {
        while (m_hal.gpio_get(m_cfg.pin_busy)) {
            m_hal.delay_us(100);
        }
    }

    void write_command(uint8_t opcode, const uint8_t* data, size_t len) {
        wait_busy();
        uint8_t tx_buf[260] = {};
        tx_buf[0] = opcode;
        if (data && len > 0) {
            std::memcpy(&tx_buf[1], data, len);
        }
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_write_dma(tx_buf, len + 1)) {
                return;
            }
        }
        m_hal.spi_write(tx_buf, len + 1);
    }

    void read_command(uint8_t opcode, uint8_t* data, size_t len) {
        wait_busy();
        uint8_t tx_buf[260] = {};
        uint8_t rx_buf[260] = {};
        tx_buf[0] = opcode;
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_transfer_dma(tx_buf, rx_buf, len + 1)) {
                std::memcpy(data, &rx_buf[1], len);
                return;
            }
        }
        m_hal.spi_transfer(tx_buf, rx_buf, len + 1);
        if (data) {
            std::memcpy(data, &rx_buf[1], len);
        }
    }

    void write_buffer(uint8_t offset, std::span<const uint8_t> data) {
        wait_busy();
        std::array<uint8_t, kMaxPayload + 2> tx{};
        tx[0] = sx128x::WRITE_BUFFER;
        tx[1] = offset;
        std::copy(data.begin(), data.end(), tx.begin() + 2);
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_write_dma(tx.data(), data.size() + 2)) {
                return;
            }
        }
        m_hal.spi_write(tx.data(), data.size() + 2);
    }

    void read_buffer(uint8_t offset, std::span<uint8_t> data) {
        wait_busy();
        std::array<uint8_t, kMaxPayload + 3> tx{};
        std::array<uint8_t, kMaxPayload + 3> rx{};
        tx[0] = sx128x::READ_BUFFER;
        tx[1] = offset;
        tx[2] = 0x00;
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_transfer_dma(tx.data(), rx.data(), data.size() + 3)) {
                std::copy(rx.begin() + 3, rx.begin() + 3 + data.size(), data.begin());
                return;
            }
        }
        m_hal.spi_transfer(tx.data(), rx.data(), data.size() + 3);
        std::copy(rx.begin() + 3, rx.begin() + 3 + data.size(), data.begin());
    }

    void write_register(uint16_t addr, uint8_t value) {
        uint8_t tx[4] = {
            sx128x::WRITE_REGISTER,
            static_cast<uint8_t>((addr >> 8) & 0xFF),
            static_cast<uint8_t>(addr & 0xFF),
            value
        };
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_write_dma(tx, sizeof(tx))) {
                return;
            }
        }
        m_hal.spi_write(tx, sizeof(tx));
    }

    uint8_t read_register(uint16_t addr) {
        uint8_t tx[5] = {
            sx128x::READ_REGISTER,
            static_cast<uint8_t>((addr >> 8) & 0xFF),
            static_cast<uint8_t>(addr & 0xFF),
            0x00,
            0x00
        };
        uint8_t rx[5] = {};
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_transfer_dma(tx, rx, sizeof(tx))) {
                return rx[4];
            }
        }
        m_hal.spi_transfer(tx, rx, sizeof(tx));
        return rx[4];
    }

    void set_dio_irq(uint16_t irq_mask, uint16_t dio1_mask) {
        uint8_t params[8] = {
            static_cast<uint8_t>((irq_mask >> 8) & 0xFF),
            static_cast<uint8_t>(irq_mask & 0xFF),
            static_cast<uint8_t>((dio1_mask >> 8) & 0xFF),
            static_cast<uint8_t>(dio1_mask & 0xFF),
            0x00, 0x00,
            0x00, 0x00
        };
        write_command(sx128x::SET_DIO_IRQ_PARAMS, params, 8);
    }

    uint16_t get_irq_status() {
        uint8_t status[2] = {};
        read_command(sx128x::GET_IRQ_STATUS, status, 2);
        return static_cast<uint16_t>(status[0] << 8) | status[1];
    }

    void clear_irq(uint16_t mask) {
        uint8_t params[2] = {
            static_cast<uint8_t>((mask >> 8) & 0xFF),
            static_cast<uint8_t>(mask & 0xFF)
        };
        write_command(sx128x::CLEAR_IRQ_STATUS, params, 2);
    }

    static uint8_t encode_bw(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, sx128x::kBwReg.size() - 1));
        return sx128x::kBwReg[idx];
    }

    static uint32_t bandwidth_hz(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, sx128x::kBwHz.size() - 1));
        return sx128x::kBwHz[idx];
    }

    static uint8_t encode_cr(uint8_t cr) {
        const uint8_t idx = static_cast<uint8_t>(std::clamp<int>(cr, 1, 4) - 1);
        return sx128x::kCrReg[idx];
    }

    static uint8_t encode_sf(uint8_t sf) {
        const uint8_t idx = static_cast<uint8_t>(std::clamp<int>(sf, 5, 12) - 5);
        return sx128x::kSfReg[idx];
    }

    bool ldro_needed(uint8_t sf) const {
        const uint32_t bw = bandwidth_hz(m_cfg.bandwidth);
        const float symbol_ms = (static_cast<float>(1u << sf) * 1000.0f) / static_cast<float>(bw);
        return symbol_ms >= 16.0f;
    }

    void apply_sf_config(uint8_t sf) {
        uint8_t val = 0x32;
        if (sf == 5 || sf == 6) {
            val = 0x1E;
        } else if (sf == 7 || sf == 8) {
            val = 0x37;
        }
        write_register(sx128x::REG_LORA_SF_CONFIG, val);
        uint8_t fec = read_register(sx128x::REG_FREQ_ERROR_CORRECTION);
        write_register(sx128x::REG_FREQ_ERROR_CORRECTION, static_cast<uint8_t>(fec | 0x01));
    }

    void apply_modem_config() {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(m_cfg.bandwidth, 3));
        m_cfg.coding_rate = static_cast<uint8_t>(std::clamp<int>(m_cfg.coding_rate, 1, 4));
        m_cfg.spreading_factor = static_cast<uint8_t>(std::clamp<int>(m_cfg.spreading_factor, 5, 12));

        const uint8_t bw = encode_bw(m_cfg.bandwidth);
        const uint8_t cr = encode_cr(m_cfg.coding_rate);
        const uint8_t sf = encode_sf(m_cfg.spreading_factor);

        uint8_t params[3] = {sf, bw, cr};
        write_command(sx128x::SET_MODULATION_PARAMS, params, 3);
        apply_sf_config(m_cfg.spreading_factor);

        apply_packet_params(use_implicit_header() ? select_implicit_len(kMaxPayload) : 0);
    }

    void apply_packet_params(uint8_t payload_len) {
        uint8_t hdr = use_implicit_header() ? sx128x::LORA_HEADER_IMPLICIT : sx128x::LORA_HEADER_EXPLICIT;
        uint8_t len = payload_len;
        if (!use_implicit_header()) {
            len = static_cast<uint8_t>(std::min<size_t>(payload_len == 0 ? kMaxPayload : payload_len, kMaxPayload));
        }
        uint8_t params[7] = {
            0x08,
            hdr,
            len,
            sx128x::LORA_CRC_ON,
            sx128x::LORA_IQ_STANDARD,
            0x00,
            0x00
        };
        write_command(sx128x::SET_PACKET_PARAMS, params, 7);
    }

    bool use_implicit_header() const {
        return m_cfg.implicit_header || (m_cfg.spreading_factor == 5 || m_cfg.spreading_factor == 6);
    }

    uint8_t select_implicit_len(size_t buf_len) const {
        if (!use_implicit_header()) {
            return 0;
        }
        if (m_cfg.implicit_header_len != 0) {
            return m_cfg.implicit_header_len;
        }
        return static_cast<uint8_t>(std::min<size_t>(buf_len, kMaxPayload));
    }
};

// Variant aliases
template <typename Hal>
using SX1280 = SX128x<Hal>;

template <typename Hal>
using SX1281 = SX128x<Hal>;

template <typename Hal>
using SX1282 = SX128x<Hal>;
