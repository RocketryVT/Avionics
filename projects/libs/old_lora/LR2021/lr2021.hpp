#pragma once

// LR2021 LoRa transceiver driver.
// LoRa-only, blocking and non-blocking TX/RX.

#include "../lora_interface.hpp"
#include "../hal/hal_interface.hpp"

#include <array>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <span>

namespace lr2021 {

// Commands (subset)
inline constexpr uint16_t CMD_READ_RX_FIFO             = 0x0001;
inline constexpr uint16_t CMD_WRITE_TX_FIFO            = 0x0002;
inline constexpr uint16_t CMD_SET_SLEEP                = 0x0127;
inline constexpr uint16_t CMD_SET_STANDBY              = 0x0128;
inline constexpr uint16_t CMD_CALIBRATE                = 0x0122;
inline constexpr uint16_t CMD_SET_DIO_FUNCTION         = 0x0112;
inline constexpr uint16_t CMD_SET_DIO_IRQ_CONFIG       = 0x0115;
inline constexpr uint16_t CMD_CLEAR_IRQ                = 0x0116;
inline constexpr uint16_t CMD_GET_AND_CLEAR_IRQ_STATUS = 0x0117;
inline constexpr uint16_t CMD_SET_RX_TX_FALLBACK_MODE  = 0x0206;
inline constexpr uint16_t CMD_SET_RF_FREQUENCY         = 0x0200;
inline constexpr uint16_t CMD_SET_RX_PATH              = 0x0201;
inline constexpr uint16_t CMD_SET_PA_CONFIG            = 0x0202;
inline constexpr uint16_t CMD_SET_TX_PARAMS            = 0x0203;
inline constexpr uint16_t CMD_SET_PACKET_TYPE          = 0x0207;
inline constexpr uint16_t CMD_SET_RX                   = 0x020C;
inline constexpr uint16_t CMD_SET_TX                   = 0x020D;
inline constexpr uint16_t CMD_GET_RX_PKT_LENGTH        = 0x0212;
inline constexpr uint16_t CMD_SET_LORA_MODULATION_PARAMS = 0x0220;
inline constexpr uint16_t CMD_SET_LORA_PACKET_PARAMS   = 0x0221;
inline constexpr uint16_t CMD_SET_LORA_SYNCWORD        = 0x0223;
inline constexpr uint16_t CMD_GET_LORA_PACKET_STATUS   = 0x022A;

// Packet type
inline constexpr uint8_t PACKET_TYPE_LORA = 0x00;

// Standby
inline constexpr uint8_t STDBY_RC = 0x00;

// IRQ flags
inline constexpr uint32_t IRQ_RX_DONE   = 0x01UL << 18;
inline constexpr uint32_t IRQ_TX_DONE   = 0x01UL << 19;
inline constexpr uint32_t IRQ_TIMEOUT   = 0x01UL << 21;
inline constexpr uint32_t IRQ_CRC_ERROR = 0x01UL << 22;
inline constexpr uint32_t IRQ_ALL       = 0xFFFFFFFFUL;

// LoRa params
inline constexpr uint8_t LORA_HEADER_EXPLICIT = 0x00;
inline constexpr uint8_t LORA_HEADER_IMPLICIT = 0x01;
inline constexpr uint8_t LORA_CRC_ON          = 0x01;
inline constexpr uint8_t LORA_IQ_STANDARD     = 0x00;
inline constexpr uint8_t LORA_SYNC_WORD_PRIVATE = 0x12;

// LoRa BW codes (kHz)
inline constexpr std::array<uint8_t, 12> kBwReg = {
    0x02, 0x0A, 0x03, 0x0B, 0x0C, 0x04, 0x0D, 0x05, 0x0E, 0x06, 0x0F, 0x07
};
inline constexpr std::array<uint32_t, 12> kBwHz = {
    31'250, 41'667, 62'500, 83'340, 101'000, 125'000, 203'000, 250'000, 406'000, 500'000, 812'000, 1'000'000
};

inline constexpr std::array<uint8_t, 4> kCrReg = {
    0x01, 0x02, 0x03, 0x04
};

// DIO function
inline constexpr uint8_t DIO_FUNCTION_IRQ  = 0x10;
inline constexpr uint8_t DIO_SLEEP_PULL_NONE = 0x00;
inline constexpr uint8_t DIO_SLEEP_PULL_UP   = 0x02;

// Sleep / calibration
inline constexpr uint8_t SLEEP_RETENTION_ENABLED = 0x02;
inline constexpr uint8_t CALIBRATE_ALL = 0x6F;

// RX/TX
inline constexpr uint32_t RX_TIMEOUT_NONE = 0x000000;
inline constexpr uint32_t RX_TIMEOUT_INF  = 0xFFFFFF;
inline constexpr uint8_t  FALLBACK_STDBY_RC = 0x01;

// RX path
inline constexpr uint8_t RX_PATH_LF  = 0x00;
inline constexpr uint8_t RX_PATH_HF  = 0x01;
inline constexpr uint8_t RX_BOOST_LF = 0x00;
inline constexpr uint8_t RX_BOOST_HF = 0x04;

// PA
inline constexpr uint8_t PA_LF_MODE_FSM          = 0x00;
inline constexpr uint8_t PA_LF_DUTY_CYCLE_UNUSED = 0x60;
inline constexpr uint8_t PA_LF_SLICES_UNUSED     = 0x07;
inline constexpr uint8_t PA_HF_DUTY_CYCLE_UNUSED = 0x10;
inline constexpr uint8_t PA_RAMP_48U             = 0x05;

}  // namespace lr2021

template <typename Hal>
    requires HalImpl<Hal>
struct LR2021
{
    struct Config {
        uint8_t  pin_reset;
        uint8_t  pin_busy;
        uint8_t  pin_dio1;
        uint8_t  dio_irq_num         = 5;   // LR2021 DIO number (5..11)
        uint32_t frequency_hz        = 915'000'000;
        uint8_t  bandwidth           = 5;   // index into kBwReg (0..11), default 125 kHz
        uint8_t  spreading_factor    = 9;   // 5..12
        uint8_t  coding_rate         = 1;   // 1..4 (4/5..4/8)
        int8_t   tx_power_dbm        = 14;
        bool     implicit_header     = false;
        uint8_t  implicit_header_len = 0;
    };

    LR2021(Hal& hal, Config cfg) : m_hal{hal}, m_cfg{cfg} {}

    bool initialize() {
        reset();
        standby();

        set_rx_tx_fallback(lr2021::FALLBACK_STDBY_RC);

        const uint8_t dio = irq_dio_num();
        const uint8_t pull = (dio == 5) ? lr2021::DIO_SLEEP_PULL_UP : lr2021::DIO_SLEEP_PULL_NONE;
        set_dio_function(dio, lr2021::DIO_FUNCTION_IRQ, pull);
        set_dio_irq(dio, lr2021::IRQ_TX_DONE | lr2021::IRQ_RX_DONE | lr2021::IRQ_TIMEOUT | lr2021::IRQ_CRC_ERROR);
        clear_irq(lr2021::IRQ_ALL);

        calibrate_all();

        uint8_t pkt = lr2021::PACKET_TYPE_LORA;
        write_command(lr2021::CMD_SET_PACKET_TYPE, &pkt, 1);

        set_frequency(m_cfg.frequency_hz);
        apply_modem_config();
        set_lora_syncword(lr2021::LORA_SYNC_WORD_PRIVATE);
        set_tx_power(m_cfg.tx_power_dbm);

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
        uint8_t params[4] = {
            static_cast<uint8_t>((hz >> 24) & 0xFF),
            static_cast<uint8_t>((hz >> 16) & 0xFF),
            static_cast<uint8_t>((hz >> 8) & 0xFF),
            static_cast<uint8_t>(hz & 0xFF)
        };
        write_command(lr2021::CMD_SET_RF_FREQUENCY, params, 4);
    }

    void set_bandwidth(uint8_t bw) {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(bw, 11));
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
        m_cfg.tx_power_dbm = clamp_tx_power(dbm);
        const bool high_freq = is_high_freq();
        const uint8_t pa = static_cast<uint8_t>(high_freq ? 1 : 0);
        uint8_t pa_cfg[5] = {
            static_cast<uint8_t>(pa << 7),
            lr2021::PA_LF_MODE_FSM,
            lr2021::PA_LF_DUTY_CYCLE_UNUSED,
            lr2021::PA_LF_SLICES_UNUSED,
            lr2021::PA_HF_DUTY_CYCLE_UNUSED
        };
        write_command(lr2021::CMD_SET_PA_CONFIG, pa_cfg, 5);

        uint8_t params[2] = {
            static_cast<uint8_t>(m_cfg.tx_power_dbm * 2),
            lr2021::PA_RAMP_48U
        };
        write_command(lr2021::CMD_SET_TX_PARAMS, params, 2);
    }

    int16_t get_rssi() { return m_last_rssi; }
    int8_t  get_snr()  { return m_last_snr; }

    void sleep() {
        uint8_t cfg[5] = {lr2021::SLEEP_RETENTION_ENABLED, 0x00, 0x00, 0x00, 0x00};
        write_command(lr2021::CMD_SET_SLEEP, cfg, sizeof(cfg));
    }

    void standby() {
        uint8_t mode = lr2021::STDBY_RC;
        write_command(lr2021::CMD_SET_STANDBY, &mode, 1);
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
        clear_irq(lr2021::IRQ_ALL);

        write_tx_fifo(data);
        apply_packet_params(static_cast<uint8_t>(data.size()));

        uint8_t timeout[3] = {0x00, 0x00, 0x00};
        write_command(lr2021::CMD_SET_TX, timeout, 3);

        m_pending = Pending::Tx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_send() {
        if (m_pending != Pending::Tx) {
            return 0;
        }

        if ((m_hal.millis() - m_op_start_ms) > kTxTimeoutMs) {
            clear_irq(lr2021::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return -1;
        }

        if (m_hal.gpio_get(m_cfg.pin_dio1)) {
            const uint32_t irq = get_and_clear_irq_status();
            if (irq & lr2021::IRQ_TX_DONE) {
                standby();
                m_pending = Pending::None;
                return 1;
            }
            if (irq & (lr2021::IRQ_TIMEOUT | lr2021::IRQ_CRC_ERROR)) {
                standby();
                m_pending = Pending::None;
                return -1;
            }
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
        clear_irq(lr2021::IRQ_ALL);

        set_rx_path(is_high_freq() ? lr2021::RX_PATH_HF : lr2021::RX_PATH_LF,
                    is_high_freq() ? lr2021::RX_BOOST_HF : lr2021::RX_BOOST_LF);

        const uint8_t implicit_len = select_implicit_len(buf.size());
        if (use_implicit_header()) {
            apply_packet_params(implicit_len);
        }

        m_rx_buf = buf.data();
        m_rx_buf_len = buf.size();
        m_rx_expected = implicit_len;
        m_rx_continuous = continuous;

        const uint32_t timeout = continuous ? lr2021::RX_TIMEOUT_INF : lr2021::RX_TIMEOUT_NONE;
        uint8_t timeout_buf[3] = {
            static_cast<uint8_t>((timeout >> 16) & 0xFF),
            static_cast<uint8_t>((timeout >> 8) & 0xFF),
            static_cast<uint8_t>(timeout & 0xFF)
        };
        write_command(lr2021::CMD_SET_RX, timeout_buf, 3);

        m_pending = Pending::Rx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_receive() {
        if (m_pending != Pending::Rx) {
            return 0;
        }

        if (!m_rx_continuous && (m_hal.millis() - m_op_start_ms) > kRxTimeoutMs) {
            clear_irq(lr2021::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return -1;
        }

        if (m_hal.gpio_get(m_cfg.pin_dio1)) {
            const uint32_t irq = get_and_clear_irq_status();
            if (irq & lr2021::IRQ_TIMEOUT) {
                standby();
                m_pending = Pending::None;
                return -1;
            }
            if (irq & lr2021::IRQ_CRC_ERROR) {
                if (!m_rx_continuous) {
                    standby();
                }
                m_pending = Pending::None;
                return -1;
            }
            if (irq & lr2021::IRQ_RX_DONE) {
                uint16_t len = 0;
                if (use_implicit_header() && m_rx_expected != 0) {
                    len = m_rx_expected;
                } else {
                    len = get_rx_packet_length();
                }
                if (len == 0 && m_rx_expected != 0) {
                    len = m_rx_expected;
                }

                const uint8_t copy_len = static_cast<uint8_t>(
                    std::min<size_t>(len, m_rx_buf_len)
                );
                read_rx_fifo({m_rx_buf, copy_len});
                update_packet_status();

                if (!m_rx_continuous) {
                    standby();
                }
                m_pending = Pending::None;
                return copy_len;
            }
        }

        return 0;
    }

private:
    static constexpr size_t kMaxPayload = 255;
    static constexpr uint32_t kTxTimeoutMs = 5'000;
    static constexpr uint32_t kRxTimeoutMs = 6'000;
    static constexpr size_t kStatusBytes = 2;

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

    uint8_t irq_dio_num() const {
        return static_cast<uint8_t>(std::clamp<int>(m_cfg.dio_irq_num, 5, 11));
    }

    bool is_high_freq() const {
        return m_cfg.frequency_hz >= 1'000'000'000ULL;
    }

    void write_command(uint16_t cmd, const uint8_t* data, size_t len) {
        wait_busy();
        uint8_t tx[260] = {};
        tx[0] = static_cast<uint8_t>((cmd >> 8) & 0xFF);
        tx[1] = static_cast<uint8_t>(cmd & 0xFF);
        if (data && len > 0) {
            std::memcpy(&tx[2], data, len);
        }
        m_hal.spi_write(tx, len + 2);
    }

    void read_command(uint16_t cmd, uint8_t* data, size_t len, const uint8_t* out, size_t out_len) {
        wait_busy();
        uint8_t tx[260] = {};
        tx[0] = static_cast<uint8_t>((cmd >> 8) & 0xFF);
        tx[1] = static_cast<uint8_t>(cmd & 0xFF);
        if (out && out_len > 0) {
            std::memcpy(&tx[2], out, out_len);
        }
        m_hal.spi_write(tx, out_len + 2);

        wait_busy();
        uint8_t rx[260] = {};
        uint8_t zeros[260] = {};
        const size_t read_len = len + kStatusBytes;
        m_hal.spi_transfer(zeros, rx, read_len);
        if (data) {
            std::memcpy(data, &rx[kStatusBytes], len);
        }
    }

    void write_tx_fifo(std::span<const uint8_t> data) {
        std::array<uint8_t, kMaxPayload + 2> tx{};
        tx[0] = static_cast<uint8_t>((lr2021::CMD_WRITE_TX_FIFO >> 8) & 0xFF);
        tx[1] = static_cast<uint8_t>(lr2021::CMD_WRITE_TX_FIFO & 0xFF);
        std::copy(data.begin(), data.end(), tx.begin() + 2);
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_write_dma(tx.data(), data.size() + 2)) {
                return;
            }
        }
        m_hal.spi_write(tx.data(), data.size() + 2);
    }

    void read_rx_fifo(std::span<uint8_t> data) {
        std::array<uint8_t, kMaxPayload + 2> tx{};
        std::array<uint8_t, kMaxPayload + 2> rx{};
        tx[0] = static_cast<uint8_t>((lr2021::CMD_READ_RX_FIFO >> 8) & 0xFF);
        tx[1] = static_cast<uint8_t>(lr2021::CMD_READ_RX_FIFO & 0xFF);
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_transfer_dma(tx.data(), rx.data(), data.size() + 2)) {
                std::copy(rx.begin() + 2, rx.begin() + 2 + data.size(), data.begin());
                return;
            }
        }
        m_hal.spi_transfer(tx.data(), rx.data(), data.size() + 2);
        std::copy(rx.begin() + 2, rx.begin() + 2 + data.size(), data.begin());
    }

    void set_dio_function(uint8_t dio, uint8_t function, uint8_t pull) {
        uint8_t params[2] = {dio, static_cast<uint8_t>((function & 0xF0) | (pull & 0x0F))};
        write_command(lr2021::CMD_SET_DIO_FUNCTION, params, sizeof(params));
    }

    void set_dio_irq(uint8_t dio, uint32_t mask) {
        uint8_t params[5] = {
            dio,
            static_cast<uint8_t>((mask >> 24) & 0xFF),
            static_cast<uint8_t>((mask >> 16) & 0xFF),
            static_cast<uint8_t>((mask >> 8) & 0xFF),
            static_cast<uint8_t>(mask & 0xFF)
        };
        write_command(lr2021::CMD_SET_DIO_IRQ_CONFIG, params, sizeof(params));
    }

    void clear_irq(uint32_t mask) {
        uint8_t params[4] = {
            static_cast<uint8_t>((mask >> 24) & 0xFF),
            static_cast<uint8_t>((mask >> 16) & 0xFF),
            static_cast<uint8_t>((mask >> 8) & 0xFF),
            static_cast<uint8_t>(mask & 0xFF)
        };
        write_command(lr2021::CMD_CLEAR_IRQ, params, sizeof(params));
    }

    uint32_t get_and_clear_irq_status() {
        uint8_t buf[4] = {};
        read_command(lr2021::CMD_GET_AND_CLEAR_IRQ_STATUS, buf, sizeof(buf), nullptr, 0);
        return (static_cast<uint32_t>(buf[0]) << 24) |
               (static_cast<uint32_t>(buf[1]) << 16) |
               (static_cast<uint32_t>(buf[2]) << 8) |
               static_cast<uint32_t>(buf[3]);
    }

    void set_rx_tx_fallback(uint8_t mode) {
        write_command(lr2021::CMD_SET_RX_TX_FALLBACK_MODE, &mode, 1);
    }

    void calibrate_all() {
        uint8_t blocks = lr2021::CALIBRATE_ALL;
        write_command(lr2021::CMD_CALIBRATE, &blocks, 1);
        wait_busy();
    }

    void set_rx_path(uint8_t path, uint8_t boost) {
        uint8_t params[2] = {static_cast<uint8_t>(path & 0x01), static_cast<uint8_t>(boost & 0x07)};
        write_command(lr2021::CMD_SET_RX_PATH, params, sizeof(params));
    }

    void set_lora_syncword(uint8_t syncword) {
        write_command(lr2021::CMD_SET_LORA_SYNCWORD, &syncword, 1);
    }

    uint16_t get_rx_packet_length() {
        uint8_t buf[2] = {};
        read_command(lr2021::CMD_GET_RX_PKT_LENGTH, buf, sizeof(buf), nullptr, 0);
        return static_cast<uint16_t>(buf[0] << 8) | buf[1];
    }

    void update_packet_status() {
        uint8_t buf[6] = {};
        read_command(lr2021::CMD_GET_LORA_PACKET_STATUS, buf, sizeof(buf), nullptr, 0);
        m_last_snr = static_cast<int8_t>(static_cast<int8_t>(buf[2]) / 4);
        const uint16_t raw = static_cast<uint16_t>(buf[3] << 1) |
                             static_cast<uint16_t>((buf[5] & 0x02) >> 1);
        m_last_rssi = static_cast<int16_t>(raw / -2);
    }

    static uint8_t encode_bw(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, lr2021::kBwReg.size() - 1));
        return lr2021::kBwReg[idx];
    }

    static uint32_t bandwidth_hz(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, lr2021::kBwHz.size() - 1));
        return lr2021::kBwHz[idx];
    }

    static uint8_t encode_cr(uint8_t cr) {
        const uint8_t idx = static_cast<uint8_t>(std::clamp<int>(cr, 1, 4) - 1);
        return lr2021::kCrReg[idx];
    }

    bool ldro_needed(uint8_t sf) const {
        const uint32_t bw = bandwidth_hz(m_cfg.bandwidth);
        const float symbol_ms = (static_cast<float>(1u << sf) * 1000.0f) / static_cast<float>(bw);
        return symbol_ms >= 16.0f;
    }

    void apply_modem_config() {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(m_cfg.bandwidth, 11));
        m_cfg.coding_rate = static_cast<uint8_t>(std::clamp<int>(m_cfg.coding_rate, 1, 4));
        m_cfg.spreading_factor = static_cast<uint8_t>(std::clamp<int>(m_cfg.spreading_factor, 5, 12));

        const uint8_t bw = encode_bw(m_cfg.bandwidth);
        const uint8_t cr = encode_cr(m_cfg.coding_rate);
        const uint8_t sf = m_cfg.spreading_factor;
        const uint8_t ldro = ldro_needed(m_cfg.spreading_factor) ? 0x01 : 0x00;

        uint8_t params[2] = {
            static_cast<uint8_t>((sf << 4) | (bw & 0x0F)),
            static_cast<uint8_t>((cr << 4) | (ldro & 0x01))
        };
        write_command(lr2021::CMD_SET_LORA_MODULATION_PARAMS, params, sizeof(params));
        apply_packet_params(use_implicit_header() ? select_implicit_len(kMaxPayload) : 0);
    }

    void apply_packet_params(uint8_t payload_len) {
        const uint8_t hdr = use_implicit_header() ? lr2021::LORA_HEADER_IMPLICIT : lr2021::LORA_HEADER_EXPLICIT;
        const uint8_t len = use_implicit_header()
            ? payload_len
            : static_cast<uint8_t>(std::min<size_t>(payload_len == 0 ? kMaxPayload : payload_len, kMaxPayload));

        uint8_t params[4] = {
            0x00, 0x08,
            len,
            static_cast<uint8_t>(((hdr & 0x01) << 2) | ((lr2021::LORA_CRC_ON & 0x01) << 1) | (lr2021::LORA_IQ_STANDARD & 0x01))
        };
        write_command(lr2021::CMD_SET_LORA_PACKET_PARAMS, params, sizeof(params));
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

    int8_t clamp_tx_power(int8_t dbm) const {
        if (is_high_freq()) {
            return static_cast<int8_t>(std::clamp<int>(dbm, -19, 12));
        }
        return static_cast<int8_t>(std::clamp<int>(dbm, -9, 22));
    }
};
