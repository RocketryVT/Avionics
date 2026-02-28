#pragma once

// LR11x0 LoRa transceiver driver (LR1110, LR1120, LR1121).
// LoRa-only, blocking and non-blocking TX/RX.

#include "../lora_interface.hpp"
#include "../hal/hal_interface.hpp"

#include <array>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <span>

namespace lr11x0 {

enum class Variant : uint8_t {
    LR1110,
    LR1120,
    LR1121,
};

// Commands (subset)
inline constexpr uint16_t CMD_WRITE_BUFFER        = 0x0109;
inline constexpr uint16_t CMD_READ_BUFFER         = 0x010A;
inline constexpr uint16_t CMD_GET_STATUS          = 0x0100;
inline constexpr uint16_t CMD_SET_DIO_IRQ_PARAMS  = 0x0113;
inline constexpr uint16_t CMD_CLEAR_IRQ           = 0x0114;
inline constexpr uint16_t CMD_SET_SLEEP           = 0x011B;
inline constexpr uint16_t CMD_SET_STANDBY         = 0x011C;
inline constexpr uint16_t CMD_SET_RX              = 0x0209;
inline constexpr uint16_t CMD_SET_TX              = 0x020A;
inline constexpr uint16_t CMD_SET_RF_FREQUENCY    = 0x020B;
inline constexpr uint16_t CMD_GET_RX_BUFFER_STATUS= 0x0203;
inline constexpr uint16_t CMD_GET_PACKET_STATUS   = 0x0204;
inline constexpr uint16_t CMD_SET_PACKET_TYPE     = 0x020E;
inline constexpr uint16_t CMD_SET_MODULATION_PARAMS=0x020F;
inline constexpr uint16_t CMD_SET_PACKET_PARAMS   = 0x0210;
inline constexpr uint16_t CMD_SET_TX_PARAMS       = 0x0211;
inline constexpr uint16_t CMD_SET_PA_CONFIG       = 0x0215;

// Packet type
inline constexpr uint8_t PACKET_TYPE_LORA = 0x02;

// Standby
inline constexpr uint8_t STDBY_RC = 0x00;

// IRQ flags
inline constexpr uint32_t IRQ_TX_DONE   = 0x00000004;
inline constexpr uint32_t IRQ_RX_DONE   = 0x00000008;
inline constexpr uint32_t IRQ_CRC_ERR   = 0x00000080;
inline constexpr uint32_t IRQ_TIMEOUT   = 0x00000400;
inline constexpr uint32_t IRQ_ALL       = 0x1BF80FFC;

// LoRa params
inline constexpr uint8_t LORA_HEADER_EXPLICIT = 0x00;
inline constexpr uint8_t LORA_HEADER_IMPLICIT = 0x01;
inline constexpr uint8_t LORA_CRC_ON          = 0x01;
inline constexpr uint8_t LORA_IQ_STANDARD     = 0x00;

// LoRa BW codes
inline constexpr std::array<uint8_t, 7> kBwReg = {
    0x03, 0x04, 0x05, 0x06, 0x0D, 0x0E, 0x0F
};
inline constexpr std::array<uint32_t, 7> kBwHz = {
    62'500, 125'000, 250'000, 500'000, 203'125, 406'250, 812'500
};

inline constexpr std::array<uint8_t, 4> kCrReg = {
    0x01, 0x02, 0x03, 0x04
};

inline constexpr uint8_t PA_RAMP_160U = 0x09;

}  // namespace lr11x0

template <typename Hal, lr11x0::Variant V = lr11x0::Variant::LR1110>
    requires HalImpl<Hal>
struct LR11x0
{
    struct Config {
        uint8_t  pin_reset;
        uint8_t  pin_busy;
        uint8_t  pin_dio1;
        uint32_t frequency_hz        = 915'000'000;
        uint8_t  bandwidth           = 1;  // index into kBwReg (0..6)
        uint8_t  spreading_factor    = 9;  // 5..12
        uint8_t  coding_rate         = 1;  // 1..4 (4/5..4/8)
        int8_t   tx_power_dbm        = 14;
        bool     implicit_header     = false;
        uint8_t  implicit_header_len = 0;
    };

    LR11x0(Hal& hal, Config cfg) : m_hal{hal}, m_cfg{cfg} {}

    bool initialize() {
        reset();
        standby();

        uint8_t pkt = lr11x0::PACKET_TYPE_LORA;
        write_command(lr11x0::CMD_SET_PACKET_TYPE, &pkt, 1);

        set_frequency(m_cfg.frequency_hz);
        apply_modem_config();
        set_tx_power(m_cfg.tx_power_dbm);

        set_dio_irq(lr11x0::IRQ_TX_DONE | lr11x0::IRQ_RX_DONE | lr11x0::IRQ_TIMEOUT | lr11x0::IRQ_CRC_ERR);
        clear_irq(lr11x0::IRQ_ALL);
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
        write_command(lr11x0::CMD_SET_RF_FREQUENCY, params, 4);
    }

    void set_bandwidth(uint8_t bw) {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(bw, 6));
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
        const PaConfig cfg = select_pa_config(m_cfg.tx_power_dbm);
        uint8_t pa[4] = {cfg.pa_sel, cfg.pa_supply, 0x04, 0x07};
        write_command(lr11x0::CMD_SET_PA_CONFIG, pa, 4);

        uint8_t params[2] = {static_cast<uint8_t>(m_cfg.tx_power_dbm), lr11x0::PA_RAMP_160U};
        write_command(lr11x0::CMD_SET_TX_PARAMS, params, 2);
    }

    int16_t get_rssi() { return m_last_rssi; }
    int8_t  get_snr()  { return m_last_snr; }

    void sleep() {
        uint8_t cfg[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
        write_command(lr11x0::CMD_SET_SLEEP, cfg, sizeof(cfg));
    }

    void standby() {
        uint8_t mode = lr11x0::STDBY_RC;
        write_command(lr11x0::CMD_SET_STANDBY, &mode, 1);
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
        clear_irq(lr11x0::IRQ_ALL);

        write_buffer(data);
        apply_packet_params(static_cast<uint8_t>(data.size()));

        uint8_t timeout[3] = {0x00, 0x00, 0x00};
        write_command(lr11x0::CMD_SET_TX, timeout, 3);

        m_pending = Pending::Tx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_send() {
        if (m_pending != Pending::Tx) {
            return 0;
        }

        if ((m_hal.millis() - m_op_start_ms) > kTxTimeoutMs) {
            clear_irq(lr11x0::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return -1;
        }

        if (m_hal.gpio_get(m_cfg.pin_dio1)) {
            clear_irq(lr11x0::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return 1;
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
        clear_irq(lr11x0::IRQ_ALL);

        const uint8_t implicit_len = select_implicit_len(buf.size());
        if (use_implicit_header()) {
            apply_packet_params(implicit_len);
        }

        m_rx_buf = buf.data();
        m_rx_buf_len = buf.size();
        m_rx_expected = implicit_len;
        m_rx_continuous = continuous;

        uint8_t timeout[3] = {0x00, 0x00, 0x00};
        write_command(lr11x0::CMD_SET_RX, timeout, 3);

        m_pending = Pending::Rx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_receive() {
        if (m_pending != Pending::Rx) {
            return 0;
        }

        if (!m_rx_continuous && (m_hal.millis() - m_op_start_ms) > kRxTimeoutMs) {
            clear_irq(lr11x0::IRQ_ALL);
            standby();
            m_pending = Pending::None;
            return -1;
        }

        if (m_hal.gpio_get(m_cfg.pin_dio1)) {
            uint8_t status[2] = {};
            read_command(lr11x0::CMD_GET_RX_BUFFER_STATUS, status, 2, nullptr, 0);
            uint8_t payload_len = status[0];
            uint8_t offset = status[1];
            if (payload_len == 0 && m_rx_expected != 0) {
                payload_len = m_rx_expected;
            }

            const uint8_t copy_len = static_cast<uint8_t>(
                std::min<size_t>(payload_len, m_rx_buf_len)
            );
            read_buffer(offset, copy_len, {m_rx_buf, copy_len});

            uint8_t pkt_status[3] = {};
            read_command(lr11x0::CMD_GET_PACKET_STATUS, pkt_status, 3, nullptr, 0);
            m_last_rssi = static_cast<int16_t>(pkt_status[0]) / -2;
            m_last_snr = static_cast<int8_t>(static_cast<int8_t>(pkt_status[1]) / 4);

            clear_irq(lr11x0::IRQ_ALL);
            if (!m_rx_continuous) {
                standby();
            }
            m_pending = Pending::None;
            return copy_len;
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

    struct PaConfig {
        uint8_t pa_sel;
        uint8_t pa_supply;
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

        uint8_t rx[260] = {};
        uint8_t zeros[260] = {};
        m_hal.spi_transfer(zeros, rx, len + 1);
        if (data) {
            std::memcpy(data, &rx[1], len);
        }
    }

    void write_buffer(std::span<const uint8_t> data) {
        write_command(lr11x0::CMD_WRITE_BUFFER, data.data(), data.size());
    }

    void read_buffer(uint8_t offset, uint8_t len, std::span<uint8_t> data) {
        std::array<uint8_t, kMaxPayload + 2> req{};
        req[0] = offset;
        req[1] = len;
        const size_t req_len = 2 + len;
        read_command(lr11x0::CMD_READ_BUFFER, data.data(), len, req.data(), req_len);
    }

    void set_dio_irq(uint32_t mask) {
        uint8_t params[8] = {
            static_cast<uint8_t>((mask >> 24) & 0xFF),
            static_cast<uint8_t>((mask >> 16) & 0xFF),
            static_cast<uint8_t>((mask >> 8) & 0xFF),
            static_cast<uint8_t>(mask & 0xFF),
            static_cast<uint8_t>((mask >> 24) & 0xFF),
            static_cast<uint8_t>((mask >> 16) & 0xFF),
            static_cast<uint8_t>((mask >> 8) & 0xFF),
            static_cast<uint8_t>(mask & 0xFF)
        };
        write_command(lr11x0::CMD_SET_DIO_IRQ_PARAMS, params, 8);
    }

    void clear_irq(uint32_t mask) {
        uint8_t params[4] = {
            static_cast<uint8_t>((mask >> 24) & 0xFF),
            static_cast<uint8_t>((mask >> 16) & 0xFF),
            static_cast<uint8_t>((mask >> 8) & 0xFF),
            static_cast<uint8_t>(mask & 0xFF)
        };
        write_command(lr11x0::CMD_CLEAR_IRQ, params, 4);
    }

    static uint8_t encode_bw(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, lr11x0::kBwReg.size() - 1));
        return lr11x0::kBwReg[idx];
    }

    static uint32_t bandwidth_hz(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, lr11x0::kBwHz.size() - 1));
        return lr11x0::kBwHz[idx];
    }

    static uint8_t encode_cr(uint8_t cr) {
        const uint8_t idx = static_cast<uint8_t>(std::clamp<int>(cr, 1, 4) - 1);
        return lr11x0::kCrReg[idx];
    }

    bool ldro_needed(uint8_t sf) const {
        const uint32_t bw = bandwidth_hz(m_cfg.bandwidth);
        const float symbol_ms = (static_cast<float>(1u << sf) * 1000.0f) / static_cast<float>(bw);
        return symbol_ms >= 16.0f;
    }

    void apply_modem_config() {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(m_cfg.bandwidth, 6));
        m_cfg.coding_rate = static_cast<uint8_t>(std::clamp<int>(m_cfg.coding_rate, 1, 4));
        m_cfg.spreading_factor = static_cast<uint8_t>(std::clamp<int>(m_cfg.spreading_factor, 5, 12));

        const uint8_t bw = encode_bw(m_cfg.bandwidth);
        const uint8_t cr = encode_cr(m_cfg.coding_rate);
        const uint8_t sf = m_cfg.spreading_factor;
        const uint8_t ldro = ldro_needed(m_cfg.spreading_factor) ? 0x01 : 0x00;

        uint8_t params[4] = {sf, bw, cr, ldro};
        write_command(lr11x0::CMD_SET_MODULATION_PARAMS, params, 4);
        apply_packet_params(use_implicit_header() ? select_implicit_len(kMaxPayload) : 0);
    }

    void apply_packet_params(uint8_t payload_len) {
        const uint8_t hdr = use_implicit_header() ? lr11x0::LORA_HEADER_IMPLICIT : lr11x0::LORA_HEADER_EXPLICIT;
        const uint8_t len = use_implicit_header() ? payload_len : static_cast<uint8_t>(std::min<size_t>(payload_len == 0 ? kMaxPayload : payload_len, kMaxPayload));
        uint8_t params[6] = {
            0x00, 0x08,
            hdr,
            len,
            lr11x0::LORA_CRC_ON,
            lr11x0::LORA_IQ_STANDARD
        };
        write_command(lr11x0::CMD_SET_PACKET_PARAMS, params, 6);
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
        const bool high_freq = (m_cfg.frequency_hz >= 1'000'000'000ULL);
        if constexpr (V == lr11x0::Variant::LR1110) {
            if (dbm > 14) {
                return static_cast<int8_t>(std::clamp<int>(dbm, -9, 22));
            }
            return static_cast<int8_t>(std::clamp<int>(dbm, -17, 14));
        } else {
            if (high_freq) {
                return static_cast<int8_t>(std::clamp<int>(dbm, -18, 13));
            }
            if (dbm > 14) {
                return static_cast<int8_t>(std::clamp<int>(dbm, -9, 22));
            }
            return static_cast<int8_t>(std::clamp<int>(dbm, -17, 14));
        }
    }

    PaConfig select_pa_config(int8_t dbm) const {
        const bool high_freq = (m_cfg.frequency_hz >= 1'000'000'000ULL);
        if constexpr (V == lr11x0::Variant::LR1110) {
            const bool use_hp = (dbm > 14);
            return {static_cast<uint8_t>(use_hp ? 1 : 0), static_cast<uint8_t>(use_hp ? 1 : 0)};
        } else {
            if (high_freq) {
                return {2, 0};
            }
            const bool use_hp = (dbm > 14);
            return {static_cast<uint8_t>(use_hp ? 1 : 0), static_cast<uint8_t>(use_hp ? 1 : 0)};
        }
    }
};

// Variant aliases
template <typename Hal>
using LR1110 = LR11x0<Hal, lr11x0::Variant::LR1110>;

template <typename Hal>
using LR1120 = LR11x0<Hal, lr11x0::Variant::LR1120>;

template <typename Hal>
using LR1121 = LR11x0<Hal, lr11x0::Variant::LR1121>;
