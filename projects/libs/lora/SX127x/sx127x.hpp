#pragma once

// SX127x LoRa transceiver driver (SX1276, SX1277, SX1278, SX1279).
// LoRa-only, blocking and non-blocking TX/RX. Based on the RadioLib register map.

#include "../lora_interface.hpp"
#include "../hal/hal_interface.hpp"

#include <array>
#include <algorithm>
#include <cstdint>
#include <span>

namespace sx127x {

// Register map (LoRa mode).
inline constexpr uint8_t kRegFifo              = 0x00;
inline constexpr uint8_t kRegOpMode            = 0x01;
inline constexpr uint8_t kRegFrfMsb            = 0x06;
inline constexpr uint8_t kRegFrfMid            = 0x07;
inline constexpr uint8_t kRegFrfLsb            = 0x08;
inline constexpr uint8_t kRegPaConfig          = 0x09;
inline constexpr uint8_t kRegPaRamp            = 0x0A;
inline constexpr uint8_t kRegOcp               = 0x0B;
inline constexpr uint8_t kRegLna               = 0x0C;
inline constexpr uint8_t kRegFifoAddrPtr       = 0x0D;
inline constexpr uint8_t kRegFifoTxBaseAddr    = 0x0E;
inline constexpr uint8_t kRegFifoRxBaseAddr    = 0x0F;
inline constexpr uint8_t kRegFifoRxCurrentAddr = 0x10;
inline constexpr uint8_t kRegIrqFlags          = 0x12;
inline constexpr uint8_t kRegRxNbBytes         = 0x13;
inline constexpr uint8_t kRegPktSnrValue       = 0x19;
inline constexpr uint8_t kRegPktRssiValue      = 0x1A;
inline constexpr uint8_t kRegModemConfig1      = 0x1D;
inline constexpr uint8_t kRegModemConfig2      = 0x1E;
inline constexpr uint8_t kRegSymbTimeoutLsb    = 0x1F;
inline constexpr uint8_t kRegPreambleMsb       = 0x20;
inline constexpr uint8_t kRegPreambleLsb       = 0x21;
inline constexpr uint8_t kRegPayloadLength     = 0x22;
inline constexpr uint8_t kRegMaxPayloadLength  = 0x23;
inline constexpr uint8_t kRegModemConfig3      = 0x26;
inline constexpr uint8_t kRegDetectOptimize    = 0x31;
inline constexpr uint8_t kRegDetectionThreshold= 0x37;
inline constexpr uint8_t kRegSyncWord          = 0x39;
inline constexpr uint8_t kRegDioMapping1       = 0x40;
inline constexpr uint8_t kRegVersion           = 0x42;
inline constexpr uint8_t kRegPaDac             = 0x4D;

// OpMode bits.
inline constexpr uint8_t kOpModeLoRa       = 0x80;
inline constexpr uint8_t kOpModeSleep      = 0x00;
inline constexpr uint8_t kOpModeStandby    = 0x01;
inline constexpr uint8_t kOpModeTx         = 0x03;
inline constexpr uint8_t kOpModeRxContinuous = 0x05;
inline constexpr uint8_t kOpModeRxSingle   = 0x06;

// PA config bits.
inline constexpr uint8_t kPaSelectBoost    = 0x80;

// IRQ flags.
inline constexpr uint8_t kIrqRxTimeout     = 0x80;
inline constexpr uint8_t kIrqRxDone        = 0x40;
inline constexpr uint8_t kIrqPayloadCrcErr = 0x20;
inline constexpr uint8_t kIrqTxDone        = 0x08;

// DIO mapping (DIO0).
inline constexpr uint8_t kDio0RxDone        = 0x00;
inline constexpr uint8_t kDio0TxDone        = 0x40;

inline constexpr std::array<uint8_t, 10> kBwReg = {
    0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90
};

inline constexpr std::array<uint32_t, 10> kBwHz = {
    7'800, 10'400, 15'600, 20'800, 31'250, 41'700, 62'500, 125'000, 250'000, 500'000
};

inline constexpr std::array<uint8_t, 4> kCrReg = {
    0x02, 0x04, 0x06, 0x08
};

inline constexpr std::array<uint8_t, 7> kSfReg = {
    0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0
};

namespace bw {
enum class Value : uint8_t {
    k7_8   = 0,
    k10_4  = 1,
    k15_6  = 2,
    k20_8  = 3,
    k31_25 = 4,
    k41_7  = 5,
    k62_5  = 6,
    k125   = 7,
    k250   = 8,
    k500   = 9,
};

constexpr uint8_t index(Value v) {
    return static_cast<uint8_t>(v);
}

constexpr uint32_t hz(Value v) {
    return kBwHz[index(v)];
}
}  // namespace bw

namespace cr {
enum class Value : uint8_t {
    k4_5 = 1,
    k4_6 = 2,
    k4_7 = 3,
    k4_8 = 4,
};
}  // namespace cr

namespace sf {
enum class Value : uint8_t {
    k6  = 6,
    k7  = 7,
    k8  = 8,
    k9  = 9,
    k10 = 10,
    k11 = 11,
    k12 = 12,
};
}  // namespace sf

}  // namespace sx127x

template <typename Hal>
    requires HalImpl<Hal>
struct SX127x
{
    struct Config {
        uint8_t  pin_reset;
        uint8_t  pin_dio0;
        uint32_t frequency_hz     = 915'000'000;
        uint8_t  bandwidth        = 7;
        uint8_t  spreading_factor = 9;
        uint8_t  coding_rate      = 1;
        int8_t   tx_power_dbm     = 14;
        bool     implicit_header  = false;
        uint8_t  implicit_header_len = 0;
    };

    SX127x(Hal& hal, Config cfg) : m_hal{hal}, m_cfg{cfg} {}

    bool initialize() {
        reset();
        write_reg(sx127x::kRegOpMode, sx127x::kOpModeLoRa | sx127x::kOpModeSleep);
        m_hal.delay_ms(10);

        standby();

        const uint8_t version = read_reg(sx127x::kRegVersion);
        if (version == 0x00 || version == 0xFF) {
            return false;
        }

        write_reg(sx127x::kRegFifoTxBaseAddr, 0x00);
        write_reg(sx127x::kRegFifoRxBaseAddr, 0x00);

        update_reg(sx127x::kRegLna, 0x03, 0x03);
        write_reg(sx127x::kRegPreambleMsb, 0x00);
        write_reg(sx127x::kRegPreambleLsb, 0x08);
        write_reg(sx127x::kRegSyncWord, 0x12);
        write_reg(sx127x::kRegMaxPayloadLength, kMaxPayload);

        set_frequency(m_cfg.frequency_hz);
        apply_modem_config();
        set_tx_power(m_cfg.tx_power_dbm);

        update_reg(sx127x::kRegModemConfig2, 0x04, 0x04);
        update_reg(sx127x::kRegModemConfig2, 0x03, 0x00);
        write_reg(sx127x::kRegSymbTimeoutLsb, 0x64);

        clear_irq_flags();
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
        const uint64_t frf = (static_cast<uint64_t>(hz) << 19) / 32'000'000ULL;
        write_reg(sx127x::kRegFrfMsb, static_cast<uint8_t>((frf >> 16) & 0xFF));
        write_reg(sx127x::kRegFrfMid, static_cast<uint8_t>((frf >> 8) & 0xFF));
        write_reg(sx127x::kRegFrfLsb, static_cast<uint8_t>(frf & 0xFF));
    }

    void set_bandwidth(uint8_t bw) {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(bw, 9));
        apply_modem_config();
    }

    void set_bandwidth(sx127x::bw::Value bw) {
        set_bandwidth(sx127x::bw::index(bw));
    }

    void set_spreading_factor(uint8_t sf) {
        m_cfg.spreading_factor = static_cast<uint8_t>(std::clamp<int>(sf, 6, 12));
        apply_modem_config();
    }

    void set_spreading_factor(sx127x::sf::Value value) {
        set_spreading_factor(static_cast<uint8_t>(value));
    }

    void set_coding_rate(uint8_t cr) {
        m_cfg.coding_rate = static_cast<uint8_t>(std::clamp<int>(cr, 1, 4));
        apply_modem_config();
    }

    void set_coding_rate(sx127x::cr::Value value) {
        set_coding_rate(static_cast<uint8_t>(value));
    }

    void set_tx_power(int8_t dbm) {
        dbm = static_cast<int8_t>(std::clamp<int>(dbm, 2, 20));
        m_cfg.tx_power_dbm = dbm;

        const bool high_power = dbm > 17;
        write_reg(sx127x::kRegPaDac, high_power ? 0x87 : 0x84);
        write_reg(sx127x::kRegOcp, high_power ? 0x32 : 0x2B);

        const uint8_t out_power = static_cast<uint8_t>(high_power ? 15 : (dbm - 2));
        write_reg(sx127x::kRegPaConfig, sx127x::kPaSelectBoost | (out_power & 0x0F));
    }

    int16_t get_rssi() { return m_last_rssi; }
    int8_t  get_snr()  { return m_last_snr; }
    void    sleep()    { set_op_mode(sx127x::kOpModeSleep); }
    void    standby()  { set_op_mode(sx127x::kOpModeStandby); }

    void set_implicit_header(uint8_t length) {
        m_cfg.implicit_header = true;
        m_cfg.implicit_header_len = length;
        apply_modem_config();
        if (length != 0) {
            write_reg(sx127x::kRegPayloadLength, length);
        }
    }

    void set_explicit_header() {
        m_cfg.implicit_header = false;
        apply_modem_config();
    }

    bool start_send(std::span<const uint8_t> data) {
        if (m_pending != Pending::None) {
            return false;
        }
        if (data.size() > kMaxPayload) {
            return false;
        }

        standby();
        clear_irq_flags();
        set_dio0_mapping(sx127x::kDio0TxDone);

        write_reg(sx127x::kRegFifoAddrPtr, 0x00);
        write_reg(sx127x::kRegPayloadLength, static_cast<uint8_t>(data.size()));
        write_fifo(data);

        set_op_mode(sx127x::kOpModeTx);
        m_pending = Pending::Tx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_send() {
        if (m_pending != Pending::Tx) {
            return 0;
        }

        if ((m_hal.millis() - m_op_start_ms) > kTxTimeoutMs) {
            clear_irq_flags();
            standby();
            m_pending = Pending::None;
            return -1;
        }

        if (m_hal.gpio_get(m_cfg.pin_dio0)) {
            clear_irq_flags();
            standby();
            m_pending = Pending::None;
            return 1;
        }

        const uint8_t irq = read_reg(sx127x::kRegIrqFlags);
        if (irq & sx127x::kIrqTxDone) {
            clear_irq_flags();
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
        clear_irq_flags();
        set_dio0_mapping(sx127x::kDio0RxDone);
        write_reg(sx127x::kRegFifoAddrPtr, 0x00);

        const uint8_t implicit_len = select_implicit_len(buf.size());
        if (use_implicit_header()) {
            write_reg(sx127x::kRegPayloadLength, implicit_len);
        }

        m_rx_buf = buf.data();
        m_rx_buf_len = buf.size();
        m_rx_expected = implicit_len;
        m_rx_continuous = continuous;

        set_op_mode(continuous ? sx127x::kOpModeRxContinuous : sx127x::kOpModeRxSingle);
        m_pending = Pending::Rx;
        m_op_start_ms = m_hal.millis();
        return true;
    }

    int poll_receive() {
        if (m_pending != Pending::Rx) {
            return 0;
        }

        if (!m_rx_continuous && (m_hal.millis() - m_op_start_ms) > kRxTimeoutMs) {
            clear_irq_flags();
            standby();
            m_pending = Pending::None;
            return -1;
        }

        const uint8_t irq = read_reg(sx127x::kRegIrqFlags);
        if (irq & sx127x::kIrqRxDone) {
            if (irq & sx127x::kIrqPayloadCrcErr) {
                clear_irq_flags();
                if (!m_rx_continuous) {
                    standby();
                }
                m_pending = Pending::None;
                return -1;
            }

            uint8_t len = read_reg(sx127x::kRegRxNbBytes);
            if (len == 0 && m_rx_expected != 0) {
                len = m_rx_expected;
            }
            const uint8_t addr = read_reg(sx127x::kRegFifoRxCurrentAddr);
            write_reg(sx127x::kRegFifoAddrPtr, addr);

            const uint8_t copy_len = static_cast<uint8_t>(
                std::min<size_t>(len, m_rx_buf_len)
            );
            read_fifo({m_rx_buf, copy_len});

            const int8_t snr = static_cast<int8_t>(read_reg(sx127x::kRegPktSnrValue)) / 4;
            const uint8_t rssi_raw = read_reg(sx127x::kRegPktRssiValue);
            m_last_snr = snr;
            m_last_rssi = compute_rssi(rssi_raw, snr);

            clear_irq_flags();
            if (!m_rx_continuous) {
                standby();
            }
            m_pending = Pending::None;
            return copy_len;
        }

        if (irq & (sx127x::kIrqRxTimeout | sx127x::kIrqPayloadCrcErr)) {
            clear_irq_flags();
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
    }

    void set_op_mode(uint8_t mode) {
        const uint8_t low_freq = (m_cfg.frequency_hz < 525'000'000) ? 0x08 : 0x00;
        write_reg(sx127x::kRegOpMode, sx127x::kOpModeLoRa | low_freq | mode);
        m_hal.delay_ms(1);
    }

    void clear_irq_flags() {
        write_reg(sx127x::kRegIrqFlags, 0xFF);
    }

    void set_dio0_mapping(uint8_t value) {
        update_reg(sx127x::kRegDioMapping1, 0xC0, value);
    }

    static uint8_t encode_bw(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, sx127x::kBwReg.size() - 1));
        return sx127x::kBwReg[idx];
    }

    static uint32_t bandwidth_hz(uint8_t index) {
        const uint8_t idx = static_cast<uint8_t>(std::min<size_t>(index, sx127x::kBwHz.size() - 1));
        return sx127x::kBwHz[idx];
    }

    static uint8_t encode_cr(uint8_t cr) {
        const uint8_t idx = static_cast<uint8_t>(std::clamp<int>(cr, 1, 4) - 1);
        return sx127x::kCrReg[idx];
    }

    static uint8_t encode_sf(uint8_t sf) {
        const uint8_t idx = static_cast<uint8_t>(std::clamp<int>(sf, 6, 12) - 6);
        return sx127x::kSfReg[idx];
    }

    bool ldro_needed(uint8_t sf) const {
        const uint32_t bw = bandwidth_hz(m_cfg.bandwidth);
        const float symbol_ms = (static_cast<float>(1u << sf) * 1000.0f) / static_cast<float>(bw);
        return symbol_ms >= 16.0f;
    }

    void apply_sf_optimize(uint8_t sf) {
        if (sf == 6) {
            update_reg(sx127x::kRegDetectOptimize, 0x07, 0x05);
            write_reg(sx127x::kRegDetectionThreshold, 0x0C);
        } else {
            update_reg(sx127x::kRegDetectOptimize, 0x07, 0x03);
            write_reg(sx127x::kRegDetectionThreshold, 0x0A);
        }
    }

    void apply_modem_config() {
        m_cfg.bandwidth = static_cast<uint8_t>(std::min<uint8_t>(m_cfg.bandwidth, 9));
        m_cfg.coding_rate = static_cast<uint8_t>(std::clamp<int>(m_cfg.coding_rate, 1, 4));
        m_cfg.spreading_factor = static_cast<uint8_t>(std::clamp<int>(m_cfg.spreading_factor, 6, 12));

        const uint8_t bw = encode_bw(m_cfg.bandwidth);
        const uint8_t cr = encode_cr(m_cfg.coding_rate);
        const uint8_t sf = encode_sf(m_cfg.spreading_factor);
        const bool implicit = use_implicit_header();
        const uint8_t header = implicit ? 0x01 : 0x00;

        write_reg(sx127x::kRegModemConfig1, bw | cr | header);
        write_reg(sx127x::kRegModemConfig2, sf | 0x04);

        const uint8_t ldro = ldro_needed(m_cfg.spreading_factor) ? 0x08 : 0x00;
        write_reg(sx127x::kRegModemConfig3, ldro | 0x04);

        apply_sf_optimize(m_cfg.spreading_factor);
        if (implicit && m_cfg.implicit_header_len != 0) {
            write_reg(sx127x::kRegPayloadLength, m_cfg.implicit_header_len);
        }
    }

    bool use_implicit_header() const {
        return m_cfg.implicit_header || (m_cfg.spreading_factor == 6);
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

    int16_t compute_rssi(uint8_t rssi_raw, int8_t snr) const {
        const int16_t offset = (m_cfg.frequency_hz >= 779'000'000) ? -157 : -164;
        int16_t rssi = static_cast<int16_t>(offset + rssi_raw);
        if (snr < 0) {
            rssi = static_cast<int16_t>(rssi + snr);
        }
        return rssi;
    }

    void write_reg(uint8_t addr, uint8_t value) {
        uint8_t tx[2] = {static_cast<uint8_t>(addr | 0x80), value};
        uint8_t rx[2] = {};
        m_hal.spi_transfer(tx, rx, 2);
    }

    uint8_t read_reg(uint8_t addr) {
        uint8_t tx[2] = {static_cast<uint8_t>(addr & 0x7F), 0x00};
        uint8_t rx[2] = {};
        m_hal.spi_transfer(tx, rx, 2);
        return rx[1];
    }

    void update_reg(uint8_t addr, uint8_t mask, uint8_t value) {
        const uint8_t current = read_reg(addr);
        write_reg(addr, static_cast<uint8_t>((current & ~mask) | (value & mask)));
    }

    void write_fifo(std::span<const uint8_t> data) {
        std::array<uint8_t, kMaxPayload + 1> tx{};
        tx[0] = static_cast<uint8_t>(sx127x::kRegFifo | 0x80);
        std::copy(data.begin(), data.end(), tx.begin() + 1);
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_write_dma(tx.data(), data.size() + 1)) {
                return;
            }
        }
        m_hal.spi_write(tx.data(), data.size() + 1);
    }

    void read_fifo(std::span<uint8_t> data) {
        std::array<uint8_t, kMaxPayload + 1> tx{};
        std::array<uint8_t, kMaxPayload + 1> rx{};
        tx[0] = static_cast<uint8_t>(sx127x::kRegFifo & 0x7F);
        if constexpr (HalSpiDmaImpl<Hal>) {
            if (m_hal.spi_transfer_dma(tx.data(), rx.data(), data.size() + 1)) {
                std::copy(rx.begin() + 1, rx.begin() + 1 + data.size(), data.begin());
                return;
            }
        }
        m_hal.spi_transfer(tx.data(), rx.data(), data.size() + 1);
        std::copy(rx.begin() + 1, rx.begin() + 1 + data.size(), data.begin());
    }
};
