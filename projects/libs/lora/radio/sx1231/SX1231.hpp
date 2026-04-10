// radio/sx1231/SX1231.hpp — SX1231 FSK driver for RP2040 / RP2350.
//
// Native Pico SDK driver — no RadioLib dependency.
// Implements radio::IRadio for the SX1231 FSK modem (433 MHz backup link).
//
// Supports:
//   • Variable-length packet mode, CRC, GFSK BT=0.3 modulation
//   • Polled RX via DIO0 (Payload Ready)
//   • Configurable frequency, bit rate, deviation, RX bandwidth, TX power
//   • Chip revision detection (2A–2D); rev-2A OOK-delta register fix applied
//
// SPI framing: CPOL=0 CPHA=0, MSB first.
// Register write: addr | 0x80, data.   Register read: addr & 0x7F, dummy.
#pragma once

#include "../Radio.hpp"
#include "../SpiDevice.hpp"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <array>
#include <cstdint>
#include <expected>
#include <span>

namespace radio::sx1231 {

// ── Error codes ───────────────────────────────────────────────────────────────

enum class Err : int8_t {
    Ok              =  0,
    ChipNotFound    = -1,
    InvalidFreq     = -2,
    InvalidBitRate  = -3,
    InvalidFdev     = -4,
    InvalidRxBw     = -5,
    InvalidPower    = -6,
    InvalidSyncWord = -7,
};

// ── Configuration ─────────────────────────────────────────────────────────────

struct Config {
    float    freq_mhz      = 433.0f;
    float    br_kbps       = 4.8f;
    float    fdev_khz      = 5.0f;
    float    rx_bw_khz     = 125.0f;
    int8_t   tx_dbm        = 10;         // -18 to +13 dBm (PA0 path)
    uint8_t  preamble_bytes = 2;
    std::array<uint8_t, 2> sync_word = { 0x2D, 0x01 };
    uint32_t spi_hz        = 1'000'000;
};

// ── Driver ────────────────────────────────────────────────────────────────────

class SX1231 final : public radio::IRadio {
public:
    static constexpr uint8_t MAX_PACKET = 64;  // SX1231 FIFO depth

    SX1231( spi_inst_t* spi,
            uint sck, uint mosi, uint miso,
            uint nss, uint dio0, uint rst,
            const Config& cfg )
        : spi_dev_( spi, nss )
        , sck_( sck ), mosi_( mosi ), miso_( miso )
        , dio0_( dio0 ), rst_( rst )
        , cfg_( cfg )
    {}

    // IRadio interface ---------------------------------------------------------

    // Initialise SPI + GPIO, verify chip revision, apply full config.
    // Returns 0 on success, negative Err on failure.
    int  begin()                       override;

    // Map DIO0 → Payload Ready, clear IRQ flags, enter RX mode.
    void start_receive()               override;

    // Returns true when DIO0 is high (Payload Ready asserted).
    bool packet_available() const      override { return gpio_get( dio0_ ); }

    // Dequeue the waiting packet.  Leaves radio in standby; call start_receive() again.
    int  read_packet( radio::Packet& pkt ) override;

    // Extended API (chip-specific, not in IRadio) ──────────────────────────────

    [[nodiscard]] std::expected<void, Err> set_frequency   ( float mhz  );
    [[nodiscard]] std::expected<void, Err> set_bit_rate    ( float kbps );
    [[nodiscard]] std::expected<void, Err> set_fdev        ( float khz  );
    [[nodiscard]] std::expected<void, Err> set_rx_bandwidth( float khz  );
    [[nodiscard]] std::expected<void, Err> set_output_power( int8_t dbm );

    // sw: 1–8 bytes, none may be 0x00.
    [[nodiscard]] std::expected<void, Err> set_sync_word( std::span<const uint8_t> sw );

    // RSSI sampled at Payload Ready (dBm).
    [[nodiscard]] float rssi() const;

    // Raw chip version register value (0x21–0x24 = rev 2A–2D).
    [[nodiscard]] uint8_t chip_revision() const { return chip_rev_; }

private:
    // ── Register addresses ────────────────────────────────────────────────────
    struct Reg {
        static constexpr uint8_t Fifo          = 0x00;
        static constexpr uint8_t OpMode        = 0x01;
        static constexpr uint8_t DataModul     = 0x02;
        static constexpr uint8_t BitrateMsb    = 0x03;
        static constexpr uint8_t BitrateLsb    = 0x04;
        static constexpr uint8_t FdevMsb       = 0x05;
        static constexpr uint8_t FdevLsb       = 0x06;
        static constexpr uint8_t FrfMsb        = 0x07;
        static constexpr uint8_t FrfMid        = 0x08;
        static constexpr uint8_t FrfLsb        = 0x09;
        static constexpr uint8_t Version       = 0x10;
        static constexpr uint8_t PaLevel       = 0x11;
        static constexpr uint8_t Ocp           = 0x13;
        static constexpr uint8_t RxBw          = 0x19;
        static constexpr uint8_t RssiValue     = 0x24;
        static constexpr uint8_t DioMapping1   = 0x25;
        static constexpr uint8_t DioMapping2   = 0x26;
        static constexpr uint8_t IrqFlags1     = 0x27;
        static constexpr uint8_t IrqFlags2     = 0x28;
        static constexpr uint8_t RssiThresh    = 0x29;
        static constexpr uint8_t RxTimeout1    = 0x2A;
        static constexpr uint8_t RxTimeout2    = 0x2B;
        static constexpr uint8_t PreambleMsb   = 0x2C;
        static constexpr uint8_t PreambleLsb   = 0x2D;
        static constexpr uint8_t SyncConfig    = 0x2E;
        static constexpr uint8_t SyncValue1    = 0x2F;  // through 0x36
        static constexpr uint8_t PacketConfig1 = 0x37;
        static constexpr uint8_t PayloadLength = 0x38;
        static constexpr uint8_t FifoThresh    = 0x3C;
        static constexpr uint8_t PacketConfig2 = 0x3D;
        static constexpr uint8_t TestPa1       = 0x5A;
        static constexpr uint8_t TestPa2       = 0x5C;
        static constexpr uint8_t TestDagc      = 0x6F;
        static constexpr uint8_t TestOok       = 0x6E;  // SX1231-specific
    };

    // ── Operating modes (OpMode register bits [4:2]) ──────────────────────────
    static constexpr uint8_t kModeSleep   = 0b00000000;
    static constexpr uint8_t kModeStandby = 0b00000100;
    static constexpr uint8_t kModeRx      = 0b00010000;

    // ── Frequency synthesis (32 MHz XOSC, 2^19 divisor) ──────────────────────
    static constexpr float    kXosc_MHz  = 32.0f;
    static constexpr uint32_t kFstepExp  = 19;

    // ── Chip revision IDs ─────────────────────────────────────────────────────
    static constexpr uint8_t kRev2A = 0x21;
    static constexpr uint8_t kRev2D = 0x24;

    // ── Private helpers ───────────────────────────────────────────────────────
    void set_mode( uint8_t mode );
    void hw_reset();
    void clear_irq_flags();
    Err  apply_config();

    // ── Members ───────────────────────────────────────────────────────────────
    radio::SpiDevice spi_dev_;
    uint    sck_, mosi_, miso_;
    uint    dio0_, rst_;
    Config  cfg_;
    uint8_t chip_rev_  = 0;
    float   rx_bw_khz_ = 125.0f;  // cached — needed for bit-rate/BW ratio check
    float   br_kbps_   = 4.8f;    // cached — needed for fdev Carson's-rule check
};

} // namespace radio::sx1231
