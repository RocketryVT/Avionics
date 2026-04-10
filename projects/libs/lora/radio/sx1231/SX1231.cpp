// radio/sx1231/SX1231.cpp — SX1231 FSK driver implementation.
#include "SX1231.hpp"

#include <cmath>
#include <cstring>

namespace radio::sx1231 {

// ── Private helpers ───────────────────────────────────────────────────────────

void SX1231::set_mode( uint8_t mode )
{
    // Write new mode into bits [4:2] of OpMode.
    spi_dev_.set_bits( Reg::OpMode, mode >> 2, 4, 2 );
    // Poll ModeReady (IrqFlags1 bit 7) before returning.
    for ( int i = 0; i < 100; ++i ) {
        if ( spi_dev_.read_reg( Reg::IrqFlags1 ) & 0x80u ) return;
        sleep_us( 100 );
    }
}

void SX1231::hw_reset()
{
    gpio_put( rst_, true );
    sleep_ms( 1 );
    gpio_put( rst_, false );
    sleep_ms( 10 );
}

void SX1231::clear_irq_flags()
{
    spi_dev_.write_reg( Reg::IrqFlags1, 0xFF );
    spi_dev_.write_reg( Reg::IrqFlags2, 0xFF );
}

// ── apply_config ──────────────────────────────────────────────────────────────

Err SX1231::apply_config()
{
    set_mode( kModeStandby );

    // Sequencer on, Listen off (bits [7:6] = 0b00).
    spi_dev_.set_bits( Reg::OpMode, 0b00, 7, 6 );

    // OCP on, 95 mA trim.
    spi_dev_.write_reg( Reg::Ocp, 0b00010000 | 0b00001010 );

    // Packet mode, FSK, Gaussian BT=0.3 shaping.
    spi_dev_.write_reg( Reg::DataModul, 0b00000011 );  // packet|FSK|BT=0.3

    // RSSI threshold -114 dBm (register = -2 × dBm).
    spi_dev_.write_reg( Reg::RssiThresh, 0xE4 );

    // Clear any stale FIFO-overrun flag.
    spi_dev_.write_reg( Reg::IrqFlags2, 0b00010000 );

    // DIO5 clock output disabled.
    spi_dev_.set_bits( Reg::DioMapping2, 0b111, 2, 0 );

    // Variable-length packets, no DC-free encoding, CRC on + autoclear, no address filtering.
    spi_dev_.write_reg( Reg::PacketConfig1,
                        0b10000000   // variable length
                      | 0b00010000   // CRC on
                                     // DC-free none, CRC autoclear on, addr filter off = 0
    );

    // Auto RX restart on, AES off, inter-packet delay = 0.
    spi_dev_.write_reg( Reg::PacketConfig2, 0b00000010 );

    // Max payload 255 (variable mode reads actual length from FIFO byte 0).
    spi_dev_.write_reg( Reg::PayloadLength, 0xFF );

    // FIFO threshold: TX starts on FifoNotEmpty; threshold = 31 bytes.
    spi_dev_.write_reg( Reg::FifoThresh, 0b10000000 | 0x1F );

    // RX timeouts: restart on no RSSI / Payload Ready within timeout period.
    spi_dev_.write_reg( Reg::RxTimeout1, 0xFF );
    spi_dev_.write_reg( Reg::RxTimeout2, 0xFF );

    // Improved DAGC (AfcLowBetaOff fading-margin mode).
    spi_dev_.write_reg( Reg::TestDagc, 0x30 );

    // Rev-2A specific: correct OOK delta threshold.
    if ( chip_rev_ == kRev2A )
        spi_dev_.write_reg( Reg::TestOok, 0x0C );

    // User-configurable parameters — abort early on any error.
    if ( auto r = set_frequency   ( cfg_.freq_mhz      ); !r ) return r.error();
    if ( auto r = set_rx_bandwidth( cfg_.rx_bw_khz     ); !r ) return r.error();
    if ( auto r = set_bit_rate    ( cfg_.br_kbps        ); !r ) return r.error();
    if ( auto r = set_fdev        ( cfg_.fdev_khz       ); !r ) return r.error();
    if ( auto r = set_output_power( cfg_.tx_dbm         ); !r ) return r.error();
    if ( auto r = set_sync_word   ( cfg_.sync_word      ); !r ) return r.error();

    spi_dev_.write_reg( Reg::PreambleMsb, 0x00 );
    spi_dev_.write_reg( Reg::PreambleLsb, cfg_.preamble_bytes );

    return Err::Ok;
}

// ── IRadio::begin ─────────────────────────────────────────────────────────────

int SX1231::begin()
{
    spi_dev_.init( sck_, mosi_, miso_, cfg_.spi_hz );

    gpio_init( rst_ );
    gpio_set_dir( rst_, GPIO_OUT );
    gpio_put( rst_, false );

    gpio_init( dio0_ );
    gpio_set_dir( dio0_, GPIO_IN );

    hw_reset();

    // Detect chip revision — retry up to 10 times.
    for ( int i = 0; i < 10; ++i ) {
        const uint8_t rev = spi_dev_.read_reg( Reg::Version );
        if ( rev >= kRev2A && rev <= kRev2D ) {
            chip_rev_ = rev;
            break;
        }
        sleep_ms( 10 );
    }

    if ( chip_rev_ == 0 )
        return static_cast<int>( Err::ChipNotFound );

    return static_cast<int>( apply_config() );
}

// ── IRadio::start_receive ─────────────────────────────────────────────────────

void SX1231::start_receive()
{
    set_mode( kModeStandby );

    // Map DIO0 → Payload Ready (packet mode bits [7:6] = 0b01).
    spi_dev_.set_bits( Reg::DioMapping1, 0b01, 7, 6 );

    clear_irq_flags();

    // Ensure PA boost is off on the RX path.
    spi_dev_.write_reg( Reg::TestPa1, 0x55 );
    spi_dev_.write_reg( Reg::TestPa2, 0x70 );

    set_mode( kModeRx );
}

// ── IRadio::read_packet ───────────────────────────────────────────────────────

int SX1231::read_packet( radio::Packet& pkt )
{
    std::memset( &pkt, 0, sizeof(pkt) );

    set_mode( kModeStandby );

    // In variable-length mode the first FIFO byte is the payload length.
    uint8_t len = spi_dev_.read_reg( Reg::Fifo );
    if ( len > MAX_PACKET ) len = MAX_PACKET;

    spi_dev_.read_burst( Reg::Fifo, std::span<uint8_t>( pkt.data, len ) );

    pkt.len  = len;
    pkt.rssi = rssi();
    pkt.snr  = 0.0f;  // FSK has no SNR

    clear_irq_flags();
    return static_cast<int>( Err::Ok );
}

// ── Extended API ──────────────────────────────────────────────────────────────

std::expected<void, Err> SX1231::set_frequency( float mhz )
{
    const bool valid = ( mhz > 290.0f && mhz < 340.0f )
                    || ( mhz > 431.0f && mhz < 510.0f )
                    || ( mhz > 862.0f && mhz < 1020.0f );
    if ( !valid ) return std::unexpected( Err::InvalidFreq );

    set_mode( kModeStandby );

    const uint32_t frf = static_cast<uint32_t>(
        ( mhz * static_cast<float>( 1u << kFstepExp ) ) / kXosc_MHz );

    spi_dev_.write_reg( Reg::FrfMsb, static_cast<uint8_t>( (frf >> 16) & 0xFF ) );
    spi_dev_.write_reg( Reg::FrfMid, static_cast<uint8_t>( (frf >>  8) & 0xFF ) );
    spi_dev_.write_reg( Reg::FrfLsb, static_cast<uint8_t>(  frf        & 0xFF ) );

    return {};
}

std::expected<void, Err> SX1231::set_bit_rate( float kbps )
{
    if ( kbps < 0.5f || kbps > 300.0f || kbps >= 2.0f * rx_bw_khz_ )
        return std::unexpected( Err::InvalidBitRate );

    set_mode( kModeStandby );

    const uint16_t raw = static_cast<uint16_t>( 32'000.0f / kbps );
    spi_dev_.write_reg( Reg::BitrateMsb, static_cast<uint8_t>( raw >> 8 ) );
    spi_dev_.write_reg( Reg::BitrateLsb, static_cast<uint8_t>( raw & 0xFF ) );

    br_kbps_ = kbps;
    return {};
}

std::expected<void, Err> SX1231::set_fdev( float khz )
{
    if ( khz < 0.6f ) khz = 0.6f;
    if ( khz + br_kbps_ / 2.0f > 500.0f )
        return std::unexpected( Err::InvalidFdev );

    set_mode( kModeStandby );

    // Fdev_raw (14-bit) = fdev_kHz * 2^19 / (Fxosc_MHz * 1000)
    const uint32_t raw = static_cast<uint32_t>(
        ( khz * static_cast<float>( 1u << kFstepExp ) ) / ( kXosc_MHz * 1000.0f ) );

    spi_dev_.write_reg( Reg::FdevMsb, static_cast<uint8_t>( (raw >> 8) & 0x3F ) );
    spi_dev_.write_reg( Reg::FdevLsb, static_cast<uint8_t>(  raw       & 0xFF ) );

    return {};
}

std::expected<void, Err> SX1231::set_rx_bandwidth( float khz )
{
    // RxBw (FSK) = Fxosc / (Mant * 2^(Exp+2))
    // Mantissa encoding: 0b00→16, 0b01→20, 0b10→24
    static constexpr float kFxosc_kHz = kXosc_MHz * 1000.0f;
    static constexpr float kMants[3]  = { 16.0f, 20.0f, 24.0f };

    uint8_t best_reg  = 0;
    float   best_diff = 1e9f;

    for ( int8_t e = 7; e >= 0; --e ) {
        for ( uint8_t m = 0; m < 3; ++m ) {
            const float bw   = kFxosc_kHz / ( kMants[m] * static_cast<float>( 1u << (e + 2) ) );
            const float diff = std::fabsf( khz - bw );
            if ( diff < best_diff ) {
                best_diff = diff;
                best_reg  = static_cast<uint8_t>( (m << 3) | static_cast<uint8_t>( e ) );
            }
        }
    }

    if ( best_diff / khz > 0.1f )
        return std::unexpected( Err::InvalidRxBw );

    set_mode( kModeStandby );

    // Preserve DCC frequency bits [7:5]; update mantissa/exp in bits [4:0].
    const uint8_t cur = spi_dev_.read_reg( Reg::RxBw );
    spi_dev_.write_reg( Reg::RxBw,
        static_cast<uint8_t>( (cur & 0b11100000u) | (best_reg & 0b00011111u) ) );

    rx_bw_khz_ = khz;
    return {};
}

std::expected<void, Err> SX1231::set_output_power( int8_t dbm )
{
    // PA0 path only: valid range -18 to +13 dBm.  PaLevel = dbm + 18.
    if ( dbm < -18 || dbm > 13 )
        return std::unexpected( Err::InvalidPower );

    set_mode( kModeStandby );

    spi_dev_.write_reg( Reg::PaLevel,
        static_cast<uint8_t>( 0b10000000u | static_cast<uint8_t>( dbm + 18 ) ) );

    return {};
}

std::expected<void, Err> SX1231::set_sync_word( std::span<const uint8_t> sw )
{
    if ( sw.empty() || sw.size() > 8 )
        return std::unexpected( Err::InvalidSyncWord );
    for ( const uint8_t b : sw )
        if ( b == 0x00 ) return std::unexpected( Err::InvalidSyncWord );

    // SyncOn | fill-on-SyncAddress | size=(len-1) | 0 error bits.
    spi_dev_.write_reg( Reg::SyncConfig,
        static_cast<uint8_t>( 0b10000000u | ((sw.size() - 1u) << 3u) ) );
    spi_dev_.write_burst( Reg::SyncValue1, sw );

    return {};
}

float SX1231::rssi() const
{
    // RssiValue register = -2 × RSSI_dBm.
    return -static_cast<float>( spi_dev_.read_reg( Reg::RssiValue ) ) / 2.0f;
}

} // namespace radio::sx1231
