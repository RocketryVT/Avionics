// radio/rf69/RF69.hpp — RF69 / RFM69HCW FSK driver.
//
// RadioLib wrapper.  Implements radio::IRadio for the RF69 FSK modem
// (433 MHz backup link, HCW high-power variant).
//
// Lifetime: declare as static or global — RadioLib holds internal pointers to
// the HAL and Module objects stored inside this class.
#pragma once

#include "../Radio.hpp"
#include "../../hal/PicoHal.h"

#include <RadioLib.h>
#include <cstdint>
#include <cstring>
#include "hardware/gpio.h"
#include "hardware/spi.h"

namespace radio::rf69 {

// ── Configuration ─────────────────────────────────────────────────────────────

struct Config {
    float   freq_mhz   = 433.0f;
    float   br_kbps    = 4.8f;    // bit rate
    float   fdev_khz   = 5.0f;    // frequency deviation
    float   rx_bw_khz  = 125.0f;  // RX channel filter bandwidth
    int8_t  tx_dbm     = 20;      // +20 dBm max with PA boost (HCW variant)
    bool    high_power = true;    // true for RFM69HCW (PA boost)
    uint8_t preamble   = 16;      // preamble length in bits
};

// ── Driver ────────────────────────────────────────────────────────────────────

class RF69 final : public radio::IRadio {
public:
    static constexpr uint8_t MAX_PACKET = 64;  // RF69 FIFO depth

    RF69( spi_inst_t* spi,
          uint sck, uint mosi, uint miso,
          uint nss, uint dio0, uint rst,
          const Config& cfg )
        : cfg_( cfg )
        , hal_( spi, static_cast<uint8_t>( sck  ),
                     static_cast<uint8_t>( mosi ),
                     static_cast<uint8_t>( miso ) )
        , module_( &hal_, nss, dio0, rst, RADIOLIB_NC )
        , radio_( &module_ )
        , dio0_( dio0 )
    {}

    int begin() override
    {
        int err = radio_.begin( cfg_.freq_mhz, cfg_.br_kbps, cfg_.fdev_khz,
                                cfg_.rx_bw_khz, cfg_.tx_dbm, cfg_.preamble );
        if ( err != RADIOLIB_ERR_NONE ) return err;

        // Re-apply output power with PA boost flag for the HCW variant.
        // begin() sets power without the flag; this corrects the PA registers.
        if ( cfg_.high_power )
            err = radio_.setOutputPower( cfg_.tx_dbm, true );

        return err;
    }

    void start_receive() override { radio_.startReceive(); }

    // DIO0 maps to Payload Ready in RX packet mode.
    bool packet_available() const override { return gpio_get( dio0_ ); }

    int read_packet( radio::Packet& pkt ) override
    {
        std::memset( &pkt, 0, sizeof(pkt) );

        size_t len = static_cast<size_t>( radio_.getPacketLength() );
        if ( len > MAX_PACKET ) len = MAX_PACKET;

        const int err = radio_.readData( pkt.data, len );
        if ( err == RADIOLIB_ERR_NONE ) {
            pkt.len  = static_cast<uint8_t>( len );
            pkt.rssi = radio_.getRSSI();
            pkt.snr  = 0.0f;  // FSK has no SNR
        }
        return err;
    }

private:
    Config  cfg_;
    PicoHal hal_;
    Module  module_;
    ::RF69  radio_;
    uint    dio0_;
};

} // namespace radio::rf69
