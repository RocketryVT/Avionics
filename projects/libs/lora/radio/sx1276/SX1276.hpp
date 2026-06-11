// radio/sx1276/SX1276.hpp — SX1276 LoRa driver.
//
// RadioLib wrapper.  Implements radio::IRadio for the SX1276 LoRa modem
// (CSS modulation, 915 MHz primary link).
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

namespace radio::sx1276 {

// -- Configuration -------------------------------------------------------------

struct Config {
    float    freq_mhz  = 915.0f;
    float    bw_khz    = 125.0f;
    uint8_t  sf        = 7;        // spreading factor 6–12
    uint8_t  cr        = 5;        // coding rate denominator 5–8
    uint8_t  sync_word = 0x12;
    int8_t   tx_dbm    = 17;       // +2 to +17 dBm (or +20 with PA boost)
    uint16_t preamble  = 8;
};

// -- Driver --------------------------------------------------------------------

class SX1276 final : public radio::IRadio {
public:
    SX1276( spi_inst_t* spi,
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
        ConfigLoRa_t config;
        config.frequency       = cfg_.freq_mhz;
        config.bandwidth       = cfg_.bw_khz;
        config.spreadingFactor = cfg_.sf;
        config.codingRate      = cfg_.cr;
        config.syncWord        = cfg_.sync_word;
        config.power           = cfg_.tx_dbm;
        config.preambleLength  = cfg_.preamble;
        return radio_.begin( config );
    }

    void start_receive() override { radio_.startReceive(); }

    // DIO0 goes high on RxDone in LoRa mode.
    bool packet_available() const override { return gpio_get( dio0_ ); }

    int read_packet( radio::Packet& pkt ) override
    {
        std::memset( &pkt, 0, sizeof(pkt) );

        const size_t len = static_cast<size_t>( radio_.getPacketLength() );
        const int    err = radio_.readData( pkt.data, sizeof(pkt.data) - 1 );

        if ( err == RADIOLIB_ERR_NONE ) {
            pkt.len  = static_cast<uint8_t>( len );
            pkt.rssi = radio_.getRSSI();
            pkt.snr  = radio_.getSNR();
        }
        return err;
    }

private:
    Config   cfg_;
    PicoHal  hal_;
    Module   module_;
    ::SX1276 radio_;
    uint     dio0_;
};

} // namespace radio::sx1276
