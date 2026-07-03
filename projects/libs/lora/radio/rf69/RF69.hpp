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

// -- Configuration -------------------------------------------------------------

struct Config {
    float   freq_mhz   = 433.0f;
    float   br_kbps    = 4.8f;    // bit rate
    float   fdev_khz   = 5.0f;    // frequency deviation
    float   rx_bw_khz  = 125.0f;  // RX channel filter bandwidth
    int8_t  tx_dbm     = 20;      // +20 dBm max with PA boost (HCW variant)
    bool    high_power = true;    // true for RFM69HCW (PA boost)
    uint8_t preamble   = 16;      // preamble length in bits
    uint8_t sync_word[2] = { 0x2D, 0x01 };
    uint8_t sync_word_len = 2;
    uint8_t fixed_len = 64;
    uint8_t data_shaping = RADIOLIB_SHAPING_0_5;
    bool    afc = false;          // auto frequency correction on each RX entry
};

// -- Driver --------------------------------------------------------------------

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

    const char* init_stage() const { return init_stage_; }

    int begin() override
    {
        const int8_t init_power =
            ( cfg_.high_power && cfg_.tx_dbm > 13 ) ? 13 : cfg_.tx_dbm;

        ConfigFSK_t config;
        config.frequency          = cfg_.freq_mhz;
        config.bitRate            = cfg_.br_kbps;
        config.frequencyDeviation = cfg_.fdev_khz;
        config.receiverBandwidth  = cfg_.rx_bw_khz;
        config.power              = init_power;
        config.preambleLength     = cfg_.preamble;

        init_stage_ = "begin";
        int err = radio_.begin( config );
        if ( err != RADIOLIB_ERR_NONE ) return err;

        init_stage_ = "setSyncWord";
        err = radio_.setSyncWord(
            static_cast<const uint8_t*>( cfg_.sync_word ), cfg_.sync_word_len, 0 );
        if ( err != RADIOLIB_ERR_NONE ) return err;

        init_stage_ = "fixedPacketLengthMode";
        err = radio_.fixedPacketLengthMode( cfg_.fixed_len );
        if ( err != RADIOLIB_ERR_NONE ) return err;

        init_stage_ = "setDataShaping";
        err = radio_.setDataShaping( cfg_.data_shaping );
        if ( err != RADIOLIB_ERR_NONE ) return err;

        // Re-apply output power with PA boost flag for the HCW variant.
        // begin() sets power without the flag; this corrects the PA registers.
        if ( cfg_.high_power ) {
            init_stage_ = "setOutputPowerHigh";
            err = radio_.setOutputPower( cfg_.tx_dbm, true );
            if ( err != RADIOLIB_ERR_NONE ) return err;
        }

        // Optional automatic frequency correction. With a narrow RX bandwidth,
        // RFM69 crystal tolerance (~±20 ppm ≈ ±8.5 kHz @ 424.5 MHz, per end)
        // can walk the signal off-centre. Auto-AFC measures the offset during
        // the preamble each time RX starts and re-centres the receiver, so the
        // narrow RxBw can be used without losing the packet. AfcBw is left at
        // the chip default (50 kHz → ±25 kHz capture, covers the combined
        // crystal spread). AUTOCLEAR resets the correction every RX entry.
        if ( cfg_.afc ) {
            init_stage_ = "afc";
            err = module_.SPIsetRegValue(
                RADIOLIB_RF69_REG_AFC_FEI,
                RADIOLIB_RF69_AFC_AUTO_ON | RADIOLIB_RF69_AFC_AUTOCLEAR_ON,
                3, 2 );
            if ( err != RADIOLIB_ERR_NONE ) return err;
        }

        init_stage_ = "ready";
        return RADIOLIB_ERR_NONE;
    }

    void start_receive() override { radio_.startReceive(); }

    // DIO0 maps to Payload Ready in RX packet mode.
    bool packet_available() const override { return gpio_get( dio0_ ); }

    int read_packet( radio::Packet& pkt ) override
    {
        std::memset( &pkt, 0, sizeof(pkt) );

        size_t len = cfg_.fixed_len ? cfg_.fixed_len : static_cast<size_t>( radio_.getPacketLength() );
        if ( len > MAX_PACKET ) len = MAX_PACKET;

        const int err = radio_.readData( pkt.data, len );
        if ( err == RADIOLIB_ERR_NONE ) {
            while ( len > 0 && pkt.data[len - 1] == 0u ) {
                --len;
            }
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
    const char* init_stage_ = "not-started";
};

} // namespace radio::rf69
