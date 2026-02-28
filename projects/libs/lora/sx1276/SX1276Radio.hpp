// SX1276Radio.hpp
//
// Thin wrapper that bundles PicoHal + RadioLib Module + SX1276 into one
// object with a receive-focused API.  Consumers never touch RadioLib or
// PicoHal types directly.
//
// Lifetime: the instance must outlive all radio calls — RadioLib keeps
// internal pointers to the HAL and Module, so use as a static or global.

#pragma once

#include "PicoHal.h"
#include <RadioLib.h>

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <string.h>

// Returned by begin() / readPacket() on success.
static constexpr int LORA_OK = 0;   // == RADIOLIB_ERR_NONE

// ── Radio configuration ───────────────────────────────────────────────────────
struct SX1276Config {
    float    freq_mhz  = 915.0f;
    float    bw_khz    = 125.0f;
    uint8_t  sf        = 7;
    uint8_t  cr        = 5;
    uint8_t  sync_word = 0x12;
    int8_t   tx_power  = 17;
    uint16_t preamble  = 8;
};

// ── Received packet ───────────────────────────────────────────────────────────
struct LoRaPacket {
    char    data[ 256 ];
    uint8_t len;
    float   rssi;
    float   snr;
};

// ── Wrapper class ─────────────────────────────────────────────────────────────
class SX1276Radio {
public:
    // Construct with SPI bus + GPIO pin numbers.
    // Initialisation is deferred to begin().
    SX1276Radio( spi_inst_t* spi,
                 uint sck, uint mosi, uint miso,
                 uint nss, uint dio0, uint rst )
        : _hal( spi, sck, mosi, miso )
        , _module( &_hal, nss, dio0, rst, RADIOLIB_NC )
        , _radio( &_module )
        , _dio0( dio0 )
    {}

    // Initialise the radio with the given config.
    // Returns LORA_OK (0) on success, a negative RadioLib error code otherwise.
    int begin( const SX1276Config& cfg )
    {
        return _radio.begin( cfg.freq_mhz, cfg.bw_khz, cfg.sf, cfg.cr,
                             cfg.sync_word, cfg.tx_power, cfg.preamble );
    }

    // Enter continuous receive mode.
    // DIO0 goes high when a packet is fully received and CRC-validated.
    // Call again after each readPacket() to re-arm.
    void startReceive()
    {
        _radio.startReceive();
    }

    // Returns true when a packet is waiting (DIO0 high).
    // Poll this in your task loop; no busy-wait needed.
    bool packetAvailable() const
    {
        return gpio_get( _dio0 );
    }

    // Read the pending packet into 'pkt'.
    // Call only when packetAvailable() returns true.
    // Returns LORA_OK on success; pkt is populated with data, rssi, snr.
    int readPacket( LoRaPacket& pkt )
    {
        memset( pkt.data, 0, sizeof( pkt.data ) );
        pkt.len  = 0;
        pkt.rssi = 0.0f;
        pkt.snr  = 0.0f;

        // Read packet length from the radio header BEFORE draining the FIFO.
        // strnlen() stops at the first null byte in a binary SIGMA frame,
        // giving a truncated length.  getPacketLength() reads the SX1276
        // explicit-header length field which is correct for any payload.
        size_t rx_len = static_cast<size_t>( _radio.getPacketLength() );

        int state = _radio.readData( reinterpret_cast<uint8_t*>( pkt.data ),
                                     sizeof( pkt.data ) - 1 );

        if ( state == RADIOLIB_ERR_NONE ) {
            pkt.len  = static_cast<uint8_t>( rx_len );
            pkt.rssi = _radio.getRSSI();
            pkt.snr  = _radio.getSNR();
        }

        return state;
    }

private:
    // Declaration order matters — each member is initialised from the one above.
    PicoHal  _hal;
    Module   _module;
    SX1276   _radio;
    uint     _dio0;
};
