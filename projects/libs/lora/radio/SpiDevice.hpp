// radio/SpiDevice.hpp — low-level Pico SPI helper for native radio drivers.
//
// Owns nothing; callers supply an already-initialised spi_inst_t* and GPIO pin
// numbers.  Used internally by radio::sx1231::SX1231 (and any future native
// drivers) so the SPI framing logic lives in one place.
//
// Register framing convention (RF69 / SX127x family):
//   Write: addr | 0x80, data...
//   Read:  addr & 0x7F, dummy...
#pragma once

#include "hardware/gpio.h"
#include "hardware/spi.h"

#include <cstdint>
#include <span>

namespace radio {

class SpiDevice {
public:
    SpiDevice( spi_inst_t* spi, uint nss )
        : spi_( spi ), nss_( nss )
    {}

    // Initialise the SPI peripheral and all GPIO pins.
    // spi_hz: desired clock rate; sck/mosi/miso are set to GPIO_FUNC_SPI.
    void init( uint sck, uint mosi, uint miso, uint32_t spi_hz = 1'000'000 )
    {
        spi_init( spi_, spi_hz );
        spi_set_format( spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST );
        gpio_set_function( sck,  GPIO_FUNC_SPI );
        gpio_set_function( mosi, GPIO_FUNC_SPI );
        gpio_set_function( miso, GPIO_FUNC_SPI );

        gpio_init( nss_ );
        gpio_set_dir( nss_, GPIO_OUT );
        gpio_put( nss_, true );
    }

    void write_reg( uint8_t addr, uint8_t val )
    {
        const uint8_t buf[2] = { static_cast<uint8_t>( addr | 0x80u ), val };
        cs_select();
        spi_write_blocking( spi_, buf, 2 );
        cs_deselect();
    }

    uint8_t read_reg( uint8_t addr ) const
    {
        const uint8_t tx[2] = { static_cast<uint8_t>( addr & 0x7Fu ), 0x00 };
        uint8_t rx[2] = {};
        cs_select();
        spi_write_read_blocking( spi_, tx, rx, 2 );
        cs_deselect();
        return rx[1];
    }

    void write_burst( uint8_t addr, std::span<const uint8_t> data )
    {
        const uint8_t hdr = addr | 0x80u;
        cs_select();
        spi_write_blocking( spi_, &hdr, 1 );
        spi_write_blocking( spi_, data.data(), data.size() );
        cs_deselect();
    }

    void read_burst( uint8_t addr, std::span<uint8_t> out ) const
    {
        const uint8_t hdr = addr & 0x7Fu;
        cs_select();
        spi_write_blocking( spi_, &hdr, 1 );
        for ( auto& byte : out ) {
            const uint8_t dummy = 0x00;
            spi_write_read_blocking( spi_, &dummy, &byte, 1 );
        }
        cs_deselect();
    }

    // Read-modify-write: set bits [msb:lsb] to val (val is pre-shifted to bit 0).
    void set_bits( uint8_t addr, uint8_t val, uint8_t msb, uint8_t lsb )
    {
        const uint8_t mask = static_cast<uint8_t>( ((1u << (msb - lsb + 1u)) - 1u) << lsb );
        const uint8_t cur  = read_reg( addr );
        write_reg( addr, static_cast<uint8_t>( (cur & ~mask) | ((val << lsb) & mask) ) );
    }

private:
    void cs_select()   const { gpio_put( nss_, false ); }
    void cs_deselect() const { gpio_put( nss_, true  ); }

    spi_inst_t* spi_;
    uint        nss_;
};

} // namespace radio
