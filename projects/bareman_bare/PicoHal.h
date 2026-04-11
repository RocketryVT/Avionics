// PicoHal.h
#pragma once
#include <RadioLib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"


class PicoHal : public RadioLibHal {
public:
    PicoHal( spi_inst_t* spi,
             uint8_t sck,
             uint8_t mosi,
             uint8_t miso )
        : RadioLibHal( GPIO_IN, GPIO_OUT, 0, 1, GPIO_IRQ_EDGE_RISE, GPIO_IRQ_EDGE_FALL ),
          _spi( spi ), _sck( sck ), _mosi( mosi ), _miso( miso ) {}

    void init() override {
        spi_init( _spi, 1000000 );
        spi_set_format( _spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST );
        gpio_set_function( _sck,  GPIO_FUNC_SPI );
        gpio_set_function( _mosi, GPIO_FUNC_SPI );
        gpio_set_function( _miso, GPIO_FUNC_SPI );
    }

    void term() override {}

    void pinMode( uint32_t pin, uint32_t mode ) override {
        gpio_init( pin );
        gpio_set_dir( pin, mode == GPIO_OUT );
    }

    void digitalWrite( uint32_t pin, uint32_t value ) override {
        gpio_put( pin, value );
    }

    uint32_t digitalRead( uint32_t pin ) override {
        return gpio_get( pin );
    }

    void attachInterrupt( uint32_t pin, void (*func)( void ), uint32_t mode ) override {
        gpio_set_irq_enabled_with_callback( pin, mode, true, (gpio_irq_callback_t)func );
    }

    void detachInterrupt( uint32_t pin ) override {
        gpio_set_irq_enabled( pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false );
    }

    void delay( unsigned long ms ) override {
        sleep_ms( ms );
    }

    void delayMicroseconds( unsigned long us ) override {
        sleep_us( us );
    }

    unsigned long millis() override {
        return to_ms_since_boot( get_absolute_time() );
    }

    unsigned long micros() override {
        return to_us_since_boot( get_absolute_time() );
    }

    long pulseIn( uint32_t pin, uint32_t state, unsigned long timeout ) override {
        return 0;
    }

    void spiBegin() override {}
    void spiEnd() override {}
    void spiBeginTransaction() override {}
    void spiEndTransaction() override {}

    void spiTransfer( uint8_t* out, size_t len, uint8_t* in ) override {
        spi_write_read_blocking( _spi, out, in, len );
    }

private:
    spi_inst_t* _spi;
    uint8_t _sck, _mosi, _miso;
};