// Not gonna lie, Claude wrote 90% of this code.

// SX1276_Test.cpp
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "PicoHal.h"
#include <RadioLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SIGMA.hpp"

// -- SPI / LoRa pin definitions ---------------------------------------------
#define PIN_SCK     26
#define PIN_MOSI    27
#define PIN_MISO    28
#define PIN_NSS     29
#define PIN_DIO0    22
#define PIN_RESET   23

// -- GPS UART pin definitions -----------------------------------------------
#define GPS_UART        uart0
#define PIN_UART_TX     16
#define PIN_UART_RX     17
#define UART_BAUD       38400    // factory default after CFG-CFG reset

// -- LED definitions --------------------------------------------------------
#define PIN_LED_TX      13

// -- HAL and radio (global) -------------------------------------------------
PicoHal hal( spi1, PIN_SCK, PIN_MOSI, PIN_MISO );
SX1276 radio = new Module( &hal, PIN_NSS, PIN_DIO0, PIN_RESET, RADIOLIB_NC );

// -- GPS state --------------------------------------------------------------
static char  gps_line[256];
static int   gps_idx = 0;
static float gps_lat = 0.0f;
static float gps_lon = 0.0f;
static bool  gps_fix = false;

// -- Read one NMEA sentence (non-blocking) ----------------------------------
bool gps_read_line( char* buf, size_t max_len ) {
    while( uart_is_readable( GPS_UART ) ) {
        char c = uart_getc( GPS_UART );

        if( c == '\n' ) {
            gps_line[gps_idx] = '\0';
            gps_idx = 0;
            if( gps_line[0] == '$' ) {
                strncpy( buf, gps_line, max_len );
                buf[max_len - 1] = '\0';
                return true;
            }
        } else if( c != '\r' ) {
            if( gps_idx < (int)sizeof(gps_line) - 1 )
                gps_line[gps_idx++] = c;
        }
    }
    return false;
}

// -- Parse $GNRMC / $GPRMC -------------------------------------------------
bool parse_rmc( const char* sentence ) {
    if( strncmp( sentence, "$GNRMC", 6 ) != 0 &&
        strncmp( sentence, "$GPRMC", 6 ) != 0 ) return false;

    const char* p = sentence;
    int field = 0;
    float raw_lat = 0, raw_lon = 0;
    char ns = 'N', ew = 'E', status = 'V';

    while( *p ) {
        if( *p == ',' ) {
            field++;
            p++;
            switch( field ) {
                case 2:  status  = *p;         break;
                case 3:  raw_lat = atof( p );  break;
                case 4:  ns      = *p;         break;
                case 5:  raw_lon = atof( p );  break;
                case 6:  ew      = *p;         break;
            }
        } else {
            p++;
        }
    }

    if( status != 'A' ) {
        gps_fix = false;
        return false;
    }

    int lat_deg = (int)( raw_lat / 100 );
    int lon_deg = (int)( raw_lon / 100 );
    gps_lat = lat_deg + ( raw_lat - lat_deg * 100 ) / 60.0f;
    gps_lon = lon_deg + ( raw_lon - lon_deg * 100 ) / 60.0f;
    if( ns == 'S' ) gps_lat = -gps_lat;
    if( ew == 'W' ) gps_lon = -gps_lon;

    gps_fix = true;
    return true;
}

// -- Entry point ------------------------------------------------------------
int main() {
    stdio_init_all();

    gpio_init( PIN_LED_TX );
    gpio_set_dir( PIN_LED_TX, GPIO_OUT );
    gpio_put( PIN_LED_TX, false );

    // GPS UART
    uart_init( GPS_UART, UART_BAUD );
    gpio_set_function( PIN_UART_TX, GPIO_FUNC_UART );
    gpio_set_function( PIN_UART_RX, GPIO_FUNC_UART );
    uart_set_format( GPS_UART, 8, 1, UART_PARITY_NONE );
    uart_set_hw_flow( GPS_UART, false, false );
    uart_set_fifo_enabled( GPS_UART, true );

    sleep_ms( 500 );

    printf( "Initializing SX1276...\n" );

    ConfigLoRa_t config;
    config.frequency       = 915.0f;
    config.bandwidth       = 125.0f;
    config.spreadingFactor = 7;
    config.codingRate      = 5;
    config.syncWord        = 0x12;
    config.power           = 20;
    config.preambleLength  = 8;

    int state = radio.begin( config );

    if( state == RADIOLIB_ERR_NONE ) {
        printf( "SX1276 ready!\n" );
    } else {
        printf( "SX1276 init failed, code: %d\n", state );
        while( true );
    }

    printf( "Waiting for GPS fix...\n" );

    char     line[256];
    uint32_t last_tx_ms = 0;

    while( true ) {
        // Drain all available NMEA sentences
        while( gps_read_line( line, sizeof(line) ) ) {
            if( strncmp( line, "$GNRMC", 6 ) == 0 ||
                strncmp( line, "$GPRMC", 6 ) == 0 ) {
                parse_rmc( line );
            }
        }

        // Transmit at 1 Hz when we have a fix
        uint32_t now = to_ms_since_boot( get_absolute_time() );
        if( now - last_tx_ms >= 1000 ) {
            last_tx_ms = now;

            if( !gps_fix ) {
                printf( "no fix\n" );
            } else {
                SIGMA::GpsNavData gn;
                gn.lat   = (double)gps_lat;
                gn.lon   = (double)gps_lon;
                gn.flags = SIGMA::FLAG_GPS_VALID;

                uint8_t frame[ SIGMA::MAX_FRAME ];
                size_t  frame_len = gn.serialize( frame, sizeof(frame) );

                int tx_state = radio.transmit( frame, frame_len );
                if( tx_state == RADIOLIB_ERR_NONE ) {
                    printf( "TX %.6f, %.6f\n", gps_lat, gps_lon );
                    gpio_put( PIN_LED_TX, true );
                    sleep_ms( 50 );
                    gpio_put( PIN_LED_TX, false );
                } else {
                    printf( "TX failed, code: %d\n", tx_state );
                }
            }
        }
    }
}
