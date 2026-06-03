// SX1276_Test.cpp
#include "pico/stdlib.h"
#include "PicoHal.h"
#include <RadioLib.h>
#include <stdio.h>

// -- Pin definitions --------------------------------------------------------
#define PIN_SCK   6
#define PIN_MOSI  7
#define PIN_MISO  4
#define PIN_NSS   5
#define PIN_DIO0  0
#define PIN_RESET 14

// -- Setup HAL and radio ----------------------------------------------------
PicoHal hal( spi0, PIN_SCK, PIN_MOSI, PIN_MISO );
SX1276 radio = new Module( &hal, PIN_NSS, PIN_DIO0, PIN_RESET, RADIOLIB_NC );

int main() {
    stdio_init_all();
    
    // Wait for USB CDC connection to be ready
    while( !stdio_usb_connected() ) {
        sleep_ms( 100 );
    }
    
    sleep_ms( 1000 );
    printf( "Initializing SX1276...\n" );

    int state = radio.begin(
        915.0,  // frequency MHz
        125.0,  // bandwidth kHz
        7,      // spreading factor
        5,      // coding rate
        0x12,   // sync word
        20,     // output power dBm
        8       // preamble length
    );

    if( state == RADIOLIB_ERR_NONE ) {
        printf( "SX1276 ready!\n" );
    } else {
        printf( "Init failed, code: %d\n", state );
        while( true );
    }

    printf( "Init state: %d\n", state );
    
    // // -- Transmitter loop ---------------------------------------------------
    while( true ) {
        printf( "Transmitting...\n" );

        int state = radio.transmit( "Hello LoRa!" );

        if( state == RADIOLIB_ERR_NONE ) {
            printf( "Sent!\n" );
        } else {
            printf( "TX failed, code: %d\n", state );
        }

        sleep_ms( 5000 );
    }

    // -- Receiver loop -----------------------------------------------------
    // char received[256];

    // while( true ) {
    //     memset( received, 0, sizeof(received) );
    //     int state = radio.receive( (uint8_t*)received, sizeof(received) );

    //     if( state == RADIOLIB_ERR_NONE ) {
    //         float rssi = radio.getRSSI();
    //         float snr = radio.getSNR();
    //         printf( "Received: %s\n", received );
    //         printf( "RSSI: %.1f dBm\n", rssi );
    //         printf( "SNR:  %.1f dB\n",  snr );
    //     } else if( state == RADIOLIB_ERR_RX_TIMEOUT ) {
    //         // No packet received within the timeout period, this is normal behavior
    //     } else {
    //         printf( "RX failed, code: %d\n", state );
    //     }
    // }
}