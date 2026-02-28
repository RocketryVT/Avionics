#include "pico/stdlib.h"
#include <stdio.h>

extern "C" {
#include "lr11xx_hal.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
}

int main( void ) {
    stdio_init_all();
    sleep_ms( 2000 );

    // Initialize hardware (setup SPI and reset radio)
    lr11xx_hal_init();
    lr11xx_hal_reset( NULL );
    lr11xx_hal_wakeup( NULL );

    // Set standby mode before configuring anything 
    lr11xx_system_set_standby( NULL, LR11XX_SYSTEM_STANDBY_CFG_RC );

    // Set packet type to LoRa 
    lr11xx_radio_set_pkt_type( NULL, LR11XX_RADIO_PKT_TYPE_LORA );

    // Set frequency (915MHz)
    lr11xx_radio_set_rf_freq( NULL, 915000000 );

    // Set TX output power and ramp time (22dBm is max) (48us is default))
    lr11xx_radio_set_tx_params( NULL, 22, LR11XX_RADIO_RAMP_48_US );
    

    // Set PA config (which PA to use) 
    lr11xx_radio_pa_cfg_t pa_config = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HP,   // high power PA
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT, // switch to VREG if low power PA
        .pa_duty_cycle = 0x04,
        .pa_hp_sel     = 0x07,
    };
    lr11xx_radio_set_pa_cfg( NULL, &pa_config );

    // Set LoRa modulation params
    lr11xx_radio_mod_params_lora_t mod_params = {
        .sf   = LR11XX_RADIO_LORA_SF7,       // spreading factor
        .bw   = LR11XX_RADIO_LORA_BW_250,    // bandwidth 250kHz // switch to 125kHz for longer range
        .cr   = LR11XX_RADIO_LORA_CR_4_5,    // coding rate 4/5
        .ldro = 0                            // low data rate optimize off
    };
    lr11xx_radio_set_lora_mod_params( NULL, &mod_params );

    // Set LoRa packet params 
    // change payload length based on size of sensor data
    lr11xx_radio_pkt_params_lora_t pkt_params = {
        .preamble_len_in_symb = 8,                              // 8 preamble symbols
        .header_type          = LR11XX_RADIO_LORA_PKT_EXPLICIT, // explicit header
        .pld_len_in_bytes     = 64,                             // payload length
        .crc                  = LR11XX_RADIO_LORA_CRC_ON,       // CRC enabled
        .iq                   = LR11XX_RADIO_LORA_IQ_STANDARD   // standard IQ
    };
    lr11xx_radio_set_lora_pkt_params( NULL, &pkt_params );

    // Verify chip is alive 
    lr11xx_system_version_t ver;
    lr11xx_system_get_version( NULL, &ver );
    printf( "LR1121 ready. FW: %d.%d\n", ver.fw >> 8, ver.fw & 0xFF );

    // Main loop
    while( true ) {
        // TX example
        uint8_t payload[] = "Wassup!";
        lr11xx_radio_set_tx( NULL, 3000 ); // 3 second timeout
        printf( "Transmitted!\n" );
        sleep_ms( 5000 );
    }
}
