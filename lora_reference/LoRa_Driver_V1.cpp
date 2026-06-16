#include "pico/stdlib.h"
#include <stdio.h>

extern "C" {
#include "lr11xx_hal.h"
#include "lr11xx_bootloader.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_regmem.h"
}

#ifndef LR11XX_RFSW1_BIT
#define LR11XX_RFSW1_BIT ( 1 << 1 )
#endif

static void fail( const lr11xx_hal_context_t* ctx, const uint led_pin, const char* msg )
{
    printf( "ERROR: %s\n", msg );
    if( ctx != NULL )
    {
        printf( "BUSY=%d NRESET=%d NSS=%d\n", lr11xx_hal_get_busy( ctx ),
                gpio_get( ctx->pin_nreset ), gpio_get( ctx->pin_nss ) );
    }
    gpio_put( led_pin, 1 );

    while( true )
    {
        gpio_xor_mask( 1u << led_pin );
        sleep_ms( 200 );
    }
}

static void check_hal( const lr11xx_hal_context_t* ctx, const uint led_pin, const lr11xx_hal_status_t status, const char* msg )
{
    if( status != LR11XX_HAL_STATUS_OK )
    {
        printf( "HAL status=%d\n", status );
        fail( ctx, led_pin, msg );
    }
}

static void check_lr11xx( const lr11xx_hal_context_t* ctx, const uint led_pin, const lr11xx_status_t status, const char* msg )
{
    if( status != LR11XX_STATUS_OK )
    {
        printf( "LR11xx status=%d\n", status );
        fail( ctx, led_pin, msg );
    }
}

int main( void )
{
    stdio_init_all();
    sleep_ms( 3000 );

    //const uint LED_PIN = 12; // 12 for Tracker
    const uint LED_PIN = 23; // 23 for ADS Control Board
    gpio_init( LED_PIN );
    gpio_set_dir( LED_PIN, GPIO_OUT );
    gpio_put( LED_PIN, 0 );

    // LoRa configuration
    const uint8_t LORA_SYNC_WORD = 0x12; // 0x34 public, 0x12 private
    const uint16_t LORA_PREAMBLE_SYMB = 8; // preamble length in symbols
    const uint8_t PAYLOAD_LEN = 16; // transmit payload length in bytes

    lr11xx_hal_context_t ctx = {
        .spi               = spi0,
        .pin_sck           = 6,
        .pin_mosi          = 7,
        .pin_miso          = 4,
        .pin_nss           = 5,
        .pin_busy          = 0,
        .pin_nreset        = 1,
        .spi_speed_hz      = 1000000,
        .busy_timeout_ms   = 1000,
        .use_busy_pullup   = false,
        .use_busy_pulldown = true,
    };

    lr11xx_hal_init( &ctx );

    printf( "hal_init complete\n" );
    printf( "pins: SCK=%u MOSI=%u MISO=%u NSS=%u BUSY=%u NRESET=%u\n",
            ctx.pin_sck, ctx.pin_mosi, ctx.pin_miso, ctx.pin_nss, ctx.pin_busy, ctx.pin_nreset );
    printf( "BUSY before reset=%d\n", lr11xx_hal_get_busy( &ctx ) );

    // Quick BUSY pin diagnostics: sample raw level and with pulls
    {
        const uint busy_pin = ctx.pin_busy;
        gpio_set_function( busy_pin, GPIO_FUNC_SIO );
        gpio_set_dir( busy_pin, GPIO_IN );
        gpio_disable_pulls( busy_pin );
        sleep_ms( 10 );
        printf( "BUSY raw=%d\n", gpio_get( busy_pin ) );

        gpio_pull_up( busy_pin );
        sleep_ms( 10 );
        printf( "BUSY with pull-up=%d\n", gpio_get( busy_pin ) );

        gpio_pull_down( busy_pin );
        sleep_ms( 10 );
        printf( "BUSY with pull-down=%d\n", gpio_get( busy_pin ) );

        gpio_disable_pulls( busy_pin );
    }

    check_hal( &ctx, LED_PIN, lr11xx_hal_reset( &ctx ), "lr11xx_hal_reset failed; check NRESET/BUSY/power" );
    printf( "reset passed, BUSY=%d\n", lr11xx_hal_get_busy( &ctx ) );

    check_hal( &ctx, LED_PIN, lr11xx_hal_wakeup( &ctx ), "lr11xx_hal_wakeup failed; check NSS/BUSY" );
    printf( "wakeup passed, BUSY=%d\n", lr11xx_hal_get_busy( &ctx ) );

    lr11xx_bootloader_version_t blver = { 0 };
    lr11xx_status_t status            = lr11xx_bootloader_get_version( &ctx, &blver );
    printf( "bootloader_get_version status=%d\n", status );
    if( status == LR11XX_STATUS_OK )
    {
        printf( "BOOTLOADER: HW=0x%02X TYPE=0x%02X FW=%u.%u\n",
                blver.hw, blver.type, blver.fw >> 8, blver.fw & 0xFF );
    }

    lr11xx_system_version_t ver = { 0 };
    status = lr11xx_system_get_version( &ctx, &ver );
    printf( "system_get_version status=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "system_get_version failed; check SPI wiring, mode, BUSY, reset" );

    printf( "LR11xx detected\n" );
    printf( "HW=0x%02X TYPE=0x%02X FW=%u.%u\n", ver.hw, ver.type, ver.fw >> 8, ver.fw & 0xFF );

    status = lr11xx_system_set_reg_mode( &ctx, LR11XX_SYSTEM_REG_MODE_DCDC );
    printf( "set_reg_mode=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "set_reg_mode failed" );

    status = lr11xx_system_cfg_lfclk( &ctx, LR11XX_SYSTEM_LFCLK_RC, true );
    printf( "cfg_lfclk=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "cfg_lfclk failed" );

    lr11xx_system_rfswitch_cfg_t rf_switch_cfg = {
        .enable  = LR11XX_RFSW1_BIT,
        .standby = 0x00,
        .rx      = 0x00,
        .tx      = 0x00,
        .tx_hp   = LR11XX_RFSW1_BIT,
        .tx_hf   = 0x00,
        .gnss    = 0x00,
        .wifi    = 0x00,
    };
    status = lr11xx_system_set_dio_as_rf_switch( &ctx, &rf_switch_cfg );
    printf( "set_dio_as_rf_switch=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "set_dio_as_rf_switch failed" );

    status = lr11xx_system_set_tcxo_mode( &ctx, LR11XX_SYSTEM_TCXO_CTRL_2_7V, 164 );
    printf( "set_tcxo_mode=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "set_tcxo_mode failed; confirm your board really uses a TCXO" );

    status = lr11xx_system_clear_errors( &ctx );
    printf( "clear_errors=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "clear_errors failed" );

    status = lr11xx_system_calibrate( &ctx, 0x3F );
    printf( "calibrate=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "calibrate failed" );

    status = lr11xx_system_calibrate_image( &ctx, 0x6B, 0x6E );
    printf( "calibrate_image=%d\n", status );
    check_lr11xx( &ctx, LED_PIN, status, "calibrate_image failed; verify image calibration bytes for your band" );

    

                // Configure for GFSK packet type (used for TX)
                status = lr11xx_radio_set_pkt_type( &ctx, LR11XX_RADIO_PKT_TYPE_GFSK );
                printf( "set_pkt_type=GFSK status=%d\n", status );
                check_lr11xx( &ctx, LED_PIN, status, "set_pkt_type failed" );

                // Optional: set GFSK sync word (8 bytes) — first two bytes = 0x2D01
                const uint8_t gfsk_sync_word[LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH] = { 0x2D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                status = lr11xx_radio_set_gfsk_sync_word( &ctx, gfsk_sync_word );
                printf( "set_gfsk_sync_word status=%d\n", status );
                check_lr11xx( &ctx, LED_PIN, status, "set_gfsk_sync_word failed" );

                status = lr11xx_radio_set_rf_freq( &ctx, 433000000);
                printf( "set_rf_freq=%d\n", status );
                check_lr11xx( &ctx, LED_PIN, status, "set_rf_freq failed" );

                lr11xx_radio_pa_cfg_t pa_cfg = {
                    .pa_sel        = LR11XX_RADIO_PA_SEL_HP,
                    .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
                    .pa_duty_cycle = 0x04,
                    .pa_hp_sel     = 0x07,
                };
                status = lr11xx_radio_set_pa_cfg( &ctx, &pa_cfg );
                printf( "set_pa_cfg=%d\n", status );
                check_lr11xx( &ctx, LED_PIN, status, "set_pa_cfg failed" );

                status = lr11xx_radio_set_tx_params( &ctx, 22, LR11XX_RADIO_RAMP_208_US );
                printf( "set_tx_params=%d\n", status );
                check_lr11xx( &ctx, LED_PIN, status, "set_tx_params failed" );

                // GFSK modulation params (example: 50 kbps, BT=0.5, Fdev=25 kHz)
                lr11xx_radio_mod_params_gfsk_t gfsk_mod_params = {
                    .br_in_bps   = 4800,
                    .pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05,
                    .bw_dsb_param = LR11XX_RADIO_GFSK_BW_117300,
                    .fdev_in_hz  = 5000,
                };
                status = lr11xx_radio_set_gfsk_mod_params( &ctx, &gfsk_mod_params );
                printf( "set_gfsk_mod_params=%d\n", status );
                check_lr11xx( &ctx, LED_PIN, status, "set_gfsk_mod_params failed" );

                // GFSK packet params
                lr11xx_radio_pkt_params_gfsk_t gfsk_pkt_params = {
                    .preamble_len_in_bits = 16,
                    .preamble_detector    = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
                    .sync_word_len_in_bits = 16,
                    .address_filtering     = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
                    .header_type           = LR11XX_RADIO_GFSK_PKT_FIX_LEN,
                    .pld_len_in_bytes      = PAYLOAD_LEN,
                    .crc_type              = LR11XX_RADIO_GFSK_CRC_2_BYTES,
                    .dc_free               = LR11XX_RADIO_GFSK_DC_FREE_OFF,
                };
                status = lr11xx_radio_set_gfsk_pkt_params( &ctx, &gfsk_pkt_params );
                printf( "set_gfsk_pkt_params=%d\n", status );
                check_lr11xx( &ctx, LED_PIN, status, "set_gfsk_pkt_params failed" );

    // --- Transmit loop (GFSK) ---
    uint8_t payload[PAYLOAD_LEN] = { 0 };
    const uint8_t payload_len = PAYLOAD_LEN;
    uint32_t tx_counter = 0;

    while ( true )
    {
        // prepare payload: counter in first two bytes, rest zeros
        payload[0] = (uint8_t)( tx_counter & 0xFF );
        payload[1] = (uint8_t)( ( tx_counter >> 8 ) & 0xFF );
        for ( uint8_t i = 2; i < payload_len; i++ ) payload[i] = 0;

        status = lr11xx_regmem_write_buffer8( &ctx, payload, payload_len );
        check_lr11xx( &ctx, LED_PIN, status, "write_buffer failed" );

        const uint32_t toa_ms = lr11xx_radio_get_gfsk_time_on_air_in_ms( &gfsk_pkt_params, &gfsk_mod_params );
        status = lr11xx_radio_set_tx( &ctx, toa_ms + 50 ); // margin
        if ( status != LR11XX_STATUS_OK )
        {
            check_lr11xx( &ctx, LED_PIN, status, "set_tx failed" );
        }
        else
        {
            printf( "Packet sent, ToA=%u ms counter=%u\n", toa_ms, tx_counter );
            gpio_xor_mask( 1u << LED_PIN );
        }

        // wait for TX_DONE (polling)
        uint32_t start = to_ms_since_boot( get_absolute_time() );
        lr11xx_system_irq_mask_t irq = 0;
        while ( true )
        {
            status = lr11xx_system_get_and_clear_irq_status( &ctx, &irq );
            if ( status != LR11XX_STATUS_OK )
            {
                check_lr11xx( &ctx, LED_PIN, status, "get_irq failed" );
            }
            if ( ( irq & LR11XX_SYSTEM_IRQ_TX_DONE ) != 0 ) break;
            if ( to_ms_since_boot( get_absolute_time() ) - start > (int)( toa_ms + 5000 ) )
            {
                fail( &ctx, LED_PIN, "TX timed out" );
            }
            sleep_ms( 10 );
        }

        tx_counter++; // advance for next transmission
        sleep_ms( 3000 ); // inter-packet delay
    }

    return 0;
}