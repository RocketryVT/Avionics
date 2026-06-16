#include "lr11xx_hal.h"

#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define LR11XX_DEFAULT_SPI_SPEED_HZ   1000000u
#define LR11XX_DEFAULT_BUSY_TIMEOUT_MS 1000u

static inline void nss_low( const lr11xx_hal_context_t* context )
{
    gpio_put( context->pin_nss, 0 );
}

static inline void nss_high( const lr11xx_hal_context_t* context )
{
    gpio_put( context->pin_nss, 1 );
}

int lr11xx_hal_get_busy( const lr11xx_hal_context_t* context )
{
    return gpio_get( context->pin_busy );
}

bool lr11xx_hal_wait_on_busy( const lr11xx_hal_context_t* context, uint32_t timeout_ms )
{
    const absolute_time_t deadline = make_timeout_time_ms( timeout_ms );

    while( gpio_get( context->pin_busy ) == 1 )
    {
        if( absolute_time_diff_us( get_absolute_time(), deadline ) <= 0 )
        {
            return false;
        }
        sleep_us( 50 );
    }
    return true;
}

void lr11xx_hal_init( lr11xx_hal_context_t* context )
{
    const uint32_t spi_speed = ( context->spi_speed_hz == 0 ) ? LR11XX_DEFAULT_SPI_SPEED_HZ : context->spi_speed_hz;
    if( context->busy_timeout_ms == 0 )
    {
        context->busy_timeout_ms = LR11XX_DEFAULT_BUSY_TIMEOUT_MS;
    }

    spi_init( context->spi, spi_speed );
    spi_set_format( context->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST );

    gpio_set_function( context->pin_sck, GPIO_FUNC_SPI );
    gpio_set_function( context->pin_mosi, GPIO_FUNC_SPI );
    gpio_set_function( context->pin_miso, GPIO_FUNC_SPI );

    gpio_init( context->pin_nss );
    gpio_set_function( context->pin_nss, GPIO_FUNC_SIO );
    gpio_disable_pulls( context->pin_nss );
    gpio_set_dir( context->pin_nss, GPIO_OUT );
    gpio_put( context->pin_nss, 1 );

    gpio_init( context->pin_busy );
    gpio_set_function( context->pin_busy, GPIO_FUNC_SIO );
    gpio_set_dir( context->pin_busy, GPIO_IN );
    gpio_disable_pulls( context->pin_busy );

    if( context->use_busy_pullup )
    {
        gpio_pull_up( context->pin_busy );
    }
    else if( context->use_busy_pulldown )
    {
        gpio_pull_down( context->pin_busy );
    }

    gpio_init( context->pin_nreset );
    gpio_set_function( context->pin_nreset, GPIO_FUNC_SIO );
    gpio_disable_pulls( context->pin_nreset );
    gpio_set_dir( context->pin_nreset, GPIO_OUT );
    gpio_put( context->pin_nreset, 1 );
}

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    const lr11xx_hal_context_t* hal_ctx = ( const lr11xx_hal_context_t* ) context;

    if( !lr11xx_hal_wait_on_busy( hal_ctx, hal_ctx->busy_timeout_ms ) )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    nss_low( hal_ctx );
    spi_write_blocking( hal_ctx->spi, command, command_length );

    if( ( data != NULL ) && ( data_length > 0 ) )
    {
        spi_write_blocking( hal_ctx->spi, data, data_length );
    }
    nss_high( hal_ctx );

    if( !lr11xx_hal_wait_on_busy( hal_ctx, hal_ctx->busy_timeout_ms ) )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    const lr11xx_hal_context_t* hal_ctx = ( const lr11xx_hal_context_t* ) context;
    uint8_t dummy = 0;
    uint8_t nop   = LR11XX_NOP;

    if( !lr11xx_hal_wait_on_busy( hal_ctx, hal_ctx->busy_timeout_ms ) )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    nss_low( hal_ctx );
    spi_write_blocking( hal_ctx->spi, command, command_length );
    nss_high( hal_ctx );

    if( !lr11xx_hal_wait_on_busy( hal_ctx, hal_ctx->busy_timeout_ms ) )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    nss_low( hal_ctx );
    spi_write_read_blocking( hal_ctx->spi, &nop, &dummy, 1 );
    spi_read_blocking( hal_ctx->spi, LR11XX_NOP, data, data_length );
    nss_high( hal_ctx );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    const lr11xx_hal_context_t* hal_ctx = ( const lr11xx_hal_context_t* ) context;

    if( !lr11xx_hal_wait_on_busy( hal_ctx, hal_ctx->busy_timeout_ms ) )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    nss_low( hal_ctx );
    spi_read_blocking( hal_ctx->spi, LR11XX_NOP, data, data_length );
    nss_high( hal_ctx );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    const lr11xx_hal_context_t* hal_ctx = ( const lr11xx_hal_context_t* ) context;

    gpio_put( hal_ctx->pin_nreset, 0 );
    sleep_ms( 10 );
    gpio_put( hal_ctx->pin_nreset, 1 );
    sleep_ms( 5 );

    if( !lr11xx_hal_wait_on_busy( hal_ctx, hal_ctx->busy_timeout_ms ) )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    const lr11xx_hal_context_t* hal_ctx = ( const lr11xx_hal_context_t* ) context;

    nss_low( hal_ctx );
    sleep_us( 150 );
    nss_high( hal_ctx );

    if( !lr11xx_hal_wait_on_busy( hal_ctx, hal_ctx->busy_timeout_ms ) )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
    const lr11xx_hal_context_t* hal_ctx = ( const lr11xx_hal_context_t* ) context;

    nss_low( hal_ctx );
    sleep_us( 5 );
    nss_high( hal_ctx );

    return LR11XX_HAL_STATUS_OK;
}