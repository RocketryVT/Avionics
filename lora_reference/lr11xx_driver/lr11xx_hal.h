#ifndef LR11XX_HAL_H
#define LR11XX_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/types.h"

#define LR11XX_NOP ( 0x00 )

typedef enum lr11xx_hal_status_e
{
    LR11XX_HAL_STATUS_OK    = 0,
    LR11XX_HAL_STATUS_ERROR = 3,
} lr11xx_hal_status_t;

typedef struct lr11xx_hal_context_s
{
    spi_inst_t* spi;

    uint pin_sck;
    uint pin_mosi;
    uint pin_miso;
    uint pin_nss;
    uint pin_busy;
    uint pin_nreset;

    uint32_t spi_speed_hz;
    uint32_t busy_timeout_ms;

    bool use_busy_pullup;
    bool use_busy_pulldown;
} lr11xx_hal_context_t;

void lr11xx_hal_init( lr11xx_hal_context_t* context );

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length );

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length );

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length );

lr11xx_hal_status_t lr11xx_hal_reset( const void* context );
lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context );
lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context );

bool lr11xx_hal_wait_on_busy( const lr11xx_hal_context_t* context, uint32_t timeout_ms );
int  lr11xx_hal_get_busy( const lr11xx_hal_context_t* context );

static inline uint8_t lr11xx_hal_compute_crc( const uint8_t initial_value, const uint8_t* buffer, uint16_t length )
{
    uint8_t crc = initial_value;

    for( uint16_t i = 0; i < length; i++ )
    {
        uint8_t extract = buffer[i];

        for( uint8_t j = 8; j > 0; j-- )
        {
            const uint8_t sum = ( crc ^ extract ) & 0x01;
            crc >>= 1;

            if( sum != 0 )
            {
                crc ^= 0x65;
            }
            extract >>= 1;
        }
    }

    return crc;
}

#ifdef __cplusplus
}
#endif

#endif