// lr11xx_hal.c — RP2350 SPI HAL for the Semtech LR11XX driver.
//
// All board-specific configuration (SPI bus, GPIO pin numbers, clock speed) is
// read from the lr11xx_hal_context_t that the caller passes through the opaque
// `context` pointer.  No pin numbers or SPI instances are hardcoded here.
//
// FreeRTOS-aware:
//   - wait_busy() yields with vTaskDelay(1) so other tasks run during long
//     operations (TX air time at high spreading factors can be several seconds).
//   - A per-context SPI mutex serialises all SPI transactions.
//   - Every busy-wait has a hard 30-second deadline; LR11XX_HAL_STATUS_ERROR
//     is returned on timeout to avoid hanging the caller indefinitely.
//
// Sub-millisecond delays (wakeup NSS pulse, abort pulse) retain sleep_us()
// because hardware-timer precision is required and vTaskDelay cannot give < 1 ms.

#include "lr11xx_hal.h"
#include "lr11xx_hal_context.h"

// lr11xx_hal_context.h already pulls in FreeRTOS.h and semphr.h (required
// for SemaphoreHandle_t / StaticSemaphore_t in the context struct).
// task.h is the only additional header needed here.
#include "task.h"

#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// --- Busy-wait timeout --------------------------------------------------------
// 30 s covers even the longest LoRa frame (SF12, BW 125 kHz, 255-byte payload).
#define BUSY_TIMEOUT_MS  30000u

// --- Internal helpers ---------------------------------------------------------

static inline void nss_low ( const lr11xx_hal_context_t* ctx ) { gpio_put( ctx->pin_nss, 0 ); }
static inline void nss_high( const lr11xx_hal_context_t* ctx ) { gpio_put( ctx->pin_nss, 1 ); }

// wait_busy — yield to FreeRTOS while BUSY is asserted.
//
// Called while holding the context mutex so that no concurrent task can begin
// a new SPI transaction while the radio is executing a command or transmitting.
//
// Returns LR11XX_HAL_STATUS_ERROR if the 30-second deadline expires.
static lr11xx_hal_status_t wait_busy( const lr11xx_hal_context_t* ctx )
{
    absolute_time_t deadline = make_timeout_time_ms( BUSY_TIMEOUT_MS );

    while ( gpio_get( ctx->pin_busy ) )
    {
        if ( time_reached( deadline ) )
        {
            return LR11XX_HAL_STATUS_ERROR;
        }
        vTaskDelay( pdMS_TO_TICKS( 1 ) );
    }

    return LR11XX_HAL_STATUS_OK;
}

// --- Init ---------------------------------------------------------------------

void lr11xx_hal_init( lr11xx_hal_context_t* ctx )
{
    configASSERT( ctx != NULL );
    configASSERT( ctx->spi != NULL );

    // Create the per-context SPI mutex.
    ctx->_mutex = xSemaphoreCreateMutex();
    configASSERT( ctx->_mutex );

    // SPI peripheral
    spi_init( ctx->spi, ctx->spi_hz );
    spi_set_format( ctx->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST );

    gpio_set_function( ctx->pin_sck,  GPIO_FUNC_SPI );
    gpio_set_function( ctx->pin_mosi, GPIO_FUNC_SPI );
    gpio_set_function( ctx->pin_miso, GPIO_FUNC_SPI );

    // Chip-select (NSS) — idle high
    gpio_init( ctx->pin_nss );
    gpio_set_dir( ctx->pin_nss, GPIO_OUT );
    gpio_put( ctx->pin_nss, 1 );

    // BUSY — input with pull-down.
    // LR11XX drives BUSY HIGH when processing, LOW when ready.
    // RP2350 gpio_init() enables a pull-up by default; override to pull-down
    // so a floating/unconnected BUSY line reads 0 (not-busy) rather than 1.
    gpio_init( ctx->pin_busy );
    gpio_set_dir( ctx->pin_busy, GPIO_IN );
    gpio_pull_down( ctx->pin_busy );

    // NRESET — idle high
    gpio_init( ctx->pin_nreset );
    gpio_set_dir( ctx->pin_nreset, GPIO_OUT );
    gpio_put( ctx->pin_nreset, 1 );
}

// --- HAL Write ----------------------------------------------------------------

lr11xx_hal_status_t lr11xx_hal_write( const void*    context,
                                       const uint8_t* command,
                                       const uint16_t command_length,
                                       const uint8_t* data,
                                       const uint16_t data_length )
{
    lr11xx_hal_context_t* ctx = ( lr11xx_hal_context_t* ) context;

    xSemaphoreTake( ctx->_mutex, portMAX_DELAY );

    lr11xx_hal_status_t status = wait_busy( ctx );
    if ( status != LR11XX_HAL_STATUS_OK )
    {
        xSemaphoreGive( ctx->_mutex );
        return status;
    }

    nss_low( ctx );
    spi_write_blocking( ctx->spi, command, command_length );
    if ( data != NULL && data_length > 0 )
    {
        spi_write_blocking( ctx->spi, data, data_length );
    }
    nss_high( ctx );

    xSemaphoreGive( ctx->_mutex );
    return LR11XX_HAL_STATUS_OK;
}

// --- HAL Read -----------------------------------------------------------------
// Two-step operation (per hal.h contract):
//   Step 1: NSS↓ -> send command -> NSS↑
//   Step 2: NSS↓ -> discard dummy byte -> read data -> NSS↑
// MOSI must be 0x00 (NOP) throughout the read phase.

lr11xx_hal_status_t lr11xx_hal_read( const void*    context,
                                      const uint8_t* command,
                                      const uint16_t command_length,
                                      uint8_t*       data,
                                      const uint16_t data_length )
{
    lr11xx_hal_context_t* ctx = ( lr11xx_hal_context_t* ) context;

    xSemaphoreTake( ctx->_mutex, portMAX_DELAY );

    // Step 1: send command
    lr11xx_hal_status_t status = wait_busy( ctx );
    if ( status != LR11XX_HAL_STATUS_OK )
    {
        xSemaphoreGive( ctx->_mutex );
        return status;
    }

    nss_low( ctx );
    spi_write_blocking( ctx->spi, command, command_length );
    nss_high( ctx );

    // Step 2: read response
    status = wait_busy( ctx );
    if ( status != LR11XX_HAL_STATUS_OK )
    {
        xSemaphoreGive( ctx->_mutex );
        return status;
    }

    nss_low( ctx );

    uint8_t dummy;
    uint8_t nop = LR11XX_NOP;
    spi_write_read_blocking( ctx->spi, &nop, &dummy, 1 );           // discard dummy byte
    spi_read_blocking( ctx->spi, LR11XX_NOP, data, data_length );   // NOP on MOSI

    nss_high( ctx );

    xSemaphoreGive( ctx->_mutex );
    return LR11XX_HAL_STATUS_OK;
}

// --- HAL Direct Read ----------------------------------------------------------
// Single-step NSS/read/NSS — no command phase.
// Used only by lr11xx_system_get_status and lr11xx_bootloader_get_status.

lr11xx_hal_status_t lr11xx_hal_direct_read( const void*    context,
                                             uint8_t*       data,
                                             const uint16_t data_length )
{
    lr11xx_hal_context_t* ctx = ( lr11xx_hal_context_t* ) context;

    xSemaphoreTake( ctx->_mutex, portMAX_DELAY );

    lr11xx_hal_status_t status = wait_busy( ctx );
    if ( status != LR11XX_HAL_STATUS_OK )
    {
        xSemaphoreGive( ctx->_mutex );
        return status;
    }

    nss_low( ctx );
    spi_read_blocking( ctx->spi, LR11XX_NOP, data, data_length );
    nss_high( ctx );

    xSemaphoreGive( ctx->_mutex );
    return LR11XX_HAL_STATUS_OK;
}

// --- HAL Reset ----------------------------------------------------------------

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    lr11xx_hal_context_t* ctx = ( lr11xx_hal_context_t* ) context;

    // Hold the mutex so no SPI transaction races with the reset pulse.
    xSemaphoreTake( ctx->_mutex, portMAX_DELAY );

    gpio_put( ctx->pin_nreset, 0 );
    vTaskDelay( pdMS_TO_TICKS( 5 ) );
    gpio_put( ctx->pin_nreset, 1 );
    vTaskDelay( pdMS_TO_TICKS( 10 ) );

    lr11xx_hal_status_t status = wait_busy( ctx );

    xSemaphoreGive( ctx->_mutex );
    return status;
}

// --- HAL Wakeup ---------------------------------------------------------------
// Per datasheet §4.3: NSS↓ -> hold 100 µs -> NSS↑ -> wait BUSY↓.
// The 100 µs pulse must be precise, so sleep_us() is kept here.

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    lr11xx_hal_context_t* ctx = ( lr11xx_hal_context_t* ) context;

    xSemaphoreTake( ctx->_mutex, portMAX_DELAY );

    nss_low( ctx );
    sleep_us( 100 );     // hardware-timer precision required
    nss_high( ctx );

    lr11xx_hal_status_t status = wait_busy( ctx );

    xSemaphoreGive( ctx->_mutex );
    return status;
}

// --- HAL Abort Blocking Command -----------------------------------------------
// Interrupts any ongoing blocking command by toggling NSS.

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
    lr11xx_hal_context_t* ctx = ( lr11xx_hal_context_t* ) context;

    xSemaphoreTake( ctx->_mutex, portMAX_DELAY );

    nss_low( ctx );
    sleep_us( 1 );       // must be a tight pulse
    nss_high( ctx );

    xSemaphoreGive( ctx->_mutex );
    return LR11XX_HAL_STATUS_OK;
}

// --- HAL Wait Busy (public) ---------------------------------------------------
// Call after lr11xx_radio_set_tx() / lr11xx_radio_set_rx() to block the
// calling task until the radio finishes (BUSY goes low) without spinning the
// CPU.  Does NOT hold the SPI mutex — the mutex is only needed around SPI
// transactions, not a bare GPIO read.

lr11xx_hal_status_t lr11xx_hal_wait_busy( const void* context )
{
    const lr11xx_hal_context_t* ctx = ( const lr11xx_hal_context_t* ) context;

    absolute_time_t deadline = make_timeout_time_ms( BUSY_TIMEOUT_MS );

    while ( gpio_get( ctx->pin_busy ) )
    {
        if ( time_reached( deadline ) )
        {
            return LR11XX_HAL_STATUS_ERROR;
        }
        vTaskDelay( pdMS_TO_TICKS( 1 ) );
    }

    return LR11XX_HAL_STATUS_OK;
}
