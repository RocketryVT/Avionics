#pragma once

#include <cstdint>

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#if (USE_FREERTOS == 1)
#include "FreeRTOS.h"
#include "task.h"
#endif

// ---------------------------------------------------------------------------
// HAL transport — injected at initialize(); no I2C task headers required
// ---------------------------------------------------------------------------

namespace ms5607 {

/// Send a single command byte to the device.
using CmdFn     = bool (*)(void* ctx, uint8_t cmd);

/// Send a command byte then receive `len` bytes.
using CmdReadFn = bool (*)(void* ctx, uint8_t cmd, uint8_t* buf, size_t len);

struct Transport {
    void*       ctx;
    CmdFn       cmd;
    CmdReadFn   cmd_read;
};

// ---------------------------------------------------------------------------
// Command encoding — bitfield union (same layout as original)
// ---------------------------------------------------------------------------

typedef union {
    struct {
        uint8_t RESERVED: 1;
        uint8_t ADDR_OSR: 3;
        uint8_t TYPE    : 1;
        bool    PROM2   : 1;
        bool    CONVERT : 1;
        bool    PROM    : 1;
    } fields;
    uint8_t data;
} ms5607_cmd;

// ---------------------------------------------------------------------------
// OSR options
// ---------------------------------------------------------------------------

static constexpr uint8_t OSR_CONVERT_256  = 0x0;
static constexpr uint8_t OSR_CONVERT_512  = 0x1;
static constexpr uint8_t OSR_CONVERT_1024 = 0x2;
static constexpr uint8_t OSR_CONVERT_2048 = 0x3;
static constexpr uint8_t OSR_CONVERT_4096 = 0x4;
static constexpr uint32_t OSR_256_CONVERSION_TIME_US = 600;

// ---------------------------------------------------------------------------
// Sample state machine
// ---------------------------------------------------------------------------

typedef enum {
    NOT_SAMPLING,
    PRESSURE_CONVERT,
    TEMPERATURE_CONVERT,
    COMPENSATE
} sample_state_t;

// ---------------------------------------------------------------------------
// MS5607 barometer driver
// ---------------------------------------------------------------------------

class MS5607 {
public:
    static constexpr uint8_t kDefaultAddress = 0x77;  ///< CSB pin -> GND

    MS5607() = default;

    /// Initialise: send reset, read PROM calibration coefficients.
    void initialize(Transport t);

    // State machine (driven by alarm callback + optional FreeRTOS tasks)
    void ms5607_write_cmd(ms5607_cmd* cmd);
    void ms5607_start_sample();
    void ms5607_sample();
    static int64_t ms5607_sample_callback(alarm_id_t id, void* user_data);

#if (USE_FREERTOS == 1)
    static void ms5607_sample_handler(void* pvParameters);
    static void update_ms5607_task(void* pvParameters);
    TaskHandle_t sample_handler_task_handle = NULL;
    TaskHandle_t update_task_handle         = NULL;
#endif

    int32_t pressure_to_altitude(int32_t pressure);

    int32_t get_pressure()    { return pressure; }
    int32_t get_temperature() { return temperature; }
    int32_t get_altitude()    { return altitude; }

    void set_threshold_altitude(int32_t threshold_altitude, alarm_callback_t callback);
    void clear_threshold_altitude();

private:
    void ms5607_compensate();

    bool transport_cmd(uint8_t c);
    bool transport_cmd_read(uint8_t c, uint8_t* buf, uint8_t len);

    Transport    m_transport{};
    const uint8_t addr = kDefaultAddress;

    uint8_t  buffer[3]{};
    uint16_t prom[6]{};             // C1..C6, 0-indexed (prom[0]=C1 .. prom[5]=C6)

    uint32_t uncompensated_pressure    = 0;
    uint32_t uncompensated_temperature = 0;
    int32_t  pressure                  = 0;
    int32_t  temperature               = 0;
    int32_t  altitude                  = 0;

    sample_state_t   sample_state     = NOT_SAMPLING;
    int32_t          threshold_altitude = 0;
    alarm_callback_t threshold_callback = nullptr;
    bool             positive_crossing  = false;
};

} // namespace ms5607
