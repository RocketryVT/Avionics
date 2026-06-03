#include "ms5607/MS5607.hpp"

#define PROM_CALIBRATION_COEFFICENT_1_ADDR 0x1
#define PROM_CALIBRATION_COEFFICENT_6_ADDR 0x6
#define RESET_COMMAND    0x1E
#define ADC_READ_COMMAND 0x0
#define ALT_SHIFT 8
#define ALT_SCALE (1 << ALT_SHIFT)
#define ALT_MASK  (ALT_SCALE - 1)
#define MS5607_SAMPLE_RATE_HZ 100
#define TYPE_UNCOMPENSATED_PRESSURE    0
#define TYPE_UNCOMPENSATED_TEMPERATURE 1

static const int32_t altitude_table[] = {
#include "ms5607/altitude-pa.h"
};

namespace ms5607 {

// ---------------------------------------------------------------------------
// Transport helpers
// ---------------------------------------------------------------------------

bool MS5607::transport_cmd(uint8_t c)
{
    return m_transport.cmd ? m_transport.cmd(m_transport.ctx, c) : false;
}

bool MS5607::transport_cmd_read(uint8_t c, uint8_t* buf, uint8_t len)
{
    return m_transport.cmd_read ? m_transport.cmd_read(m_transport.ctx, c, buf, len) : false;
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void MS5607::initialize(Transport t)
{
    m_transport  = t;
    sample_state = NOT_SAMPLING;

    alarm_pool_init_default();

    ms5607_cmd cmd;
    cmd.data = RESET_COMMAND;
    ms5607_write_cmd(&cmd);

    sleep_ms(500);

    cmd.data = 0;
    cmd.fields.PROM  = true;
    cmd.fields.PROM2 = true;

    for (uint8_t prom_addr = PROM_CALIBRATION_COEFFICENT_1_ADDR;
         prom_addr <= PROM_CALIBRATION_COEFFICENT_6_ADDR;
         prom_addr++)
    {
        sleep_ms(100);
        cmd.fields.ADDR_OSR = prom_addr;

        uint8_t buf[2];
        transport_cmd_read(cmd.data, buf, 2);

        prom[prom_addr - 1] = static_cast<uint16_t>(
            (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
    }
}

void MS5607::ms5607_write_cmd(ms5607_cmd* cmd)
{
    transport_cmd(cmd->data);
}

void MS5607::ms5607_start_sample()
{
    if (sample_state == NOT_SAMPLING) {
        sample_state = PRESSURE_CONVERT;
        ms5607_sample();
    }
}

#if (USE_FREERTOS == 1)
void MS5607::ms5607_sample_handler(void* pvParameters)
{
    const TickType_t xInterruptFrequency  = pdMS_TO_TICKS(1000 / (MS5607_SAMPLE_RATE_HZ * 2));
    const TickType_t xMaxExpectedBlockTime = xInterruptFrequency + pdMS_TO_TICKS(1);
    uint32_t ulEventsToProcess;
    while (1) {
        ulEventsToProcess = ulTaskNotifyTake(pdTRUE, xMaxExpectedBlockTime);
        if (ulEventsToProcess != 0) {
            while (ulEventsToProcess > 0) {
                static_cast<MS5607*>(pvParameters)->ms5607_sample();
                ulEventsToProcess--;
            }
        }
    }
}

void MS5607::update_ms5607_task(void* pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / MS5607_SAMPLE_RATE_HZ);
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        static_cast<MS5607*>(pvParameters)->ms5607_start_sample();
    }
}
#endif

int64_t MS5607::ms5607_sample_callback(alarm_id_t id, void* user_data)
{
    (void)id;
#if (USE_FREERTOS == 1)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    auto* alt = static_cast<MS5607*>(user_data);
    vTaskNotifyGiveFromISR(alt->sample_handler_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
    static_cast<MS5607*>(user_data)->ms5607_sample();
#endif
    return 0;
}

void MS5607::ms5607_sample()
{
    ms5607_cmd cmd = {.data = 0};

    switch (sample_state) {
        case NOT_SAMPLING:
            break;

        case PRESSURE_CONVERT: {
            cmd.fields.CONVERT  = 1;
            cmd.fields.TYPE     = TYPE_UNCOMPENSATED_PRESSURE;
            cmd.fields.ADDR_OSR = OSR_CONVERT_256;
            transport_cmd(cmd.data);
            add_alarm_in_us(OSR_256_CONVERSION_TIME_US, MS5607::ms5607_sample_callback,
                            static_cast<void*>(this), true);
            sample_state = TEMPERATURE_CONVERT;
            break;
        }

        case TEMPERATURE_CONVERT: {
            transport_cmd_read(ADC_READ_COMMAND, buffer, 3);
            uncompensated_pressure = (static_cast<uint32_t>(buffer[0]) << 16)
                                   | (static_cast<uint32_t>(buffer[1]) <<  8)
                                   |  static_cast<uint32_t>(buffer[2]);

            cmd.fields.CONVERT  = 1;
            cmd.fields.TYPE     = TYPE_UNCOMPENSATED_TEMPERATURE;
            cmd.fields.ADDR_OSR = OSR_CONVERT_256;
            transport_cmd(cmd.data);
            add_alarm_in_us(OSR_256_CONVERSION_TIME_US, MS5607::ms5607_sample_callback,
                            static_cast<void*>(this), true);
            sample_state = COMPENSATE;
            break;
        }

        case COMPENSATE: {
            transport_cmd_read(ADC_READ_COMMAND, buffer, 3);
            uncompensated_temperature = (static_cast<uint32_t>(buffer[0]) << 16)
                                      | (static_cast<uint32_t>(buffer[1]) <<  8)
                                      |  static_cast<uint32_t>(buffer[2]);
            ms5607_compensate();
            altitude = pressure_to_altitude(pressure);

            sample_state = NOT_SAMPLING;
            if (threshold_callback != nullptr) {
                bool fire = positive_crossing ? (altitude >= threshold_altitude)
                                              : (altitude <= threshold_altitude);
                if (fire)
                    add_alarm_in_ms(1, threshold_callback, nullptr, true);
            }
            break;
        }
    }
}

void MS5607::ms5607_compensate()
{
    int32_t dT = static_cast<int32_t>(uncompensated_temperature)
                 - (static_cast<int32_t>(prom[4]) << 8);
    temperature = 2000 + static_cast<int32_t>(
        (static_cast<int64_t>(dT) * static_cast<int64_t>(prom[5])) >> 23);
    int64_t OFF  = (static_cast<int64_t>(prom[1]) << 17)
                 + ((static_cast<int64_t>(prom[3]) * static_cast<int64_t>(dT)) >> 6);
    int64_t SENS = (static_cast<int64_t>(prom[0]) << 16)
                 + ((static_cast<int64_t>(prom[2]) * static_cast<int64_t>(dT)) >> 7);
    pressure = static_cast<int32_t>(
        ((static_cast<int64_t>(uncompensated_pressure) * SENS >> 21) - OFF) >> 15);
}

int32_t MS5607::pressure_to_altitude(int32_t p)
{
    if (p < 0)      p = 0;
    if (p > 120000) p = 120000;

    uint16_t o    = static_cast<uint16_t>(p >> ALT_SHIFT);
    int16_t  part = static_cast<int16_t>(p & ALT_MASK);

    int32_t low  = static_cast<int32_t>(altitude_table[o])   * (ALT_SCALE - part);
    int32_t high = static_cast<int32_t>(altitude_table[o + 1]) * part;
    return (low + high + (ALT_SCALE >> 1)) >> ALT_SHIFT;
}

void MS5607::set_threshold_altitude(int32_t threshold, alarm_callback_t callback)
{
    threshold_altitude  = threshold;
    positive_crossing   = (threshold > altitude);
    threshold_callback  = callback;
}

void MS5607::clear_threshold_altitude()
{
    threshold_callback = nullptr;
}

} // namespace ms5607
