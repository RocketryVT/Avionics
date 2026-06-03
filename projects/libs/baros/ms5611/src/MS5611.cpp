#include "ms5611/MS5611.hpp"

#include <cmath>
#include <cstring>

namespace ms5611 {

namespace reg {
    static constexpr uint8_t CMD_RESET      = 0x1E;
    static constexpr uint8_t CMD_CONVERT_D1 = 0x48;  // pressure,    OSR=4096 (~9.04 ms)
    static constexpr uint8_t CMD_CONVERT_D2 = 0x58;  // temperature, OSR=4096 (~9.04 ms)
    static constexpr uint8_t CMD_ADC_READ   = 0x00;
    // PROM: 8 words at 0xA0, 0xA2, … 0xAE (address = 0xA0 + i*2)
    static constexpr uint8_t CMD_PROM_BASE  = 0xA0;
}

// ---------------------------------------------------------------------------

bool Device::cmd(uint8_t c) const
{
    return m_transport.cmd(m_transport.ctx, c);
}

bool Device::cmd_read(uint8_t c, uint8_t* buf, size_t len) const
{
    return m_transport.cmd_read(m_transport.ctx, c, buf, len);
}

// ---------------------------------------------------------------------------

bool Device::initialize(Transport t, Config cfg)
{
    m_transport = t;
    m_cfg       = cfg;

    if (!cmd(reg::CMD_RESET))
        return false;

    if (m_cfg.delay_ms)
        m_cfg.delay_ms(5);

    for (int i = 0; i < 8; ++i) {
        uint8_t buf[2];
        if (!cmd_read(static_cast<uint8_t>(reg::CMD_PROM_BASE + i * 2), buf, 2))
            return false;
        m_prom[i] = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    }

    // Prime the pipeline with a temperature conversion
    return start_conversion(false);
}

bool Device::start_conversion(bool pressure) const
{
    return cmd(pressure ? reg::CMD_CONVERT_D1 : reg::CMD_CONVERT_D2);
}

bool Device::read_adc(uint32_t& out) const
{
    uint8_t buf[3];
    if (!cmd_read(reg::CMD_ADC_READ, buf, 3))
        return false;
    out = (static_cast<uint32_t>(buf[0]) << 16)
        | (static_cast<uint32_t>(buf[1]) <<  8)
        |  static_cast<uint32_t>(buf[2]);
    return true;
}

// Full 2nd-order temperature compensation per MS5611 datasheet AN520.
void Device::calculate(uint32_t D1, uint32_t D2,
                       float& pressure_pa, float& temp_c) const
{
    const int32_t C1 = m_prom[1];
    const int32_t C2 = m_prom[2];
    const int32_t C3 = m_prom[3];
    const int32_t C4 = m_prom[4];
    const int32_t C5 = m_prom[5];
    const int32_t C6 = m_prom[6];

    int32_t dT   = static_cast<int32_t>(D2) - C5 * 256;
    int32_t TEMP = 2000 + static_cast<int32_t>(static_cast<int64_t>(dT) * C6 / 8388608L);

    int64_t OFF  = static_cast<int64_t>(C2) * 65536L  + static_cast<int64_t>(C4) * dT / 128L;
    int64_t SENS = static_cast<int64_t>(C1) * 32768L  + static_cast<int64_t>(C3) * dT / 256L;

    int32_t T2   = 0;
    int64_t OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        int32_t dt2 = TEMP - 2000;
        T2    = static_cast<int32_t>(static_cast<int64_t>(dT) * dT / 2147483648LL);
        OFF2  = 5LL * dt2 * dt2 / 2;
        SENS2 = 5LL * dt2 * dt2 / 4;
        if (TEMP < -1500) {
            int32_t dt3 = TEMP + 1500;
            OFF2  += 7LL  * dt3 * dt3;
            SENS2 += 11LL * dt3 * dt3 / 2;
        }
    }
    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    int32_t P = static_cast<int32_t>((static_cast<int64_t>(D1) * SENS / 2097152L - OFF) / 32768L);

    temp_c      = TEMP / 100.0f;
    pressure_pa = P    / 100.0f;
}

float Device::altitude(float pressure_pa) const
{
    // Standard ISA barometric formula
    return 44330.0f * (1.0f - powf(pressure_pa / m_cfg.qnh_pa, 0.1902949f));
}

} // namespace ms5611
