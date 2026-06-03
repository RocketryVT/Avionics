#include "adxl375/ADXL375.hpp"

#include <cstring>

namespace adxl375 {

namespace reg {
    static constexpr uint8_t DEVID       = 0x00;
    static constexpr uint8_t THRESH_TAP  = 0x1D;
    static constexpr uint8_t OFSX        = 0x1E;
    static constexpr uint8_t OFSY        = 0x1F;
    static constexpr uint8_t OFSZ        = 0x20;
    static constexpr uint8_t BW_RATE     = 0x2C;
    static constexpr uint8_t POWER_CTL  = 0x2D;
    static constexpr uint8_t DATA_FORMAT = 0x31;
    static constexpr uint8_t DATAX0      = 0x32;  // burst: X0 X1 Y0 Y1 Z0 Z1
}

// Scale: 49 mg/LSB × 9.80665 m/s² / 1000
static constexpr float kScaleMs2 = 49.0f * 9.80665f / 1000.0f;

// ---------------------------------------------------------------------------
// Transport helpers
// ---------------------------------------------------------------------------

bool Device::write_reg(uint8_t r, uint8_t val) const
{
    return m_transport.write_reg(m_transport.ctx, m_cfg.address, r, val);
}

bool Device::read_regs(uint8_t r, uint8_t* buf, size_t len) const
{
    return m_transport.read_regs(m_transport.ctx, m_cfg.address, r, buf, len);
}

bool Device::rmw_reg(uint8_t r, uint8_t mask, uint8_t bits) const
{
    uint8_t val;
    if (!read_regs(r, &val, 1)) return false;
    val = static_cast<uint8_t>((val & ~mask) | (bits & mask));
    return write_reg(r, val);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool Device::initialize(Transport t, Config cfg)
{
    m_transport = t;
    m_cfg       = cfg;

    uint8_t id;
    if (!read_device_id(id) || id != kDeviceId)
        return false;

    // DATA_FORMAT: FULL_RES=1, ±200g fixed range → 0x0B
    if (!write_reg(reg::DATA_FORMAT, 0x0B))
        return false;

    if (m_cfg.delay_ms)
        m_cfg.delay_ms(10);

    if (!set_bandwidth(m_cfg.bandwidth))
        return false;

    if (!set_power_mode(m_cfg.power_mode))
        return false;

    return true;
}

bool Device::read_device_id(uint8_t& id) const
{
    return read_regs(reg::DEVID, &id, 1);
}

bool Device::read_sample(Sample& s) const
{
    uint8_t buf[6];
    if (!read_regs(reg::DATAX0, buf, 6))
        return false;

    auto to_int16 = [](uint8_t lo, uint8_t hi) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    s.x = static_cast<float>(to_int16(buf[0], buf[1])) * kScaleMs2;
    s.y = static_cast<float>(to_int16(buf[2], buf[3])) * kScaleMs2;
    s.z = static_cast<float>(to_int16(buf[4], buf[5])) * kScaleMs2;
    return true;
}

bool Device::set_bandwidth(BandWidth bw)
{
    m_cfg.bandwidth = bw;
    // BW_RATE bits [3:0] = rate code; preserve LOW_POWER bit [4]
    return rmw_reg(reg::BW_RATE, 0x0F, static_cast<uint8_t>(bw));
}

bool Device::set_power_mode(PowerMode mode)
{
    m_cfg.power_mode = mode;
    return write_reg(reg::POWER_CTL, static_cast<uint8_t>(mode));
}

} // namespace adxl375
