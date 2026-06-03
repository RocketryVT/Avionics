#include "lsm6dso32/LSM6DSO32.hpp"

#include <cstring>

namespace lsm6dso32 {

namespace reg {
    static constexpr uint8_t WHO_AM_I       = 0x0F;
    static constexpr uint8_t CTRL1_XL       = 0x10;
    static constexpr uint8_t CTRL2_G        = 0x11;
    static constexpr uint8_t CTRL3_C        = 0x12;
    static constexpr uint8_t CTRL7_G        = 0x16;
    static constexpr uint8_t CTRL9_XL       = 0x18;
    static constexpr uint8_t STATUS_REG     = 0x1E;
    static constexpr uint8_t OUT_TEMP_L     = 0x20;
    static constexpr uint8_t OUTX_L_G       = 0x22;
    static constexpr uint8_t OUTX_L_A       = 0x28;
    static constexpr uint8_t FIFO_STATUS1   = 0x3A;
    static constexpr uint8_t FIFO_STATUS2   = 0x3B;
    static constexpr uint8_t FIFO_DATA_OUT  = 0x78;
}

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
// Sensitivity tables
// ---------------------------------------------------------------------------

float Device::accel_sensitivity() const noexcept
{
    // mg/LSB ÷ 1000 → g/LSB
    // LSM6DSO32 range encoding: g4=0x00, g32=0x04, g8=0x08, g16=0x0C
    switch (m_cfg.accel_range) {
        case AccelRange::g4:  return 0.000122f;  ///< 0.122 mg/LSB
        case AccelRange::g32: return 0.000976f;  ///< 0.976 mg/LSB
        case AccelRange::g8:  return 0.000244f;  ///< 0.244 mg/LSB
        case AccelRange::g16: return 0.000488f;  ///< 0.488 mg/LSB
        default:              return 0.000122f;
    }
}

float Device::gyro_sensitivity() const noexcept
{
    // mdps/LSB ÷ 1000 → dps/LSB
    switch (m_cfg.gyro_range) {
        case GyroRange::dps_125:  return 0.004375f;
        case GyroRange::dps_250:  return 0.008750f;
        case GyroRange::dps_500:  return 0.017500f;
        case GyroRange::dps_1000: return 0.035000f;
        case GyroRange::dps_2000: return 0.070000f;
        default:                  return 0.008750f;
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool Device::initialize(Transport t, Config cfg)
{
    m_transport = t;
    m_cfg       = cfg;

    // Software reset
    if (!write_reg(reg::CTRL3_C, 0x01)) return false;
    if (m_cfg.delay_ms) m_cfg.delay_ms(10);

    // Verify WHO_AM_I
    uint8_t id;
    if (!read_who_am_i(id) || id != kWhoAmIExpected) return false;

    // BDU (block data update, bit6) + IF_INC (auto-increment, bit2)
    if (!write_reg(reg::CTRL3_C, 0x44)) return false;

    // CTRL9_XL: I3C_DISABLE (bit1) — recommended for SPI/I2C-only designs
    const uint8_t ctrl9 = m_cfg.disable_i3c ? 0x02u : 0x00u;
    if (!write_reg(reg::CTRL9_XL, ctrl9)) return false;

    // Accelerometer: ODR[7:4] | FS[3:2]
    if (!write_reg(reg::CTRL1_XL,
            static_cast<uint8_t>(m_cfg.accel_odr) | static_cast<uint8_t>(m_cfg.accel_range)))
        return false;

    // Gyroscope: ODR[7:4] | FS[3:0]
    if (!write_reg(reg::CTRL2_G,
            static_cast<uint8_t>(m_cfg.gyro_odr) | static_cast<uint8_t>(m_cfg.gyro_range)))
        return false;

    // CTRL7_G: HP filter off
    if (!write_reg(reg::CTRL7_G, 0x00)) return false;

    return true;
}

bool Device::reset(void (*delay_ms)(uint32_t))
{
    if (!write_reg(reg::CTRL3_C, 0x01)) return false;
    if (delay_ms) delay_ms(10);
    return true;
}

bool Device::read_who_am_i(uint8_t& value) const
{
    return read_regs(reg::WHO_AM_I, &value, 1);
}

DataReady Device::data_ready() const
{
    uint8_t sr = 0;
    read_regs(reg::STATUS_REG, &sr, 1);
    return DataReady{
        .accel = (sr & 0x01) != 0,
        .gyro  = (sr & 0x02) != 0,
        .temp  = (sr & 0x04) != 0,
    };
}

bool Device::read_sample(Sample& s) const
{
    // Burst-read 14 bytes from OUT_TEMP_L:
    //   [0:1]  = TEMP_L / TEMP_H
    //   [2:7]  = OUTX_L_G .. OUTZ_H_G
    //   [8:13] = OUTX_L_A .. OUTZ_H_A
    uint8_t buf[14];
    if (!read_regs(reg::OUT_TEMP_L, buf, 14)) return false;

    auto to16 = [](uint8_t lo, uint8_t hi) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    const int16_t raw_temp = to16(buf[0], buf[1]);
    s.temp_c = static_cast<float>(raw_temp) / 256.0f + 25.0f;

    const float gs = gyro_sensitivity();
    s.gyro_x = static_cast<float>(to16(buf[2],  buf[3]))  * gs;
    s.gyro_y = static_cast<float>(to16(buf[4],  buf[5]))  * gs;
    s.gyro_z = static_cast<float>(to16(buf[6],  buf[7]))  * gs;

    const float as = accel_sensitivity();
    s.accel_x = static_cast<float>(to16(buf[8],  buf[9]))  * as;
    s.accel_y = static_cast<float>(to16(buf[10], buf[11])) * as;
    s.accel_z = static_cast<float>(to16(buf[12], buf[13])) * as;

    return true;
}

bool Device::read_temperature(float& temp_c) const
{
    uint8_t buf[2];
    if (!read_regs(reg::OUT_TEMP_L, buf, 2)) return false;
    const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(buf[1]) << 8) | buf[0]);
    temp_c = static_cast<float>(raw) / 256.0f + 25.0f;
    return true;
}

bool Device::read_gyroscope(float& x, float& y, float& z) const
{
    uint8_t buf[6];
    if (!read_regs(reg::OUTX_L_G, buf, 6)) return false;
    const float gs = gyro_sensitivity();
    auto to16 = [](uint8_t lo, uint8_t hi) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };
    x = static_cast<float>(to16(buf[0], buf[1])) * gs;
    y = static_cast<float>(to16(buf[2], buf[3])) * gs;
    z = static_cast<float>(to16(buf[4], buf[5])) * gs;
    return true;
}

bool Device::read_accelerometer(float& x, float& y, float& z) const
{
    uint8_t buf[6];
    if (!read_regs(reg::OUTX_L_A, buf, 6)) return false;
    const float as = accel_sensitivity();
    auto to16 = [](uint8_t lo, uint8_t hi) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };
    x = static_cast<float>(to16(buf[0], buf[1])) * as;
    y = static_cast<float>(to16(buf[2], buf[3])) * as;
    z = static_cast<float>(to16(buf[4], buf[5])) * as;
    return true;
}

uint16_t Device::fifo_count() const
{
    uint8_t buf[2];
    if (!read_regs(reg::FIFO_STATUS1, buf, 2)) return 0;
    return static_cast<uint16_t>(((buf[1] & 0x03u) << 8u) | buf[0]);
}

bool Device::fifo_pop(FifoEntry& entry) const
{
    // Each FIFO word is 7 bytes: 1 tag byte + 6 data bytes
    uint8_t buf[7];
    if (!read_regs(reg::FIFO_DATA_OUT, buf, 7)) return false;

    const uint8_t raw_tag = buf[0] >> 3u;
    switch (raw_tag) {
        case 0x01: entry.tag = FifoTag::gyro_nc;     break;
        case 0x02: entry.tag = FifoTag::accel_nc;    break;
        case 0x03: entry.tag = FifoTag::temperature; break;
        case 0x04: entry.tag = FifoTag::timestamp;   break;
        default:   entry.tag = FifoTag::other;        break;
    }
    memcpy(entry.data, buf + 1, 6);
    return true;
}

} // namespace lsm6dso32
