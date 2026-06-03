#include "ism330dlc/ISM330DLC.hpp"

namespace ism330dlc {

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------

namespace reg {
    constexpr uint8_t WHO_AM_I   = 0x0F;
    constexpr uint8_t CTRL1_XL   = 0x10;  ///< Accel ODR[7:4], FS[3:2], BW_SEL[1], BW0[0]
    constexpr uint8_t CTRL2_G    = 0x11;  ///< Gyro  ODR[7:4], FS_G[3:2], FS_125[1]
    constexpr uint8_t CTRL3_C    = 0x12;  ///< BDU[6], IF_INC[2], SW_RESET[0]
    constexpr uint8_t CTRL5_C    = 0x14;  ///< ST_G[5:4], ST_XL[1:0]
    constexpr uint8_t STATUS_REG = 0x1E;  ///< TDA[2], GDA[1], XLDA[0]
    constexpr uint8_t OUT_TEMP_L = 0x20;  ///< Burst start: TEMP(2) GYRO(6) ACCEL(6) = 14 bytes
} // namespace reg

// CTRL1_XL / CTRL2_G field masks
constexpr uint8_t ODR_MASK      = 0xF0;  ///< Output data rate bits [7:4]
constexpr uint8_t ACCEL_FS_MASK = 0x0C;  ///< Accel FS bits [3:2]
constexpr uint8_t GYRO_FS_MASK  = 0x0E;  ///< Gyro FS_G[3:2] + FS_125[1]

// CTRL3_C bits
constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_IF_INC   = 0x04;
constexpr uint8_t CTRL3_BDU      = 0x40;

// CTRL5_C self-test masks
constexpr uint8_t ST_XL_MASK = 0x03;  ///< Accel ST bits [1:0]: 01 = positive
constexpr uint8_t ST_G_MASK  = 0x30;  ///< Gyro  ST bits [5:4]: 01 = positive

// STATUS_REG data-ready bits
constexpr uint8_t STATUS_XLDA = 0x01;
constexpr uint8_t STATUS_GDA  = 0x02;
constexpr uint8_t STATUS_TDA  = 0x04;

// Temperature decode constants
constexpr float TEMP_OFFSET_C    = 25.0f;
constexpr float TEMP_SENS_C_PER_LSB = 1.0f / 16.0f;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

constexpr int16_t le_i16(const uint8_t* p) noexcept
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8u));
}

/// Datasheet sensitivity values — do NOT derive from 2^15.
constexpr float accel_sens(AccelRange r) noexcept
{
    switch (r) {
        case AccelRange::g2:  return 0.000061f;
        case AccelRange::g16: return 0.000488f;
        case AccelRange::g4:  return 0.000122f;
        case AccelRange::g8:  return 0.000244f;
    }
    return 0.000488f;
}

constexpr float gyro_sens(GyroRange r) noexcept
{
    switch (r) {
        case GyroRange::dps_125:  return 0.004375f;
        case GyroRange::dps_250:  return 0.00875f;
        case GyroRange::dps_500:  return 0.0175f;
        case GyroRange::dps_1000: return 0.035f;
        case GyroRange::dps_2000: return 0.07f;
    }
    return 0.07f;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Device::Device(Transport transport, uint8_t device_address)
    : m_transport(transport)
    , m_device_address(device_address)
{}

// ---------------------------------------------------------------------------
// Private I/O helpers
// ---------------------------------------------------------------------------

bool Device::write_reg(uint8_t reg, uint8_t val) const
{
    return m_transport.write_reg != nullptr &&
           m_transport.write_reg(m_transport.ctx, m_device_address, reg, val);
}

bool Device::read_regs(uint8_t reg, uint8_t* buf, size_t len) const
{
    return m_transport.read_regs != nullptr &&
           m_transport.read_regs(m_transport.ctx, m_device_address, reg, buf, len);
}

/// Read-modify-write: clear `mask` bits, apply `bits` under mask.
bool Device::rmw_reg(uint8_t reg, uint8_t mask, uint8_t bits) const
{
    uint8_t val = 0;
    if (!read_regs(reg, &val, 1)) return false;
    val = static_cast<uint8_t>((val & ~mask) | (bits & mask));
    return write_reg(reg, val);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool Device::initialize(Config const& cfg)
{
    uint8_t id = 0;
    if (!read_who_am_i(id) || id != kWhoAmIExpected) return false;

    if (!reset(cfg.delay_ms)) return false;

    // Enable block data update (no register tearing) + address auto-increment
    if (!write_reg(reg::CTRL3_C, CTRL3_BDU | CTRL3_IF_INC)) return false;

    return set_accel_range(cfg.accel_range)
        && set_gyro_range(cfg.gyro_range)
        && set_accel_odr(cfg.accel_odr)
        && set_gyro_odr(cfg.gyro_odr);
}

bool Device::reset(void (*delay_ms)(uint32_t))
{
    if (!write_reg(reg::CTRL3_C, CTRL3_SW_RESET)) return false;
    if (delay_ms) delay_ms(10);

    // Reset brings all regs to default — sync cache to power-on defaults
    m_accel_range = AccelRange::g2;
    m_gyro_range  = GyroRange::dps_250;
    m_accel_odr   = AccelODR::power_down;
    m_gyro_odr    = GyroODR::power_down;
    return true;
}

// ---------------------------------------------------------------------------
// Runtime configuration
// ---------------------------------------------------------------------------

bool Device::set_accel_odr(AccelODR odr)
{
    if (!rmw_reg(reg::CTRL1_XL, ODR_MASK, static_cast<uint8_t>(odr))) return false;
    if (odr != AccelODR::power_down) m_accel_last_odr = odr;
    m_accel_odr = odr;
    return true;
}

bool Device::set_gyro_odr(GyroODR odr)
{
    if (!rmw_reg(reg::CTRL2_G, ODR_MASK, static_cast<uint8_t>(odr))) return false;
    if (odr != GyroODR::power_down) m_gyro_last_odr = odr;
    m_gyro_odr = odr;
    return true;
}

bool Device::set_accel_range(AccelRange range)
{
    if (!rmw_reg(reg::CTRL1_XL, ACCEL_FS_MASK, static_cast<uint8_t>(range))) return false;
    m_accel_range = range;
    return true;
}

bool Device::set_gyro_range(GyroRange range)
{
    // GYRO_FS_MASK covers FS_G[3:2] and FS_125[1] in one write
    if (!rmw_reg(reg::CTRL2_G, GYRO_FS_MASK, static_cast<uint8_t>(range))) return false;
    m_gyro_range = range;
    return true;
}

bool Device::set_accel_self_test(bool enable)
{
    // ST_XL: 00 = normal, 01 = positive self-test
    const uint8_t bits = enable ? 0x01u : 0x00u;
    return rmw_reg(reg::CTRL5_C, ST_XL_MASK, bits);
}

bool Device::set_gyro_self_test(bool enable)
{
    // ST_G: 00 = normal, 01 = positive self-test (bits [5:4])
    const uint8_t bits = enable ? 0x10u : 0x00u;
    return rmw_reg(reg::CTRL5_C, ST_G_MASK, bits);
}

// ---------------------------------------------------------------------------
// Power control
// ---------------------------------------------------------------------------

bool Device::enable_accel()
{
    if (m_accel_odr != AccelODR::power_down) return true;  // already on
    return set_accel_odr(m_accel_last_odr);
}

bool Device::disable_accel()
{
    if (m_accel_odr == AccelODR::power_down) return true;  // already off
    m_accel_last_odr = m_accel_odr;
    return set_accel_odr(AccelODR::power_down);
}

bool Device::enable_gyro()
{
    if (m_gyro_odr != GyroODR::power_down) return true;
    return set_gyro_odr(m_gyro_last_odr);
}

bool Device::disable_gyro()
{
    if (m_gyro_odr == GyroODR::power_down) return true;
    m_gyro_last_odr = m_gyro_odr;
    return set_gyro_odr(GyroODR::power_down);
}

// ---------------------------------------------------------------------------
// Data acquisition
// ---------------------------------------------------------------------------

DataReady Device::data_ready() const
{
    uint8_t status = 0;
    read_regs(reg::STATUS_REG, &status, 1);
    return DataReady{
        .accel = (status & STATUS_XLDA) != 0,
        .gyro  = (status & STATUS_GDA)  != 0,
        .temp  = (status & STATUS_TDA)  != 0,
    };
}

bool Device::read_sample(Sample& sample) const
{
    // Single burst: OUT_TEMP_L(0x20) through OUT_Z_H_XL(0x2D) = 14 bytes
    // Layout: TEMP[2] | GYRO_X[2] GYRO_Y[2] GYRO_Z[2] | ACCEL_X[2] ACCEL_Y[2] ACCEL_Z[2]
    uint8_t buf[14] = {};
    if (!read_regs(reg::OUT_TEMP_L, buf, sizeof(buf))) return false;

    const float as = accel_sens(m_accel_range);
    const float gs = gyro_sens(m_gyro_range);

    sample.temp_c  = TEMP_OFFSET_C + static_cast<float>(le_i16(&buf[0])) * TEMP_SENS_C_PER_LSB;
    sample.gyro_x  = static_cast<float>(le_i16(&buf[2]))  * gs;
    sample.gyro_y  = static_cast<float>(le_i16(&buf[4]))  * gs;
    sample.gyro_z  = static_cast<float>(le_i16(&buf[6]))  * gs;
    sample.accel_x = static_cast<float>(le_i16(&buf[8]))  * as;
    sample.accel_y = static_cast<float>(le_i16(&buf[10])) * as;
    sample.accel_z = static_cast<float>(le_i16(&buf[12])) * as;
    return true;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

bool Device::read_who_am_i(uint8_t& value) const
{
    return read_regs(reg::WHO_AM_I, &value, 1);
}

} // namespace ism330dlc
