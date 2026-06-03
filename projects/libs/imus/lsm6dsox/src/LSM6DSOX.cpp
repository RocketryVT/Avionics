#include "lsm6dsox/LSM6DSOX.hpp"

namespace lsm6dsox {

namespace reg {
constexpr uint8_t WHO_AM_I   = 0x0F;
constexpr uint8_t CTRL1_XL   = 0x10;
constexpr uint8_t CTRL2_G    = 0x11;
constexpr uint8_t CTRL3_C    = 0x12;
constexpr uint8_t CTRL5_C    = 0x14;
constexpr uint8_t CTRL7_G    = 0x16;
constexpr uint8_t CTRL8_XL   = 0x17;
constexpr uint8_t CTRL9_XL   = 0x18;
constexpr uint8_t STATUS_REG = 0x1E;
constexpr uint8_t OUT_TEMP_L = 0x20;
constexpr uint8_t OUTX_L_G   = 0x22;
constexpr uint8_t OUTX_L_XL  = 0x28;
} // namespace reg

constexpr uint8_t ODR_MASK = 0xF0;
constexpr uint8_t ACCEL_FS_MASK = 0x0C;
constexpr uint8_t GYRO_FS_MASK = 0x0E;
constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_IF_INC = 0x04;
constexpr uint8_t CTRL3_BDU = 0x40;
constexpr uint8_t CTRL5_ST_XL_MASK = 0x03;
constexpr uint8_t CTRL5_ST_G_MASK = 0x30;
constexpr uint8_t CTRL8_ACCEL_LPF2_ODR_DIV4 = 0x09;
constexpr uint8_t CTRL9_I3C_DISABLE = 0x02;
constexpr uint8_t STATUS_XLDA = 0x01;
constexpr uint8_t STATUS_GDA = 0x02;
constexpr uint8_t STATUS_TDA = 0x04;
constexpr float TEMP_OFFSET_C = 25.0f;
constexpr float TEMP_SENS_C_PER_LSB = 1.0f / 256.0f;

namespace {

constexpr int16_t le_i16(const uint8_t* p) noexcept
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8u));
}

constexpr float accel_sens(AccelRange range) noexcept
{
    switch (range) {
        case AccelRange::g2:  return 0.000061f;
        case AccelRange::g16: return 0.000488f;
        case AccelRange::g4:  return 0.000122f;
        case AccelRange::g8:  return 0.000244f;
    }
    return 0.000122f;
}

constexpr float gyro_sens(GyroRange range) noexcept
{
    switch (range) {
        case GyroRange::dps_125:  return 0.004375f;
        case GyroRange::dps_250:  return 0.00875f;
        case GyroRange::dps_500:  return 0.0175f;
        case GyroRange::dps_1000: return 0.035f;
        case GyroRange::dps_2000: return 0.07f;
    }
    return 0.07f;
}

constexpr bool valid_who_am_i(uint8_t id) noexcept
{
    return id == Device::kWhoAmIExpected || id == Device::kWhoAmICompatible;
}

} // namespace

Device::Device(Transport transport, uint8_t device_address) noexcept
    : m_transport(transport)
    , m_device_address(device_address)
{}

bool Device::write_reg(uint8_t reg_addr, uint8_t val) const
{
    return m_transport.write_reg != nullptr &&
           m_transport.write_reg(m_transport.ctx, m_device_address, reg_addr, val);
}

bool Device::read_regs(uint8_t reg_addr, uint8_t* buf, size_t len) const
{
    return m_transport.read_regs != nullptr &&
           m_transport.read_regs(m_transport.ctx, m_device_address, reg_addr, buf, len);
}

bool Device::rmw_reg(uint8_t reg_addr, uint8_t mask, uint8_t bits) const
{
    uint8_t val = 0;
    if (!read_regs(reg_addr, &val, 1)) return false;
    val = static_cast<uint8_t>((val & ~mask) | (bits & mask));
    return write_reg(reg_addr, val);
}

bool Device::initialize(Config const& cfg)
{
    uint8_t id = 0;
    if (!read_who_am_i(id) || !valid_who_am_i(id)) return false;

    if (!reset(cfg.delay_ms)) return false;

    uint8_t ctrl3 = 0;
    if (cfg.auto_increment) ctrl3 |= CTRL3_IF_INC;
    if (cfg.block_data_update) ctrl3 |= CTRL3_BDU;
    if (!write_reg(reg::CTRL3_C, ctrl3)) return false;

    if (cfg.disable_i3c && !rmw_reg(reg::CTRL9_XL, CTRL9_I3C_DISABLE, CTRL9_I3C_DISABLE)) {
        return false;
    }

    if (!write_reg(reg::CTRL7_G, 0x00)) return false;
    if (!write_reg(reg::CTRL8_XL, cfg.accel_lpf2_odr_div4 ? CTRL8_ACCEL_LPF2_ODR_DIV4 : 0x00)) {
        return false;
    }

    return set_accel_range(cfg.accel_range)
        && set_gyro_range(cfg.gyro_range)
        && set_accel_odr(cfg.accel_odr)
        && set_gyro_odr(cfg.gyro_odr);
}

bool Device::reset(void (*delay_ms)(uint32_t))
{
    if (!write_reg(reg::CTRL3_C, CTRL3_SW_RESET)) return false;
    if (delay_ms) delay_ms(10);

    m_accel_range = AccelRange::g2;
    m_gyro_range = GyroRange::dps_250;
    m_accel_odr = AccelODR::power_down;
    m_gyro_odr = GyroODR::power_down;
    return true;
}

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
    if (!rmw_reg(reg::CTRL2_G, GYRO_FS_MASK, static_cast<uint8_t>(range))) return false;
    m_gyro_range = range;
    return true;
}

bool Device::set_accel_self_test(bool enable)
{
    return rmw_reg(reg::CTRL5_C, CTRL5_ST_XL_MASK, enable ? 0x01u : 0x00u);
}

bool Device::set_gyro_self_test(bool enable)
{
    return rmw_reg(reg::CTRL5_C, CTRL5_ST_G_MASK, enable ? 0x10u : 0x00u);
}

bool Device::enable_accel()
{
    if (m_accel_odr != AccelODR::power_down) return true;
    return set_accel_odr(m_accel_last_odr);
}

bool Device::disable_accel()
{
    if (m_accel_odr == AccelODR::power_down) return true;
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

DataReady Device::data_ready() const
{
    uint8_t status = 0;
    if (!read_regs(reg::STATUS_REG, &status, 1)) return {};
    return DataReady{
        .accel = (status & STATUS_XLDA) != 0,
        .gyro = (status & STATUS_GDA) != 0,
        .temp = (status & STATUS_TDA) != 0,
    };
}

bool Device::read_sample(Sample& sample) const
{
    uint8_t buf[14] = {};
    if (!read_regs(reg::OUT_TEMP_L, buf, sizeof(buf))) return false;

    const float as = accel_sens(m_accel_range);
    const float gs = gyro_sens(m_gyro_range);

    sample.temp_c = TEMP_OFFSET_C + static_cast<float>(le_i16(&buf[0])) * TEMP_SENS_C_PER_LSB;
    sample.gyro_x = static_cast<float>(le_i16(&buf[2])) * gs;
    sample.gyro_y = static_cast<float>(le_i16(&buf[4])) * gs;
    sample.gyro_z = static_cast<float>(le_i16(&buf[6])) * gs;
    sample.accel_x = static_cast<float>(le_i16(&buf[8])) * as;
    sample.accel_y = static_cast<float>(le_i16(&buf[10])) * as;
    sample.accel_z = static_cast<float>(le_i16(&buf[12])) * as;
    return true;
}

bool Device::read_accelerometer(float& x, float& y, float& z) const
{
    uint8_t buf[6] = {};
    if (!read_regs(reg::OUTX_L_XL, buf, sizeof(buf))) return false;
    const float as = accel_sens(m_accel_range);
    x = static_cast<float>(le_i16(&buf[0])) * as;
    y = static_cast<float>(le_i16(&buf[2])) * as;
    z = static_cast<float>(le_i16(&buf[4])) * as;
    return true;
}

bool Device::read_gyroscope(float& x, float& y, float& z) const
{
    uint8_t buf[6] = {};
    if (!read_regs(reg::OUTX_L_G, buf, sizeof(buf))) return false;
    const float gs = gyro_sens(m_gyro_range);
    x = static_cast<float>(le_i16(&buf[0])) * gs;
    y = static_cast<float>(le_i16(&buf[2])) * gs;
    z = static_cast<float>(le_i16(&buf[4])) * gs;
    return true;
}

bool Device::read_temperature(float& temp_c) const
{
    uint8_t buf[2] = {};
    if (!read_regs(reg::OUT_TEMP_L, buf, sizeof(buf))) return false;
    temp_c = TEMP_OFFSET_C + static_cast<float>(le_i16(buf)) * TEMP_SENS_C_PER_LSB;
    return true;
}

bool Device::read_who_am_i(uint8_t& value) const
{
    return read_regs(reg::WHO_AM_I, &value, 1);
}

} // namespace lsm6dsox
