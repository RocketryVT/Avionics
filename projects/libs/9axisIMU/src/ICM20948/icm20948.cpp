#include "nine_axis_imu/ICM20948/ICM20948.hpp"
#include <cstdint>

namespace nine_axis_imu {

namespace {

namespace reg {
constexpr uint8_t REG_BANK_SEL = 0x7F;

constexpr uint8_t WHO_AM_I = 0x00;
constexpr uint8_t USER_CTRL = 0x03;
constexpr uint8_t LP_CONFIG = 0x05;
constexpr uint8_t PWR_MGMT_1 = 0x06;
constexpr uint8_t PWR_MGMT_2 = 0x07;
constexpr uint8_t INT_PIN_CFG = 0x0F;
constexpr uint8_t I2C_MST_STATUS = 0x17;
constexpr uint8_t ACCEL_XOUT_H = 0x2D;
constexpr uint8_t DATA_RDY_STATUS = 0x74;

constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;
constexpr uint8_t GYRO_CONFIG_1 = 0x01;
constexpr uint8_t ODR_ALIGN_EN = 0x09;
constexpr uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t ACCEL_CONFIG = 0x14;

constexpr uint8_t I2C_MST_ODR_CONFIG = 0x00;
constexpr uint8_t I2C_MST_CTRL = 0x01;
constexpr uint8_t I2C_MST_DELAY_CTRL = 0x02;
constexpr uint8_t I2C_PERIPH0_ADDR = 0x03;
constexpr uint8_t I2C_PERIPH0_REG = 0x04;
constexpr uint8_t I2C_PERIPH0_CTRL = 0x05;
constexpr uint8_t I2C_PERIPH0_DO = 0x06;
constexpr uint8_t I2C_PERIPH1_ADDR = 0x07;
constexpr uint8_t I2C_PERIPH1_REG = 0x08;
constexpr uint8_t I2C_PERIPH1_CTRL = 0x09;
constexpr uint8_t I2C_PERIPH1_DO = 0x0A;
constexpr uint8_t I2C_PERIPH4_ADDR = 0x13;
constexpr uint8_t I2C_PERIPH4_REG = 0x14;
constexpr uint8_t I2C_PERIPH4_CTRL = 0x15;
constexpr uint8_t I2C_PERIPH4_DO = 0x16;
constexpr uint8_t I2C_PERIPH4_DI = 0x17;
} // namespace reg

namespace ak09916 {
constexpr uint8_t I2C_ADDR = 0x0C;
constexpr uint8_t WIA1 = 0x00;
constexpr uint8_t WIA2 = 0x01;
constexpr uint8_t ST1 = 0x10;
constexpr uint8_t CNTL2 = 0x31;
constexpr uint8_t CNTL3 = 0x32;
} // namespace ak09916

constexpr uint8_t USER_CTRL_I2C_MST_RST = 0x02;
constexpr uint8_t USER_CTRL_I2C_MST_EN = 0x20;
constexpr uint8_t PWR_MGMT_1_DEVICE_RESET = 0x80;
constexpr uint8_t PWR_MGMT_1_SLEEP = 0x40;
constexpr uint8_t PWR_MGMT_1_CLKSEL_AUTO = 0x01;
constexpr uint8_t INT_PIN_CFG_BYPASS_EN = 0x02;
constexpr uint8_t I2C_MST_STATUS_PERIPH4_DONE = 0x40;
constexpr uint8_t I2C_MST_STATUS_PERIPH4_NACK = 0x10;
constexpr uint8_t DATA_RDY_STATUS_RAW_DATA_RDY = 0x01;
constexpr uint8_t MAG_ST1_DRDY = 0x01;
constexpr uint8_t MAG_ST2_HOFL = 0x08;

constexpr uint16_t kImuBaseRateHz = 1125;
constexpr float kMagUtPerLsb = 0.15f;

constexpr int16_t be_i16(const uint8_t* p) noexcept
{
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8u) |
                                static_cast<uint16_t>(p[1]));
}

constexpr int16_t le_i16(const uint8_t* p) noexcept
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8u));
}

constexpr float accel_lsb_per_g(AccelRange range) noexcept
{
    switch (range) {
        case AccelRange::g2: return 16384.0f;
        case AccelRange::g4: return 8192.0f;
        case AccelRange::g8: return 4096.0f;
        case AccelRange::g16: return 2048.0f;
    }
    return 2048.0f;
}

constexpr float gyro_lsb_per_dps(GyroRange range) noexcept
{
    switch (range) {
        case GyroRange::dps_250: return 131.0f;
        case GyroRange::dps_500: return 65.5f;
        case GyroRange::dps_1000: return 32.8f;
        case GyroRange::dps_2000: return 16.4f;
    }
    return 16.4f;
}

constexpr uint16_t sample_divider_for_rate(uint16_t hz) noexcept
{
    if (hz == 0) {
        hz = 1;
    }
    const uint16_t div = static_cast<uint16_t>((kImuBaseRateHz + (hz / 2u)) / hz);
    return static_cast<uint16_t>((div == 0 ? 1 : div) - 1u);
}

} // namespace

ICM20948::ICM20948(Transport transport, uint8_t device_address) noexcept
    : m_transport(transport)
    , m_device_address(device_address)
{}

bool ICM20948::write_reg(uint8_t reg_addr, uint8_t val) const
{
    return m_transport.write_reg != nullptr &&
           m_transport.write_reg(m_transport.ctx, m_device_address, reg_addr, val);
}

bool ICM20948::read_regs(uint8_t reg_addr, uint8_t* buf, size_t len) const
{
    return m_transport.read_regs != nullptr &&
           m_transport.read_regs(m_transport.ctx, m_device_address, reg_addr, buf, len);
}

bool ICM20948::set_bank(Bank bank) const
{
    if (m_bank_known && m_current_bank == bank) {
        return true;
    }
    const uint8_t value = static_cast<uint8_t>(static_cast<uint8_t>(bank) << 4u);
    if (!write_reg(reg::REG_BANK_SEL, value)) {
        return false;
    }
    m_current_bank = bank;
    m_bank_known = true;
    return true;
}

bool ICM20948::write_bank(Bank bank, uint8_t reg_addr, uint8_t val) const
{
    return set_bank(bank) && write_reg(reg_addr, val);
}

bool ICM20948::read_bank(Bank bank, uint8_t reg_addr, uint8_t* buf, size_t len) const
{
    return set_bank(bank) && read_regs(reg_addr, buf, len);
}

bool ICM20948::rmw_bank(Bank bank, uint8_t reg_addr, uint8_t mask, uint8_t bits) const
{
    uint8_t value = 0;
    if (!read_bank(bank, reg_addr, &value, 1)) {
        return false;
    }
    value = static_cast<uint8_t>((value & ~mask) | (bits & mask));
    return write_bank(bank, reg_addr, value);
}

bool ICM20948::initialize(Config const& cfg)
{
    uint8_t who = 0;
    if (!read_who_am_i(who) || who != kWhoAmIExpected) {
        return false;
    }

    if (!reset(cfg.delay_ms)) {
        return false;
    }

    if (!write_bank(Bank::bank0, reg::PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO)) {
        return false;
    }
    if (cfg.delay_ms) {
        cfg.delay_ms(10);
    }

    if (!write_bank(Bank::bank0, reg::PWR_MGMT_2, 0x00)) {
        return false;
    }
    if (!write_bank(Bank::bank0, reg::LP_CONFIG, 0x00)) {
        return false;
    }

    if (!set_accel_range(cfg.accel_range) ||
        !set_gyro_range(cfg.gyro_range) ||
        !set_accel_rate(cfg.accel_rate_hz) ||
        !set_gyro_rate(cfg.gyro_rate_hz) ||
        !set_accel_dlpf(cfg.accel_dlpf_enable, cfg.accel_dlpf) ||
        !set_gyro_dlpf(cfg.gyro_dlpf_enable, cfg.gyro_dlpf)) {
        return false;
    }

    if (!write_bank(Bank::bank2, reg::ODR_ALIGN_EN, 0x01)) {
        return false;
    }

    if (!enable_i2c_master()) {
        return false;
    }

    uint16_t mag_id = 0;
    bool mag_ok = false;
    for (uint8_t tries = 0; tries < 10 && !mag_ok; ++tries) {
        if (read_mag_who_am_i(mag_id) && mag_id == kAk09916WhoAmIExpected) {
            mag_ok = true;
            break;
        }
        if (!reset_i2c_master()) {
            return false;
        }
        if (cfg.delay_ms) {
            cfg.delay_ms(10);
        }
    }
    if (!mag_ok) {
        return false;
    }

    if (!aux_write(ak09916::I2C_ADDR, ak09916::CNTL3, 0x01)) {
        return false;
    }
    if (cfg.delay_ms) {
        cfg.delay_ms(10);
    }
    return set_mag_mode(cfg.mag_mode) && configure_mag_readout();
}

bool ICM20948::reset(void (*delay_ms)(uint32_t))
{
    m_bank_known = false;
    if (!write_bank(Bank::bank0, reg::PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET)) {
        return false;
    }
    if (delay_ms) {
        delay_ms(100);
    }
    m_bank_known = false;
    m_current_bank = Bank::bank0;
    m_accel_range = AccelRange::g2;
    m_gyro_range = GyroRange::dps_250;
    return true;
}

bool ICM20948::set_accel_range(AccelRange range)
{
    constexpr uint8_t mask = 0x06;
    const uint8_t bits = static_cast<uint8_t>(static_cast<uint8_t>(range) << 1u);
    if (!rmw_bank(Bank::bank2, reg::ACCEL_CONFIG, mask, bits)) {
        return false;
    }
    m_accel_range = range;
    return true;
}

bool ICM20948::set_gyro_range(GyroRange range)
{
    constexpr uint8_t mask = 0x06;
    const uint8_t bits = static_cast<uint8_t>(static_cast<uint8_t>(range) << 1u);
    if (!rmw_bank(Bank::bank2, reg::GYRO_CONFIG_1, mask, bits)) {
        return false;
    }
    m_gyro_range = range;
    return true;
}

bool ICM20948::set_accel_rate(uint16_t hz)
{
    const uint16_t div = sample_divider_for_rate(hz);
    if (!write_bank(Bank::bank2, reg::ACCEL_SMPLRT_DIV_1, static_cast<uint8_t>((div >> 8u) & 0x0Fu))) {
        return false;
    }
    if (!write_bank(Bank::bank2, reg::ACCEL_SMPLRT_DIV_2, static_cast<uint8_t>(div & 0xFFu))) {
        return false;
    }
    m_accel_rate_hz = hz;
    return true;
}

bool ICM20948::set_gyro_rate(uint16_t hz)
{
    const uint16_t div = sample_divider_for_rate(hz);
    if (!write_bank(Bank::bank2, reg::GYRO_SMPLRT_DIV, static_cast<uint8_t>(div & 0xFFu))) {
        return false;
    }
    m_gyro_rate_hz = hz;
    return true;
}

bool ICM20948::set_accel_dlpf(bool enable, DlpfBandwidth bandwidth)
{
    const uint8_t bits = static_cast<uint8_t>((static_cast<uint8_t>(bandwidth) << 3u) |
                                             (enable ? 0x01u : 0x00u));
    return rmw_bank(Bank::bank2, reg::ACCEL_CONFIG, 0x39, bits);
}

bool ICM20948::set_gyro_dlpf(bool enable, DlpfBandwidth bandwidth)
{
    const uint8_t bits = static_cast<uint8_t>((static_cast<uint8_t>(bandwidth) << 3u) |
                                             (enable ? 0x01u : 0x00u));
    return rmw_bank(Bank::bank2, reg::GYRO_CONFIG_1, 0x39, bits);
}

bool ICM20948::set_mag_mode(MagMode mode)
{
    return aux_write(ak09916::I2C_ADDR, ak09916::CNTL2, static_cast<uint8_t>(mode));
}

bool ICM20948::enable_i2c_master()
{
    if (!rmw_bank(Bank::bank0, reg::INT_PIN_CFG, INT_PIN_CFG_BYPASS_EN, 0x00)) {
        return false;
    }
    if (!write_bank(Bank::bank3, reg::I2C_MST_CTRL, 0x17)) {
        return false;
    }
    return rmw_bank(Bank::bank0, reg::USER_CTRL, USER_CTRL_I2C_MST_EN, USER_CTRL_I2C_MST_EN);
}

bool ICM20948::reset_i2c_master()
{
    return rmw_bank(Bank::bank0, reg::USER_CTRL, USER_CTRL_I2C_MST_RST, USER_CTRL_I2C_MST_RST);
}

bool ICM20948::configure_aux_peripheral(uint8_t slot, uint8_t addr, uint8_t addr_reg,
                                        uint8_t len, bool read, bool enable,
                                        uint8_t data_out) const
{
    if (slot > 1 || len > 15) {
        return false;
    }

    const uint8_t base = (slot == 0) ? reg::I2C_PERIPH0_ADDR : reg::I2C_PERIPH1_ADDR;
    const uint8_t ctrl = static_cast<uint8_t>(base + 2u);
    const uint8_t data = static_cast<uint8_t>(base + 3u);
    const uint8_t rw_addr = static_cast<uint8_t>((read ? 0x80u : 0x00u) | (addr & 0x7Fu));
    const uint8_t control = static_cast<uint8_t>((enable ? 0x80u : 0x00u) | (len & 0x0Fu));

    return write_bank(Bank::bank3, base, rw_addr) &&
           write_bank(Bank::bank3, static_cast<uint8_t>(base + 1u), addr_reg) &&
           (read || write_bank(Bank::bank3, data, data_out)) &&
           write_bank(Bank::bank3, ctrl, control);
}

bool ICM20948::configure_mag_readout()
{
    if (!configure_aux_peripheral(0, ak09916::I2C_ADDR, ak09916::ST1, 9, true, true)) {
        return false;
    }
    if (!write_bank(Bank::bank3, reg::I2C_MST_ODR_CONFIG, 0x00)) {
        return false;
    }
    return write_bank(Bank::bank3, reg::I2C_MST_DELAY_CTRL, 0x01);
}

bool ICM20948::aux_read(uint8_t addr, uint8_t addr_reg, uint8_t& data) const
{
    const uint8_t rw_addr = static_cast<uint8_t>(0x80u | (addr & 0x7Fu));
    if (!write_bank(Bank::bank3, reg::I2C_PERIPH4_ADDR, rw_addr) ||
        !write_bank(Bank::bank3, reg::I2C_PERIPH4_REG, addr_reg) ||
        !write_bank(Bank::bank3, reg::I2C_PERIPH4_CTRL, 0x80)) {
        return false;
    }

    uint8_t status = 0;
    for (uint16_t i = 0; i < 1000; ++i) {
        if (!read_bank(Bank::bank0, reg::I2C_MST_STATUS, &status, 1)) {
            return false;
        }
        if ((status & I2C_MST_STATUS_PERIPH4_DONE) != 0) {
            break;
        }
    }
    if ((status & (I2C_MST_STATUS_PERIPH4_DONE | I2C_MST_STATUS_PERIPH4_NACK)) !=
        I2C_MST_STATUS_PERIPH4_DONE) {
        return false;
    }
    return read_bank(Bank::bank3, reg::I2C_PERIPH4_DI, &data, 1);
}

bool ICM20948::aux_write(uint8_t addr, uint8_t addr_reg, uint8_t data) const
{
    const uint8_t rw_addr = static_cast<uint8_t>(addr & 0x7Fu);
    if (!write_bank(Bank::bank3, reg::I2C_PERIPH4_ADDR, rw_addr) ||
        !write_bank(Bank::bank3, reg::I2C_PERIPH4_REG, addr_reg) ||
        !write_bank(Bank::bank3, reg::I2C_PERIPH4_DO, data) ||
        !write_bank(Bank::bank3, reg::I2C_PERIPH4_CTRL, 0x80)) {
        return false;
    }

    uint8_t status = 0;
    for (uint16_t i = 0; i < 1000; ++i) {
        if (!read_bank(Bank::bank0, reg::I2C_MST_STATUS, &status, 1)) {
            return false;
        }
        if ((status & I2C_MST_STATUS_PERIPH4_DONE) != 0) {
            break;
        }
    }
    return (status & (I2C_MST_STATUS_PERIPH4_DONE | I2C_MST_STATUS_PERIPH4_NACK)) ==
           I2C_MST_STATUS_PERIPH4_DONE;
}

DataReady ICM20948::data_ready() const
{
    uint8_t status = 0;
    if (!read_bank(Bank::bank0, reg::DATA_RDY_STATUS, &status, 1)) {
        return {};
    }

    uint8_t mag[9] = {};
    if (!read_bank(Bank::bank0, static_cast<uint8_t>(reg::ACCEL_XOUT_H + 14u), mag, sizeof(mag))) {
        return DataReady{
            .accel = (status & DATA_RDY_STATUS_RAW_DATA_RDY) != 0,
            .gyro = (status & DATA_RDY_STATUS_RAW_DATA_RDY) != 0,
            .mag = false,
            .mag_overflow = false,
        };
    }

    return DataReady{
        .accel = (status & DATA_RDY_STATUS_RAW_DATA_RDY) != 0,
        .gyro = (status & DATA_RDY_STATUS_RAW_DATA_RDY) != 0,
        .mag = (mag[0] & MAG_ST1_DRDY) != 0,
        .mag_overflow = (mag[8] & MAG_ST2_HOFL) != 0,
    };
}

bool ICM20948::read_sample(Sample& sample) const
{
    uint8_t buf[23] = {};
    if (!read_bank(Bank::bank0, reg::ACCEL_XOUT_H, buf, sizeof(buf))) {
        return false;
    }

    const float accel_scale = accel_lsb_per_g(m_accel_range);
    const float gyro_scale = gyro_lsb_per_dps(m_gyro_range);

    sample.accel_x = static_cast<float>(be_i16(&buf[0])) / accel_scale;
    sample.accel_y = static_cast<float>(be_i16(&buf[2])) / accel_scale;
    sample.accel_z = static_cast<float>(be_i16(&buf[4])) / accel_scale;
    sample.gyro_x = static_cast<float>(be_i16(&buf[6])) / gyro_scale;
    sample.gyro_y = static_cast<float>(be_i16(&buf[8])) / gyro_scale;
    sample.gyro_z = static_cast<float>(be_i16(&buf[10])) / gyro_scale;
    sample.temp_c = ((static_cast<float>(be_i16(&buf[12])) - 21.0f) / 333.87f) + 21.0f;

    const uint8_t* mag = &buf[14];
    sample.mag_valid = (mag[0] & MAG_ST1_DRDY) != 0;
    sample.mag_overflow = (mag[8] & MAG_ST2_HOFL) != 0;
    if (sample.mag_valid && !sample.mag_overflow) {
        sample.mag_x_ut = static_cast<float>(le_i16(&mag[1])) * kMagUtPerLsb;
        sample.mag_y_ut = static_cast<float>(le_i16(&mag[3])) * kMagUtPerLsb;
        sample.mag_z_ut = static_cast<float>(le_i16(&mag[5])) * kMagUtPerLsb;
    } else {
        sample.mag_x_ut = 0.0f;
        sample.mag_y_ut = 0.0f;
        sample.mag_z_ut = 0.0f;
    }
    return true;
}

bool ICM20948::read_who_am_i(uint8_t& value) const
{
    return read_bank(Bank::bank0, reg::WHO_AM_I, &value, 1);
}

bool ICM20948::read_mag_who_am_i(uint16_t& value) const
{
    uint8_t wia1 = 0;
    uint8_t wia2 = 0;
    if (!aux_read(ak09916::I2C_ADDR, ak09916::WIA1, wia1) ||
        !aux_read(ak09916::I2C_ADDR, ak09916::WIA2, wia2)) {
        return false;
    }
    value = static_cast<uint16_t>((static_cast<uint16_t>(wia1) << 8u) | wia2);
    return true;
}

} // namespace nine_axis_imu
