#include "lis3mdl/LIS3MDL.hpp"

namespace lis3mdl {

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------

namespace reg {
    constexpr uint8_t WHO_AM_I = 0x0F;
    constexpr uint8_t CTRL1    = 0x20;
    constexpr uint8_t CTRL2    = 0x21;
    constexpr uint8_t CTRL3    = 0x22;
    constexpr uint8_t CTRL4    = 0x23;
    constexpr uint8_t STATUS   = 0x27;
    constexpr uint8_t OUT_X_L  = 0x28;
    constexpr uint8_t INT_CFG  = 0x30;
    constexpr uint8_t INT_THS_L = 0x32;
    constexpr uint8_t INT_THS_H = 0x33;
} // namespace reg

// CTRL1 bit positions
constexpr uint8_t CTRL1_ST_BIT      = 0;  // self-test
constexpr uint8_t CTRL1_FAST_ODR    = 1;  // FAST_ODR bit
constexpr uint8_t CTRL1_DO_SHIFT    = 2;  // DO[2:0]
constexpr uint8_t CTRL1_OM_SHIFT    = 5;  // OM[1:0] XY perf mode

// CTRL2 bit positions
constexpr uint8_t CTRL2_FS_SHIFT    = 5;  // FS[1:0]
constexpr uint8_t CTRL2_SOFT_RST    = 2;

// CTRL3 bit positions
constexpr uint8_t CTRL3_MD_SHIFT    = 0;  // MD[1:0]

// CTRL4 bit positions
constexpr uint8_t CTRL4_OMZ_SHIFT   = 2;  // OMZ[1:0] Z perf mode

// STATUS bit
constexpr uint8_t STATUS_ZYXDA      = 3;

// INT_CFG bits (see datasheet table 36)
constexpr uint8_t INT_CFG_IEN       = 0;
constexpr uint8_t INT_CFG_LIR       = 1;
constexpr uint8_t INT_CFG_IEA       = 2;
constexpr uint8_t INT_CFG_DEFAULT   = 0x08; // bit 3 set per datasheet default state
constexpr uint8_t INT_CFG_ZIEN      = 5;
constexpr uint8_t INT_CFG_YIEN      = 6;
constexpr uint8_t INT_CFG_XIEN      = 7;

// Multi-byte read flag (I2C auto-increment via MSB of sub-address)
constexpr uint8_t MULTI_READ        = 0x80;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

constexpr int16_t read_le_i16(const uint8_t* p) noexcept
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

/// Returns the performance mode implied by a FAST_ODR data rate.
constexpr PerformanceMode perf_for_fast_odr(DataRate rate) noexcept
{
    switch (rate) {
        case DataRate::hz_155:  return PerformanceMode::ultra_high;
        case DataRate::hz_300:  return PerformanceMode::high;
        case DataRate::hz_560:  return PerformanceMode::medium;
        case DataRate::hz_1000: return PerformanceMode::low_power;
        default:                return PerformanceMode::ultra_high;
    }
}

/// Returns true when a DataRate uses FAST_ODR (bit 0 of the enum value).
constexpr bool is_fast_odr(DataRate rate) noexcept
{
    return (static_cast<uint8_t>(rate) & 0x01u) != 0;
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

/// Read-modify-write: clear bits in mask, then OR in bits.
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

    // For non-FAST_ODR rates, performance mode must be applied explicitly.
    // FAST_ODR rates (hz_155/300/560/1000) couple perf mode to the rate and
    // are handled inside set_data_rate.
    if (!is_fast_odr(cfg.data_rate)) {
        if (!set_performance_mode(cfg.perf_mode)) return false;
    }

    return set_data_rate(cfg.data_rate)
        && set_range(cfg.range)
        && set_operation_mode(cfg.op_mode);
}

bool Device::reset(void (*delay_ms)(uint32_t))
{
    if (!rmw_reg(reg::CTRL2, 1u << CTRL2_SOFT_RST, 1u << CTRL2_SOFT_RST)) {
        return false;
    }
    if (delay_ms) {
        delay_ms(10);
    }
    // Re-sync cached range from hardware after reset
    uint8_t ctrl2 = 0;
    if (read_regs(reg::CTRL2, &ctrl2, 1)) {
        m_range = static_cast<Range>((ctrl2 >> CTRL2_FS_SHIFT) & 0x03u);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Runtime configuration
// ---------------------------------------------------------------------------

bool Device::set_range(Range range)
{
    constexpr uint8_t mask = 0b11u << CTRL2_FS_SHIFT;
    if (!rmw_reg(reg::CTRL2, mask, static_cast<uint8_t>(static_cast<uint8_t>(range) << CTRL2_FS_SHIFT))) {
        return false;
    }
    m_range = range;
    return true;
}

bool Device::set_performance_mode(PerformanceMode mode)
{
    // XY in CTRL1 bits [6:5]
    constexpr uint8_t xy_mask = 0b11u << CTRL1_OM_SHIFT;
    if (!rmw_reg(reg::CTRL1, xy_mask, static_cast<uint8_t>(static_cast<uint8_t>(mode) << CTRL1_OM_SHIFT))) {
        return false;
    }
    // Z in CTRL4 bits [3:2]
    constexpr uint8_t z_mask = 0b11u << CTRL4_OMZ_SHIFT;
    if (!rmw_reg(reg::CTRL4, z_mask, static_cast<uint8_t>(static_cast<uint8_t>(mode) << CTRL4_OMZ_SHIFT))) {
        return false;
    }
    m_perf_mode = mode;
    return true;
}

bool Device::set_data_rate(DataRate rate)
{
    // FAST_ODR rates require a specific performance mode
    if (is_fast_odr(rate)) {
        if (!set_performance_mode(perf_for_fast_odr(rate))) return false;
    }

    // DO[2:0] in CTRL1[4:2], FAST_ODR in CTRL1[1].
    // The enum value encodes these as bits [3:1] and [0] respectively —
    // writing them to register requires shifting left by 1 to align DO[0] at bit 2.
    constexpr uint8_t rate_mask = 0b1111u << CTRL1_FAST_ODR; // bits [4:1]
    uint8_t rate_bits = static_cast<uint8_t>(static_cast<uint8_t>(rate) << CTRL1_FAST_ODR);
    if (!rmw_reg(reg::CTRL1, rate_mask, rate_bits)) return false;

    m_data_rate = rate;
    return true;
}

bool Device::set_operation_mode(OperationMode mode)
{
    constexpr uint8_t mask = 0b11u << CTRL3_MD_SHIFT;
    return rmw_reg(reg::CTRL3, mask, static_cast<uint8_t>(static_cast<uint8_t>(mode) << CTRL3_MD_SHIFT));
}

bool Device::set_self_test(bool enable)
{
    return rmw_reg(reg::CTRL1, 1u << CTRL1_ST_BIT, enable ? (1u << CTRL1_ST_BIT) : 0u);
}

// ---------------------------------------------------------------------------
// Interrupt support
// ---------------------------------------------------------------------------

bool Device::set_int_threshold(uint16_t raw)
{
    raw &= 0x7FFFu;  // bit 15 must be 0 per datasheet
    const uint8_t lo = static_cast<uint8_t>(raw & 0xFF);
    const uint8_t hi = static_cast<uint8_t>(raw >> 8);
    return write_reg(reg::INT_THS_L, lo) && write_reg(reg::INT_THS_H, hi);
}

bool Device::get_int_threshold(uint16_t& raw) const
{
    uint8_t buf[2] = {};
    if (!read_regs(reg::INT_THS_L | MULTI_READ, buf, 2)) return false;
    raw = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    raw &= 0x7FFFu;
    return true;
}

bool Device::config_interrupt(IntConfig const& cfg)
{
    uint8_t val = INT_CFG_DEFAULT;
    if (cfg.enable_x)   val |= 1u << INT_CFG_XIEN;
    if (cfg.enable_y)   val |= 1u << INT_CFG_YIEN;
    if (cfg.enable_z)   val |= 1u << INT_CFG_ZIEN;
    if (cfg.active_high) val |= 1u << INT_CFG_IEA;
    if (cfg.latch)       val |= 1u << INT_CFG_LIR;
    if (cfg.enable)      val |= 1u << INT_CFG_IEN;
    return write_reg(reg::INT_CFG, val);
}

// ---------------------------------------------------------------------------
// Data acquisition
// ---------------------------------------------------------------------------

bool Device::data_ready() const
{
    uint8_t status = 0;
    if (!read_regs(reg::STATUS, &status, 1)) return false;
    return (status & (1u << STATUS_ZYXDA)) != 0;
}

bool Device::read_sample(Sample& sample) const
{
    uint8_t buf[6] = {};
    if (!read_regs(reg::OUT_X_L | MULTI_READ, buf, sizeof(buf))) return false;

    const float sensitivity = kSensitivity[static_cast<uint8_t>(m_range)];
    sample.x_gauss = read_le_i16(&buf[0]) / sensitivity;
    sample.y_gauss = read_le_i16(&buf[2]) / sensitivity;
    sample.z_gauss = read_le_i16(&buf[4]) / sensitivity;
    return true;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

bool Device::read_who_am_i(uint8_t& value) const
{
    return read_regs(reg::WHO_AM_I, &value, 1);
}

} // namespace lis3mdl
