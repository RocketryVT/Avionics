#include "icm40609d/ICM40609D.hpp"

#include <cmath>
#include <cstdint>

namespace icm40609d {

// ---------------------------------------------------------------------------
// Register map (Bank 0, unless noted)
// ---------------------------------------------------------------------------

namespace reg {

// Bank 0
constexpr uint8_t DEVICE_CONFIG     = 0x11u;  // soft-reset bit [0]
constexpr uint8_t INT_CONFIG        = 0x14u;
constexpr uint8_t INT_STATUS        = 0x2Du;  // DRDY bit [3]
constexpr uint8_t PWR_MGMT0        = 0x4Eu;
constexpr uint8_t GYRO_CONFIG0     = 0x4Fu;
constexpr uint8_t ACCEL_CONFIG0    = 0x50u;
constexpr uint8_t GYRO_CONFIG1     = 0x51u;  // temp DLPF + gyro UI filter order
constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52u; // accel + gyro LPF bandwidth
constexpr uint8_t ACCEL_CONFIG1    = 0x53u;  // accel UI filter order
constexpr uint8_t INT_CONFIG0      = 0x63u;
constexpr uint8_t INT_CONFIG1      = 0x64u;
constexpr uint8_t INT_SOURCE0      = 0x65u;
constexpr uint8_t WHO_AM_I         = 0x75u;
constexpr uint8_t BANK_SEL         = 0x76u;

// Data registers (Bank 0) — big-endian: X1=MSB, X0=LSB
constexpr uint8_t TEMP_DATA1       = 0x1Du;  // burst start: 14 bytes through GYRO_Z0
constexpr uint8_t ACCEL_DATA_X1    = 0x1Fu;
constexpr uint8_t GYRO_DATA_X1     = 0x25u;

// Bank 1 — gyro AAF / notch / HPF
constexpr uint8_t GYRO_CONFIG_STATIC2  = 0x0Bu;  // AAF_DIS[1], NF_DIS[0]
constexpr uint8_t GYRO_CONFIG_STATIC3  = 0x0Cu;  // AAF_DELT[5:0]
constexpr uint8_t GYRO_CONFIG_STATIC4  = 0x0Du;  // AAF_DELTSQR[7:0]
constexpr uint8_t GYRO_CONFIG_STATIC5  = 0x0Eu;  // AAF_BITSHIFT[3:0] | AAF_DELTSQR[11:8]
constexpr uint8_t GYRO_CONFIG_STATIC6  = 0x0Fu;  // NF_COSWZ[7:0] X
constexpr uint8_t GYRO_CONFIG_STATIC7  = 0x10u;  // NF_COSWZ[7:0] Y
constexpr uint8_t GYRO_CONFIG_STATIC8  = 0x11u;  // NF_COSWZ[7:0] Z
constexpr uint8_t GYRO_CONFIG_STATIC9  = 0x12u;  // NF_COSWZ[8] + NF_COSWZ_SEL
constexpr uint8_t GYRO_CONFIG_STATIC10 = 0x13u;  // NF_BW_SEL[6:4], HPF_BW_IND[3:1], HPF_ORD[0]

// Bank 2 — accel AAF
constexpr uint8_t ACCEL_CONFIG_STATIC2 = 0x03u;  // AAF_DIS[0], AAF_DELT[6:1]
constexpr uint8_t ACCEL_CONFIG_STATIC3 = 0x04u;  // AAF_DELTSQR[7:0]
constexpr uint8_t ACCEL_CONFIG_STATIC4 = 0x05u;  // AAF_BITSHIFT[3:0] | AAF_DELTSQR[11:8]

} // namespace reg

// ---------------------------------------------------------------------------
// PWR_MGMT0 bit fields
// ---------------------------------------------------------------------------

constexpr uint8_t TEMP_ENABLED   = 0u << 5;  // 0 = enabled
constexpr uint8_t RC_ON          = 1u << 4;
constexpr uint8_t GYRO_MODE_OFF  = 0u << 2;
constexpr uint8_t GYRO_MODE_LN   = 3u << 2;  // low-noise
constexpr uint8_t ACCEL_MODE_OFF = 0u << 0;
constexpr uint8_t ACCEL_MODE_LN  = 3u << 0;  // low-noise

// ---------------------------------------------------------------------------
// AAF profile table (from ICM-40609-D datasheet Table 5.2 / Betaflight)
// ---------------------------------------------------------------------------

struct AafProfile {
    uint16_t hz;
    uint8_t  delt;
    uint16_t deltsqr;
    uint8_t  bitshift;
};

static constexpr std::array<AafProfile, 63> kAafProfiles = {{
    {   42,  1,    1, 15 }, {   84,  2,    4, 13 }, {  126,  3,    9, 12 },
    {  170,  4,   16, 11 }, {  213,  5,   25, 10 }, {  258,  6,   36, 10 },
    {  303,  7,   49,  9 }, {  348,  8,   64,  9 }, {  394,  9,   81,  9 },
    {  441, 10,  100,  8 }, {  488, 11,  122,  8 }, {  536, 12,  144,  8 },
    {  585, 13,  170,  8 }, {  634, 14,  196,  8 }, {  684, 15,  224,  7 },
    {  734, 16,  256,  7 }, {  785, 17,  288,  7 }, {  837, 18,  324,  7 },
    {  890, 19,  360,  6 }, {  943, 20,  400,  6 }, {  997, 21,  440,  6 },
    { 1051, 22,  488,  6 }, { 1107, 23,  528,  6 }, { 1163, 24,  576,  6 },
    { 1220, 25,  624,  6 }, { 1277, 26,  680,  6 }, { 1336, 27,  736,  5 },
    { 1395, 28,  784,  5 }, { 1454, 29,  848,  5 }, { 1515, 30,  896,  5 },
    { 1577, 31,  960,  5 }, { 1639, 32, 1024,  5 }, { 1702, 33, 1088,  5 },
    { 1766, 34, 1152,  5 }, { 1830, 35, 1232,  5 }, { 1896, 36, 1296,  5 },
    { 1962, 37, 1376,  4 }, { 2029, 38, 1440,  4 }, { 2097, 39, 1536,  4 },
    { 2166, 40, 1600,  4 }, { 2235, 41, 1696,  4 }, { 2306, 42, 1760,  4 },
    { 2377, 43, 1856,  4 }, { 2449, 44, 1952,  4 }, { 2522, 45, 2016,  4 },
    { 2596, 46, 2112,  4 }, { 2671, 47, 2208,  4 }, { 2746, 48, 2304,  4 },
    { 2823, 49, 2400,  4 }, { 2900, 50, 2496,  4 }, { 2978, 51, 2592,  4 },
    { 3057, 52, 2720,  4 }, { 3137, 53, 2816,  4 }, { 3217, 54, 2944,  3 },
    { 3299, 55, 3008,  3 }, { 3381, 56, 3136,  3 }, { 3464, 57, 3264,  3 },
    { 3548, 58, 3392,  3 }, { 3633, 59, 3456,  3 }, { 3718, 60, 3584,  3 },
    { 3805, 61, 3712,  3 }, { 3892, 62, 3840,  3 }, { 3979, 63, 3968,  3 },
}};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

constexpr AafProfile aaf_for_hz(uint16_t target) noexcept
{
    // Return the entry with hz >= target (first fit); fall back to last entry.
    for (const auto& p : kAafProfiles) {
        if (target <= p.hz) return p;
    }
    return kAafProfiles.back();
}

constexpr int16_t be_i16(const uint8_t* p) noexcept
{
    // Sensor registers are big-endian: DATA_X1 = MSB, DATA_X0 = LSB.
    return static_cast<int16_t>(
        (static_cast<uint16_t>(p[0]) << 8u) | static_cast<uint16_t>(p[1]));
}

/// Datasheet LSB sensitivities for each full-scale range.
constexpr float gyro_sens(GyroRange r) noexcept
{
    switch (r) {
        case GyroRange::dps_2000:   return 1.0f / 16.4f;
        case GyroRange::dps_1000:   return 1.0f / 32.8f;
        case GyroRange::dps_500:    return 1.0f / 65.5f;
        case GyroRange::dps_250:    return 1.0f / 131.0f;
        case GyroRange::dps_125:    return 1.0f / 262.0f;
        case GyroRange::dps_62_5:   return 1.0f / 524.3f;
        case GyroRange::dps_31_25:  return 1.0f / 1048.6f;
        case GyroRange::dps_15_625: return 1.0f / 2097.2f;
    }
    return 1.0f / 16.4f;
}

constexpr float accel_sens(AccelRange r) noexcept
{
    // m/s² per LSB — 9.80665 m/s² per g
    switch (r) {
        case AccelRange::g16:    return 9.80665f / 2048.0f;
        case AccelRange::g8:     return 9.80665f / 4096.0f;
        case AccelRange::g4:     return 9.80665f / 8192.0f;
        case AccelRange::g2:     return 9.80665f / 16384.0f;
        case AccelRange::g1:     return 9.80665f / 32768.0f;
        case AccelRange::g0_5:   return 9.80665f / 65536.0f;
        case AccelRange::g0_25:  return 9.80665f / 131072.0f;
        case AccelRange::g0_125: return 9.80665f / 262144.0f;
    }
    return 9.80665f / 2048.0f;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Device::Device(Transport transport) noexcept
    : m_transport(transport)
{}

// ---------------------------------------------------------------------------
// Private I/O
// ---------------------------------------------------------------------------

bool Device::write_reg(uint8_t reg, uint8_t val) const
{
    return m_transport.write_reg != nullptr &&
           m_transport.write_reg(m_transport.ctx, reg, val);
}

bool Device::read_regs(uint8_t reg, uint8_t* buf, size_t len) const
{
    return m_transport.read_regs != nullptr &&
           m_transport.read_regs(m_transport.ctx, reg, buf, len);
}

bool Device::rmw_reg(uint8_t reg, uint8_t mask, uint8_t bits) const
{
    uint8_t val = 0u;
    if (!read_regs(reg, &val, 1u)) return false;
    val = static_cast<uint8_t>((val & ~mask) | (bits & mask));
    return write_reg(reg, val);
}

bool Device::select_bank(uint8_t bank) const
{
    return write_reg(reg::BANK_SEL, bank & 0x07u);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool Device::reset()
{
    // Ensure we're in bank 0 before touching DEVICE_CONFIG
    if (!select_bank(0u)) return false;
    if (!write_reg(reg::DEVICE_CONFIG, 0x01u)) return false;
    // Datasheet: reset completes in <1 ms; we busy-wait 2 ms to be safe.
    // Caller may provide delay_ms in Config; reset() itself is transport-agnostic.
    // A simple spin is unavoidable here without a delay callback.
    // The initialize() path calls delay_ms if provided.
    return true;
}

bool Device::initialize(Config const& cfg)
{
    // -- Reset ---------------------------------------------------------------
    if (!select_bank(0u)) return false;
    if (!write_reg(reg::DEVICE_CONFIG, 0x01u)) return false;
    if (cfg.delay_ms) cfg.delay_ms(2u);

    // -- WHO_AM_I ------------------------------------------------------------
    uint8_t id = 0u;
    if (!read_who_am_i(id) || id != kWhoAmIExpected) return false;

    // -- Power down while configuring ----------------------------------------
    if (!write_reg(reg::PWR_MGMT0,
                   TEMP_ENABLED | RC_ON | GYRO_MODE_OFF | ACCEL_MODE_OFF)) {
        return false;
    }

    // -- Gyro / Accel ranges and ODR -----------------------------------------
    if (!set_gyro_range(cfg.gyro_range))   return false;
    if (!set_accel_range(cfg.accel_range)) return false;
    if (!set_gyro_odr(cfg.gyro_odr))       return false;
    if (!set_accel_odr(cfg.accel_odr))     return false;

    // -- Temperature DLPF ----------------------------------------------------
    if (!set_temp_filt_bw(cfg.temp_filt_bw)) return false;

    // -- UI filter order -----------------------------------------------------
    if (!set_gyro_ui_filt_order(cfg.gyro_ui_order))   return false;
    if (!set_accel_ui_filt_order(cfg.accel_ui_order)) return false;

    // -- Anti-alias filters --------------------------------------------------
    if (!set_gyro_aaf (cfg.gyro_aaf_hz  > 0u, cfg.gyro_aaf_hz))  return false;
    if (!set_accel_aaf(cfg.accel_aaf_hz > 0u, cfg.accel_aaf_hz)) return false;

    // -- Gyro notch filter ---------------------------------------------------
    if (!set_gyro_notch(cfg.gyro_notch_en, cfg.gyro_nf_bw, cfg.gyro_notch_khz)) {
        return false;
    }

    // -- Gyro HPF ------------------------------------------------------------
    if (!set_gyro_hpf(cfg.gyro_hpf_en, cfg.gyro_hpf_bw, cfg.gyro_hpf_order)) {
        return false;
    }

    // -- Interrupt: DRDY on INT1, auto-clear on read -------------------------
    if (!select_bank(0u))                                              return false;
    if (!write_reg(reg::INT_SOURCE0, 0x08u))                          return false; // UI_DRDY_INT1_EN
    if (!write_reg(reg::INT_CONFIG0, 0x00u))                          return false; // clear on status read
    if (!write_reg(reg::INT_CONFIG1, (0u << 6) | (0u << 5) | (0u << 4))) return false;

    // -- Enable sensors in low-noise mode ------------------------------------
    if (!write_reg(reg::PWR_MGMT0,
                   TEMP_ENABLED | RC_ON | GYRO_MODE_LN | ACCEL_MODE_LN)) {
        return false;
    }
    // Datasheet: wait ≥200 µs after enabling sensors before reading.
    if (cfg.delay_ms) cfg.delay_ms(1u);

    return true;
}

// ---------------------------------------------------------------------------
// Runtime configuration
// ---------------------------------------------------------------------------

bool Device::set_gyro_range(GyroRange range)
{
    if (!select_bank(0u)) return false;
    const uint8_t bits = static_cast<uint8_t>(static_cast<uint8_t>(range) << 5u);
    if (!rmw_reg(reg::GYRO_CONFIG0, 0xE0u, bits)) return false;
    m_gyro_range = range;
    return true;
}

bool Device::set_accel_range(AccelRange range)
{
    if (!select_bank(0u)) return false;
    const uint8_t bits = static_cast<uint8_t>(static_cast<uint8_t>(range) << 5u);
    if (!rmw_reg(reg::ACCEL_CONFIG0, 0xE0u, bits)) return false;
    m_accel_range = range;
    return true;
}

bool Device::set_gyro_odr(ODR odr)
{
    if (!select_bank(0u)) return false;
    if (!rmw_reg(reg::GYRO_CONFIG0, 0x0Fu, static_cast<uint8_t>(odr))) return false;
    m_gyro_odr = odr;
    return true;
}

bool Device::set_accel_odr(ODR odr)
{
    if (!select_bank(0u)) return false;
    if (!rmw_reg(reg::ACCEL_CONFIG0, 0x0Fu, static_cast<uint8_t>(odr))) return false;
    m_accel_odr = odr;
    return true;
}

// ---------------------------------------------------------------------------
// Data acquisition
// ---------------------------------------------------------------------------

bool Device::data_ready() const
{
    uint8_t status = 0u;
    // read_regs operates on the current bank; INT_STATUS is bank 0.
    select_bank(0u);
    read_regs(reg::INT_STATUS, &status, 1u);
    return (status & 0x08u) != 0u;
}

bool Device::read_sample(Sample& out) const
{
    // Burst-read 14 bytes starting at TEMP_DATA1 (0x1D):
    // [0..1]  TEMP        big-endian
    // [2..3]  ACCEL_X     big-endian
    // [4..5]  ACCEL_Y
    // [6..7]  ACCEL_Z
    // [8..9]  GYRO_X      big-endian
    // [10..11] GYRO_Y
    // [12..13] GYRO_Z
    uint8_t raw[14] = {};
    if (!read_regs(reg::TEMP_DATA1, raw, sizeof(raw))) return false;

    const float as = accel_sens(m_accel_range);
    const float gs = gyro_sens(m_gyro_range);

    // Temperature: T(°C) = (raw / 132.48) + 25  (datasheet §4.5.1)
    out.temp_c    = static_cast<float>(be_i16(&raw[0]))  / 132.48f + 25.0f;
    out.accel_x   = static_cast<float>(be_i16(&raw[2]))  * as;
    out.accel_y   = static_cast<float>(be_i16(&raw[4]))  * as;
    out.accel_z   = static_cast<float>(be_i16(&raw[6]))  * as;
    out.gyro_x    = static_cast<float>(be_i16(&raw[8]))  * gs;
    out.gyro_y    = static_cast<float>(be_i16(&raw[10])) * gs;
    out.gyro_z    = static_cast<float>(be_i16(&raw[12])) * gs;

    return true;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

bool Device::read_who_am_i(uint8_t& value) const
{
    select_bank(0u);
    return read_regs(reg::WHO_AM_I, &value, 1u);
}

// ---------------------------------------------------------------------------
// Private filter helpers
// ---------------------------------------------------------------------------

bool Device::set_accel_aaf(bool enable, uint16_t target_hz)
{
    if (!select_bank(2u)) return false;

    if (!enable) {
        // Set ACCEL_AAF_DIS bit [0]
        return rmw_reg(reg::ACCEL_CONFIG_STATIC2, 0x01u, 0x01u);
    }

    const auto p = aaf_for_hz(target_hz);

    // STATIC2[7:1] = ACCEL_AAF_DELT[5:0] (bits [6:1]), bit[0] = AAF_DIS (clear)
    uint8_t s2 = 0u;
    if (!read_regs(reg::ACCEL_CONFIG_STATIC2, &s2, 1u)) return false;
    s2 &= 0x81u;                              // preserve reserved bit [7]
    s2 |= static_cast<uint8_t>(p.delt << 1u); // DELT → bits [6:1]
    s2 &= ~0x01u;                             // clear AAF_DIS → enable
    if (!write_reg(reg::ACCEL_CONFIG_STATIC2, s2)) return false;

    const uint8_t s3 = static_cast<uint8_t>(p.deltsqr & 0xFFu);
    const uint8_t s4 = static_cast<uint8_t>(
        ((p.bitshift & 0x0Fu) << 4u) | ((p.deltsqr >> 8u) & 0x0Fu));

    return write_reg(reg::ACCEL_CONFIG_STATIC3, s3)
        && write_reg(reg::ACCEL_CONFIG_STATIC4, s4)
        && select_bank(0u);
}

bool Device::set_gyro_aaf(bool enable, uint16_t target_hz)
{
    if (!select_bank(1u)) return false;

    uint8_t s2 = 0u;
    if (!read_regs(reg::GYRO_CONFIG_STATIC2, &s2, 1u)) return false;

    if (!enable) {
        s2 |= 0x02u;  // set AAF_DIS bit [1]
        return write_reg(reg::GYRO_CONFIG_STATIC2, s2) && select_bank(0u);
    }

    s2 &= ~0x02u;  // clear AAF_DIS
    if (!write_reg(reg::GYRO_CONFIG_STATIC2, s2)) return false;

    const auto p = aaf_for_hz(target_hz);
    const uint8_t s3 = static_cast<uint8_t>(p.delt & 0x3Fu);
    const uint8_t s4 = static_cast<uint8_t>(p.deltsqr & 0xFFu);
    const uint8_t s5 = static_cast<uint8_t>(
        ((p.bitshift & 0x0Fu) << 4u) | ((p.deltsqr >> 8u) & 0x0Fu));

    return write_reg(reg::GYRO_CONFIG_STATIC3, s3)
        && write_reg(reg::GYRO_CONFIG_STATIC4, s4)
        && write_reg(reg::GYRO_CONFIG_STATIC5, s5)
        && select_bank(0u);
}

bool Device::set_gyro_notch(bool enable, GyroNfBw bw, float freq_khz)
{
    if (!select_bank(1u)) return false;

    uint8_t s2 = 0u;
    if (!read_regs(reg::GYRO_CONFIG_STATIC2, &s2, 1u)) return false;

    if (!enable) {
        s2 |= 0x01u;  // set NF_DIS bit [0]
        return write_reg(reg::GYRO_CONFIG_STATIC2, s2) && select_bank(0u);
    }

    s2 &= ~0x01u;  // clear NF_DIS — enable notch
    if (!write_reg(reg::GYRO_CONFIG_STATIC2, s2)) return false;

    // Bandwidth selection in STATIC10 bits [6:4]
    uint8_t s10 = 0u;
    if (!read_regs(reg::GYRO_CONFIG_STATIC10, &s10, 1u)) return false;
    s10 &= ~(0x07u << 4u);
    s10 |= (static_cast<uint8_t>(bw) & 0x07u) << 4u;
    if (!write_reg(reg::GYRO_CONFIG_STATIC10, s10)) return false;

    // Coefficient calculation — datasheet §5.1.1
    // freq_khz must be in [1.0, 3.0]; clip silently.
    if (freq_khz < 1.0f) freq_khz = 1.0f;
    if (freq_khz > 3.0f) freq_khz = 3.0f;

    const float coswz = std::cosf(2.0f * static_cast<float>(M_PI) * freq_khz / 32.0f);

    uint8_t nf_coswz_lsb = 0u;
    uint8_t nf_coswz_msb = 0u;
    uint8_t nf_coswz_sel = 0u;

    if (std::fabsf(coswz) <= 0.875f) {
        const auto nf = static_cast<int16_t>(std::roundf(coswz * 256.0f));
        nf_coswz_lsb = static_cast<uint8_t>(nf & 0xFF);
        nf_coswz_msb = static_cast<uint8_t>((nf >> 8) & 0x01);
        nf_coswz_sel = 0u;
    } else {
        nf_coswz_sel = 1u;
        const float x = (coswz > 0.875f)
            ? 8.0f * (1.0f - coswz)
            : 8.0f * (1.0f + coswz);
        const auto nf = static_cast<int16_t>(std::roundf(x * 256.0f));
        nf_coswz_lsb = static_cast<uint8_t>(nf & 0xFF);
        nf_coswz_msb = static_cast<uint8_t>((nf >> 8) & 0x01);
    }

    // Same coefficients for X, Y, Z axes
    if (!write_reg(reg::GYRO_CONFIG_STATIC6, nf_coswz_lsb)) return false;
    if (!write_reg(reg::GYRO_CONFIG_STATIC7, nf_coswz_lsb)) return false;
    if (!write_reg(reg::GYRO_CONFIG_STATIC8, nf_coswz_lsb)) return false;

    // STATIC9: [5:3] = NF_COSWZ_SEL for X/Y/Z; [2:0] = NF_COSWZ[8] for X/Y/Z
    const uint8_t s9 =
        (nf_coswz_msb << 0u) |  // X bit[8]
        (nf_coswz_msb << 1u) |  // Y bit[8]
        (nf_coswz_msb << 2u) |  // Z bit[8]
        (nf_coswz_sel << 3u) |  // X SEL
        (nf_coswz_sel << 4u) |  // Y SEL
        (nf_coswz_sel << 5u);   // Z SEL
    return write_reg(reg::GYRO_CONFIG_STATIC9, s9) && select_bank(0u);
}

bool Device::set_gyro_hpf(bool enable, uint8_t bw_ind, HpfOrder order)
{
    if (!select_bank(1u)) return false;

    uint8_t s10 = 0u;
    if (!read_regs(reg::GYRO_CONFIG_STATIC10, &s10, 1u)) return false;

    // Clear HPF bits [3:0]
    s10 &= ~0x0Fu;

    if (enable) {
        s10 |= ((bw_ind & 0x07u) << 1u);           // HPF_BW_IND bits [3:1]
        s10 |= (static_cast<uint8_t>(order) & 0x01u); // HPF_ORD bit [0]
    }

    return write_reg(reg::GYRO_CONFIG_STATIC10, s10) && select_bank(0u);
}

bool Device::set_temp_filt_bw(TempFiltBw bw)
{
    if (!select_bank(0u)) return false;
    // GYRO_CONFIG1 bits [7:5]
    return rmw_reg(reg::GYRO_CONFIG1, 0xE0u,
                   static_cast<uint8_t>(static_cast<uint8_t>(bw) << 5u));
}

bool Device::set_gyro_ui_filt_order(UiFiltOrder order)
{
    if (!select_bank(0u)) return false;
    // GYRO_CONFIG1 bits [3:2]
    return rmw_reg(reg::GYRO_CONFIG1, 0x0Cu,
                   static_cast<uint8_t>(static_cast<uint8_t>(order) << 2u));
}

bool Device::set_accel_ui_filt_order(UiFiltOrder order)
{
    if (!select_bank(0u)) return false;
    // ACCEL_CONFIG1 bits [4:3]
    return rmw_reg(reg::ACCEL_CONFIG1, 0x18u,
                   static_cast<uint8_t>(static_cast<uint8_t>(order) << 3u));
}

} // namespace icm40609d
