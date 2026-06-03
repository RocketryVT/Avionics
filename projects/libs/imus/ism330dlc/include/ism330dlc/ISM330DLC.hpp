#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace ism330dlc {

// ---------------------------------------------------------------------------
// HAL transport — injected at construction; no virtual dispatch
// ---------------------------------------------------------------------------

using WriteRegFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t val);
using ReadRegsFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);

struct Transport {
    void*       ctx;
    WriteRegFn  write_reg;
    ReadRegsFn  read_regs;
};

// ---------------------------------------------------------------------------
// Enumerations — raw register byte values for direct masked writes
// ---------------------------------------------------------------------------

/// Accelerometer output data rate (CTRL1_XL bits [7:4]).
enum class AccelODR : uint8_t {
    power_down = 0x00,
    hz_1_6     = 0xB0,  ///< Low-power mode only
    hz_12_5    = 0x10,
    hz_26      = 0x20,
    hz_52      = 0x30,
    hz_104     = 0x40,  ///< 104 Hz (default)
    hz_208     = 0x50,
    hz_416     = 0x60,
    hz_833     = 0x70,
    hz_1660    = 0x80,
    hz_3330    = 0x90,
    hz_6660    = 0xA0,
};

/// Gyroscope output data rate (CTRL2_G bits [7:4]).
enum class GyroODR : uint8_t {
    power_down = 0x00,
    hz_12_5    = 0x10,
    hz_26      = 0x20,
    hz_52      = 0x30,
    hz_104     = 0x40,  ///< 104 Hz (default)
    hz_208     = 0x50,
    hz_416     = 0x60,
    hz_833     = 0x70,
    hz_1660    = 0x80,
    hz_3330    = 0x90,
    hz_6660    = 0xA0,
};

/// Accelerometer full-scale range (CTRL1_XL bits [3:2]).
/// Note the non-sequential encoding from the datasheet.
enum class AccelRange : uint8_t {
    g2  = 0x00,  ///< ±2g  — 0.061 mg/LSB
    g16 = 0x04,  ///< ±16g — 0.488 mg/LSB
    g4  = 0x08,  ///< ±4g  — 0.122 mg/LSB
    g8  = 0x0C,  ///< ±8g  — 0.244 mg/LSB
};

/// Gyroscope full-scale range (CTRL2_G bits [3:1], covering FS_G[3:2] + FS_125[1]).
/// ±125 dps uses the dedicated FS_125 bit; all others use FS_G with FS_125=0.
enum class GyroRange : uint8_t {
    dps_125  = 0x02,  ///< ±125 dps  — 4.375 mdps/LSB (FS_125 bit)
    dps_250  = 0x00,  ///< ±250 dps  — 8.75 mdps/LSB
    dps_500  = 0x04,  ///< ±500 dps  — 17.5 mdps/LSB
    dps_1000 = 0x08,  ///< ±1000 dps — 35.0 mdps/LSB
    dps_2000 = 0x0C,  ///< ±2000 dps — 70.0 mdps/LSB (default)
};

// ---------------------------------------------------------------------------
// Configuration passed to initialize()
// ---------------------------------------------------------------------------

struct Config {
    AccelRange  accel_range = AccelRange::g16;
    GyroRange   gyro_range  = GyroRange::dps_2000;
    AccelODR    accel_odr   = AccelODR::hz_104;
    GyroODR     gyro_odr    = GyroODR::hz_104;
    /// Optional blocking delay called after soft-reset.
    void (*delay_ms)(uint32_t) = nullptr;
};

// ---------------------------------------------------------------------------
// Output types
// ---------------------------------------------------------------------------

/// STATUS_REG data-ready flags polled before reading.
struct DataReady {
    bool accel{};
    bool gyro{};
    bool temp{};
};

/// One 6-axis + temperature sample converted to SI units.
struct Sample {
    float accel_x{};  ///< X acceleration (g)
    float accel_y{};  ///< Y acceleration (g)
    float accel_z{};  ///< Z acceleration (g)
    float gyro_x{};   ///< X angular rate (deg/s)
    float gyro_y{};   ///< Y angular rate (deg/s)
    float gyro_z{};   ///< Z angular rate (deg/s)
    float temp_c{};   ///< Chip temperature (°C)
};

// ---------------------------------------------------------------------------
// Device driver
// ---------------------------------------------------------------------------

class Device {
public:
    static constexpr uint8_t kDefaultAddress = 0x6A;
    static constexpr uint8_t kWhoAmIExpected = 0x6B;  // ISM330DHCX (successor, same register map)

    explicit Device(Transport transport, uint8_t device_address = kDefaultAddress);

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /// Verify WHO_AM_I, soft-reset, apply Config.  Returns false on comm error
    /// or unexpected device ID.
    [[nodiscard]] bool initialize(Config const& cfg = {});

    /// Software reset via CTRL3_C SW_RESET.  Calls delay_ms(10) if provided.
    [[nodiscard]] bool reset(void (*delay_ms)(uint32_t) = nullptr);

    // -----------------------------------------------------------------------
    // Runtime configuration
    // -----------------------------------------------------------------------

    /// Set accelerometer output data rate.  AccelODR::power_down disables.
    [[nodiscard]] bool set_accel_odr(AccelODR odr);

    /// Set gyroscope output data rate.  GyroODR::power_down disables.
    [[nodiscard]] bool set_gyro_odr(GyroODR odr);

    /// Set accelerometer full-scale range; updates internal sensitivity cache.
    [[nodiscard]] bool set_accel_range(AccelRange range);

    /// Set gyroscope full-scale range (includes ±125 dps via FS_125 bit).
    [[nodiscard]] bool set_gyro_range(GyroRange range);

    /// Enable/disable accelerometer built-in self-test (CTRL5_C ST_XL bits).
    [[nodiscard]] bool set_accel_self_test(bool enable);

    /// Enable/disable gyroscope built-in self-test (CTRL5_C ST_G bits).
    [[nodiscard]] bool set_gyro_self_test(bool enable);

    // -----------------------------------------------------------------------
    // Power control — preserves last active ODR across disable/re-enable
    // -----------------------------------------------------------------------

    /// Power accel back on at the last active ODR.
    [[nodiscard]] bool enable_accel();

    /// Power accel down, saving current ODR for enable_accel().
    [[nodiscard]] bool disable_accel();

    /// Power gyro back on at the last active ODR.
    [[nodiscard]] bool enable_gyro();

    /// Power gyro down, saving current ODR for enable_gyro().
    [[nodiscard]] bool disable_gyro();

    // -----------------------------------------------------------------------
    // Data acquisition
    // -----------------------------------------------------------------------

    /// Poll STATUS_REG for new-data flags (non-blocking).
    [[nodiscard]] DataReady data_ready() const;

    /// Burst-read all 14 bytes (temp + gyro + accel) in one I2C transaction.
    [[nodiscard]] bool read_sample(Sample& sample) const;

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    [[nodiscard]] bool read_who_am_i(uint8_t& value) const;

    [[nodiscard]] AccelODR   current_accel_odr()   const noexcept { return m_accel_odr; }
    [[nodiscard]] GyroODR    current_gyro_odr()    const noexcept { return m_gyro_odr; }
    [[nodiscard]] AccelRange current_accel_range() const noexcept { return m_accel_range; }
    [[nodiscard]] GyroRange  current_gyro_range()  const noexcept { return m_gyro_range; }

private:
    [[nodiscard]] bool write_reg(uint8_t reg, uint8_t val)              const;
    [[nodiscard]] bool read_regs(uint8_t reg, uint8_t* buf, size_t len) const;
    [[nodiscard]] bool rmw_reg(uint8_t reg, uint8_t mask, uint8_t bits) const;

    Transport m_transport;
    uint8_t   m_device_address;

    // Cached state — kept in sync with hardware
    AccelRange  m_accel_range = AccelRange::g16;
    GyroRange   m_gyro_range  = GyroRange::dps_2000;
    AccelODR    m_accel_odr   = AccelODR::power_down;
    GyroODR     m_gyro_odr    = GyroODR::power_down;

    // Saved ODR for enable/disable round-trips
    AccelODR    m_accel_last_odr = AccelODR::hz_104;
    GyroODR     m_gyro_last_odr  = GyroODR::hz_104;
};

} // namespace ism330dlc
