#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace lsm6dso32 {

// ---------------------------------------------------------------------------
// HAL transport — injected at initialize(); no platform headers required here
// ---------------------------------------------------------------------------

using WriteRegFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t val);
using ReadRegsFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);

struct Transport {
    void*       ctx;
    WriteRegFn  write_reg;
    ReadRegsFn  read_regs;
};

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Accelerometer output data rate (CTRL1_XL bits [7:4]).
enum class AccelODR : uint8_t {
    power_down = 0x00,
    hz_1_6     = 0xB0,  ///< Low-power only
    hz_12_5    = 0x10,
    hz_26      = 0x20,
    hz_52      = 0x30,
    hz_104     = 0x40,  ///< Default
    hz_208     = 0x50,
    hz_416     = 0x60,
    hz_833     = 0x70,
    hz_1667    = 0x80,
    hz_3333    = 0x90,
    hz_6667    = 0xA0,
};

/// Accelerometer full-scale range (CTRL1_XL bits [3:2]).
/// LSM6DSO32 supports ±4/8/16/32 g (higher than ISM330DHCX's ±2/4/8/16 g).
enum class AccelRange : uint8_t {
    g4  = 0x00,  ///< ±4 g  — 0.122 mg/LSB  (default)
    g32 = 0x04,  ///< ±32 g — 0.976 mg/LSB
    g8  = 0x08,  ///< ±8 g  — 0.244 mg/LSB
    g16 = 0x0C,  ///< ±16 g — 0.488 mg/LSB
};

/// Gyroscope output data rate (CTRL2_G bits [7:4]).
enum class GyroODR : uint8_t {
    power_down = 0x00,
    hz_12_5    = 0x10,
    hz_26      = 0x20,
    hz_52      = 0x30,
    hz_104     = 0x40,  ///< Default
    hz_208     = 0x50,
    hz_416     = 0x60,
    hz_833     = 0x70,
    hz_1667    = 0x80,
    hz_3333    = 0x90,
    hz_6667    = 0xA0,
};

/// Gyroscope full-scale range (CTRL2_G bits [3:0]).
enum class GyroRange : uint8_t {
    dps_125  = 0x02,  ///< ±125 dps  — 4.375 mdps/LSB  (FS_125 bit)
    dps_250  = 0x00,  ///< ±250 dps  — 8.75 mdps/LSB
    dps_500  = 0x04,  ///< ±500 dps  — 17.5 mdps/LSB
    dps_1000 = 0x08,  ///< ±1000 dps — 35.0 mdps/LSB
    dps_2000 = 0x0C,  ///< ±2000 dps — 70.0 mdps/LSB
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

struct Config {
    AccelRange  accel_range  = AccelRange::g16;
    GyroRange   gyro_range   = GyroRange::dps_2000;
    AccelODR    accel_odr    = AccelODR::hz_104;
    GyroODR     gyro_odr     = GyroODR::hz_104;
    bool        disable_i3c  = true;   ///< Disable I3C interface (recommended)
    uint8_t     address      = kDefaultAddress;
    void (*delay_ms)(uint32_t) = nullptr;

    static constexpr uint8_t kDefaultAddress     = 0x6B;  ///< SA0 = 1
    static constexpr uint8_t kAlternativeAddress = 0x6A;  ///< SA0 = 0
};

// ---------------------------------------------------------------------------
// Output types
// ---------------------------------------------------------------------------

/// Data-ready flags polled from STATUS_REG before reading.
struct DataReady {
    bool accel{};
    bool gyro{};
    bool temp{};
};

/// One 6-axis + temperature sample burst-read in a single transaction.
struct Sample {
    float accel_x{};  ///< X acceleration (g)
    float accel_y{};  ///< Y acceleration (g)
    float accel_z{};  ///< Z acceleration (g)
    float gyro_x{};   ///< X angular rate (dps)
    float gyro_y{};   ///< Y angular rate (dps)
    float gyro_z{};   ///< Z angular rate (dps)
    float temp_c{};   ///< Die temperature (°C)
};

// ---------------------------------------------------------------------------
// FIFO types
// ---------------------------------------------------------------------------

enum class FifoTag : uint8_t {
    empty         = 0x00,
    gyro_nc       = 0x01,
    accel_nc      = 0x02,
    temperature   = 0x03,
    timestamp     = 0x04,
    other         = 0xFF,
};

struct FifoEntry {
    FifoTag tag{};
    uint8_t data[6]{};
};

// ---------------------------------------------------------------------------
// LSM6DSO32 6-axis IMU driver (3-axis accel ±4/8/16/32 g, 3-axis gyro ±125..2000 dps)
//
// Usage:
//   lsm6dso32::Device imu;
//   imu.initialize(transport);
//   lsm6dso32::Sample s;
//   imu.read_sample(s);
// ---------------------------------------------------------------------------

class Device {
public:
    static constexpr uint8_t kDefaultAddress = 0x6B;
    static constexpr uint8_t kWhoAmIExpected = 0x6C;

    /// Verify WHO_AM_I, configure registers.  Returns false on comm error or bad ID.
    [[nodiscard]] bool initialize(Transport t, Config cfg = {});

    /// Software reset via CTRL3_C SW_RESET.  Calls delay_ms(10) if provided.
    [[nodiscard]] bool reset(void (*delay_ms)(uint32_t) = nullptr);

    // -----------------------------------------------------------------------
    // Data acquisition
    // -----------------------------------------------------------------------

    /// Poll STATUS_REG for data-ready flags (non-blocking).
    [[nodiscard]] DataReady data_ready() const;

    /// Burst-read all 14 bytes (temp + gyro + accel) in one I2C transaction.
    [[nodiscard]] bool read_sample(Sample& s) const;

    [[nodiscard]] bool read_temperature(float& temp_c) const;
    [[nodiscard]] bool read_gyroscope(float& x, float& y, float& z) const;
    [[nodiscard]] bool read_accelerometer(float& x, float& y, float& z) const;

    // -----------------------------------------------------------------------
    // FIFO
    // -----------------------------------------------------------------------

    [[nodiscard]] uint16_t fifo_count() const;
    [[nodiscard]] bool fifo_pop(FifoEntry& entry) const;

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    [[nodiscard]] bool read_who_am_i(uint8_t& value) const;

    [[nodiscard]] AccelODR   current_accel_odr()   const noexcept { return m_cfg.accel_odr;   }
    [[nodiscard]] GyroODR    current_gyro_odr()    const noexcept { return m_cfg.gyro_odr;    }
    [[nodiscard]] AccelRange current_accel_range() const noexcept { return m_cfg.accel_range; }
    [[nodiscard]] GyroRange  current_gyro_range()  const noexcept { return m_cfg.gyro_range;  }

private:
    [[nodiscard]] bool write_reg(uint8_t reg, uint8_t val)              const;
    [[nodiscard]] bool read_regs(uint8_t reg, uint8_t* buf, size_t len) const;
    [[nodiscard]] bool rmw_reg(uint8_t reg, uint8_t mask, uint8_t bits) const;

    [[nodiscard]] float accel_sensitivity() const noexcept;
    [[nodiscard]] float gyro_sensitivity()  const noexcept;

    Transport m_transport{};
    Config    m_cfg{};
};

} // namespace lsm6dso32
