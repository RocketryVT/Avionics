#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace adxl375 {

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

/// Output data rate / bandwidth (BW_RATE register bits [3:0]).
enum class BandWidth : uint8_t {
    hz_0_10  = 0x00,
    hz_0_20  = 0x01,
    hz_0_39  = 0x02,
    hz_0_78  = 0x03,
    hz_1_56  = 0x04,
    hz_3_13  = 0x05,
    hz_6_25  = 0x06,
    hz_12_5  = 0x07,
    hz_25    = 0x08,
    hz_50    = 0x09,
    hz_100   = 0x0A,
    hz_200   = 0x0B,  ///< Recommended default
    hz_400   = 0x0C,
    hz_800   = 0x0D,
    hz_1600  = 0x0E,
    hz_3200  = 0x0F,
};

/// Power control modes (POWER_CTL register).
enum class PowerMode : uint8_t {
    standby     = 0x00,  ///< Standby — no measurements
    measurement = 0x08,  ///< Measure bit (D3) = 1 — normal measurement mode
};

// ---------------------------------------------------------------------------
// Configuration passed to initialize()
// ---------------------------------------------------------------------------

struct Config {
    BandWidth  bandwidth   = BandWidth::hz_200;
    PowerMode  power_mode  = PowerMode::measurement;
    uint8_t    address     = kDefaultAddress;
    void (*delay_ms)(uint32_t) = nullptr;  ///< Optional delay (used after init)

    static constexpr uint8_t kDefaultAddress   = 0x1D;  ///< ALT ADDRESS pin high
    static constexpr uint8_t kAlternateAddress = 0x53;  ///< ALT ADDRESS pin low
};

// ---------------------------------------------------------------------------
// Output type
// ---------------------------------------------------------------------------

/// Calibrated 3-axis acceleration in m/s².
struct Sample {
    float x{};
    float y{};
    float z{};
};

// ---------------------------------------------------------------------------
// ADXL375 high-g accelerometer driver (±200 g)
//
// Usage:
//   adxl375::Device accel;
//   accel.initialize(transport);
//   adxl375::Sample s;
//   accel.read_sample(s);   // x,y,z in m/s²
// ---------------------------------------------------------------------------

class Device {
public:
    static constexpr uint8_t kDefaultAddress = 0x1D;
    static constexpr uint8_t kDeviceId       = 0xE5;  ///< Expected DEVID register value

    /// Verify device ID, configure bandwidth and power mode.
    /// Returns false on comm error or unexpected device ID.
    [[nodiscard]] bool initialize(Transport t, Config cfg = {});

    /// Read the raw DEVID register (should be 0xE5).
    [[nodiscard]] bool read_device_id(uint8_t& id) const;

    /// Read calibrated acceleration (m/s²) from all three axes in one burst.
    [[nodiscard]] bool read_sample(Sample& sample) const;

    /// Update bandwidth / output data rate (BW_RATE register bits [3:0]).
    [[nodiscard]] bool set_bandwidth(BandWidth bw);

    /// Update power mode (POWER_CTL register).
    [[nodiscard]] bool set_power_mode(PowerMode mode);

private:
    [[nodiscard]] bool write_reg(uint8_t reg, uint8_t val) const;
    [[nodiscard]] bool read_regs(uint8_t reg, uint8_t* buf, size_t len) const;
    [[nodiscard]] bool rmw_reg(uint8_t reg, uint8_t mask, uint8_t bits) const;

    Transport m_transport{};
    Config    m_cfg{};
};

} // namespace adxl375
