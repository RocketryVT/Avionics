#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace lis3mdl {

// ---------------------------------------------------------------------------
// HAL transport — inject at construction time; no virtual dispatch
// ---------------------------------------------------------------------------

using WriteRegFn  = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t val);
using ReadRegsFn  = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);

struct Transport {
    void*        ctx;
    WriteRegFn   write_reg;
    ReadRegsFn   read_regs;
};

// ---------------------------------------------------------------------------
// Enumerations (all scoped to avoid polluting namespace)
// ---------------------------------------------------------------------------

/// Full-scale range selection (CTRL_REG2 FS[1:0])
enum class Range : uint8_t {
    gauss_4  = 0b00,   ///< ±4 gauss  — 6842 LSB/gauss (default)
    gauss_8  = 0b01,   ///< ±8 gauss  — 3421 LSB/gauss
    gauss_12 = 0b10,   ///< ±12 gauss — 2281 LSB/gauss
    gauss_16 = 0b11,   ///< ±16 gauss — 1711 LSB/gauss
};

/// XY (and Z) performance mode (CTRL_REG1 OM[1:0], CTRL_REG4 OMZ[1:0])
enum class PerformanceMode : uint8_t {
    low_power   = 0b00,
    medium      = 0b01,
    high        = 0b10,
    ultra_high  = 0b11,
};

/// Output data rate + FAST_ODR (CTRL_REG1 bits [4:1])
/// Encoding: bits [3:1] = DO[2:0], bit [0] = FAST_ODR
enum class DataRate : uint8_t {
    hz_0_625  = 0b0000,  ///<   0.625 Hz
    hz_1_25   = 0b0010,  ///<   1.25 Hz
    hz_2_5    = 0b0100,  ///<   2.5 Hz
    hz_5      = 0b0110,  ///<   5 Hz
    hz_10     = 0b1000,  ///<   10 Hz
    hz_20     = 0b1010,  ///<   20 Hz
    hz_40     = 0b1100,  ///<   40 Hz
    hz_80     = 0b1110,  ///<   80 Hz
    hz_155    = 0b0001,  ///<  155 Hz  (FAST_ODR + UHP, default)
    hz_300    = 0b0011,  ///<  300 Hz  (FAST_ODR + HP)
    hz_560    = 0b0101,  ///<  560 Hz  (FAST_ODR + MP)
    hz_1000   = 0b0111,  ///< 1000 Hz  (FAST_ODR + LP)
};

/// Operating mode (CTRL_REG3 MD[1:0])
enum class OperationMode : uint8_t {
    continuous  = 0b00,
    single      = 0b01,
    power_down  = 0b11,
};

// ---------------------------------------------------------------------------
// Configuration passed to initialize()
// ---------------------------------------------------------------------------

struct Config {
    Range           range     = Range::gauss_4;
    DataRate        data_rate = DataRate::hz_155;            ///< 155 Hz default (FAST_ODR sets perf automatically)
    PerformanceMode perf_mode = PerformanceMode::ultra_high; ///< Applied for non-FAST_ODR rates
    OperationMode   op_mode   = OperationMode::continuous;
    /// Optional blocking delay — called after soft-reset to let the chip settle.
    /// Pass e.g. `sleep_ms` (Pico SDK) or `vTaskDelay`-wrapper; nullptr skips the wait.
    void (*delay_ms)(uint32_t) = nullptr;
};

// ---------------------------------------------------------------------------
// Interrupt configuration
// ---------------------------------------------------------------------------

struct IntConfig {
    bool enable_x   = false;   ///< Interrupt on X-axis threshold
    bool enable_y   = false;   ///< Interrupt on Y-axis threshold
    bool enable_z   = false;   ///< Interrupt on Z-axis threshold
    bool active_high = true;   ///< INT pin polarity (true = active high)
    bool latch       = false;  ///< Latch INT pin until INT_SRC is read
    bool enable      = false;  ///< Assert INT pin when triggered
};

// ---------------------------------------------------------------------------
// Output sample
// ---------------------------------------------------------------------------

struct Sample {
    float x_gauss{};
    float y_gauss{};
    float z_gauss{};
};

// ---------------------------------------------------------------------------
// Device driver
// ---------------------------------------------------------------------------

class Device {
public:
    static constexpr uint8_t kDefaultAddress = 0x1C;
    static constexpr uint8_t kWhoAmIExpected = 0x3D;

    /// Sensitivity in LSB/gauss indexed by Range enum value (0..3)
    static constexpr std::array<float, 4> kSensitivity = {
        6842.0f,  // gauss_4
        3421.0f,  // gauss_8
        2281.0f,  // gauss_12
        1711.0f,  // gauss_16
    };

    explicit Device(Transport transport, uint8_t device_address = kDefaultAddress);

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /// Verify WHO_AM_I, reset, then apply config.  Returns false on comm error
    /// or unexpected device ID.
    [[nodiscard]] bool initialize(Config const& cfg = {});

    /// Software reset (CTRL_REG2 SOFT_RST).  Waits ~10 ms for the chip to
    /// settle — the caller must supply a blocking delay via delay_ms.
    [[nodiscard]] bool reset(void (*delay_ms)(uint32_t) = nullptr);

    // -----------------------------------------------------------------------
    // Runtime configuration
    // -----------------------------------------------------------------------

    /// Set full-scale range and update the internal sensitivity cache.
    [[nodiscard]] bool set_range(Range range);

    /// Set output data rate.  For FAST_ODR rates (hz_155/300/560/1000) the
    /// corresponding performance mode is set automatically, matching the
    /// Adafruit driver behaviour.
    [[nodiscard]] bool set_data_rate(DataRate rate);

    /// Explicitly set XY and Z performance mode without changing data rate.
    [[nodiscard]] bool set_performance_mode(PerformanceMode mode);

    /// Set operating mode (continuous / single / power-down).
    [[nodiscard]] bool set_operation_mode(OperationMode mode);

    /// Enable or disable the built-in self-test (CTRL_REG1 ST bit).
    [[nodiscard]] bool set_self_test(bool enable);

    // -----------------------------------------------------------------------
    // Interrupt support
    // -----------------------------------------------------------------------

    /// Write INT_THS (15-bit unsigned raw threshold, MSB of high byte forced 0).
    [[nodiscard]] bool set_int_threshold(uint16_t raw);

    /// Read INT_THS.
    [[nodiscard]] bool get_int_threshold(uint16_t& raw) const;

    /// Configure INT_CFG register.
    [[nodiscard]] bool config_interrupt(IntConfig const& cfg);

    // -----------------------------------------------------------------------
    // Data acquisition
    // -----------------------------------------------------------------------

    /// True when new XYZ data is available (STATUS ZYXDA bit).
    [[nodiscard]] bool data_ready() const;

    /// Read XYZ sample, converting to gauss using the current range.
    [[nodiscard]] bool read_sample(Sample& sample) const;

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    [[nodiscard]] bool read_who_am_i(uint8_t& value) const;

    [[nodiscard]] Range           current_range()            const noexcept { return m_range; }
    [[nodiscard]] DataRate        current_data_rate()        const noexcept { return m_data_rate; }
    [[nodiscard]] PerformanceMode current_performance_mode() const noexcept { return m_perf_mode; }

private:
    [[nodiscard]] bool write_reg(uint8_t reg, uint8_t val)                         const;
    [[nodiscard]] bool read_regs(uint8_t reg, uint8_t* buf, size_t len)            const;
    [[nodiscard]] bool rmw_reg(uint8_t reg, uint8_t mask, uint8_t bits)            const;

    Transport m_transport;
    uint8_t   m_device_address;

    // Cached state — kept in sync with hardware writes
    Range           m_range     = Range::gauss_4;
    DataRate        m_data_rate = DataRate::hz_155;
    PerformanceMode m_perf_mode = PerformanceMode::ultra_high;
};

} // namespace lis3mdl
