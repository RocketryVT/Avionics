#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace icm40609d {

// ---------------------------------------------------------------------------
// HAL transport — injected at construction; same pattern as ism330dlc
// ---------------------------------------------------------------------------

/// Write a single register byte.
using WriteRegFn = bool (*)(void* ctx, uint8_t reg, uint8_t val);

/// Read `len` consecutive bytes starting at `reg` (auto-increment).
using ReadRegsFn = bool (*)(void* ctx, uint8_t reg, uint8_t* buf, size_t len);

struct Transport {
    void*       ctx;
    WriteRegFn  write_reg;
    ReadRegsFn  read_regs;
};

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Gyroscope full-scale range (GYRO_CONFIG0 bits [7:5]).
enum class GyroRange : uint8_t {
    dps_2000   = 0,  ///< ±2000 dps — 16.4 LSB/(°/s)   (default)
    dps_1000   = 1,  ///< ±1000 dps — 32.8 LSB/(°/s)
    dps_500    = 2,  ///< ±500  dps — 65.5 LSB/(°/s)
    dps_250    = 3,  ///< ±250  dps — 131  LSB/(°/s)
    dps_125    = 4,  ///< ±125  dps — 262  LSB/(°/s)
    dps_62_5   = 5,  ///< ±62.5 dps — 524  LSB/(°/s)
    dps_31_25  = 6,  ///< ±31.25 dps
    dps_15_625 = 7,  ///< ±15.625 dps
};

/// Accelerometer full-scale range (ACCEL_CONFIG0 bits [7:5]).
enum class AccelRange : uint8_t {
    g16    = 0,  ///< ±16 g  — 2048 LSB/g   (default)
    g8     = 1,  ///< ±8  g  — 4096 LSB/g
    g4     = 2,  ///< ±4  g  — 8192 LSB/g
    g2     = 3,  ///< ±2  g  — 16384 LSB/g
    g1     = 4,  ///< ±1  g
    g0_5   = 5,  ///< ±0.5 g
    g0_25  = 6,  ///< ±0.25 g
    g0_125 = 7,  ///< ±0.125 g
};

/// Output data rate shared by both sensors (low nibble of CONFIG0 registers).
enum class ODR : uint8_t {
    hz_32k  = 0x01,
    hz_16k  = 0x02,
    hz_8k   = 0x03,
    hz_4k   = 0x04,
    hz_2k   = 0x05,
    hz_1k   = 0x06,  ///< default
    hz_200  = 0x07,
    hz_100  = 0x08,
    hz_50   = 0x09,
    hz_25   = 0x0A,
    hz_12_5 = 0x0B,
    hz_500  = 0x0F,
};

/// UI anti-alias filter (AAF) — 3 dB cutoff targets; nearest profile selected.
enum class AafHz : uint16_t {
    hz_42   =   42,
    hz_84   =   84,
    hz_126  =  126,
    hz_213  =  213,
    hz_258  =  258,
    hz_303  =  303,
    hz_450  =  450,   
    hz_997  =  997,
    hz_1220 = 1220,
    hz_max  = 3979,
};

/// UI low-pass filter order (GYRO_CONFIG1 bits [3:2], ACCEL_CONFIG1 bits [4:3]).
enum class UiFiltOrder : uint8_t {
    order_1st = 0,  ///< Lowest latency
    order_2nd = 1,  
    order_3rd = 2,
};

/// Temperature signal low-pass filter bandwidth (GYRO_CONFIG1 bits [7:5]).
enum class TempFiltBw : uint8_t {
    hz_4000 = 0,  ///< 0.125 ms latency (default)
    hz_170  = 1,
    hz_82   = 2,
    hz_40   = 3,
    hz_20   = 4,
    hz_10   = 5,
    hz_5    = 6,
};

/// Gyroscope notch-filter bandwidth (GYRO_CONFIG_STATIC10 bits [6:4]).
enum class GyroNfBw : uint8_t {
    hz_1449 = 0,
    hz_680  = 1,
    hz_329  = 2,
    hz_162  = 3,  
    hz_80   = 4,
    hz_40   = 5,
    hz_20   = 6,
    hz_10   = 7,
};

/// Gyroscope high-pass filter order.
enum class HpfOrder : uint8_t {
    order_1st = 0,
    order_2nd = 1,
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

struct Config {
    GyroRange    gyro_range     = GyroRange::dps_2000;
    AccelRange   accel_range    = AccelRange::g16;
    ODR          gyro_odr       = ODR::hz_1k;
    ODR          accel_odr      = ODR::hz_1k;

    // Anti-alias filter — set to 0 to disable
    uint16_t     accel_aaf_hz   = 450;
    uint16_t     gyro_aaf_hz    = 450;

    // UI low-pass filter order
    UiFiltOrder  gyro_ui_order  = UiFiltOrder::order_2nd;
    UiFiltOrder  accel_ui_order = UiFiltOrder::order_2nd;

    // Temperature DLPF
    TempFiltBw   temp_filt_bw   = TempFiltBw::hz_4000;

    // Notch filter on gyro (set freq_khz to 0 to disable)
    bool         gyro_notch_en  = true;
    GyroNfBw     gyro_nf_bw     = GyroNfBw::hz_162;
    float        gyro_notch_khz = 1.0f;  ///< Must be in [1.0, 3.0] kHz

    // High-pass filter on gyro (hpf_bw_ind=0 to disable)
    bool         gyro_hpf_en    = true;
    uint8_t      gyro_hpf_bw    = 1;     ///< bits [3:1] of STATIC10
    HpfOrder     gyro_hpf_order = HpfOrder::order_1st;

    /// Optional blocking delay — provide e.g. vTaskDelay wrapper for FreeRTOS.
    void (*delay_ms)(uint32_t) = nullptr;
};

// ---------------------------------------------------------------------------
// Output types
// ---------------------------------------------------------------------------

struct Sample {
    float accel_x{};  ///< m/s²
    float accel_y{};
    float accel_z{};
    float gyro_x{};   ///< °/s
    float gyro_y{};
    float gyro_z{};
    float temp_c{};   ///< °C
};

// ---------------------------------------------------------------------------
// Device driver
// ---------------------------------------------------------------------------

class Device {
public:
    static constexpr uint8_t kWhoAmIExpected = 0x3Bu;

    explicit Device(Transport transport) noexcept;

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /// Verify WHO_AM_I, soft-reset, apply Config.
    /// Returns false on comm error or unexpected device ID.
    [[nodiscard]] bool initialize(Config const& cfg = {});

    /// Software reset; brings all registers to POR defaults.
    [[nodiscard]] bool reset();

    // -----------------------------------------------------------------------
    // Runtime configuration
    // -----------------------------------------------------------------------

    [[nodiscard]] bool set_gyro_range(GyroRange range);
    [[nodiscard]] bool set_accel_range(AccelRange range);
    [[nodiscard]] bool set_gyro_odr(ODR odr);
    [[nodiscard]] bool set_accel_odr(ODR odr);

    // -----------------------------------------------------------------------
    // Data acquisition
    // -----------------------------------------------------------------------

    /// Returns true if the data-ready bit (INT_STATUS bit 3) is set.
    [[nodiscard]] bool data_ready() const;

    /// Burst-read all 14 sensor bytes (temp + accel + gyro) and convert to SI.
    [[nodiscard]] bool read_sample(Sample& out) const;

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    [[nodiscard]] bool     read_who_am_i(uint8_t& value) const;
    [[nodiscard]] GyroRange  current_gyro_range()  const noexcept { return m_gyro_range; }
    [[nodiscard]] AccelRange current_accel_range() const noexcept { return m_accel_range; }
    [[nodiscard]] ODR        current_gyro_odr()    const noexcept { return m_gyro_odr; }
    [[nodiscard]] ODR        current_accel_odr()   const noexcept { return m_accel_odr; }

private:
    // -- I/O -----------------------------------------------------------------
    [[nodiscard]] bool write_reg (uint8_t reg, uint8_t val)               const;
    [[nodiscard]] bool read_regs (uint8_t reg, uint8_t* buf, size_t len)  const;
    [[nodiscard]] bool rmw_reg   (uint8_t reg, uint8_t mask, uint8_t bits) const;

    // -- Bank switching ------------------------------------------------------
    [[nodiscard]] bool select_bank(uint8_t bank) const;

    // -- Filter helpers (private; called from initialize) --------------------
    [[nodiscard]] bool set_accel_aaf(bool enable, uint16_t target_hz);
    [[nodiscard]] bool set_gyro_aaf (bool enable, uint16_t target_hz);
    [[nodiscard]] bool set_gyro_notch(bool enable, GyroNfBw bw, float freq_khz);
    [[nodiscard]] bool set_gyro_hpf  (bool enable, uint8_t bw_ind, HpfOrder order);
    [[nodiscard]] bool set_temp_filt_bw(TempFiltBw bw);
    [[nodiscard]] bool set_gyro_ui_filt_order(UiFiltOrder order);
    [[nodiscard]] bool set_accel_ui_filt_order(UiFiltOrder order);

    // -- Data ----------------------------------------------------------------
    Transport   m_transport;
    GyroRange   m_gyro_range  = GyroRange::dps_2000;
    AccelRange  m_accel_range = AccelRange::g16;
    ODR         m_gyro_odr    = ODR::hz_1k;
    ODR         m_accel_odr   = ODR::hz_1k;
};

} // namespace icm40609d
