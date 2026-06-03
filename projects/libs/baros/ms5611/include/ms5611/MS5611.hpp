#pragma once

#include <cstddef>
#include <cstdint>

namespace ms5611 {

// ---------------------------------------------------------------------------
// HAL transport — injected at initialize(); no platform headers required here
// ---------------------------------------------------------------------------

/// Send a single command byte to the device (e.g. reset, start conversion).
using CmdFn     = bool (*)(void* ctx, uint8_t cmd);

/// Send a command byte then receive `len` bytes (e.g. PROM read, ADC read).
using CmdReadFn = bool (*)(void* ctx, uint8_t cmd, uint8_t* buf, size_t len);

struct Transport {
    void*       ctx;
    CmdFn       cmd;
    CmdReadFn   cmd_read;
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

struct Config {
    float qnh_pa = 101325.f;               ///< Reference pressure for altitude (Pa)
    void (*delay_ms)(uint32_t) = nullptr;  ///< Optional blocking delay (used after reset)
};

// ---------------------------------------------------------------------------
// MS5611 barometer driver
//
// Usage (non-blocking alternating pipeline):
//   Device baro;
//   baro.initialize(transport);
//   // First conversion triggered inside initialize() — call read_adc / start_conversion
//   // in alternating pairs at >= 10 ms intervals (OSR-4096 takes 9.04 ms).
//
//   // Each tick:
//   uint32_t adc;
//   baro.read_adc(adc);                    // read result from previous conversion
//   baro.start_conversion(want_pressure);  // kick next conversion
//
//   // Once both D1 and D2 are fresh:
//   float pressure_pa, temp_c;
//   baro.calculate(D1, D2, pressure_pa, temp_c);
//   float alt_m = baro.altitude(pressure_pa);
// ---------------------------------------------------------------------------

class Device {
public:
    static constexpr uint8_t kDefaultAddress = 0x77;  ///< CSB pin -> GND

    /// Verify comms (PROM read), apply config; triggers first D2 conversion.
    /// Returns false if any PROM word read fails.
    [[nodiscard]] bool initialize(Transport t, Config cfg = {});

    /// Send a convert command.  pressure=true → D1 (OSR-4096), false → D2.
    [[nodiscard]] bool start_conversion(bool pressure) const;

    /// Read back the 24-bit ADC result from the previous conversion.
    [[nodiscard]] bool read_adc(uint32_t& out) const;

    /// Full 2nd-order temperature-compensated calculation (per datasheet AN520).
    /// D1 = raw pressure ADC, D2 = raw temperature ADC.
    void calculate(uint32_t D1, uint32_t D2,
                   float& pressure_pa, float& temp_c) const;

    /// ISA barometric altitude in metres MSL.
    float altitude(float pressure_pa) const;

private:
    [[nodiscard]] bool cmd(uint8_t c) const;
    [[nodiscard]] bool cmd_read(uint8_t c, uint8_t* buf, size_t len) const;

    Transport m_transport{};
    Config    m_cfg{};
    uint16_t  m_prom[8]{};   // factory calibration words C0..C7; use C1..C6
};

} // namespace ms5611
