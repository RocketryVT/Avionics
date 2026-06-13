#pragma once

// boards/devices.hpp — shared device registry for the avionics fleet.
//
// Two things live here, defined ONCE and reused by every board/project:
//   1. Model enums per peripheral category (ImuModel, BaroModel, ...).
//   2. Each model's INTRINSIC specs (ranges / max rate / resolution) via
//      spec_of(model). These are chip facts — they don't change per board.
//
// What does NOT live here: which devices a given firmware actually has and how
// they're wired. That is per-project and goes in board_profile.hpp as arrays of
// the *Instance structs below (model + bus + addr/CS + rate + role).
//
// Spec values are nominal datasheet figures meant for sizing/sanity, not a
// substitute for the datasheet — refine as needed.

#include <cstdint>
#include <cstddef>
#include <iterator>   // std::size (used by profiles to size their instance arrays)

namespace Board {

// -- Where a device is attached on the carrier/board --------------------------
enum class Bus : uint8_t { I2C0, I2C1, SPI0, SPI1, UART0, UART1, PIO0, PIO1 };

// -- Selectable<T> ------------------------------------------------------------
// A register field that exposes a discrete set of values (ODR, FSR, ...). Holds
// a pointer to a static constexpr table so the struct stays small/POD and works
// in constant expressions. Build with selectable(some_static_array).
//   spec_of(m).gyro.fsr.max()              -> largest selectable FSR
//   spec_of(m).accel.odr.supports(1000.f)  -> is 1 kHz a real setting?
//   spec_of(m).accel.odr.nearest(750.f)    -> closest supported value
template <class T>
struct Selectable {
    const T*    values;
    std::size_t count;

    constexpr T max() const {
        T m = values[0];
        for (std::size_t i = 1; i < count; ++i) if (values[i] > m) m = values[i];
        return m;
    }
    constexpr T min() const {
        T m = values[0];
        for (std::size_t i = 1; i < count; ++i) if (values[i] < m) m = values[i];
        return m;
    }
    constexpr bool supports(T v) const {
        for (std::size_t i = 0; i < count; ++i) if (values[i] == v) return true;
        return false;
    }
    constexpr T nearest(T v) const {
        T best = values[0];
        T bestd = v > best ? v - best : best - v;
        for (std::size_t i = 1; i < count; ++i) {
            T d = v > values[i] ? v - values[i] : values[i] - v;
            if (d < bestd) { bestd = d; best = values[i]; }
        }
        return best;
    }
};

template <class T, std::size_t N>
constexpr Selectable<T> selectable(const T (&arr)[N]) { return { arr, N }; }

// =============================================================================
// IMU (combo 6-axis: INDEPENDENT accel + gyro sub-sensors)
// =============================================================================
// Accel and gyro have separate ODR and FSR register fields, each exposing its
// own discrete set of values (see the IIM-42653 datasheet tables). So the spec
// models each sub-sensor as a Selectable ODR + Selectable FSR rather than a
// single "max rate / full scale".
enum class ImuModel : uint8_t { IIM42653, ICM40609D, ICM42688P, ICM20948 };

struct ImuAxisSpec {
    Selectable<float> odr_hz;   // selectable output data rates (Hz)
    Selectable<float> fsr;      // selectable full-scale ranges (accel: g, gyro: dps)
};
struct ImuSpec {
    const char* name;
    ImuAxisSpec accel;
    ImuAxisSpec gyro;
};

// Register-map option tables. IIM-42653 is verified against the datasheet; the
// other TDK parts follow the same family map — VERIFY per datasheet before relying
// on them for FSR/ODR validation.
namespace detail {
inline constexpr float IIM42653_ACC_ODR[] = {1.5625f,3.125f,6.25f,12.5f,25,50,100,200,500,1000,2000,4000,8000,16000,32000};
inline constexpr float IIM42653_ACC_FSR[] = {4,8,16,32};
inline constexpr float IIM42653_GYR_ODR[] = {12.5f,25,50,100,200,500,1000,2000,4000,8000,16000,32000};
inline constexpr float IIM42653_GYR_FSR[] = {31.25f,62.5f,125,250,500,1000,2000,4000};

inline constexpr float ICM40609_ACC_ODR[] = {1.5625f,3.125f,6.25f,12.5f,25,50,100,200,500,1000,2000,4000,8000}; // max 8 kHz
inline constexpr float ICM40609_ACC_FSR[] = {4,8,16,32};
inline constexpr float ICM40609_GYR_ODR[] = {12.5f,25,50,100,200,500,1000,2000,4000,8000};
inline constexpr float ICM40609_GYR_FSR[] = {31.25f,62.5f,125,250,500,1000,2000,4000};

inline constexpr float ICM42688_ACC_ODR[] = {1.5625f,3.125f,6.25f,12.5f,25,50,100,200,500,1000,2000,4000,8000,16000,32000};
inline constexpr float ICM42688_ACC_FSR[] = {2,4,8,16};
inline constexpr float ICM42688_GYR_ODR[] = {12.5f,25,50,100,200,500,1000,2000,4000,8000,16000,32000};
inline constexpr float ICM42688_GYR_FSR[] = {15.625f,31.25f,62.5f,125,250,500,1000,2000};

// ICM-20948 (older family, sample-rate-divider based) — FSR exact, ODR nominal.
inline constexpr float ICM20948_ACC_ODR[] = {12.5f,25,50,100,200,500,1000,2000,4500}; // VERIFY
inline constexpr float ICM20948_ACC_FSR[] = {2,4,8,16};
inline constexpr float ICM20948_GYR_ODR[] = {12.5f,25,50,100,200,500,1000,2000,4000,9000}; // VERIFY
inline constexpr float ICM20948_GYR_FSR[] = {250,500,1000,2000};
} // namespace detail

constexpr ImuSpec spec_of(ImuModel m) {
    switch (m) {
    case ImuModel::IIM42653: return { "IIM-42653",
        { selectable(detail::IIM42653_ACC_ODR), selectable(detail::IIM42653_ACC_FSR) },
        { selectable(detail::IIM42653_GYR_ODR), selectable(detail::IIM42653_GYR_FSR) } };
    case ImuModel::ICM40609D: return { "ICM-40609-D",
        { selectable(detail::ICM40609_ACC_ODR), selectable(detail::ICM40609_ACC_FSR) },
        { selectable(detail::ICM40609_GYR_ODR), selectable(detail::ICM40609_GYR_FSR) } };
    case ImuModel::ICM42688P: return { "ICM-42688-P",
        { selectable(detail::ICM42688_ACC_ODR), selectable(detail::ICM42688_ACC_FSR) },
        { selectable(detail::ICM42688_GYR_ODR), selectable(detail::ICM42688_GYR_FSR) } };
    case ImuModel::ICM20948: return { "ICM-20948",
        { selectable(detail::ICM20948_ACC_ODR), selectable(detail::ICM20948_ACC_FSR) },
        { selectable(detail::ICM20948_GYR_ODR), selectable(detail::ICM20948_GYR_FSR) } };
    }
    return { "unknown-imu", {}, {} };
}

struct ImuInstance {
    ImuModel    model;
    Bus         bus;
    uint8_t     addr_or_cs;     // I2C address, or CS pin for SPI
    // Configured per-sub-sensor settings (validate against spec_of(model)).
    float       accel_odr_hz;
    float       accel_fsr_g;
    float       gyro_odr_hz;
    float       gyro_fsr_dps;
    const char* role;           // e.g. "primary", "highg"
};

// =============================================================================
// High-g / dedicated accelerometer
// =============================================================================
enum class AccelModel : uint8_t { ADXL375 };

struct AccelSpec {
    const char* name;
    float       fs_g;           // full-scale (±g)
    uint32_t    max_odr_hz;
};

constexpr AccelSpec spec_of(AccelModel m) {
    switch (m) {
    case AccelModel::ADXL375: return { "ADXL375", 200.f, 3200 };
    }
    return { "unknown-accel", 0.f, 0 };
}

struct AccelInstance {
    AccelModel  model;
    Bus         bus;
    uint8_t     addr_or_cs;
    uint32_t    odr_hz;
    const char* role;
};

// =============================================================================
// Magnetometer
// =============================================================================
enum class MagModel : uint8_t { LIS3MDL, MMC5983MA };

struct MagSpec {
    const char* name;
    float       fs_gauss;       // full-scale (±gauss)
    uint32_t    max_odr_hz;
};

constexpr MagSpec spec_of(MagModel m) {
    switch (m) {
    case MagModel::LIS3MDL:   return { "LIS3MDL",    16.f, 1000 };
    case MagModel::MMC5983MA: return { "MMC5983MA",   8.f, 1000 };
    }
    return { "unknown-mag", 0.f, 0 };
}

struct MagInstance {
    MagModel    model;
    Bus         bus;
    uint8_t     addr_or_cs;
    uint32_t    odr_hz;
    const char* role;
};

// =============================================================================
// Barometer / pressure altimeter
// =============================================================================
enum class BaroModel : uint8_t { MS5607, MS5611, MPL3115A2 };

struct BaroSpec {
    const char* name;
    float       p_min_mbar;
    float       p_max_mbar;
    uint32_t    max_odr_hz;
};

constexpr BaroSpec spec_of(BaroModel m) {
    switch (m) {
    case BaroModel::MS5607:    return { "MS5607",      10.f, 1200.f, 200 };
    case BaroModel::MS5611:    return { "MS5611",      10.f, 1200.f, 200 };
    case BaroModel::MPL3115A2: return { "MPL3115A2",  500.f, 1100.f,  16 };
    }
    return { "unknown-baro", 0.f, 0.f, 0 };
}

struct BaroInstance {
    BaroModel   model;
    Bus         bus;
    uint8_t     addr_or_cs;
    uint32_t    odr_hz;
    const char* role;
};

// =============================================================================
// Packet radio
// =============================================================================
enum class RadioModel : uint8_t { SX1276, SX1262, RFM69HCW, SX1231H = RFM69HCW, LR1121 };

// Modulations mapped to binary positions
enum class ModulationMask : uint16_t {
    NONE     = 0,
    LORA     = 1 << 0,
    FSK      = 1 << 1,
    GFSK     = 1 << 2,
    OOK      = 1 << 3,
    MSK      = 1 << 4,
    GMSK     = 1 << 5,
    FLRC     = 1 << 6,
    LR_FHSS  = 1 << 7,
    SIGFOX   = 1 << 8,
    FOUR_FSK = 1 << 9   // 4-FSK for advanced protocols
};

// Spreading Factors as individual bits
enum class LoraSfMask : uint8_t {
    SF5  = 1 << 0,
    SF6  = 1 << 1,
    SF7  = 1 << 2,
    SF8  = 1 << 3,
    SF9  = 1 << 4,
    SF10 = 1 << 5,
    SF11 = 1 << 6,
    SF12 = 1 << 7
};

// Bandwidths as individual bits
enum class LoraBwMask : uint16_t {
    BW7_8   = 1 << 0,
    BW10_4  = 1 << 1,
    BW15_6  = 1 << 2,
    BW20_8  = 1 << 3,
    BW31_25 = 1 << 4,
    BW41_7  = 1 << 5,
    BW62_5  = 1 << 6,
    BW125   = 1 << 7,
    BW250   = 1 << 8,
    BW500   = 1 << 9,
    BW1000  = 1 << 10,
    BW1625  = 1 << 11
};

// Coding Rates as individual bits (including Gen2/Gen4 long interleaving)
enum class LoraCrMask : uint8_t {
    CR4_5      = 1 << 0,
    CR4_6      = 1 << 1,
    CR4_7      = 1 << 2,
    CR4_8      = 1 << 3,
    CR4_5_LONG = 1 << 4,
    CR4_6_LONG = 1 << 5,
    CR4_8_LONG = 1 << 6
};

constexpr LoraSfMask operator|(LoraSfMask a, LoraSfMask b) { 
    return static_cast<LoraSfMask>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b)); 
}
constexpr LoraBwMask operator|(LoraBwMask a, LoraBwMask b) { 
    return static_cast<LoraBwMask>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b)); 
}
constexpr LoraCrMask operator|(LoraCrMask a, LoraCrMask b) { 
    return static_cast<LoraCrMask>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b)); 
}
constexpr ModulationMask operator|(ModulationMask a, ModulationMask b) { 
    return static_cast<ModulationMask>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b)); 
}

constexpr bool operator&(LoraSfMask a, LoraSfMask b) { return (static_cast<uint8_t>(a) & static_cast<uint8_t>(b)) != 0; }
constexpr bool operator&(LoraBwMask a, LoraBwMask b) { return (static_cast<uint16_t>(a) & static_cast<uint16_t>(b)) != 0; }
constexpr bool operator&(LoraCrMask a, LoraCrMask b) { return (static_cast<uint8_t>(a) & static_cast<uint8_t>(b)) != 0; }
constexpr bool operator&(ModulationMask a, ModulationMask b) { return (static_cast<uint16_t>(a) & static_cast<uint16_t>(b)) != 0; }



namespace LoraPresets {
    // Gen 1 Capabilities (SX1276)
    constexpr LoraSfMask SF_GEN1 = LoraSfMask::SF6 | LoraSfMask::SF7 | LoraSfMask::SF8 | 
                                LoraSfMask::SF9 | LoraSfMask::SF10 | LoraSfMask::SF11 | LoraSfMask::SF12;
                    
    constexpr LoraBwMask BW_SUB_GHZ = LoraBwMask::BW7_8 | LoraBwMask::BW10_4 | LoraBwMask::BW15_6 | 
                                    LoraBwMask::BW20_8 | LoraBwMask::BW31_25 | LoraBwMask::BW41_7 | 
                                    LoraBwMask::BW62_5 | LoraBwMask::BW125 | LoraBwMask::BW250 | LoraBwMask::BW500;

    constexpr LoraBwMask BW_MULTI_BAND = BW_SUB_GHZ | LoraBwMask::BW1000 | LoraBwMask::BW1625;

    constexpr LoraCrMask CR_STANDARD = LoraCrMask::CR4_5 | LoraCrMask::CR4_6 | LoraCrMask::CR4_7 | LoraCrMask::CR4_8;

    // Gen 2/3 Capabilities (SX1262, LR1121) - Adds SF5
    constexpr LoraSfMask SF_GEN2_3 = LoraSfMask::SF5 | SF_GEN1;
}

namespace ModPresets {
    // Gen 1 Modulations (SX1276)
    constexpr ModulationMask GEN1_MODS = ModulationMask::LORA | ModulationMask::FSK | 
                                   ModulationMask::GFSK | ModulationMask::OOK | 
                                   ModulationMask::MSK;

    // Gen 2 Sub-GHz Modulations (SX1262 drops OOK/MSK native modes)
    constexpr ModulationMask GEN2_MODS = ModulationMask::LORA | ModulationMask::FSK | 
                                   ModulationMask::GFSK;

    // Pure Legacy FSK Radios (RFM69HCW uses Semtech SX1231H core)
    constexpr ModulationMask RFM69_MODS = ModulationMask::FSK | ModulationMask::GFSK | 
                                    ModulationMask::OOK;

    // Gen 3 "Edge" Radios (LR1121 adds LR-FHSS and Sigfox PHY)
    constexpr ModulationMask GEN3_MODS = ModulationMask::LORA | ModulationMask::FSK | 
                                   ModulationMask::GFSK | ModulationMask::LR_FHSS | 
                                   ModulationMask::SIGFOX;
}

enum class LoraGeneration : uint8_t { None = 0, Gen1 = 1, Gen2 = 2 };
enum class LoraSpreadingFactor : uint8_t { SF5 = 5, SF6 = 6, SF7 = 7, SF8 = 8, SF9 = 9, SF10 = 10, SF11 = 11, SF12 = 12 };
enum class LoraBandwidth : uint32_t { BW7_8 = 7800, BW10_4 = 10400, BW15_6 = 15600, BW20_8 = 20800, BW31_25 = 31250, BW41_7 = 41700, BW62_5 = 62500, BW125 = 125000, BW250 = 250000, BW500 = 500000 };
enum class LoraCodingRate : uint8_t { CR4_5 = 4, CR4_6 = 5, CR4_7 = 6, CR4_8 = 7 };
enum class LoraPreambleLength : uint16_t { PL3 = 3, PL4 = 4, PL5 = 5, PL6 = 6, PL7 = 7, PL8 = 8, PL12 = 12, PL16 = 16, PL24 = 24, PL32 = 32, PL48 = 48, PL64 = 64 };
enum class LoraSyncWord : uint8_t { Public = 0x34, Private = 0x12 };
enum class LoraPowerLevel : uint8_t { P0 = 0, P1 = 1, P2 = 2, P3 = 3, P4 = 4, P5 = 5, P6 = 6, P7 = 7 };
enum class MaxPayloadLength : uint8_t { L51 = 51, L115 = 115, L222 = 222, L255 = 255 };

struct RadioSpec {
    const char* name;
    float freq_min_mhz;
    float freq_max_mhz;
    uint8_t lora_generation; 
    bool has_rfo;            // Still needed to track physical PA hardware pinout paths
    
    // Condensed Capability Bitmasks
    ModulationMask supported_modulations; // Holds combined ModulationMask bits
    LoraSfMask  supported_sf;          // Holds combined LoraSfMask bits
    LoraBwMask supported_bw;          // Holds combined LoraBwMask bits
    LoraCrMask  supported_cr;          // Holds combined LoraCrMask bits
};

constexpr RadioSpec spec_of(RadioModel m) {
    switch (m) {
        case RadioModel::SX1276: 
            return { 
                "SX1276", 
                137.f, 
                1020.f,
                1,
                true,
                ModPresets::GEN1_MODS, 
                LoraPresets::SF_GEN1, 
                LoraPresets::BW_SUB_GHZ, 
                LoraPresets::CR_STANDARD 
            }; 

        case RadioModel::SX1262: 
            return { 
                "SX1262", 
                150.f,
                960.f,
                2,
                false,
                ModPresets::GEN2_MODS, 
                LoraPresets::SF_GEN2_3, 
                LoraPresets::BW_SUB_GHZ, 
                LoraPresets::CR_STANDARD 
            }; 

        case RadioModel::RFM69HCW: 
            return { 
                "SX1231H/RFM69HCW", 
                290.f, 
                1020.f, 
                0, 
                false, 
                ModPresets::RFM69_MODS, 
                static_cast<LoraSfMask>(0), 
                static_cast<LoraBwMask>(0), 
                static_cast<LoraCrMask>(0) 
            };

        case RadioModel::LR1121: 
            return { 
                "LR1121", 
                150.f, 
                2500.f, 
                3, 
                true,
                ModPresets::GEN3_MODS, 
                LoraPresets::SF_GEN2_3, 
                LoraPresets::BW_SUB_GHZ, 
                LoraPresets::CR_STANDARD 
            }; 
    }
    return { "unknown", 0.f, 0.f, 0, false, 
            ModulationMask::NONE, 
            static_cast<LoraSfMask>(0), 
            static_cast<LoraBwMask>(0), 
            static_cast<LoraCrMask>(0) };

}

struct RadioInstance {
    RadioModel  model;
    Bus         bus;
    uint8_t     cs_pin;
    float       freq_mhz;       // configured operating frequency
    const char* role;           // e.g. "915-lora", "433-gfsk"
};

// =============================================================================
// GNSS / GPS
// =============================================================================
enum class GpsModel : uint8_t { UbloxM10, MaxM10S, NeoM8Q, UbloxM9N, UbloxM8U, Generic };

struct GpsSpec {
    const char* name;
    uint16_t    max_nav_hz;
};

constexpr GpsSpec spec_of(GpsModel m) {
    switch (m) {
    case GpsModel::UbloxM10: return { "u-blox M10", 25 };
    case GpsModel::MaxM10S:  return { "MAX-M10S",   25 };
    case GpsModel::NeoM8Q:   return { "NEO-M8Q",    10 };
    case GpsModel::UbloxM9N: return { "u-blox M9",  10 };
    case GpsModel::UbloxM8U: return { "u-blox M8U", 18 };  // dead-reckoning M8
    case GpsModel::Generic:  return { "generic NMEA", 10 };
    }
    return { "unknown-gps", 0 };
}

struct GpsInstance {
    GpsModel    model;
    Bus         bus;            // UART0/UART1 (or I2C for DDC)
    uint32_t    baud;
    uint16_t    nav_hz;
    const char* role;
};

// =============================================================================
// RF path / antenna switch (passive N-way mux on an antenna line, GPIO-selected)
// =============================================================================
// No spec_of(): it's a passive mux. Characterized by how many ports it selects
// and the GPIO select bits (LSB first). e.g. a 4-way switch uses 2 select pins.
struct AntSwitchInstance {
    uint8_t     ways;           // number of selectable antenna ports (e.g. 4)
    uint8_t     sel_pins[2];    // select GPIOs, LSB first (2 bits -> up to 4-way)
    uint8_t     sel_count;      // number of select pins actually used
    const char* role;           // e.g. "gps", "radio"
};

// =============================================================================
// Generic instance-array queries (constexpr — usable in static_assert / if constexpr)
// =============================================================================
// Count instances of a given model in a profile's array, e.g.
//   static_assert(Board::count_model(Board::Radios, RadioModel::RFM69HCW) >= 1);
template <class Inst, class Model, std::size_t N>
constexpr int count_model(const Inst (&arr)[N], Model m) {
    int n = 0;
    for (std::size_t i = 0; i < N; ++i)
        if (arr[i].model == m) ++n;
    return n;
}

template <class Inst, class Model, std::size_t N>
constexpr bool has_model(const Inst (&arr)[N], Model m) {
    return count_model(arr, m) > 0;
}

} // namespace Board
