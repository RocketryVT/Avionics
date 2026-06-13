#pragma once

// gps/types.hpp — shared types for the GPS driver stack
//
// Included by nmea_parser.hpp, ubx_parser.hpp, and gps_driver.hpp.
// Can also be included on its own when only the data types are needed.

#include <cstdint>
#include <string_view>

namespace gps {

// ===============================================================================
// Enumerations
// ===============================================================================

// UBX port identifiers (CFG-PRT portID field).
enum class Port : uint8_t {
    I2C   = 0,
    UART1 = 1,
    UART2 = 2,
    USB   = 3,
    SPI   = 4,
};

// Input protocol mask (CFG-PRT inProtoMask).
enum class InProto : uint16_t {
    UBX  = (1u << 0),
    NMEA = (1u << 1),
    RTCM = (1u << 5),
};
constexpr InProto operator|(InProto a, InProto b) noexcept {
    return static_cast<InProto>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

// Output protocol mask (CFG-PRT outProtoMask).
enum class OutProto : uint16_t {
    UBX  = (1u << 0),
    NMEA = (1u << 1),
};
constexpr OutProto operator|(OutProto a, OutProto b) noexcept {
    return static_cast<OutProto>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

// Fix type from UBX-NAV-PVT fixType field.
enum class FixType : uint8_t {
    None     = 0,
    DR       = 1,      // dead reckoning only
    Fix2D    = 2,
    Fix3D    = 3,
    GnssDR   = 4,      // GNSS + dead reckoning combined
    TimeOnly = 5,
};

// Carrier-phase RTK solution status (NAV-PVT flags bits [6:5]).
enum class CarrierSolution : uint8_t {
    None     = 0,
    FloatRTK = 1,
    FixedRTK = 2,
};

// NMEA GGA fix-quality indicator (field 6).
enum class FixQuality : uint8_t {
    Invalid   = '0',
    GPS       = '1',
    DGPS      = '2',
    PPS       = '3',
    RTK       = '4',
    FloatRTK  = '5',
    Estimated = '6',
    Manual    = '7',
    Simulated = '8',
};

// NMEA RMC positioning mode (field 12).
enum class FixMode : uint8_t {
    None         = 'N',
    Autonomous   = 'A',
    Differential = 'D',
    Estimated    = 'E',
};

// Provenance for Coordinate::vel_*_mms.
enum class NedVelocitySource : uint8_t {
    None,
    UbxNavPvt,          // Direct receiver-reported NED velocity.
    NmeaPositionDelta,  // Calculated from consecutive NMEA lat/lon/alt fixes.
};

// ===============================================================================
// Raw UBX payload structs
//
// packed + little-endian.  Always memcpy from rx buffer into these — never
// access the rx buffer through a reinterpret_cast.
// ===============================================================================

// UBX-NAV-PVT (class 0x01, id 0x07) — 92-byte payload.
struct [[gnu::packed]] NavPvt {
    uint32_t iTOW;          // GPS time of week, ms
    uint16_t year;
    uint8_t  month;         // 1–12
    uint8_t  day;           // 1–31
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;         // bit0=validDate  bit1=validTime  bit2=fullyResolved
    uint32_t tAcc;          // time accuracy estimate, ns
    int32_t  nano;          // UTC sub-second fraction, ns  (–1e9..+1e9)
    uint8_t  fixType;
    uint8_t  flags;         // bit0=gnssFixOK  bits[6:5]=carrSoln
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;           // longitude, deg × 1e-7
    int32_t  lat;           // latitude,  deg × 1e-7
    int32_t  height;        // height above ellipsoid, mm
    int32_t  hMSL;          // height above MSL, mm
    uint32_t hAcc;          // horizontal accuracy estimate, mm
    uint32_t vAcc;          // vertical accuracy estimate, mm
    int32_t  velN;          // NED north velocity, mm/s
    int32_t  velE;          // NED east  velocity, mm/s
    int32_t  velD;          // NED down  velocity, mm/s
    int32_t  gSpeed;        // 2-D ground speed, mm/s
    int32_t  headMot;       // heading of motion, deg × 1e-5
    uint32_t sAcc;          // speed accuracy estimate, mm/s
    uint32_t headAcc;       // heading accuracy estimate, deg × 1e-5
    uint16_t pDOP;          // position DOP × 0.01
    uint8_t  flags3;
    uint8_t  reserved0[5];
    int32_t  headVeh;       // heading of vehicle, deg × 1e-5 (ADR/UDR only)
    int16_t  magDec;        // magnetic declination, deg × 1e-2
    uint16_t magAcc;        // magnetic declination accuracy, deg × 1e-2
};
static_assert(sizeof(NavPvt) == 92);

// UBX-NAV-HPPOSLLH (class 0x01, id 0x14) — 36-byte payload.
// Combine with NavPvt for full precision; lat/lon accurate to ~1 cm.
struct [[gnu::packed]] NavHpPosLlh {
    uint8_t  version;
    uint8_t  reserved0[2];
    uint8_t  flags;         // bit1 = invalidLlh  (1 = position invalid, discard)
    uint32_t iTOW;
    int32_t  lon;           // longitude, deg × 1e-7
    int32_t  lat;           // latitude,  deg × 1e-7
    int32_t  height;        // height above ellipsoid, mm
    int32_t  hMSL;          // height above MSL, mm
    int8_t   lonHp;         // high-precision lon fraction, deg × 1e-9  (–99..+99)
    int8_t   latHp;         // high-precision lat fraction, deg × 1e-9
    int8_t   heightHp;      // high-precision ellipsoid height fraction, 0.1 mm
    int8_t   hMSLHp;        // high-precision MSL height fraction, 0.1 mm
    uint32_t hAcc;          // horizontal accuracy, mm × 0.1
    uint32_t vAcc;          // vertical accuracy,   mm × 0.1
};
static_assert(sizeof(NavHpPosLlh) == 36);

// ===============================================================================
// Coordinate — the unified decoded output written by both parsers
// ===============================================================================

struct Coordinate {
    // Position
    double   latitude     = 0.0;    // degrees WGS-84
    double   longitude    = 0.0;
    float    altitude     = 0.0f;   // metres MSL
    bool     valid        = false;
    bool     hp_valid     = false;  // true when NAV-HPPOSLLH has been merged in

    // Kinematics
    float    speed_mps    = 0.0f;   // 2-D ground speed, m/s
    float    course_deg   = 0.0f;   // course over ground, degrees true (0–360)

    // NED velocity. Check ned_velocity_source to see whether this is direct
    // receiver output or calculated by the parser.
    int32_t  vel_north_mms = 0;     // mm/s positive north
    int32_t  vel_east_mms  = 0;
    int32_t  vel_down_mms  = 0;     // positive = descending (useful for apogee detection)
    NedVelocitySource ned_velocity_source = NedVelocitySource::None;

    // Accuracy estimates — UBX only; zero when using NMEA
    uint32_t h_acc_mm     = 0;
    uint32_t v_acc_mm     = 0;

    // Time
    uint32_t utc_ms       = 0;      // milliseconds since midnight
    uint16_t utc_year     = 0;      // e.g. 2026  (0 = unknown)
    uint8_t  utc_month    = 0;      // 1–12
    uint8_t  utc_day      = 0;      // 1–31
    uint32_t gps_tow_ms   = 0;      // GPS time-of-week, ms (UBX only)

    // Fix classification
    FixType         fix_type    = FixType::None;
    CarrierSolution carr_soln   = CarrierSolution::None;
    FixQuality      fix_quality = FixQuality::Invalid;
    FixMode         fix_mode    = FixMode::None;

    // Dilution of precision
    float    hdop         = 0.0f;   // horizontal DOP (NMEA GGA or UBX NAV-DOP)
    float    vdop         = 0.0f;   // vertical DOP   (UBX NAV-DOP only)
    float    pdop         = 0.0f;   // position DOP   (UBX NAV-PVT pDOP)

    // Satellite tracking — UBX NAV-SAT only; zero with NMEA
    int      satellites   = 0;
    uint8_t  best_cno     = 0;      // highest C/N0 among tracked SVs, dBHz
    uint8_t  num_sv_used  = 0;      // count of SVs flagged svUsed
};

// ===============================================================================
// Diagnostics — counters maintained by both parsers
// ===============================================================================

struct Diagnostics {
    uint32_t ubx_frames     = 0;  // UBX frames successfully verified
    uint32_t ubx_pvt        = 0;  // NAV-PVT messages decoded
    uint32_t ubx_hp         = 0;  // NAV-HPPOSLLH messages decoded
    uint32_t ubx_dop        = 0;  // NAV-DOP messages decoded
    uint32_t ubx_odo        = 0;  // NAV-ODO messages decoded
    uint32_t ubx_ack        = 0;  // ACK-ACK responses received
    uint32_t ubx_nak        = 0;  // ACK-NAK responses received
    uint32_t nmea_good      = 0;  // NMEA sentences with valid checksum
    uint32_t nmea_bad_cksum = 0;  // NMEA sentences with bad checksum
};

// ===============================================================================
// Navigation helpers — free functions, no state
// ===============================================================================

// Haversine distance in metres between two WGS-84 positions.
[[nodiscard]] double distance_between(double lat1, double lon1,
                                      double lat2, double lon2) noexcept;

// Bearing in degrees clockwise from north (0 = N, 90 = E).
[[nodiscard]] double course_to(double lat1, double lon1,
                                double lat2, double lon2) noexcept;

// 16-point cardinal direction string from a bearing in degrees.
[[nodiscard]] std::string_view cardinal(float course) noexcept;

// Human-readable fix label: "RTK", "FRTK", "3D", "2D", "DR", "---".
[[nodiscard]] std::string_view fix_label(const Coordinate& c) noexcept;

} // namespace gps
