#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

namespace gps {

// -- Enumerations --------------------------------------------------------------

// Fix quality from NMEA GGA field 6.
enum class FixQuality : uint8_t {
    Invalid   = '0',
    GPS       = '1',  // Standard GPS
    DGPS      = '2',  // Differential GPS
    PPS       = '3',  // Precise Point Positioning
    RTK       = '4',  // Real-Time Kinematic (fixed integer)
    FloatRTK  = '5',  // Real-Time Kinematic (float)
    Estimated = '6',  // Dead reckoning / estimated
    Manual    = '7',  // Manual input mode
    Simulated = '8',  // Simulation mode
};

// Positioning mode from NMEA RMC field 12.
enum class FixMode : uint8_t {
    None         = 'N',
    Autonomous   = 'A',
    Differential = 'D',
    Estimated    = 'E',
};

// UBX fix type from NAV-PVT byte 20.
enum class FixType : uint8_t {
    None    = 0,
    DR      = 1,  // Dead reckoning only
    Fix2D   = 2,
    Fix3D   = 3,
    GnssDR  = 4,  // GNSS + dead reckoning
    TimeOnly = 5,
};

// Carrier-phase solution extracted from NAV-PVT flags bits [6:5].
// Only valid when fixType >= Fix3D and gnssFixOK is set.
enum class CarrierSolution : uint8_t {
    None     = 0,
    FloatRTK = 1,
    FixedRTK = 2,
};

// -- Raw UBX payload structs ---------------------------------------------------
// Exposed so callers can access full precision without going through Coordinate.

// UBX-NAV-PVT (class 0x01, id 0x07) — 92-byte payload, little-endian.
struct [[gnu::packed]] NavPvt {
    uint32_t iTOW;       // GPS time of week, ms
    uint16_t year;       // UTC year
    uint8_t  month;      // UTC month (1–12)
    uint8_t  day;        // UTC day   (1–31)
    uint8_t  hour;       // UTC hour  (0–23)
    uint8_t  min;        // UTC minute
    uint8_t  sec;        // UTC second
    uint8_t  valid;      // bit0=validDate bit1=validTime bit2=fullyResolved bit3=validMag
    uint32_t tAcc;       // Time accuracy estimate, ns
    int32_t  nano;       // Sub-second fraction, ns  (-1e9..+1e9)
    uint8_t  fixType;    // 0=none 1=DR 2=2D 3=3D 4=GNSS+DR 5=time-only
    uint8_t  flags;      // bit0=gnssFixOK bit1=diffSoln bits[6:5]=carrSoln
    uint8_t  flags2;
    uint8_t  numSV;      // Number of SVs used in fix
    int32_t  lon;        // Longitude,  deg × 1e-7
    int32_t  lat;        // Latitude,   deg × 1e-7
    int32_t  height;     // Height above ellipsoid, mm
    int32_t  hMSL;       // Height above MSL, mm
    uint32_t hAcc;       // Horizontal accuracy estimate, mm
    uint32_t vAcc;       // Vertical accuracy estimate, mm
    int32_t  velN;       // NED north velocity, mm/s
    int32_t  velE;       // NED east  velocity, mm/s
    int32_t  velD;       // NED down  velocity, mm/s
    int32_t  gSpeed;     // Ground speed (2-D), mm/s
    int32_t  headMot;    // Heading of motion, deg × 1e-5
    uint32_t sAcc;       // Speed accuracy estimate, mm/s
    uint32_t headAcc;    // Heading accuracy estimate, deg × 1e-5
    uint16_t pDOP;       // Position DOP × 0.01
    uint8_t  flags3;
    uint8_t  reserved0[5];
    int32_t  headVeh;    // Heading of vehicle, deg × 1e-5 (fusion only)
    int16_t  magDec;     // Magnetic declination, deg × 1e-2
    uint16_t magAcc;     // Magnetic declination accuracy, deg × 1e-2
};
static_assert(sizeof(NavPvt) == 92);

// UBX-NAV-HPPOSLLH (class 0x01, id 0x14) — 36-byte payload, little-endian.
// High-precision geodetic position. Combine with NavPvt for velocity/time.
// True position:
//   lat_deg = (lat × 1e-7) + (latHp  × 1e-9)
//   lon_deg = (lon × 1e-7) + (lonHp  × 1e-9)
//   hMSL_m  = (hMSL + hMSLHp × 0.1) × 1e-3
struct [[gnu::packed]] NavHpPosLlh {
    uint8_t  version;     // 0
    uint8_t  reserved0[2];
    uint8_t  flags;       // bit1 = invalidLlh (1 = position invalid)
    uint32_t iTOW;        // GPS time of week, ms
    int32_t  lon;         // Longitude, deg × 1e-7
    int32_t  lat;         // Latitude,  deg × 1e-7
    int32_t  height;      // Height above ellipsoid, mm
    int32_t  hMSL;        // Height above MSL, mm
    int8_t   lonHp;       // High-precision lon fraction, deg × 1e-9  (-99..+99)
    int8_t   latHp;       // High-precision lat fraction, deg × 1e-9  (-99..+99)
    int8_t   heightHp;    // High-precision ellipsoid height fraction, 0.1 mm
    int8_t   hMSLHp;      // High-precision MSL height fraction, 0.1 mm
    uint32_t hAcc;        // Horizontal accuracy, mm × 0.1
    uint32_t vAcc;        // Vertical accuracy,   mm × 0.1
};
static_assert(sizeof(NavHpPosLlh) == 36);

// -- Coordinate ----------------------------------------------------------------

struct Coordinate {
    // -- Position -------------------------------------------------------------
    // Sources: NMEA GGA, UBX NAV-PVT, UBX NAV-HPPOSLLH (highest precision).
    // When a NAV-HPPOSLLH frame has been merged, hp_valid is true and the
    // lat/lon/altitude fields reflect the high-precision values.
    double   latitude;    // degrees  (double for sub-mm hp precision)
    double   longitude;   // degrees
    float    altitude;    // meters MSL
    bool     valid;       // true when a usable fix is present
    bool     hp_valid;    // true when NAV-HPPOSLLH has been merged

    // -- Kinematic -------------------------------------------------------------
    float    speed_mps;   // ground speed, m/s
    float    course_deg;  // course over ground, degrees true (0–360)

    // -- UTC -------------------------------------------------------------------
    uint32_t utc_ms;      // milliseconds since midnight (includes sub-ms from nano)
    uint16_t utc_year;    // e.g. 2026  (0 if unknown)
    uint8_t  utc_month;   // 1–12       (0 if unknown)
    uint8_t  utc_day;     // 1–31       (0 if unknown)

    // -- 3-D NED velocity ------------------------------------------------------
    // UBX NAV-PVT only; all zero when using NMEA.
    // vel_down_mms < 0 = ascending; > 0 = descending (apogee detection).
    int32_t  vel_north_mms;
    int32_t  vel_east_mms;
    int32_t  vel_down_mms;

    // -- Accuracy estimates ----------------------------------------------------
    // UBX NAV-PVT only; 0 when using NMEA.
    uint32_t h_acc_mm;    // horizontal accuracy estimate, mm
    uint32_t v_acc_mm;    // vertical accuracy estimate, mm

    // -- Fix classification ----------------------------------------------------
    FixType          fix_type;     // UBX fix type (None with NMEA)
    CarrierSolution  carr_soln;    // RTK carrier solution (None with NMEA)
    FixQuality       fix_quality;  // NMEA GGA field 6 (Invalid with UBX-only)
    FixMode          fix_mode;     // NMEA RMC field 12 (None with UBX-only)

    // -- Dilution of Precision -------------------------------------------------
    float    hdop;   // horizontal DOP (from NMEA GGA or UBX NAV-DOP)
    float    vdop;   // vertical DOP   (from UBX NAV-DOP only)
    float    pdop;   // position DOP   (from UBX NAV-PVT pDOP field)

    // -- Satellite tracking ----------------------------------------------------
    int      satellites;   // SVs used in fix
    uint8_t  best_cno;     // highest C/N0 among tracked SVs, dBHz (NAV-SAT)
    uint8_t  num_sv_used;  // SVs flagged svUsed in NAV-SAT
};

// -- GPSParser -----------------------------------------------------------------

class GPSParser {
public:
    GPSParser();

    // Feed one character (NMEA or UBX byte stream — both protocols interleaved).
    void parse(char c);

    // Current decoded coordinate. Always valid to read; check coord.valid for fix.
    [[nodiscard]] const Coordinate &coordinate() const noexcept { return coord_; }

    // Convenience: true when coord_.valid is set.
    [[nodiscard]] bool hasFix() const noexcept { return coord_.valid; }

    // Returns the label string for the current fix, e.g. "RTK", "FRTK", "3D", "2D", "DR", "---".
    [[nodiscard]] std::string_view fixLabel() const noexcept;

    // -- Diagnostic counters ---------------------------------------------------
    [[nodiscard]] uint32_t ubxFrameCount()       const noexcept { return ubx_frame_count_; }
    [[nodiscard]] uint32_t ubxPvtCount()         const noexcept { return ubx_pvt_count_; }
    [[nodiscard]] uint32_t ubxHpCount()          const noexcept { return ubx_hp_count_; }
    [[nodiscard]] uint32_t nmeaGoodSentences()   const noexcept { return nmea_good_; }
    [[nodiscard]] uint32_t nmeaFailedChecksums() const noexcept { return nmea_bad_; }

    // -- Static navigation helpers ---------------------------------------------
    // Haversine distance in metres between two lat/lon positions.
    [[nodiscard]] static double distanceBetween(double lat1, double lon1,
                                                double lat2, double lon2) noexcept;
    // Bearing in degrees (0 = N, 90 = E) from position 1 to position 2.
    [[nodiscard]] static double courseTo(double lat1, double lon1,
                                         double lat2, double lon2) noexcept;
    // 16-point cardinal direction string from a course angle in degrees.
    [[nodiscard]] static std::string_view cardinal(float course) noexcept;

private:
    // -- Parser state machine --------------------------------------------------
    enum class State : uint8_t {
        Idle,
        // NMEA
        NmeaType, NmeaData, NmeaCk1, NmeaCk2,
        // UBX
        UbxSync2, UbxClass, UbxId, UbxLen1, UbxLen2,
        UbxPayload, UbxCkA, UbxCkB,
    };

    State    state_           = State::Idle;

    // -- NMEA ------------------------------------------------------------------
    // Fixed-size buffer avoids heap allocation; 128 bytes covers all sentences.
    static constexpr std::size_t NMEA_BUF = 128;
    std::array<char, NMEA_BUF> nmea_buf_{};
    std::size_t                 nmea_len_    = 0;
    uint8_t                     nmea_ck_     = 0;  // running XOR
    uint8_t                     nmea_parsed_ = 0;  // two-nibble accumulator

    uint32_t nmea_good_ = 0;
    uint32_t nmea_bad_  = 0;

    void nmea_reset() noexcept;
    void processSentence() noexcept;
    void parseGGA() noexcept;
    void parseRMC() noexcept;

    // -- UBX -------------------------------------------------------------------
    uint8_t  ubx_class_       = 0;
    uint8_t  ubx_id_          = 0;
    uint16_t ubx_length_      = 0;
    uint16_t ubx_payload_idx_ = 0;
    uint8_t  ubx_ck_a_        = 0;
    uint8_t  ubx_ck_b_        = 0;

    // Buffer sized for the largest accepted message (NavPvt = 92 bytes).
    static constexpr uint16_t UBX_BUF = 92;
    std::array<uint8_t, UBX_BUF> ubx_buf_{};

    // NAV-SAT streaming accumulators (reset per message).
    uint8_t  ubx_best_cno_    = 0;
    uint8_t  ubx_num_sv_used_ = 0;

    // Pending high-precision position waiting to be merged on the next NAV-PVT.
    bool           hp_pending_ = false;
    NavHpPosLlh    hp_pending_frame_{};

    uint32_t ubx_frame_count_ = 0;
    uint32_t ubx_pvt_count_   = 0;
    uint32_t ubx_hp_count_    = 0;

    // Returns true if (ubx_class_, ubx_id_, ubx_length_) is a message we handle.
    [[nodiscard]] bool ubxAccepted() const noexcept;

    void processUbxMessage() noexcept;
    void parseNavPvt()       noexcept;
    void parseNavHpPosLlh()  noexcept;
    void parseNavDop()       noexcept;
    void parseNavSat()       noexcept;
    void navSatStreamByte(uint8_t b) noexcept;

    // Little-endian payload readers.
    [[nodiscard]] uint16_t u16(int off) const noexcept;
    [[nodiscard]] uint32_t u32(int off) const noexcept;
    [[nodiscard]] int32_t  i32(int off) const noexcept;

    // Output state.
    Coordinate coord_{};
};

} // namespace gps
