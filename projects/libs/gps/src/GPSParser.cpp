#include "gps/GPSParser.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <numbers>

namespace gps {

// -- Constructor ---------------------------------------------------------------

GPSParser::GPSParser() {
    coord_ = Coordinate{
        .latitude      = 0.0,
        .longitude     = 0.0,
        .altitude      = 0.0f,
        .valid         = false,
        .hp_valid      = false,
        .speed_mps     = 0.0f,
        .course_deg    = 0.0f,
        .utc_ms        = 0,
        .utc_year      = 0,
        .utc_month     = 0,
        .utc_day       = 0,
        .vel_north_mms = 0,
        .vel_east_mms  = 0,
        .vel_down_mms  = 0,
        .h_acc_mm      = 0,
        .v_acc_mm      = 0,
        .fix_type      = FixType::None,
        .carr_soln     = CarrierSolution::None,
        .fix_quality   = FixQuality::Invalid,
        .fix_mode      = FixMode::None,
        .hdop          = 0.0f,
        .vdop          = 0.0f,
        .pdop          = 0.0f,
        .satellites    = 0,
        .best_cno      = 0,
        .num_sv_used   = 0,
    };
}

// -- UBX payload read helpers (little-endian) ----------------------------------

uint16_t GPSParser::u16(int off) const noexcept {
    return static_cast<uint16_t>(ubx_buf_[off]) |
           static_cast<uint16_t>(static_cast<uint16_t>(ubx_buf_[off + 1]) << 8);
}

uint32_t GPSParser::u32(int off) const noexcept {
    return static_cast<uint32_t>(ubx_buf_[off])           |
           static_cast<uint32_t>(ubx_buf_[off + 1]) <<  8 |
           static_cast<uint32_t>(ubx_buf_[off + 2]) << 16 |
           static_cast<uint32_t>(ubx_buf_[off + 3]) << 24;
}

int32_t GPSParser::i32(int off) const noexcept {
    return static_cast<int32_t>(u32(off));
}

// -- Character-by-character parser ---------------------------------------------

void GPSParser::parse(char c) {
    const uint8_t b = static_cast<uint8_t>(c);

    switch (state_) {

    // -- Idle: detect NMEA '$' or UBX 0xB5 ------------------------------------
    case State::Idle:
        if (c == '$') {
            nmea_reset();
            state_ = State::NmeaType;
        } else if (b == 0xB5u) {
            state_ = State::UbxSync2;
        }
        break;

    // -- NMEA sentence type -----------------------------------------------------
    // Accumulates "$GNGGA," (type + leading comma) before switching to NmeaData.
    case State::NmeaType:
        if (c == '$') {
            nmea_reset();               // restart on embedded '$'
        } else if (c == '*') {
            state_ = State::NmeaCk1;
        } else if (c == ',') {
            nmea_ck_ ^= static_cast<uint8_t>(c);
            if (nmea_len_ < NMEA_BUF) nmea_buf_[nmea_len_++] = c;
            state_ = State::NmeaData;
        } else {
            nmea_ck_ ^= static_cast<uint8_t>(c);
            if (nmea_len_ < NMEA_BUF) nmea_buf_[nmea_len_++] = c;
        }
        break;

    // -- NMEA data fields -------------------------------------------------------
    case State::NmeaData:
        if (c == '*') {
            state_ = State::NmeaCk1;
        } else if (c == '$') {
            nmea_reset();
            state_ = State::NmeaType;
        } else {
            nmea_ck_ ^= static_cast<uint8_t>(c);
            if (nmea_len_ < NMEA_BUF) nmea_buf_[nmea_len_++] = c;
        }
        break;

    // -- NMEA checksum ----------------------------------------------------------
    case State::NmeaCk1:
        nmea_parsed_  = (c >= 'A') ? static_cast<uint8_t>(c - 'A' + 10)
                                   : static_cast<uint8_t>(c - '0');
        nmea_parsed_ *= 16u;
        state_ = State::NmeaCk2;
        break;

    case State::NmeaCk2: {
        const uint8_t lo = (c >= 'A') ? static_cast<uint8_t>(c - 'A' + 10)
                                      : static_cast<uint8_t>(c - '0');
        if ((nmea_parsed_ + lo) == nmea_ck_) {
            nmea_good_++;
            nmea_buf_[nmea_len_] = '\0';
            processSentence();
        } else {
            nmea_bad_++;
        }
        state_ = State::Idle;
        break;
    }

    // -- UBX framing -----------------------------------------------------------
    case State::UbxSync2:
        state_ = (b == 0x62u) ? State::UbxClass : State::Idle;
        break;

    case State::UbxClass:
        ubx_class_ = b;
        ubx_ck_a_  = b;
        ubx_ck_b_  = b;
        state_ = State::UbxId;
        break;

    case State::UbxId:
        ubx_id_   = b;
        ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
        state_ = State::UbxLen1;
        break;

    case State::UbxLen1:
        ubx_length_  = b;
        ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
        state_ = State::UbxLen2;
        break;

    case State::UbxLen2:
        ubx_length_ |= static_cast<uint16_t>(b) << 8;
        ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
        ubx_payload_idx_ = 0;
        // Reject unknown/oversized messages immediately — no need to buffer them.
        if (!ubxAccepted()) {
            state_ = State::Idle;
            break;
        }
        // Reset NAV-SAT streaming accumulators for each new frame.
        if (ubx_class_ == 0x01u && ubx_id_ == 0x35u) {
            ubx_best_cno_    = 0;
            ubx_num_sv_used_ = 0;
        }
        state_ = (ubx_length_ > 0) ? State::UbxPayload : State::UbxCkA;
        break;

    case State::UbxPayload:
        ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
        if (ubx_payload_idx_ < UBX_BUF)
            ubx_buf_[ubx_payload_idx_] = b;
        // NAV-SAT is too large to buffer fully — extract inline as bytes arrive.
        if (ubx_class_ == 0x01u && ubx_id_ == 0x35u)
            navSatStreamByte(b);
        if (++ubx_payload_idx_ >= ubx_length_)
            state_ = State::UbxCkA;
        break;

    case State::UbxCkA:
        state_ = (b == ubx_ck_a_) ? State::UbxCkB : State::Idle;
        break;

    case State::UbxCkB:
        if (b == ubx_ck_b_) {
            ubx_frame_count_++;
            processUbxMessage();
        }
        state_ = State::Idle;
        break;
    }
}

// -- NMEA helpers --------------------------------------------------------------

void GPSParser::nmea_reset() noexcept {
    nmea_len_ = 0;
    nmea_ck_  = 0;
}

// Returns a pointer to the start of field `n` (0-indexed) in buf, or nullptr.
// Fields are comma-separated; field 0 is everything before the first comma.
static const char *nmeaField(const char *buf, int n) noexcept {
    const char *p = buf;
    for (int i = 0; i < n; ++i) {
        p = std::strchr(p, ',');
        if (!p) return nullptr;
        ++p;
    }
    return p;
}

// Parses HHMMSS.ss → milliseconds since midnight.
static uint32_t parseUtcMs(const char *s) noexcept {
    if (!s || s[0] < '0' || s[0] > '2') return 0;
    const unsigned h  = static_cast<unsigned>(s[0] - '0') * 10u + static_cast<unsigned>(s[1] - '0');
    const unsigned m  = static_cast<unsigned>(s[2] - '0') * 10u + static_cast<unsigned>(s[3] - '0');
    const unsigned sc = static_cast<unsigned>(s[4] - '0') * 10u + static_cast<unsigned>(s[5] - '0');
    // Centiseconds from optional ".cs" fractional part.
    const unsigned cs = (s[6] == '.') ? (static_cast<unsigned>(s[7] - '0') * 10u +
                                          static_cast<unsigned>(s[8] - '0')) : 0u;
    return ((h * 3600u + m * 60u + sc) * 100u + cs) * 10u;
}

// Converts NMEA DDDMM.mmm → decimal degrees.
static double parseDegMin(const char *s) noexcept {
    if (!s || !*s) return 0.0;
    const double raw = std::strtod(s, nullptr);
    const int    deg = static_cast<int>(raw / 100.0);
    return static_cast<double>(deg) + (raw - static_cast<double>(deg) * 100.0) / 60.0;
}

// -- NMEA sentence dispatch -----------------------------------------------------

void GPSParser::processSentence() noexcept {
    // Accept any G[PNABL] talker prefix (GP, GN, GA, GB, GL).
    if (nmea_len_ < 5) return;
    if (nmea_buf_[0] != 'G') return;
    const char t = nmea_buf_[1];
    if (t != 'P' && t != 'N' && t != 'A' && t != 'B' && t != 'L') return;

    const std::string_view type{nmea_buf_.data() + 2, 3};
    if (type == "GGA") parseGGA();
    else if (type == "RMC") parseRMC();
}

// -- NMEA GGA parser ------------------------------------------------------------
// $GxGGA,hhmmss.ss,lat,N,lon,E,Q,numSV,hdop,alt,M,...*cs

void GPSParser::parseGGA() noexcept {
    const char *buf = nmea_buf_.data();

    const char *f1  = nmeaField(buf, 1);  // time
    const char *f2  = nmeaField(buf, 2);  // lat
    const char *f3  = nmeaField(buf, 3);  // N/S
    const char *f4  = nmeaField(buf, 4);  // lon
    const char *f5  = nmeaField(buf, 5);  // E/W
    const char *f6  = nmeaField(buf, 6);  // fix quality
    const char *f7  = nmeaField(buf, 7);  // num satellites
    const char *f8  = nmeaField(buf, 8);  // HDOP
    const char *f9  = nmeaField(buf, 9);  // altitude

    if (!f1 || !f2 || !f3 || !f4 || !f5 || !f6) return;

    const uint32_t utc = parseUtcMs(f1);
    if (utc > 0) coord_.utc_ms = utc;

    const int q = f6[0] - '0';
    coord_.fix_quality = static_cast<FixQuality>('0' + q);

    if (q <= 0) {
        coord_.fix_type = FixType::None;
        coord_.valid    = false;
        return;
    }

    double lat = parseDegMin(f2);
    if (f3[0] == 'S') lat = -lat;

    double lon = parseDegMin(f4);
    if (f5[0] == 'W') lon = -lon;

    coord_.latitude    = lat;
    coord_.longitude   = lon;
    coord_.altitude    = f9 ? std::strtof(f9, nullptr) : 0.0f;
    coord_.satellites  = f7 ? std::atoi(f7) : 0;
    coord_.hdop        = f8 ? std::strtof(f8, nullptr) : 0.0f;
    coord_.fix_type    = FixType::Fix3D;
    coord_.valid       = true;
}

// -- NMEA RMC parser ------------------------------------------------------------
// $GxRMC,hhmmss.ss,A,lat,N,lon,E,spd,cog,ddmmyy,mv,mvE,mode*cs

void GPSParser::parseRMC() noexcept {
    const char *buf = nmea_buf_.data();

    const char *f1  = nmeaField(buf, 1);   // time
    const char *f2  = nmeaField(buf, 2);   // status (A/V)
    const char *f3  = nmeaField(buf, 3);   // lat
    const char *f4  = nmeaField(buf, 4);   // N/S
    const char *f5  = nmeaField(buf, 5);   // lon
    const char *f6  = nmeaField(buf, 6);   // E/W
    const char *f7  = nmeaField(buf, 7);   // speed (knots)
    const char *f8  = nmeaField(buf, 8);   // COG
    const char *f9  = nmeaField(buf, 9);   // date (ddmmyy)
    const char *f12 = nmeaField(buf, 12);  // positioning mode

    if (!f1 || !f2) return;

    const uint32_t utc = parseUtcMs(f1);
    if (utc > 0) coord_.utc_ms = utc;

    if (f2[0] != 'A') { coord_.valid = false; return; }

    if (!f3 || !f4 || !f5 || !f6) return;

    double lat = parseDegMin(f3);
    if (f4[0] == 'S') lat = -lat;

    double lon = parseDegMin(f5);
    if (f6[0] == 'W') lon = -lon;

    coord_.latitude    = lat;
    coord_.longitude   = lon;
    coord_.speed_mps   = f7 ? std::strtof(f7, nullptr) * 0.514444f : 0.0f;
    coord_.course_deg  = f8 ? std::strtof(f8, nullptr)              : 0.0f;
    coord_.valid       = true;

    // Date — ddmmyy.
    if (f9 && f9[0] >= '0') {
        const uint8_t  day   = static_cast<uint8_t>( (f9[0]-'0')*10 + (f9[1]-'0'));
        const uint8_t  month = static_cast<uint8_t>( (f9[2]-'0')*10 + (f9[3]-'0'));
        const uint16_t year  = static_cast<uint16_t>(2000 + (f9[4]-'0')*10 + (f9[5]-'0'));
        if (day > 0 && month > 0) {
            coord_.utc_day   = day;
            coord_.utc_month = month;
            coord_.utc_year  = year;
        }
    }

    // Positioning mode — field 12.
    if (f12) {
        const char mode = f12[0];
        if (mode == 'A' || mode == 'D' || mode == 'E' || mode == 'N')
            coord_.fix_mode = static_cast<FixMode>(static_cast<uint8_t>(mode));
    }
}

// -- UBX accepted-message filter -----------------------------------------------

bool GPSParser::ubxAccepted() const noexcept {
    if (ubx_class_ != 0x01u) return false;
    switch (ubx_id_) {
    case 0x07u: return ubx_length_ == sizeof(NavPvt);        // NAV-PVT
    case 0x14u: return ubx_length_ == sizeof(NavHpPosLlh);   // NAV-HPPOSLLH
    case 0x04u: return ubx_length_ == 18u;                   // NAV-DOP
    case 0x35u: return true;                                  // NAV-SAT (variable length)
    default:    return false;
    }
}

// -- UBX message dispatch -------------------------------------------------------

void GPSParser::processUbxMessage() noexcept {
    if (ubx_class_ != 0x01u) return;
    switch (ubx_id_) {
    case 0x07u: ubx_pvt_count_++; parseNavPvt();      break;
    case 0x14u: ubx_hp_count_++;  parseNavHpPosLlh(); break;
    case 0x04u:                   parseNavDop();       break;
    case 0x35u:                   parseNavSat();       break;
    }
}

// -- UBX-NAV-PVT ---------------------------------------------------------------

void GPSParser::parseNavPvt() noexcept {
    // Overlay the typed struct directly onto our buffer for zero-copy field access.
    NavPvt pvt;
    std::memcpy(&pvt, ubx_buf_.data(), sizeof(NavPvt));

    // UTC date.
    if ((pvt.valid & 0x01u) && pvt.year > 0) {
        coord_.utc_year  = pvt.year;
        coord_.utc_month = pvt.month;
        coord_.utc_day   = pvt.day;
    }

    // UTC time — include sub-second nanoseconds for full precision.
    if (pvt.valid & 0x02u) {
        const int32_t  nano_clamped = (pvt.nano < 0) ? 0 : pvt.nano;
        const uint32_t ms_from_nano = static_cast<uint32_t>(nano_clamped / 1'000'000);
        coord_.utc_ms = (static_cast<uint32_t>(pvt.hour)  * 3600u +
                         static_cast<uint32_t>(pvt.min)   * 60u   +
                         static_cast<uint32_t>(pvt.sec)) * 1000u  + ms_from_nano;
    }

    coord_.fix_type  = static_cast<FixType>(pvt.fixType);
    coord_.carr_soln = static_cast<CarrierSolution>((pvt.flags >> 5u) & 0x03u);
    coord_.h_acc_mm  = pvt.hAcc;
    coord_.v_acc_mm  = pvt.vAcc;
    coord_.pdop      = static_cast<float>(pvt.pDOP) * 0.01f;
    coord_.satellites = pvt.numSV;

    if (pvt.flags & 0x01u) { // gnssFixOK
        // If a high-precision frame arrived before this PVT, merge it now.
        if (hp_pending_) {
            const NavHpPosLlh &hp = hp_pending_frame_;
            if (!(hp.flags & 0x02u)) { // invalidLlh bit clear
                coord_.latitude  = (hp.lat  * 1e-7) + (hp.latHp  * 1e-9);
                coord_.longitude = (hp.lon  * 1e-7) + (hp.lonHp  * 1e-9);
                coord_.altitude  = static_cast<float>((hp.hMSL + hp.hMSLHp * 0.1) * 1e-3);
                coord_.hp_valid  = true;
            }
            hp_pending_ = false;
        } else {
            coord_.latitude    = pvt.lat  * 1e-7;
            coord_.longitude   = pvt.lon  * 1e-7;
            coord_.altitude    = static_cast<float>(pvt.hMSL  * 0.001);
            coord_.hp_valid    = false;
        }

        coord_.vel_north_mms = pvt.velN;
        coord_.vel_east_mms  = pvt.velE;
        coord_.vel_down_mms  = pvt.velD;
        coord_.speed_mps     = static_cast<float>(pvt.gSpeed)  * 0.001f;
        coord_.course_deg    = static_cast<float>(pvt.headMot) * 1e-5f;
        coord_.valid         = true;
    } else {
        coord_.valid = false;
    }
}

// -- UBX-NAV-HPPOSLLH ----------------------------------------------------------

void GPSParser::parseNavHpPosLlh() noexcept {
    std::memcpy(&hp_pending_frame_, ubx_buf_.data(), sizeof(NavHpPosLlh));
    // If invalidLlh is set, discard immediately.
    if (hp_pending_frame_.flags & 0x02u) return;
    hp_pending_ = true;
    // If we already have a valid fix from a recent PVT, apply immediately.
    // (HPPOSLLH may arrive before or after PVT — both orderings are handled.)
    if (coord_.valid) {
        coord_.latitude  = (hp_pending_frame_.lat  * 1e-7) + (hp_pending_frame_.latHp  * 1e-9);
        coord_.longitude = (hp_pending_frame_.lon  * 1e-7) + (hp_pending_frame_.lonHp  * 1e-9);
        coord_.altitude  = static_cast<float>((hp_pending_frame_.hMSL +
                                               hp_pending_frame_.hMSLHp * 0.1) * 1e-3);
        coord_.hp_valid  = true;
        hp_pending_ = false;
    }
}

// -- UBX-NAV-DOP (18-byte payload) ---------------------------------------------
// Payload: iTOW(4) gDOP(2) pDOP(2) tDOP(2) vDOP(2) hDOP(2) nDOP(2) eDOP(2)
// All DOP fields are uint16_t scaled × 0.01.

void GPSParser::parseNavDop() noexcept {
    coord_.hdop = u16(12) * 0.01f;  // hDOP at offset 12
    coord_.vdop = u16(10) * 0.01f;  // vDOP at offset 10
}

// -- UBX-NAV-SAT per-byte inline extractor -------------------------------------
// Payload: iTOW(4) version(1) numSvs(1) reserved(2) — then numSvs × 12-byte records:
//   [0] gnssId  [1] svId  [2] cno (dBHz)  [3] elev  [4-5] azim  [6-7] prRes  [8-11] flags
//   flags bit 3 = svUsed

void GPSParser::navSatStreamByte(uint8_t b) noexcept {
    const uint16_t idx = ubx_payload_idx_;
    if (idx < 8u) return;
    const uint8_t byte_in_record = static_cast<uint8_t>((idx - 8u) % 12u);
    if (byte_in_record == 2u) {
        if (b > ubx_best_cno_) ubx_best_cno_ = b;
    } else if (byte_in_record == 8u) {
        if (b & 0x08u) ubx_num_sv_used_++;
    }
}

void GPSParser::parseNavSat() noexcept {
    coord_.best_cno    = ubx_best_cno_;
    coord_.num_sv_used = ubx_num_sv_used_;
}

// -- Public API -----------------------------------------------------------------

std::string_view GPSParser::fixLabel() const noexcept {
    switch (coord_.carr_soln) {
    case CarrierSolution::FixedRTK: return "RTK";
    case CarrierSolution::FloatRTK: return "FRTK";
    default: break;
    }
    switch (coord_.fix_type) {
    case FixType::Fix3D:
    case FixType::GnssDR:  return "3D";
    case FixType::Fix2D:   return "2D";
    case FixType::DR:      return "DR";
    default:               return "---";
    }
}

// -- Static navigation helpers --------------------------------------------------

double GPSParser::distanceBetween(double lat1, double lon1,
                                   double lat2, double lon2) noexcept {
    constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;
    constexpr double EARTH_R    = 6'371'009.0;

    const double dlon  = (lon1 - lon2) * DEG_TO_RAD;
    const double sdlon = std::sin(dlon);
    const double cdlon = std::cos(dlon);
    lat1 *= DEG_TO_RAD; lat2 *= DEG_TO_RAD;
    const double slat1 = std::sin(lat1), clat1 = std::cos(lat1);
    const double slat2 = std::sin(lat2), clat2 = std::cos(lat2);
    double d = (clat1 * slat2) - (slat1 * clat2 * cdlon);
    d = d * d + (clat2 * sdlon) * (clat2 * sdlon);
    d = std::sqrt(d);
    return std::atan2(d, (slat1 * slat2) + (clat1 * clat2 * cdlon)) * EARTH_R;
}

double GPSParser::courseTo(double lat1, double lon1,
                            double lat2, double lon2) noexcept {
    constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / std::numbers::pi;

    const double dlon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 *= DEG_TO_RAD; lat2 *= DEG_TO_RAD;
    const double a1 = std::sin(dlon) * std::cos(lat2);
    const double a2 = std::cos(lat1) * std::sin(lat2) -
                      std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing = std::atan2(a1, a2);
    if (bearing < 0.0) bearing += 2.0 * std::numbers::pi;
    return bearing * RAD_TO_DEG;
}

std::string_view GPSParser::cardinal(float course) noexcept {
    using sv = std::string_view;
    static constexpr std::array<sv, 16> dirs = {
        "N","NNE","NE","ENE","E","ESE","SE","SSE",
        "S","SSW","SW","WSW","W","WNW","NW","NNW"
    };
    return dirs[static_cast<int>((course + 11.25f) / 22.5f) % 16];
}

} // namespace gps
