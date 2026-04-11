#pragma once

// gps/nmea_parser.hpp — NMEA-0183 parser
//
// Parses GGA and RMC sentences from a byte stream and writes decoded fields
// into a Coordinate reference supplied at construction.
//
// Can be used standalone:
//
//   gps::Coordinate coord;
//   gps::NmeaParser nmea(coord);
//   while (uart_is_readable(uart0))
//       nmea.feed(uart_getc(uart0));
//   if (coord.valid) { ... }
//
// Or let GpsDriver own it (the typical case) — see gps_driver.hpp.
//
// Sentences decoded
// ------------------
//   GxGGA  position, altitude, HDOP, satellite count, fix quality
//   GxRMC  position, speed, course, date, positioning mode
//
// Talker prefixes accepted: GP, GN, GA, GB, GL  (the 'x' above)
// All other sentences are silently discarded after checksum verification.

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string_view>

#include "types.hpp"

namespace gps {

class NmeaParser {
public:
    // coord is written every time a valid sentence is decoded.
    explicit NmeaParser(Coordinate& coord, Diagnostics& diag) noexcept
        : coord_(coord), diag_(diag) {}

    // Feed one byte from the transport stream.
    void feed(uint8_t b) noexcept { parse_byte(b); }

private:
    Coordinate&  coord_;
    Diagnostics& diag_;

    // -- State machine ---------------------------------------------------------
    enum class State : uint8_t { Idle, Type, Data, Ck1, Ck2 };
    State state_ = State::Idle;

    // -- Sentence buffer -------------------------------------------------------
    // 128 bytes covers the longest standard NMEA sentence (GGA/RMC < 82 chars,
    // but some talkers emit slightly longer ones).
    static constexpr std::size_t BUF_SIZE = 128;
    std::array<char, BUF_SIZE> buf_{};
    std::size_t len_    = 0;
    uint8_t     ck_run_ = 0;   // running XOR checksum
    uint8_t     ck_hi_  = 0;   // first checksum nibble × 16

    // -- Byte pump -------------------------------------------------------------
    void parse_byte(uint8_t b) noexcept {
        const char c = static_cast<char>(b);
        switch (state_) {

        case State::Idle:
            if (c == '$') { reset(); state_ = State::Type; }
            break;

        case State::Type:
            if (c == '$') {
                reset();                        // embedded '$' — restart
            } else if (c == '*') {
                state_ = State::Ck1;
            } else if (c == ',') {
                ck_run_ ^= b;
                push(c);
                state_ = State::Data;
            } else {
                ck_run_ ^= b;
                push(c);
            }
            break;

        case State::Data:
            if (c == '*') {
                state_ = State::Ck1;
            } else if (c == '$') {
                reset();
                state_ = State::Type;
            } else {
                ck_run_ ^= b;
                push(c);
            }
            break;

        case State::Ck1:
            ck_hi_  = hex_val(c) * 16u;
            state_  = State::Ck2;
            break;

        case State::Ck2: {
            const uint8_t expected = ck_hi_ + hex_val(c);
            if (expected == ck_run_) {
                diag_.nmea_good++;
                buf_[len_] = '\0';
                dispatch();
            } else {
                diag_.nmea_bad_cksum++;
            }
            state_ = State::Idle;
            break;
        }
        }
    }

    void reset() noexcept { len_ = 0; ck_run_ = 0; }

    void push(char c) noexcept {
        if (len_ < BUF_SIZE - 1) buf_[len_++] = c;
    }

    static uint8_t hex_val(char c) noexcept {
        return (c >= 'A') ? static_cast<uint8_t>(c - 'A' + 10)
                          : static_cast<uint8_t>(c - '0');
    }

    // -- Sentence dispatcher ---------------------------------------------------
    void dispatch() noexcept {
        // Require at least "GxGGA," (6 chars) and one of the accepted talkers.
        if (len_ < 5 || buf_[0] != 'G') return;
        const char t = buf_[1];
        if (t != 'P' && t != 'N' && t != 'A' && t != 'B' && t != 'L') return;

        const std::string_view type{buf_.data() + 2, 3};
        if      (type == "GGA") parse_gga();
        else if (type == "RMC") parse_rmc();
    }

    // -- Field helpers ---------------------------------------------------------

    // Return pointer to the start of comma-delimited field n (0-indexed).
    // Field 0 is everything before the first comma.
    static const char* field(const char* buf, int n) noexcept {
        const char* p = buf;
        for (int i = 0; i < n; ++i) {
            p = std::strchr(p, ',');
            if (!p) return nullptr;
            ++p;
        }
        return p;
    }

    // Parse NMEA HHMMSS.ss or HHMMSS.sss → milliseconds since midnight.
    static uint32_t parse_utc_ms(const char* s) noexcept {
        if (!s || s[0] < '0' || s[0] > '2') return 0;
        const unsigned h  = static_cast<unsigned>(s[0]-'0')*10u + static_cast<unsigned>(s[1]-'0');
        const unsigned m  = static_cast<unsigned>(s[2]-'0')*10u + static_cast<unsigned>(s[3]-'0');
        const unsigned sc = static_cast<unsigned>(s[4]-'0')*10u + static_cast<unsigned>(s[5]-'0');
        const unsigned cs = (s[6] == '.') ? (static_cast<unsigned>(s[7]-'0')*10u +
                                              static_cast<unsigned>(s[8]-'0')) : 0u;
        return ((h * 3600u + m * 60u + sc) * 100u + cs) * 10u;
    }

    // Parse NMEA DDDMM.mmm → decimal degrees.
    static double parse_degmin(const char* s) noexcept {
        if (!s || !*s) return 0.0;
        const double raw = ::strtod(s, nullptr);
        const int    deg = static_cast<int>(raw / 100.0);
        return static_cast<double>(deg) + (raw - static_cast<double>(deg) * 100.0) / 60.0;
    }

    // -- GGA -------------------------------------------------------------------
    // $GxGGA,hhmmss.ss,lat,N,lon,E,Q,numSV,hdop,alt,M,...*cs
    void parse_gga() noexcept {
        const char* buf = buf_.data();
        const char* f1  = field(buf, 1);   // time
        const char* f2  = field(buf, 2);   // lat
        const char* f3  = field(buf, 3);   // N/S
        const char* f4  = field(buf, 4);   // lon
        const char* f5  = field(buf, 5);   // E/W
        const char* f6  = field(buf, 6);   // fix quality
        const char* f7  = field(buf, 7);   // num satellites
        const char* f8  = field(buf, 8);   // HDOP
        const char* f9  = field(buf, 9);   // altitude MSL

        if (!f1 || !f2 || !f3 || !f4 || !f5 || !f6) return;

        const uint32_t utc = parse_utc_ms(f1);
        if (utc > 0) coord_.utc_ms = utc;

        const int q = f6[0] - '0';
        coord_.fix_quality = static_cast<FixQuality>('0' + q);

        if (q <= 0) {
            coord_.fix_type = FixType::None;
            coord_.valid    = false;
            return;
        }

        double lat = parse_degmin(f2); if (f3[0] == 'S') lat = -lat;
        double lon = parse_degmin(f4); if (f5[0] == 'W') lon = -lon;

        coord_.latitude   = lat;
        coord_.longitude  = lon;
        coord_.altitude   = f9 ? ::strtof(f9, nullptr) : 0.0f;
        coord_.satellites = f7 ? ::atoi(f7)            : 0;
        coord_.hdop       = f8 ? ::strtof(f8, nullptr) : 0.0f;
        coord_.fix_type   = FixType::Fix3D;
        coord_.valid      = true;
    }

    // -- RMC -------------------------------------------------------------------
    // $GxRMC,hhmmss.ss,A,lat,N,lon,E,spd,cog,ddmmyy,mv,mvE,mode*cs
    void parse_rmc() noexcept {
        const char* buf = buf_.data();
        const char* f1  = field(buf, 1);   // time
        const char* f2  = field(buf, 2);   // status A/V
        const char* f3  = field(buf, 3);   // lat
        const char* f4  = field(buf, 4);   // N/S
        const char* f5  = field(buf, 5);   // lon
        const char* f6  = field(buf, 6);   // E/W
        const char* f7  = field(buf, 7);   // speed, knots
        const char* f8  = field(buf, 8);   // course over ground
        const char* f9  = field(buf, 9);   // date ddmmyy
        const char* f12 = field(buf, 12);  // positioning mode

        if (!f1 || !f2) return;

        const uint32_t utc = parse_utc_ms(f1);
        if (utc > 0) coord_.utc_ms = utc;

        if (f2[0] != 'A') { coord_.valid = false; return; }
        if (!f3 || !f4 || !f5 || !f6) return;

        double lat = parse_degmin(f3); if (f4[0] == 'S') lat = -lat;
        double lon = parse_degmin(f5); if (f6[0] == 'W') lon = -lon;

        coord_.latitude   = lat;
        coord_.longitude  = lon;
        coord_.speed_mps  = f7 ? ::strtof(f7, nullptr) * 0.514444f : 0.0f;
        coord_.course_deg = f8 ? ::strtof(f8, nullptr)              : 0.0f;
        coord_.valid      = true;

        // Date field: ddmmyy
        if (f9 && f9[0] >= '0') {
            const uint8_t  day   = static_cast<uint8_t> ((f9[0]-'0')*10 + (f9[1]-'0'));
            const uint8_t  month = static_cast<uint8_t> ((f9[2]-'0')*10 + (f9[3]-'0'));
            const uint16_t year  = static_cast<uint16_t>(2000 + (f9[4]-'0')*10 + (f9[5]-'0'));
            if (day > 0 && month > 0) {
                coord_.utc_day   = day;
                coord_.utc_month = month;
                coord_.utc_year  = year;
            }
        }

        // Positioning mode: field 12 (A/D/E/N)
        if (f12) {
            const char mode = f12[0];
            if (mode == 'A' || mode == 'D' || mode == 'E' || mode == 'N')
                coord_.fix_mode = static_cast<FixMode>(static_cast<uint8_t>(mode));
        }
    }
};

} // namespace gps
