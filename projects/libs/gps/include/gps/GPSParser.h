#pragma once

#include <cstdint>
#include <stdint.h>
#include <string>

namespace gps {

struct Coordinate {
  // Position — from GNGGA or UBX-NAV-PVT
  float    latitude;    // degrees
  float    longitude;   // degrees
  float    altitude;    // meters MSL
  int      satellites;  // number of SVs used
  bool     valid;

  // Kinematic — from GNRMC or UBX-NAV-PVT
  float    speed_mps;   // ground speed, m/s
  float    course_deg;  // course over ground, degrees true (0–360)

  // UTC — time-of-day from GNRMC/GNGGA; full date from UBX-NAV-PVT
  uint32_t utc_ms;      // milliseconds since midnight (0 if unknown)
  uint16_t utc_year;    // e.g. 2026 (0 if unknown)
  uint8_t  utc_month;   // 1–12 (0 if unknown)
  uint8_t  utc_day;     // 1–31 (0 if unknown)

  // 3-D NED velocity — UBX-NAV-PVT only (all 0 with NMEA only)
  // vel_down_mms < 0 = ascending; > 0 = descending (use for apogee detection)
  int32_t  vel_north_mms;
  int32_t  vel_east_mms;
  int32_t  vel_down_mms;

  // Fix quality — UBX-NAV-PVT only (0 with NMEA only)
  // 0=no fix  2=2D-fix  3=3D-fix  4=GNSS+DR
  uint8_t  fix_type;

  // Dilution of Precision — UBX-NAV-DOP (0.0 if not received)
  float    hdop;   // horizontal DOP
  float    vdop;   // vertical DOP

  // Signal quality — UBX-NAV-SAT (0 if not received)
  uint8_t  best_cno;     // highest C/N0 among all tracked SVs, dBHz
  uint8_t  num_sv_used;  // number of SVs flagged svUsed in NAV-SAT
};

class GPSParser {
public:
  GPSParser();

  void parse(char c);

  const Coordinate &getCoordinate() const;

  bool hasFix() const;

  // Diagnostic counters — reset to 0 on construction
  uint32_t ubxFrameCount() const { return ubx_frame_count_; }  // valid UBX frames (any)
  uint32_t ubxPvtCount()   const { return ubx_pvt_count_; }    // valid NAV-PVT frames

private:
  enum class State {
    IDLE,
    // ── NMEA ──────────────────────────────────────────────────────────────
    NMEA_TYPE,
    NMEA_DATA,
    NMEA_CK1,
    NMEA_CK2,
    // ── UBX ───────────────────────────────────────────────────────────────
    UBX_SYNC2,    // received 0xB5, waiting for 0x62
    UBX_CLASS,
    UBX_ID,
    UBX_LEN1,
    UBX_LEN2,
    UBX_PAYLOAD,
    UBX_CK_A,
    UBX_CK_B,
  };

  State state_;

  // NMEA
  std::string buffer_;
  int currentChecksum_;
  int parsedChecksum_;

  // UBX — shared framing state
  uint8_t  ubx_class_;
  uint8_t  ubx_id_;
  uint16_t ubx_length_;
  uint16_t ubx_payload_idx_;
  uint8_t  ubx_ck_a_;
  uint8_t  ubx_ck_b_;
  static constexpr uint16_t UBX_BUF_SIZE = 96;
  uint8_t  ubx_payload_[UBX_BUF_SIZE];

  // NAV-SAT streaming accumulators (reset at start of each NAV-SAT message)
  uint8_t  ubx_best_cno_;
  uint8_t  ubx_num_sv_used_;

  // Diagnostic counters
  uint32_t ubx_frame_count_;  // total valid UBX frames received
  uint32_t ubx_pvt_count_;    // valid NAV-PVT frames specifically

  Coordinate currentCoordinate_;

  // NMEA parsers
  void processSentence();
  void parseGPGGA();
  void parseGPRMC();

  // UBX parsers
  void processUbxMessage();
  void parseUbxNavPvt();   // class=0x01 id=0x07 — position/velocity/time
  void parseUbxNavDop();   // class=0x01 id=0x04 — dilution of precision
  void parseUbxNavSat();   // class=0x01 id=0x35 — finalise streaming results

  // NAV-SAT per-byte inline extractor (called from UBX_PAYLOAD)
  void navSatStreamByte(uint8_t b);

  // UBX payload read helpers (little-endian)
  uint16_t ubxU16(int off) const;
  uint32_t ubxU32(int off) const;
  int32_t  ubxI32(int off) const;
};

} // namespace gps
