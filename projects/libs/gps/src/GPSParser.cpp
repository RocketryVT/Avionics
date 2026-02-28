#include "gps/GPSParser.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>


namespace gps {

GPSParser::GPSParser() {
  state_           = State::IDLE;
  currentChecksum_ = 0;
  parsedChecksum_  = 0;

  ubx_class_       = 0;
  ubx_id_          = 0;
  ubx_length_      = 0;
  ubx_payload_idx_ = 0;
  ubx_ck_a_        = 0;
  ubx_ck_b_        = 0;
  ubx_best_cno_    = 0;
  ubx_num_sv_used_ = 0;
  ubx_frame_count_ = 0;
  ubx_pvt_count_   = 0;

  currentCoordinate_.latitude      = 0.0f;
  currentCoordinate_.longitude     = 0.0f;
  currentCoordinate_.altitude      = 0.0f;
  currentCoordinate_.satellites    = 0;
  currentCoordinate_.valid         = false;
  currentCoordinate_.speed_mps     = 0.0f;
  currentCoordinate_.course_deg    = 0.0f;
  currentCoordinate_.utc_ms        = 0;
  currentCoordinate_.utc_year      = 0;
  currentCoordinate_.utc_month     = 0;
  currentCoordinate_.utc_day       = 0;
  currentCoordinate_.vel_north_mms = 0;
  currentCoordinate_.vel_east_mms  = 0;
  currentCoordinate_.vel_down_mms  = 0;
  currentCoordinate_.fix_type      = 0;
  currentCoordinate_.hdop          = 0.0f;
  currentCoordinate_.vdop          = 0.0f;
  currentCoordinate_.best_cno      = 0;
  currentCoordinate_.num_sv_used   = 0;
}

// ── UBX payload read helpers (little-endian) ─────────────────────────────────
uint16_t GPSParser::ubxU16(int off) const {
  return (uint16_t)ubx_payload_[off] |
        ((uint16_t)ubx_payload_[off + 1] << 8);
}

uint32_t GPSParser::ubxU32(int off) const {
  return (uint32_t)ubx_payload_[off]           |
        ((uint32_t)ubx_payload_[off + 1] <<  8) |
        ((uint32_t)ubx_payload_[off + 2] << 16) |
        ((uint32_t)ubx_payload_[off + 3] << 24);
}

int32_t GPSParser::ubxI32(int off) const {
  return (int32_t)ubxU32(off);
}

// ── Main character-by-character parser ───────────────────────────────────────
void GPSParser::parse(char c) {
  uint8_t b = (uint8_t)c;

  switch (state_) {

  // ── Idle: detect NMEA '$' or UBX 0xB5 ──────────────────────────────────
  case State::IDLE:
    if (c == '$') {
      buffer_.clear();
      currentChecksum_ = 0;
      state_ = State::NMEA_TYPE;
    } else if (b == 0xB5) {
      state_ = State::UBX_SYNC2;
    }
    break;

  // ── NMEA ────────────────────────────────────────────────────────────────
  case State::NMEA_TYPE:
    if (c == ',') {
      currentChecksum_ ^= c;
      buffer_ += ',';
      state_ = State::NMEA_DATA;
    } else if (c == '*') {
      state_ = State::NMEA_CK1;
    } else if (c == '$') {
      buffer_.clear();
      currentChecksum_ = 0;
    } else {
      buffer_ += c;
      currentChecksum_ ^= c;
    }
    break;

  case State::NMEA_DATA:
    if (c == '*') {
      state_ = State::NMEA_CK1;
    } else if (c == '$') {
      buffer_.clear();
      currentChecksum_ = 0;
      state_ = State::NMEA_TYPE;
    } else {
      buffer_ += c;
      currentChecksum_ ^= c;
    }
    break;

  case State::NMEA_CK1:
    parsedChecksum_ = (c >= '0' && c <= '9') ? (c - '0') : (c - 'A' + 10);
    parsedChecksum_ *= 16;
    state_ = State::NMEA_CK2;
    break;

  case State::NMEA_CK2:
    parsedChecksum_ += (c >= '0' && c <= '9') ? (c - '0') : (c - 'A' + 10);
    if (parsedChecksum_ == currentChecksum_)
      processSentence();
    state_ = State::IDLE;
    break;

  // ── UBX ─────────────────────────────────────────────────────────────────
  case State::UBX_SYNC2:
    if (b == 0x62) {
      ubx_ck_a_ = 0;
      ubx_ck_b_ = 0;
      state_ = State::UBX_CLASS;
    } else {
      state_ = State::IDLE;
    }
    break;

  case State::UBX_CLASS:
    ubx_class_ = b;
    ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
    state_ = State::UBX_ID;
    break;

  case State::UBX_ID:
    ubx_id_ = b;
    ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
    state_ = State::UBX_LEN1;
    break;

  case State::UBX_LEN1:
    ubx_length_ = b;
    ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
    state_ = State::UBX_LEN2;
    break;

  case State::UBX_LEN2:
    ubx_length_ |= ((uint16_t)b << 8);
    ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
    ubx_payload_idx_ = 0;
    // Reset NAV-SAT streaming accumulators for each new NAV-SAT message
    if (ubx_class_ == 0x01 && ubx_id_ == 0x35) {
      ubx_best_cno_    = 0;
      ubx_num_sv_used_ = 0;
    }
    state_ = (ubx_length_ > 0) ? State::UBX_PAYLOAD : State::UBX_CK_A;
    break;

  case State::UBX_PAYLOAD:
    ubx_ck_a_ += b; ubx_ck_b_ += ubx_ck_a_;
    if (ubx_payload_idx_ < UBX_BUF_SIZE)
      ubx_payload_[ubx_payload_idx_] = b;
    // NAV-SAT: too large to buffer fully — extract inline as bytes arrive
    if (ubx_class_ == 0x01 && ubx_id_ == 0x35)
      navSatStreamByte(b);
    ubx_payload_idx_++;
    if (ubx_payload_idx_ >= ubx_length_)
      state_ = State::UBX_CK_A;
    break;

  case State::UBX_CK_A:
    state_ = (b == ubx_ck_a_) ? State::UBX_CK_B : State::IDLE;
    break;

  case State::UBX_CK_B:
    if (b == ubx_ck_b_)
      processUbxMessage();
    state_ = State::IDLE;
    break;
  }
}

// ── NMEA helpers ─────────────────────────────────────────────────────────────

static size_t findNextToken(const std::string &str, size_t start) {
  size_t pos = str.find(',', start);
  if (pos == std::string::npos)
    return std::string::npos;
  return pos + 1;
}

static uint32_t parseUtcMs(const char* s) {
  if (s[0] < '0' || s[0] > '2') return 0;
  unsigned h  = (unsigned)(s[0] - '0') * 10 + (unsigned)(s[1] - '0');
  unsigned m  = (unsigned)(s[2] - '0') * 10 + (unsigned)(s[3] - '0');
  unsigned sc = (unsigned)(s[4] - '0') * 10 + (unsigned)(s[5] - '0');
  unsigned cs = (s[6] == '.') ? (unsigned)(s[7] - '0') * 10 + (unsigned)(s[8] - '0') : 0;
  return ((h * 3600u + m * 60u + sc) * 100u + cs) * 10u;
}

// ── NMEA sentence dispatch ────────────────────────────────────────────────────

void GPSParser::processSentence() {
  if (buffer_.size() >= 5 && buffer_.substr(2, 3) == "GGA")
    parseGPGGA();
  else if (buffer_.size() >= 5 && buffer_.substr(2, 3) == "RMC")
    parseGPRMC();
}

void GPSParser::parseGPGGA() {
  size_t pos = findNextToken(buffer_, 0);
  if (pos == std::string::npos) return;

  uint32_t utc = parseUtcMs(&buffer_[pos]);
  if (utc > 0) currentCoordinate_.utc_ms = utc;

  pos = findNextToken(buffer_, pos);
  if (pos == std::string::npos) return;

  float latRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);
  char ns = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  float lonRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);
  char ew = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  int fixQuality = atoi(&buffer_[pos]);
  pos = findNextToken(buffer_, pos);

  int satellites = atoi(&buffer_[pos]);
  pos = findNextToken(buffer_, pos);

  float hdop = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);

  float altitude = strtof(&buffer_[pos], nullptr);

  if (fixQuality > 0) {
    int latDeg = (int)(latRaw / 100);
    currentCoordinate_.latitude = latDeg + (latRaw - latDeg * 100) / 60.0f;
    if (ns == 'S') currentCoordinate_.latitude *= -1;

    int lonDeg = (int)(lonRaw / 100);
    currentCoordinate_.longitude = lonDeg + (lonRaw - lonDeg * 100) / 60.0f;
    if (ew == 'W') currentCoordinate_.longitude *= -1;

    currentCoordinate_.altitude   = altitude;
    currentCoordinate_.satellites = satellites;
    currentCoordinate_.hdop       = hdop;
    currentCoordinate_.fix_type   = 3;  // GGA fix quality > 0 → treat as 3D fix
    currentCoordinate_.valid      = true;
  } else {
    currentCoordinate_.fix_type = 0;
    currentCoordinate_.valid    = false;
  }
}

void GPSParser::parseGPRMC() {
  size_t pos = findNextToken(buffer_, 0);
  if (pos == std::string::npos) return;

  uint32_t utc = parseUtcMs(&buffer_[pos]);
  if (utc > 0) currentCoordinate_.utc_ms = utc;

  pos = findNextToken(buffer_, pos);
  char status = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  if (status != 'A') { currentCoordinate_.valid = false; return; }

  float latRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);
  char ns = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  float lonRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);
  char ew = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  float speed_knots = (pos != std::string::npos) ? strtof(&buffer_[pos], nullptr) : 0.0f;
  if (pos != std::string::npos) pos = findNextToken(buffer_, pos);
  float course = (pos != std::string::npos) ? strtof(&buffer_[pos], nullptr) : 0.0f;

  // Parse date field (ddmmyy) — field 9 in RMC
  if (pos != std::string::npos) pos = findNextToken(buffer_, pos);
  if (pos != std::string::npos) {
    const char* d = &buffer_[pos];
    if (d[0] >= '0' && d[1] >= '0' && d[2] >= '0' &&
        d[3] >= '0' && d[4] >= '0' && d[5] >= '0') {
      uint8_t  day   = (uint8_t) ((d[0]-'0')*10 + (d[1]-'0'));
      uint8_t  month = (uint8_t) ((d[2]-'0')*10 + (d[3]-'0'));
      uint16_t year  = (uint16_t)(2000 + (d[4]-'0')*10 + (d[5]-'0'));
      if (day > 0 && month > 0) {
        currentCoordinate_.utc_day   = day;
        currentCoordinate_.utc_month = month;
        currentCoordinate_.utc_year  = year;
      }
    }
  }

  int latDeg = (int)(latRaw / 100);
  currentCoordinate_.latitude = latDeg + (latRaw - latDeg * 100) / 60.0f;
  if (ns == 'S') currentCoordinate_.latitude *= -1;

  int lonDeg = (int)(lonRaw / 100);
  currentCoordinate_.longitude = lonDeg + (lonRaw - lonDeg * 100) / 60.0f;
  if (ew == 'W') currentCoordinate_.longitude *= -1;

  currentCoordinate_.speed_mps  = speed_knots * 0.514444f;
  currentCoordinate_.course_deg = course;
  currentCoordinate_.valid      = true;
}

// ── UBX message dispatch ──────────────────────────────────────────────────────

void GPSParser::processUbxMessage() {
  ubx_frame_count_++;

  if (ubx_class_ != 0x01) return;

  switch (ubx_id_) {
  case 0x07:   // NAV-PVT — position / velocity / time
    ubx_pvt_count_++;
    if (ubx_length_ <= UBX_BUF_SIZE)
      parseUbxNavPvt();
    break;
  case 0x04:   // NAV-DOP — dilution of precision
    if (ubx_length_ <= UBX_BUF_SIZE)
      parseUbxNavDop();
    break;
  case 0x35:   // NAV-SAT — satellite signal info (streaming, may be >buffer)
    parseUbxNavSat();
    break;
  }
}

// ── UBX-NAV-PVT (class=0x01 id=0x07, 92 bytes) ───────────────────────────────
void GPSParser::parseUbxNavPvt() {
  if (ubx_length_ < 68) return;

  uint16_t year   = ubxU16(4);
  uint8_t  month  = ubx_payload_[6];
  uint8_t  day    = ubx_payload_[7];
  uint8_t  hour   = ubx_payload_[8];
  uint8_t  minute = ubx_payload_[9];
  uint8_t  sec    = ubx_payload_[10];
  uint8_t  valid  = ubx_payload_[11]; // bit0=validDate, bit1=validTime

  uint8_t  fixType = ubx_payload_[20];
  uint8_t  flags   = ubx_payload_[21]; // bit0=gnssFixOK
  uint8_t  numSV   = ubx_payload_[23];

  int32_t  lat_deg7 = ubxI32(28);  // 1e-7 degrees
  int32_t  lon_deg7 = ubxI32(24);  // 1e-7 degrees
  int32_t  hMSL_mm  = ubxI32(36);  // mm above MSL

  int32_t  velN    = ubxI32(48);   // mm/s
  int32_t  velE    = ubxI32(52);
  int32_t  velD    = ubxI32(56);
  int32_t  gSpd    = ubxI32(60);   // ground speed mm/s
  int32_t  headMot = ubxI32(64);   // heading of motion, 1e-5 degrees

  if (valid & 0x02)
    currentCoordinate_.utc_ms = ((uint32_t)hour * 3600u + minute * 60u + sec) * 1000u;
  if ((valid & 0x01) && year > 0) {
    currentCoordinate_.utc_year  = year;
    currentCoordinate_.utc_month = month;
    currentCoordinate_.utc_day   = day;
  }

  currentCoordinate_.fix_type = fixType;

  if (flags & 0x01) { // gnssFixOK
    currentCoordinate_.latitude      = lat_deg7 * 1e-7f;
    currentCoordinate_.longitude     = lon_deg7 * 1e-7f;
    currentCoordinate_.altitude      = hMSL_mm  * 0.001f;
    currentCoordinate_.satellites    = numSV;
    currentCoordinate_.speed_mps     = gSpd    * 0.001f;
    currentCoordinate_.course_deg    = headMot * 1e-5f;
    currentCoordinate_.vel_north_mms = velN;
    currentCoordinate_.vel_east_mms  = velE;
    currentCoordinate_.vel_down_mms  = velD;
    currentCoordinate_.valid         = true;
  } else {
    currentCoordinate_.valid = false;
  }
}

// ── UBX-NAV-DOP (class=0x01 id=0x04, 18 bytes) ───────────────────────────────
// Payload: iTOW(4) gDOP(2) pDOP(2) tDOP(2) vDOP(2) hDOP(2) nDOP(2) eDOP(2)
// All DOP fields are uint16_t scaled ×0.01.
void GPSParser::parseUbxNavDop() {
  if (ubx_length_ < 18) return;
  currentCoordinate_.hdop = ubxU16(12) * 0.01f;  // hDOP at offset 12
  currentCoordinate_.vdop = ubxU16(10) * 0.01f;  // vDOP at offset 10
}

// ── UBX-NAV-SAT per-byte inline extractor ────────────────────────────────────
// Called from UBX_PAYLOAD state for every byte of a NAV-SAT message.
// Payload layout:
//   [0-3]  iTOW (uint32)
//   [4]    version (uint8)
//   [5]    numSvs (uint8)
//   [6-7]  reserved (uint16)
//   -- repeated numSvs times, 12 bytes per SV: --
//   [8 + n*12 + 0]  gnssId
//   [8 + n*12 + 1]  svId
//   [8 + n*12 + 2]  cno      ← C/N0 in dBHz
//   [8 + n*12 + 3]  elev
//   [8 + n*12 + 4-5] azim
//   [8 + n*12 + 6-7] prRes
//   [8 + n*12 + 8-11] flags  ← bit 3 = svUsed
void GPSParser::navSatStreamByte(uint8_t b) {
  uint16_t idx = ubx_payload_idx_;  // index of this byte within payload
  if (idx < 8) return;              // skip the 8-byte header

  uint8_t byte_in_record = (uint8_t)((idx - 8) % 12);

  if (byte_in_record == 2) {          // cno field
    if (b > ubx_best_cno_)
      ubx_best_cno_ = b;
  } else if (byte_in_record == 8) {   // flags byte 0; bit 3 = svUsed
    if (b & 0x08)
      ubx_num_sv_used_++;
  }
}

// ── UBX-NAV-SAT finaliser ────────────────────────────────────────────────────
// Called after checksum validated — copies streaming results to Coordinate.
void GPSParser::parseUbxNavSat() {
  currentCoordinate_.best_cno    = ubx_best_cno_;
  currentCoordinate_.num_sv_used = ubx_num_sv_used_;
}

// ── Public API ────────────────────────────────────────────────────────────────

const Coordinate &GPSParser::getCoordinate() const { return currentCoordinate_; }

bool GPSParser::hasFix() const { return currentCoordinate_.valid; }

} // namespace gps
