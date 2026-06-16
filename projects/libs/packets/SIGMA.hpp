#pragma once

// SIGMA.hpp - Rocket Telemetry Packet Protocol
// Statically Improbable Giga Muffin Aggregator
//
// All types live inside the SIGMA namespace.
//
// Design rules
// ------------
//   • No __packed__. Wire structs use naturally-aligned fixed-width types
//     ordered largest-to-smallest; the compiler adds zero implicit padding.
//     static_assert guards verify sizes at compile time.
//   • memcpy is used for all serialisation/deserialisation to satisfy
//     strict-aliasing rules and to work on any architecture.
//   • All multi-byte fields are little-endian (RP2350, ESP32, STM32 are all
//     little-endian; adjust serialize/deserialize if targeting BE).
//   • Scaled integers minimise wire size while retaining useful resolution.
//     See SIGMA::Convert helpers for physical ↔ wire conversions.
//
// Frame layout (all transports)
// ------------------------------
//   [AA][55]         start magic        2 bytes
//   [type]           PacketType         1 byte
//   [seq]            per-type sequence  1 byte   (wraps 0x00..0xFF)
//   [len_lo][len_hi] payload length     2 bytes  (little-endian)
//   [... payload ...]                   plen bytes
//   [crc_lo][crc_hi] CRC-16/CCITT       2 bytes  (covers type+seq+len+payload)
//   [BB][66]         end magic          2 bytes
//   ---------------------------------------------
//   Total overhead: 10 bytes
//
// Sequence counter:
//   Each PacketType has its own independent uint8_t counter incremented by the
//   sender before every transmission.  The receiver detects gaps with:
//     uint8_t dropped = (uint8_t)(new_seq - last_seq - 1);
//   A result of 0 means no gap; the uint8_t subtraction handles wraparound.
//   Counters are stored inside each builder type's serialize() as a static.
//
// Magic byte rationale:
//   Start AA 55 (10101010 01010101) and end BB 66 (10111011 01100110) are
//   deliberately not byte-reversals of each other, so a stream parser can
//   unambiguously identify start vs end without length-counting.  Neither
//   value is a valid PacketType byte.
//
// LoRa note: the radio PHY already provides a CRC; the protocol CRC is
// redundant but enables receiver-side validation before application use.
//
// Inter-Pico UDP note:
//   The secondary Pico (low-gain LoRa hub) forwards received packets to the
//   primary Pico (antenna tracker) immediately on receipt - no field-gathering
//   delay.  InterPico::Payload mirrors LoRa::Payload and appends radio metadata
//   plus optional SIGMA2 vector/accuracy fields when the source packet has them.

#include <stddef.h>
#include <stdint.h>
#include <string.h>   // memcpy

namespace SIGMA {

// ===============================================================================
// Frame constants
// ===============================================================================

static constexpr uint8_t FRAME_START_0  = 0xAA;
static constexpr uint8_t FRAME_START_1  = 0x55;
static constexpr uint8_t FRAME_END_0    = 0xBB;
static constexpr uint8_t FRAME_END_1    = 0x66;

static constexpr size_t  FRAME_OVERHEAD = 10;   // start(2)+type(1)+seq(1)+len(2)+crc(2)+end(2)
static constexpr size_t  MAX_PAYLOAD    = 245;
static constexpr size_t  MAX_FRAME      = MAX_PAYLOAD + FRAME_OVERHEAD;

// ===============================================================================
// Enumerations
// ===============================================================================

enum class PacketType : uint8_t {
    LORA_TELEMETRY   = 0x01,   ///< Compact LoRa uplink  (~45 bytes on-air)
    WIFI_TELEMETRY   = 0x02,   ///< Full float telemetry (WiFi / USB)
    INTER_PICO       = 0x03,   ///< Secondary -> primary Pico UDP forwarding
    TIME_SYNC        = 0x04,   ///< Occasional GPS time-sync beacon
    GPS_NAV          = 0x05,   ///< GPS position + NED velocity
    NAV_STATE        = 0x06,   ///< Baro/fused altitude, fused NED velocity, quaternion
    STORAGE_FULL     = 0x10,   ///< Maximum-fidelity SD / flash record
    STORAGE_IMU      = 0x11,   ///< High-rate IMU burst  (flash ring-buffer)
    COMMAND          = 0x20,   ///< Ground -> rocket command
    ACK              = 0x21,   ///< Rocket -> ground acknowledgement
    HEARTBEAT        = 0xFE,   ///< Keepalive / link-quality beacon
    INVALID          = 0xFF,
};

enum class FlightState : uint8_t {
    GROUND_IDLE      = 0,
    ARMED            = 1,
    POWERED_ASCENT   = 2,
    COAST_ASCENT     = 3,
    APOGEE           = 4,
    DESCENT_DROGUE   = 5,
    DESCENT_MAIN     = 6,
    LANDED           = 7,
    FAULT            = 0xFF,
};

// --- Data-validity flags (bitmask in flags field) -----------------------------
static constexpr uint8_t FLAG_GPS_VALID   = (1u << 0);
static constexpr uint8_t FLAG_BARO_VALID  = (1u << 1);
static constexpr uint8_t FLAG_IMU_VALID   = (1u << 2);
static constexpr uint8_t FLAG_MAG_VALID   = (1u << 3);
static constexpr uint8_t FLAG_TIME_VALID  = (1u << 4);

// ===============================================================================
// Wire scaling constants
//
// All scaled-integer wire fields:
//   stored   = physical x SCALE   (encode)
//   physical = stored   ÷ SCALE   (decode)
// ===============================================================================

static constexpr int32_t  LATLON_SCALE = 10'000'000; ///< degrees x 10^7 -> int32_t (~1.1 cm)
static constexpr int32_t  GPS_ALT_SCALE = 100;        ///< metres -> centimetres
static constexpr float    ANGLE_SCALE  = 10.0f;       ///< degrees x 10 -> int16_t (0.1° res)
static constexpr float    QUAT_SCALE   = 32767.0f;    ///< Q1.15 fixed-point
static constexpr float    SPEED_SCALE  = 100.0f;      ///< m/s x 100 -> uint16_t (0.01 m/s res)
static constexpr float    MACH_SCALE   = 1000.0f;     ///< Mach x 1000 -> uint16_t
static constexpr float    ACCEL_SCALE  = 1000.0f;     ///< g -> milli-g -> int16_t
static constexpr float    GYRO_SCALE   = 10.0f;       ///< deg/s x 10 -> int16_t

// ===============================================================================
// Wire-format payload structs  (namespace SIGMA::Wire)
//
// Fields ordered largest-alignment-first - no implicit padding.
// static_assert guards catch any future reordering.
// ===============================================================================

namespace Wire {

// -- LoRa Telemetry ------------------------------------------------------------
// On-air frame = 36 + 9 = 45 bytes.
// Minimised for throughput and link-budget.  No heading or UTC - those add
// bytes on-air for fields that are rarely needed at range.
//
// Offset  Size  Field
//      0     4  boot_ms
//      4     4  lat_deg_e7
//      8     4  lon_deg_e7
//     12     4  alt_baro_dm
//     16     4  alt_gps_cm
//     20     8  q[4]           int16_t Q1.15 x 4
//     28     2  speed_cms
//     30     1  state
//     31     1  satellites
//     32     1  flags
//     33     3  _pad[3]
//    ----   --
//     36 bytes
struct LoRa {
    uint32_t boot_ms;       ///< ms since power-on
    int32_t  lat_deg_e7;    ///< latitude  x 10^7  (÷10^7 -> °, WGS-84)
    int32_t  lon_deg_e7;    ///< longitude x 10^7  (÷10^7 -> °, WGS-84)
    int32_t  alt_baro_dm;   ///< baro altitude, decimetres  (÷10 -> m)
    int32_t  alt_gps_cm;    ///< GPS  altitude, centimetres (÷100 -> m)
    int16_t  q[4];          ///< quaternion [w,x,y,z] Q1.15  (÷32767 -> [-1,+1])
    uint16_t speed_cms;     ///< scalar speed, cm/s  (÷100 -> m/s)
    uint8_t  state;         ///< FlightState
    uint8_t  satellites;    ///< GPS satellites in use
    uint8_t  flags;         ///< FLAG_* validity bits
    uint8_t  _pad[3];
};
static_assert(sizeof(LoRa) == 36, "Wire::LoRa layout changed");

// -- Inter-Pico UDP ------------------------------------------------------------
// The secondary Pico forwards each received LoRa packet immediately on receipt
// - no buffering or field-gathering.  Wire::LoRa is reproduced verbatim, with
// RSSI and SNR appended from the radio driver.  The primary Pico can therefore
// reuse the same LoRa decoder for the data fields.
//
// Offset  Size  Field
//      0    36  (same as Wire::LoRa)
//     36     6  vel_*_cms      NED velocity, cm/s
//     42     6  *_acc_*        GPS h/v/s accuracy, cm or cm/s
//     48     6  acc_*_mg       fused NED acceleration, milli-g
//     54     1  nav_source     SIGMA2 nav-source bitfield when available
//     55     1  rssi           receive RSSI, dBm (int8_t)
//     56     1  snr_q2         SNR x 4, dB  (int8_t, ÷4 -> float dB)
//     57     3  _pad[3]
//    ----   --
//     60 bytes
struct InterPico {
    // -- LoRa fields (identical layout to Wire::LoRa) -------------------------
    uint32_t boot_ms;
    int32_t  lat_deg_e7;
    int32_t  lon_deg_e7;
    int32_t  alt_baro_dm;
    int32_t  alt_gps_cm;
    int16_t  q[4];
    uint16_t speed_cms;
    int16_t  vel_n_cms;
    int16_t  vel_e_cms;
    int16_t  vel_d_cms;
    uint16_t h_acc_cm;
    uint16_t v_acc_cm;
    uint16_t s_acc_cms;
    int16_t  acc_n_mg;
    int16_t  acc_e_mg;
    int16_t  acc_d_mg;
    uint8_t  nav_source;
    uint8_t  state;
    uint8_t  satellites;
    uint8_t  flags;
    // -- Radio metadata (appended by secondary Pico) ---------------------------
    int8_t   rssi;          ///< receive RSSI, dBm
    int8_t   snr_q2;        ///< SNR x 4, dB  (÷4 -> float dB)
    uint8_t  _pad[3];
};
static_assert(sizeof(InterPico) == 60, "Wire::InterPico layout changed");

// -- WiFi / Full Telemetry -----------------------------------------------------
// Sent over WiFi or USB.  Uses float/double - no conversion before use.
//
// Offset  Size  Field
//      0     8  lat
//      8     8  lon
//     16     8  utc_unix_ms
//     24     4  boot_ms
//     28     4  alt_gps_m
//     32     4  alt_baro_m
//     36     4  pressure_pa
//     40     4  temp_c
//     44     4  mach
//     48    16  q[4]
//     64    12  rpy_deg[3]
//     76    12  accel_g[3]
//     88    12  gyro_dps[3]
//    100    12  vel_ned_ms[3]
//    112     1  state
//    113     1  satellites
//    114     1  flags
//    115     5  _pad[5]
//    ----   --
//    120 bytes
struct WiFi {
    double   lat;
    double   lon;
    uint64_t utc_unix_ms;   ///< UTC ms since Unix epoch
    uint32_t boot_ms;
    float    alt_gps_m;
    float    alt_baro_m;
    float    pressure_pa;
    float    temp_c;
    float    mach;
    float    q[4];          ///< quaternion [w,x,y,z]
    float    rpy_deg[3];    ///< roll / pitch / yaw, degrees
    float    accel_g[3];    ///< body-frame acceleration, g
    float    gyro_dps[3];   ///< angular rate, °/s
    float    vel_ned_ms[3]; ///< NED velocity, m/s
    uint8_t  state;
    uint8_t  satellites;
    uint8_t  flags;
    uint8_t  _pad[5];
};
static_assert(sizeof(WiFi) == 120, "Wire::WiFi layout changed");

// -- Full Storage Record -------------------------------------------------------
// Maximum-fidelity record for SD card / flash.
//
// Offset  Size  Field
//      0     8  lat
//      8     8  lon
//     16     8  utc_unix_ms
//     24     4  boot_ms
//     28     4  alt_gps_m
//     32     4  alt_baro_m
//     36     4  pressure_pa
//     40     4  temp_c
//     44     4  mach
//     48    16  q[4]
//     64    12  rpy_deg[3]
//     76    12  accel_g[3]
//     88    12  gyro_dps[3]
//    100    12  mag_uT[3]
//    112    12  vel_ned_ms[3]
//    124     1  state
//    125     1  satellites
//    126     1  flags
//    127     1  _pad
//    ----   --
//    128 bytes  (power-of-2 for flash page alignment)
struct StorageFull {
    double   lat;
    double   lon;
    uint64_t utc_unix_ms;
    uint32_t boot_ms;
    float    alt_gps_m;
    float    alt_baro_m;
    float    pressure_pa;
    float    temp_c;
    float    mach;
    float    q[4];
    float    rpy_deg[3];
    float    accel_g[3];
    float    gyro_dps[3];
    float    mag_uT[3];
    float    vel_ned_ms[3];
    uint8_t  state;
    uint8_t  satellites;
    uint8_t  flags;
    uint8_t  _pad;
};
static_assert(sizeof(StorageFull) == 128, "Wire::StorageFull layout changed");

// -- High-rate IMU Storage Record ----------------------------------------------
// Written at IMU sample rate (500 Hz–4 kHz) to a flash ring-buffer.
// Compact: scaled integers only.
//
// Offset  Size  Field
//      0     4  boot_us
//      4     8  q[4]           int16_t Q1.15 x 4
//     12     6  accel_mg[3]
//     18     6  gyro_ddps[3]   0.1 °/s units
//     24     1  state
//     25     1  flags
//     26     2  _pad[2]
//    ----   --
//     28 bytes
struct StorageIMU {
    uint32_t boot_us;       ///< µs since power-on (rolls ~71 min)
    int16_t  q[4];          ///< quaternion Q1.15
    int16_t  accel_mg[3];   ///< acceleration, mg   (÷1000 -> g)
    int16_t  gyro_ddps[3];  ///< angular rate, 0.1°/s (÷10 -> °/s)
    uint8_t  state;
    uint8_t  flags;
    uint8_t  _pad[2];
};
static_assert(sizeof(StorageIMU) == 28, "Wire::StorageIMU layout changed");

// -- Heartbeat -----------------------------------------------------------------
// Used pre-launch for bidirectional link characterisation.
// Both ground and rocket send these; each side echoes the sender's seq back
// in rx_seq so the originator can compute round-trip time.
//
// RSSI/SNR fields carry what *this sender heard* from the last packet it
// received from the other side — so each Heartbeat gives the remote node
// its own receive quality without a separate ACK.
//
// Offset  Size  Field
//      0     4  boot_ms
//      4     4  tx_time_ms     absolute time this frame was sent (for RTT)
//      8     2  seq            monotonic counter, wraps freely
//     10     2  rx_seq         echoed seq from last received heartbeat
//     12     1  state          FlightState
//     13     1  flags          FLAG_* validity bits
//     14     1  rx_rssi        RSSI of last received heartbeat, dBm (int8_t)
//     15     1  rx_snr_q2      SNR of last received heartbeat × 4 (int8_t, ÷4 -> dB)
//    ----   --
//     16 bytes
struct Heartbeat {
    uint32_t boot_ms;       ///< sender boot time, ms
    uint32_t tx_time_ms;    ///< sender's clock at transmit (for RTT calc)
    uint16_t seq;           ///< this frame's sequence number
    uint16_t rx_seq;        ///< seq echoed from last received heartbeat
    uint8_t  state;         ///< FlightState
    uint8_t  flags;         ///< FLAG_* validity bits
    int8_t   rx_rssi;       ///< RSSI of last received heartbeat, dBm
    int8_t   rx_snr_q2;     ///< SNR of last received heartbeat × 4 (÷4 -> dB)
};
static_assert(sizeof(Heartbeat) == 16, "Wire::Heartbeat layout changed");

// -- Time Sync -----------------------------------------------------------------
// Occasional beacon to let the ground station synchronise its software clock.
// Carries GPS time-of-week, UTC Unix epoch in ms, and the sender's boot
// microsecond counter so the receiver can correlate all three time bases.
//
// utc_unix_ms split into lo/hi uint32_t to avoid uint64_t alignment padding.
// Reassemble: (uint64_t)utc_ms_hi << 32 | utc_ms_lo
//
// Offset  Size  Field
//      0     4  boot_us        µs since sender power-on (low 32 bits of time_us_64())
//      4     4  gps_tow_ms     GPS time-of-week, ms (0..604800000)
//      8     4  utc_ms_lo      UTC Unix ms, low  32 bits
//     12     4  utc_ms_hi      UTC Unix ms, high 32 bits
//     16     1  flags          FLAG_GPS_VALID | FLAG_TIME_VALID
//     17     3  _pad[3]
//    ----   --
//     20 bytes
struct TimeSync {
    uint32_t boot_us;      ///< µs since power-on (low 32 bits of time_us_64(), rolls ~71 min)
    uint32_t gps_tow_ms;   ///< GPS time-of-week, ms (0..604 800 000)
    uint32_t utc_ms_lo;    ///< UTC ms since Unix epoch, low  32 bits
    uint32_t utc_ms_hi;    ///< UTC ms since Unix epoch, high 32 bits
    uint8_t  flags;        ///< FLAG_GPS_VALID | FLAG_TIME_VALID
    uint8_t  _pad[3];
};
static_assert(sizeof(TimeSync) == 20, "Wire::TimeSync layout changed");

// -- GPS Navigation ------------------------------------------------------------
// Position + NED velocity.  Lean over-the-air packet focused on navigation.
//
// Offset  Size  Field
//      0     4  lat_deg_e7     degrees x 10^7  (÷10^7 -> °)
//      4     4  lon_deg_e7     degrees x 10^7  (÷10^7 -> °)
//      8     4  alt_gps_cm     GPS MSL altitude, cm  (÷100 -> m)
//     12     2  vel_n_cms      NED north velocity, cm/s  (÷100 -> m/s)
//     14     2  vel_e_cms      NED east  velocity, cm/s
//     16     2  vel_d_cms      NED down  velocity, cm/s
//     18     1  satellites
//     19     1  flags
//    ----   --
//     20 bytes
struct GpsNav {
    int32_t  lat_deg_e7;   ///< latitude  x 10^7 (÷10^7 -> °, WGS-84)
    int32_t  lon_deg_e7;   ///< longitude x 10^7 (÷10^7 -> °, WGS-84)
    int32_t  alt_gps_cm;   ///< GPS MSL altitude, cm (÷100 -> m)
    int16_t  vel_n_cms;    ///< NED north velocity, cm/s (÷100 -> m/s)
    int16_t  vel_e_cms;    ///< NED east  velocity, cm/s
    int16_t  vel_d_cms;    ///< NED down  velocity, cm/s
    uint8_t  satellites;
    uint8_t  flags;
};
static_assert(sizeof(GpsNav) == 20, "Wire::GpsNav layout changed");

// -- Navigation State ----------------------------------------------------------
// Fused altitude, fused NED velocity, and attitude quaternion.  Sent alongside
// GpsNav to give the ground station a complete navigation picture.
//
// Offset  Size  Field
//      0     4  alt_baro_dm    baro altitude, decimetres  (÷10 -> m)
//      4     4  alt_fused_dm   fused altitude, decimetres (÷10 -> m)
//      8     8  q[4]           quaternion [w,x,y,z] Q1.15 (int16_t x4, ÷32767 -> [-1,+1])
//     16     2  vel_n_cms      fused NED north velocity, cm/s
//     18     2  vel_e_cms      fused NED east  velocity, cm/s
//     20     2  vel_d_cms      fused NED down  velocity, cm/s
//     22     1  state          FlightState
//     23     1  flags
//    ----   --
//     24 bytes
struct NavState {
    int32_t  alt_baro_dm;  ///< baro altitude, decimetres   (÷10 -> m)
    int32_t  alt_fused_dm; ///< fused altitude, decimetres  (÷10 -> m)
    int16_t  q[4];         ///< quaternion [w,x,y,z] Q1.15  (÷32767 -> [-1,+1])
    int16_t  vel_n_cms;    ///< fused NED north velocity, cm/s (÷100 -> m/s)
    int16_t  vel_e_cms;    ///< fused NED east  velocity, cm/s
    int16_t  vel_d_cms;    ///< fused NED down  velocity, cm/s
    uint8_t  state;        ///< FlightState
    uint8_t  flags;
};
static_assert(sizeof(NavState) == 24, "Wire::NavState layout changed");

} // namespace Wire

// ===============================================================================
// CRC-16/CCITT  (polynomial 0x1021, init 0xFFFF)
// ===============================================================================

inline uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < 8; ++i)
        crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                               : static_cast<uint16_t>(crc << 1);
    return crc;
}

inline uint16_t crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
        crc = crc16_update(crc, data[i]);
    return crc;
}

// ===============================================================================
// Framing - serialize / deserialize / find_frame
// ===============================================================================

template <typename Payload>
inline size_t serialize(PacketType type, uint8_t seq, const Payload& payload,
                        uint8_t* buf, size_t buf_len)
{
    const size_t plen      = sizeof(Payload);
    const size_t frame_len = plen + FRAME_OVERHEAD;
    if (buf_len < frame_len) return 0;

    size_t i = 0;
    buf[i++] = FRAME_START_0;
    buf[i++] = FRAME_START_1;
    buf[i++] = static_cast<uint8_t>(type);
    buf[i++] = seq;
    buf[i++] = static_cast<uint8_t>(plen & 0xFFu);
    buf[i++] = static_cast<uint8_t>((plen >> 8) & 0xFFu);
    memcpy(buf + i, &payload, plen);
    i += plen;

    // CRC covers: type(1) + seq(1) + len(2) + payload(plen)
    const uint16_t c = crc16(buf + 2, 4 + plen);
    buf[i++] = static_cast<uint8_t>(c & 0xFFu);
    buf[i++] = static_cast<uint8_t>((c >> 8) & 0xFFu);
    buf[i++] = FRAME_END_0;
    buf[i++] = FRAME_END_1;
    return i;
}

// Deserialize a frame and return the sequence number via out_seq.
template <typename Payload>
inline bool deserialize(const uint8_t* buf, size_t buf_len,
                        PacketType expected_type, Payload& out,
                        uint8_t* out_seq = nullptr)
{
    const size_t plen  = sizeof(Payload);
    const size_t total = plen + FRAME_OVERHEAD;

    if (buf_len < total)                                                return false;
    if (buf[0] != FRAME_START_0 || buf[1] != FRAME_START_1)            return false;
    if (static_cast<PacketType>(buf[2]) != expected_type)               return false;

    // buf[3] = seq — extract before length check
    const uint8_t seq = buf[3];

    const uint16_t got_plen = static_cast<uint16_t>(buf[4]) |
                              (static_cast<uint16_t>(buf[5]) << 8);
    if (got_plen != plen)                                               return false;

    // CRC covers: type(1) + seq(1) + len(2) + payload(plen)
    const uint16_t expected_crc = crc16(buf + 2, 4 + plen);
    const uint16_t actual_crc   = static_cast<uint16_t>(buf[6 + plen]) |
                                  (static_cast<uint16_t>(buf[7 + plen]) << 8);
    if (actual_crc != expected_crc)                                     return false;
    if (buf[8 + plen] != FRAME_END_0 || buf[9 + plen] != FRAME_END_1)  return false;

    memcpy(&out, buf + 6, plen);
    if (out_seq) *out_seq = seq;
    return true;
}

// Scan a stream buffer for the next start magic pair.
// Returns offset of first AA 55, or buf_len if not found.
inline size_t find_frame(const uint8_t* buf, size_t buf_len)
{
    for (size_t i = 0; i + 1 < buf_len; ++i)
        if (buf[i] == FRAME_START_0 && buf[i + 1] == FRAME_START_1)
            return i;
    return buf_len;
}

// ===============================================================================
// Unit-conversion helpers   (SIGMA::Convert)
// ===============================================================================

namespace Convert {

inline int32_t  lat_to_wire(double deg)      { return static_cast<int32_t>(deg * LATLON_SCALE); }
inline int32_t  lon_to_wire(double deg)      { return static_cast<int32_t>(deg * LATLON_SCALE); }
inline double   lat_from_wire(int32_t v)     { return static_cast<double>(v)  / LATLON_SCALE; }
inline double   lon_from_wire(int32_t v)     { return static_cast<double>(v)  / LATLON_SCALE; }

inline int32_t  gps_alt_to_wire(float m)     { return static_cast<int32_t>(m * GPS_ALT_SCALE); }
inline float    gps_alt_from_wire(int32_t v) { return static_cast<float>(v)  / static_cast<float>(GPS_ALT_SCALE); }

inline int32_t  baro_m_to_dm(float m)        { return static_cast<int32_t>(m * 10.0f); }
inline float    baro_dm_to_m(int32_t dm)     { return static_cast<float>(dm) * 0.1f; }

inline int16_t  angle_to_wire(float deg)     { return static_cast<int16_t>(deg * ANGLE_SCALE); }
inline float    angle_from_wire(int16_t v)   { return static_cast<float>(v)  / ANGLE_SCALE; }

inline int16_t  quat_to_wire(float q)        { return static_cast<int16_t>(q  * QUAT_SCALE); }
inline float    quat_from_wire(int16_t v)    { return static_cast<float>(v)  / QUAT_SCALE; }

inline uint16_t speed_to_wire(float ms)      { return static_cast<uint16_t>(ms   * SPEED_SCALE); }
inline float    speed_from_wire(uint16_t v)  { return static_cast<float>(v)  / SPEED_SCALE; }

inline uint16_t mach_to_wire(float mach)     { return static_cast<uint16_t>(mach * MACH_SCALE); }
inline float    mach_from_wire(uint16_t v)   { return static_cast<float>(v)  / MACH_SCALE; }

inline int16_t  accel_to_wire(float g)       { return static_cast<int16_t>(g   * ACCEL_SCALE); }
inline float    accel_from_wire(int16_t v)   { return static_cast<float>(v)  / ACCEL_SCALE; }

inline int16_t  gyro_to_wire(float dps)      { return static_cast<int16_t>(dps * GYRO_SCALE); }
inline float    gyro_from_wire(int16_t v)    { return static_cast<float>(v)  / GYRO_SCALE; }

inline float    snr_from_wire(int8_t v)      { return static_cast<float>(v)  * 0.25f; }
inline int8_t   snr_to_wire(float dB)        { return static_cast<int8_t>(dB * 4.0f); }

} // namespace Convert

// ===============================================================================
// High-level builder/decoder types   (physical units, human-friendly API)
//
// Usage (transmit):
//   SIGMA::LoRaData d;
//   d.boot_ms  = to_ms_since_boot(get_absolute_time());
//   d.lat      = gps.lat;  d.lon = gps.lon;
//   d.alt_gps_m = gps.alt_m;
//   d.alt_baro_m = baro.alt_m;
//   d.q[0..3]  = { qw, qx, qy, qz };
//   d.speed_ms = vel_mag;
//   d.state    = SIGMA::FlightState::POWERED_ASCENT;
//   d.flags    = SIGMA::FLAG_GPS_VALID | SIGMA::FLAG_BARO_VALID;
//   uint8_t frame[64];
//   size_t n = d.serialize(frame, sizeof(frame));
//
// Usage (receive):
//   SIGMA::LoRaData d;
//   if (SIGMA::LoRaData::deserialize(buf, len, d)) { /* use d */ }
//
// Usage (secondary Pico - forward immediately on LoRa receive):
//   SIGMA::InterPicoData fwd = SIGMA::InterPicoData::from_lora(lora_data, rssi, snr);
//   uint8_t frame[64];
//   size_t n = fwd.serialize(frame, sizeof(frame));
//   udp_sendto(frame, n, PRIMARY_PICO_IP, INTER_PICO_UDP_PORT);
// ===============================================================================

// -- LoRa builder -------------------------------------------------------------

struct LoRaData {
    uint32_t    boot_ms     = 0;
    double      lat         = 0.0;
    double      lon         = 0.0;
    float       alt_baro_m  = 0.0f;
    float       alt_gps_m   = 0.0f;
    float       q[4]        = {1,0,0,0};  ///< [w,x,y,z]
    float       speed_ms    = 0.0f;
    FlightState state       = FlightState::GROUND_IDLE;
    uint8_t     satellites  = 0;
    uint8_t     flags       = 0;

    Wire::LoRa convert() const {
        Wire::LoRa p;
        p.boot_ms     = boot_ms;
        p.lat_deg_e7  = Convert::lat_to_wire(lat);
        p.lon_deg_e7  = Convert::lon_to_wire(lon);
        p.alt_baro_dm = Convert::baro_m_to_dm(alt_baro_m);
        p.alt_gps_cm  = Convert::gps_alt_to_wire(alt_gps_m);
        for (int i = 0; i < 4; ++i) p.q[i] = Convert::quat_to_wire(q[i]);
        p.speed_cms   = Convert::speed_to_wire(speed_ms);
        p.state       = static_cast<uint8_t>(state);
        p.satellites  = satellites;
        p.flags       = flags;
        p._pad[0] = p._pad[1] = p._pad[2] = 0;
        return p;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::LoRa p = convert();
        return SIGMA::serialize(PacketType::LORA_TELEMETRY, s_seq++, p, buf, buf_len);
    }

    static LoRaData from_wire(const Wire::LoRa& p) {
        LoRaData d;
        d.boot_ms    = p.boot_ms;
        d.lat        = Convert::lat_from_wire(p.lat_deg_e7);
        d.lon        = Convert::lon_from_wire(p.lon_deg_e7);
        d.alt_baro_m = Convert::baro_dm_to_m(p.alt_baro_dm);
        d.alt_gps_m  = Convert::gps_alt_from_wire(p.alt_gps_cm);
        for (int i = 0; i < 4; ++i) d.q[i] = Convert::quat_from_wire(p.q[i]);
        d.speed_ms   = Convert::speed_from_wire(p.speed_cms);
        d.state      = static_cast<FlightState>(p.state);
        d.satellites = p.satellites;
        d.flags      = p.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, LoRaData& out) {
        Wire::LoRa p;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::LORA_TELEMETRY, p)) return false;
        out = from_wire(p);
        return true;
    }
};

// -- Inter-Pico builder --------------------------------------------------------
// Construct immediately from a decoded LoRaData + radio metadata and send.
// No buffering - forward on every received LoRa packet.

struct InterPicoData {
    // LoRa fields (identical physical units to LoRaData)
    uint32_t    boot_ms     = 0;
    double      lat         = 0.0;
    double      lon         = 0.0;
    float       alt_baro_m  = 0.0f;
    float       alt_gps_m   = 0.0f;
    float       q[4]        = {1,0,0,0};
    float       speed_ms    = 0.0f;
    float       vel_ned_ms[3] = {0.0f, 0.0f, 0.0f};
    float       h_acc_m     = 0.0f;
    float       v_acc_m     = 0.0f;
    float       s_acc_ms    = 0.0f;
    float       acc_ned_ms2[3] = {0.0f, 0.0f, 0.0f};
    uint8_t     nav_source  = 0;
    FlightState state       = FlightState::GROUND_IDLE;
    uint8_t     satellites  = 0;
    uint8_t     flags       = 0;
    // Radio metadata
    int         rssi        = 0;    ///< dBm (from radio driver)
    float       snr_dB      = 0.0f; ///< dB  (from radio driver)

    /// Build directly from a decoded LoRa packet + radio metadata.
    /// Call this in the LoRa receive handler and send immediately.
    static InterPicoData from_lora(const LoRaData& lora, int rssi_dBm, float snr_dB) {
        InterPicoData d;
        d.boot_ms    = lora.boot_ms;
        d.lat        = lora.lat;
        d.lon        = lora.lon;
        d.alt_baro_m = lora.alt_baro_m;
        d.alt_gps_m  = lora.alt_gps_m;
        for (int i = 0; i < 4; ++i) d.q[i] = lora.q[i];
        d.speed_ms   = lora.speed_ms;
        d.state      = lora.state;
        d.satellites = lora.satellites;
        d.flags      = lora.flags;
        d.rssi       = rssi_dBm;
        d.snr_dB     = snr_dB;
        return d;
    }

    Wire::InterPico convert() const {
        Wire::InterPico p;
        p.boot_ms     = boot_ms;
        p.lat_deg_e7  = Convert::lat_to_wire(lat);
        p.lon_deg_e7  = Convert::lon_to_wire(lon);
        p.alt_baro_dm = Convert::baro_m_to_dm(alt_baro_m);
        p.alt_gps_cm  = Convert::gps_alt_to_wire(alt_gps_m);
        for (int i = 0; i < 4; ++i) p.q[i] = Convert::quat_to_wire(q[i]);
        p.speed_cms   = Convert::speed_to_wire(speed_ms);
        p.vel_n_cms   = static_cast<int16_t>(vel_ned_ms[0] * SPEED_SCALE);
        p.vel_e_cms   = static_cast<int16_t>(vel_ned_ms[1] * SPEED_SCALE);
        p.vel_d_cms   = static_cast<int16_t>(vel_ned_ms[2] * SPEED_SCALE);
        p.h_acc_cm    = static_cast<uint16_t>(h_acc_m * GPS_ALT_SCALE);
        p.v_acc_cm    = static_cast<uint16_t>(v_acc_m * GPS_ALT_SCALE);
        p.s_acc_cms   = static_cast<uint16_t>(s_acc_ms * SPEED_SCALE);
        p.acc_n_mg    = static_cast<int16_t>((acc_ned_ms2[0] / 9.80665f) * ACCEL_SCALE);
        p.acc_e_mg    = static_cast<int16_t>((acc_ned_ms2[1] / 9.80665f) * ACCEL_SCALE);
        p.acc_d_mg    = static_cast<int16_t>((acc_ned_ms2[2] / 9.80665f) * ACCEL_SCALE);
        p.nav_source  = nav_source;
        p.state       = static_cast<uint8_t>(state);
        p.satellites  = satellites;
        p.flags       = flags;
        p.rssi        = static_cast<int8_t>(rssi);
        p.snr_q2      = Convert::snr_to_wire(snr_dB);
        p._pad[0] = p._pad[1] = p._pad[2] = 0;
        return p;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::InterPico p = convert();
        return SIGMA::serialize(PacketType::INTER_PICO, s_seq++, p, buf, buf_len);
    }

    static InterPicoData from_wire(const Wire::InterPico& p) {
        InterPicoData d;
        d.boot_ms    = p.boot_ms;
        d.lat        = Convert::lat_from_wire(p.lat_deg_e7);
        d.lon        = Convert::lon_from_wire(p.lon_deg_e7);
        d.alt_baro_m = Convert::baro_dm_to_m(p.alt_baro_dm);
        d.alt_gps_m  = Convert::gps_alt_from_wire(p.alt_gps_cm);
        for (int i = 0; i < 4; ++i) d.q[i] = Convert::quat_from_wire(p.q[i]);
        d.speed_ms   = Convert::speed_from_wire(p.speed_cms);
        d.vel_ned_ms[0] = static_cast<float>(p.vel_n_cms) / SPEED_SCALE;
        d.vel_ned_ms[1] = static_cast<float>(p.vel_e_cms) / SPEED_SCALE;
        d.vel_ned_ms[2] = static_cast<float>(p.vel_d_cms) / SPEED_SCALE;
        d.h_acc_m    = static_cast<float>(p.h_acc_cm) / GPS_ALT_SCALE;
        d.v_acc_m    = static_cast<float>(p.v_acc_cm) / GPS_ALT_SCALE;
        d.s_acc_ms   = static_cast<float>(p.s_acc_cms) / SPEED_SCALE;
        d.acc_ned_ms2[0] = (static_cast<float>(p.acc_n_mg) / ACCEL_SCALE) * 9.80665f;
        d.acc_ned_ms2[1] = (static_cast<float>(p.acc_e_mg) / ACCEL_SCALE) * 9.80665f;
        d.acc_ned_ms2[2] = (static_cast<float>(p.acc_d_mg) / ACCEL_SCALE) * 9.80665f;
        d.nav_source = p.nav_source;
        d.state      = static_cast<FlightState>(p.state);
        d.satellites = p.satellites;
        d.flags      = p.flags;
        d.rssi       = p.rssi;
        d.snr_dB     = Convert::snr_from_wire(p.snr_q2);
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, InterPicoData& out) {
        Wire::InterPico p;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::INTER_PICO, p)) return false;
        out = from_wire(p);
        return true;
    }
};

// -- WiFi / full-rate builder --------------------------------------------------

struct WiFiData {
    double      lat          = 0.0;
    double      lon          = 0.0;
    uint64_t    utc_unix_ms  = 0;
    uint32_t    boot_ms      = 0;
    float       alt_gps_m    = 0.0f;
    float       alt_baro_m   = 0.0f;
    float       pressure_pa  = 0.0f;
    float       temp_c       = 0.0f;
    float       mach         = 0.0f;
    float       q[4]         = {1,0,0,0};
    float       rpy_deg[3]   = {0,0,0};
    float       accel_g[3]   = {0,0,0};
    float       gyro_dps[3]  = {0,0,0};
    float       vel_ned_ms[3]= {0,0,0};
    FlightState state        = FlightState::GROUND_IDLE;
    uint8_t     satellites   = 0;
    uint8_t     flags        = 0;

    Wire::WiFi convert() const {
        Wire::WiFi p;
        p.lat         = lat;          p.lon         = lon;
        p.utc_unix_ms = utc_unix_ms;  p.boot_ms     = boot_ms;
        p.alt_gps_m   = alt_gps_m;   p.alt_baro_m  = alt_baro_m;
        p.pressure_pa = pressure_pa;  p.temp_c      = temp_c;
        p.mach        = mach;
        for (int i = 0; i < 4; ++i) p.q[i]          = q[i];
        for (int i = 0; i < 3; ++i) p.rpy_deg[i]    = rpy_deg[i];
        for (int i = 0; i < 3; ++i) p.accel_g[i]    = accel_g[i];
        for (int i = 0; i < 3; ++i) p.gyro_dps[i]   = gyro_dps[i];
        for (int i = 0; i < 3; ++i) p.vel_ned_ms[i] = vel_ned_ms[i];
        p.state       = static_cast<uint8_t>(state);
        p.satellites  = satellites;  p.flags = flags;
        for (int i = 0; i < 5; ++i) p._pad[i] = 0;
        return p;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::WiFi p = convert();
        return SIGMA::serialize(PacketType::WIFI_TELEMETRY, s_seq++, p, buf, buf_len);
    }

    static WiFiData from_wire(const Wire::WiFi& p) {
        WiFiData d;
        d.lat         = p.lat;        d.lon         = p.lon;
        d.utc_unix_ms = p.utc_unix_ms; d.boot_ms    = p.boot_ms;
        d.alt_gps_m   = p.alt_gps_m;  d.alt_baro_m = p.alt_baro_m;
        d.pressure_pa = p.pressure_pa; d.temp_c     = p.temp_c;
        d.mach        = p.mach;
        for (int i = 0; i < 4; ++i) d.q[i]          = p.q[i];
        for (int i = 0; i < 3; ++i) d.rpy_deg[i]    = p.rpy_deg[i];
        for (int i = 0; i < 3; ++i) d.accel_g[i]    = p.accel_g[i];
        for (int i = 0; i < 3; ++i) d.gyro_dps[i]   = p.gyro_dps[i];
        for (int i = 0; i < 3; ++i) d.vel_ned_ms[i] = p.vel_ned_ms[i];
        d.state       = static_cast<FlightState>(p.state);
        d.satellites  = p.satellites; d.flags = p.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, WiFiData& out) {
        Wire::WiFi p;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::WIFI_TELEMETRY, p)) return false;
        out = from_wire(p);
        return true;
    }
};

// -- IMU storage builder -------------------------------------------------------

struct IMUData {
    uint32_t    boot_us     = 0;
    float       q[4]        = {1,0,0,0};
    float       accel_g[3]  = {0,0,0};
    float       gyro_dps[3] = {0,0,0};
    FlightState state       = FlightState::GROUND_IDLE;
    uint8_t     flags       = 0;

    Wire::StorageIMU convert() const {
        Wire::StorageIMU r;
        r.boot_us = boot_us;
        for (int i = 0; i < 4; ++i) r.q[i]         = Convert::quat_to_wire(q[i]);
        for (int i = 0; i < 3; ++i) r.accel_mg[i]  = Convert::accel_to_wire(accel_g[i]);
        for (int i = 0; i < 3; ++i) r.gyro_ddps[i] = Convert::gyro_to_wire(gyro_dps[i]);
        r.state = static_cast<uint8_t>(state);
        r.flags = flags;
        r._pad[0] = r._pad[1] = 0;
        return r;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::StorageIMU r = convert();
        return SIGMA::serialize(PacketType::STORAGE_IMU, s_seq++, r, buf, buf_len);
    }

    static IMUData from_wire(const Wire::StorageIMU& r) {
        IMUData d;
        d.boot_us = r.boot_us;
        for (int i = 0; i < 4; ++i) d.q[i]        = Convert::quat_from_wire(r.q[i]);
        for (int i = 0; i < 3; ++i) d.accel_g[i]  = Convert::accel_from_wire(r.accel_mg[i]);
        for (int i = 0; i < 3; ++i) d.gyro_dps[i] = Convert::gyro_from_wire(r.gyro_ddps[i]);
        d.state = static_cast<FlightState>(r.state);
        d.flags = r.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, IMUData& out) {
        Wire::StorageIMU r;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::STORAGE_IMU, r)) return false;
        out = from_wire(r);
        return true;
    }
};

// -- Time Sync builder ---------------------------------------------------------
// Sent occasionally (e.g. once per second) to let the ground station align
// its software clock to GPS UTC and correlate boot_us timestamps.
//
// Usage (transmit):
//   SIGMA::TimeSyncData ts;
//   ts.boot_us     = (uint32_t)time_us_64();
//   ts.gps_tow_ms  = gps.time_of_week_ms;
//   ts.utc_unix_ms = gps.utc_unix_ms;
//   ts.flags       = SIGMA::FLAG_GPS_VALID | SIGMA::FLAG_TIME_VALID;
//   uint8_t frame[32];
//   size_t n = ts.serialize(frame, sizeof(frame));

struct TimeSyncData {
    uint32_t boot_us      = 0;   ///< low 32 bits of time_us_64() at transmit time
    uint32_t gps_tow_ms   = 0;   ///< GPS time-of-week, ms
    uint64_t utc_unix_ms  = 0;   ///< UTC ms since Unix epoch
    uint8_t  flags        = 0;   ///< FLAG_GPS_VALID | FLAG_TIME_VALID

    Wire::TimeSync convert() const {
        Wire::TimeSync p;
        p.boot_us    = boot_us;
        p.gps_tow_ms = gps_tow_ms;
        p.utc_ms_lo  = static_cast<uint32_t>(utc_unix_ms & 0xFFFFFFFFu);
        p.utc_ms_hi  = static_cast<uint32_t>(utc_unix_ms >> 32);
        p.flags      = flags;
        p._pad[0] = p._pad[1] = p._pad[2] = 0;
        return p;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::TimeSync p = convert();
        return SIGMA::serialize(PacketType::TIME_SYNC, s_seq++, p, buf, buf_len);
    }

    static TimeSyncData from_wire(const Wire::TimeSync& p) {
        TimeSyncData d;
        d.boot_us     = p.boot_us;
        d.gps_tow_ms  = p.gps_tow_ms;
        d.utc_unix_ms = static_cast<uint64_t>(p.utc_ms_hi) << 32 | p.utc_ms_lo;
        d.flags       = p.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, TimeSyncData& out) {
        Wire::TimeSync p;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::TIME_SYNC, p)) return false;
        out = from_wire(p);
        return true;
    }
};

// -- GPS Navigation builder ----------------------------------------------------
// Position + NED velocity from the GPS fix.
//
// Usage (transmit):
//   SIGMA::GpsNavData gn;
//   gn.lat        = gps.lat;  gn.lon = gps.lon;
//   gn.alt_gps_m  = gps.alt_m;
//   gn.vel_ned_ms = { gps.vel_n, gps.vel_e, gps.vel_d };
//   gn.satellites = gps.sats;
//   gn.flags      = SIGMA::FLAG_GPS_VALID;
//   uint8_t frame[32];
//   size_t n = gn.serialize(frame, sizeof(frame));

struct GpsNavData {
    double   lat         = 0.0;
    double   lon         = 0.0;
    float    alt_gps_m   = 0.0f;
    float    vel_ned_ms[3] = {0,0,0};  ///< NED velocity [N, E, D], m/s
    uint8_t  satellites  = 0;
    uint8_t  flags       = 0;

    Wire::GpsNav convert() const {
        Wire::GpsNav p;
        p.lat_deg_e7 = Convert::lat_to_wire(lat);
        p.lon_deg_e7 = Convert::lon_to_wire(lon);
        p.alt_gps_cm = Convert::gps_alt_to_wire(alt_gps_m);
        p.vel_n_cms  = static_cast<int16_t>(vel_ned_ms[0] * 100.0f);
        p.vel_e_cms  = static_cast<int16_t>(vel_ned_ms[1] * 100.0f);
        p.vel_d_cms  = static_cast<int16_t>(vel_ned_ms[2] * 100.0f);
        p.satellites = satellites;
        p.flags      = flags;
        return p;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::GpsNav p = convert();
        return SIGMA::serialize(PacketType::GPS_NAV, s_seq++, p, buf, buf_len);
    }

    static GpsNavData from_wire(const Wire::GpsNav& p) {
        GpsNavData d;
        d.lat          = Convert::lat_from_wire(p.lat_deg_e7);
        d.lon          = Convert::lon_from_wire(p.lon_deg_e7);
        d.alt_gps_m    = Convert::gps_alt_from_wire(p.alt_gps_cm);
        d.vel_ned_ms[0] = static_cast<float>(p.vel_n_cms) * 0.01f;
        d.vel_ned_ms[1] = static_cast<float>(p.vel_e_cms) * 0.01f;
        d.vel_ned_ms[2] = static_cast<float>(p.vel_d_cms) * 0.01f;
        d.satellites   = p.satellites;
        d.flags        = p.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, GpsNavData& out) {
        Wire::GpsNav p;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::GPS_NAV, p)) return false;
        out = from_wire(p);
        return true;
    }
};

// -- Navigation State builder --------------------------------------------------
// Fused baro/alt, fused NED velocity, and attitude quaternion from fusion_task.
//
// Usage (transmit):
//   SIGMA::NavStateData ns;
//   ns.alt_baro_m   = baro.alt_m;
//   ns.alt_fused_m  = fusion.alt_m;
//   ns.vel_ned_ms   = { fusion.vel_n, fusion.vel_e, fusion.vel_d };
//   ns.q            = { fusion.qw, fusion.qx, fusion.qy, fusion.qz };
//   ns.state        = current_state;
//   ns.flags        = SIGMA::FLAG_BARO_VALID | SIGMA::FLAG_IMU_VALID;
//   uint8_t frame[32];
//   size_t n = ns.serialize(frame, sizeof(frame));

struct NavStateData {
    float       alt_baro_m   = 0.0f;
    float       alt_fused_m  = 0.0f;
    float       vel_ned_ms[3]= {0,0,0};  ///< fused NED velocity [N, E, D], m/s
    float       q[4]         = {1,0,0,0}; ///< quaternion [w,x,y,z]
    FlightState state        = FlightState::GROUND_IDLE;
    uint8_t     flags        = 0;

    Wire::NavState convert() const {
        Wire::NavState p;
        p.alt_baro_dm  = Convert::baro_m_to_dm(alt_baro_m);
        p.alt_fused_dm = Convert::baro_m_to_dm(alt_fused_m);
        for (int i = 0; i < 4; ++i) p.q[i] = Convert::quat_to_wire(q[i]);
        p.vel_n_cms    = static_cast<int16_t>(vel_ned_ms[0] * 100.0f);
        p.vel_e_cms    = static_cast<int16_t>(vel_ned_ms[1] * 100.0f);
        p.vel_d_cms    = static_cast<int16_t>(vel_ned_ms[2] * 100.0f);
        p.state        = static_cast<uint8_t>(state);
        p.flags        = flags;
        return p;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::NavState p = convert();
        return SIGMA::serialize(PacketType::NAV_STATE, s_seq++, p, buf, buf_len);
    }

    static NavStateData from_wire(const Wire::NavState& p) {
        NavStateData d;
        d.alt_baro_m   = Convert::baro_dm_to_m(p.alt_baro_dm);
        d.alt_fused_m  = Convert::baro_dm_to_m(p.alt_fused_dm);
        for (int i = 0; i < 4; ++i) d.q[i] = Convert::quat_from_wire(p.q[i]);
        d.vel_ned_ms[0] = static_cast<float>(p.vel_n_cms) * 0.01f;
        d.vel_ned_ms[1] = static_cast<float>(p.vel_e_cms) * 0.01f;
        d.vel_ned_ms[2] = static_cast<float>(p.vel_d_cms) * 0.01f;
        d.state        = static_cast<FlightState>(p.state);
        d.flags        = p.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, NavStateData& out) {
        Wire::NavState p;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::NAV_STATE, p)) return false;
        out = from_wire(p);
        return true;
    }
};

// -- Heartbeat builder ---------------------------------------------------------
// Pre-launch link characterisation — sent by both ground and rocket.
//
// Typical usage on the sending side:
//   SIGMA::HeartbeatData hb;
//   hb.boot_ms    = to_ms_since_boot(get_absolute_time());
//   hb.tx_time_ms = hb.boot_ms;          // same clock for RTT
//   hb.seq        = s_seq++;
//   hb.rx_seq     = s_last_rx_seq;       // echo what we last received
//   hb.state      = current_state;
//   hb.flags      = current_flags;
//   hb.rx_rssi    = s_last_rx_rssi;      // what we heard from the other side
//   hb.rx_snr_dB  = s_last_rx_snr;
//   uint8_t frame[32];
//   size_t n = hb.serialize(frame, sizeof(frame));
//
// RTT on the receiving side:
//   float rtt_ms = (float)(now_ms - received.tx_time_ms);
//   // rx_rssi / rx_snr_dB tell you what the *sender* heard from you last.

struct HeartbeatData {
    uint32_t    boot_ms    = 0;
    uint32_t    tx_time_ms = 0;    ///< sender's clock at transmit
    uint16_t    seq        = 0;    ///< this frame's sequence number
    uint16_t    rx_seq     = 0;    ///< echoed seq from last received heartbeat
    FlightState state      = FlightState::GROUND_IDLE;
    uint8_t     flags      = 0;
    int         rx_rssi    = 0;    ///< RSSI of last received heartbeat, dBm
    float       rx_snr_dB  = 0.0f; ///< SNR of last received heartbeat, dB

    Wire::Heartbeat convert() const {
        Wire::Heartbeat h;
        h.boot_ms    = boot_ms;
        h.tx_time_ms = tx_time_ms;
        h.seq        = seq;
        h.rx_seq     = rx_seq;
        h.state      = static_cast<uint8_t>(state);
        h.flags      = flags;
        h.rx_rssi    = static_cast<int8_t>(rx_rssi);
        h.rx_snr_q2  = Convert::snr_to_wire(rx_snr_dB);
        return h;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        static uint8_t s_seq = 0;
        Wire::Heartbeat h = convert();
        return SIGMA::serialize(PacketType::HEARTBEAT, s_seq++, h, buf, buf_len);
    }

    static HeartbeatData from_wire(const Wire::Heartbeat& h) {
        HeartbeatData d;
        d.boot_ms    = h.boot_ms;
        d.tx_time_ms = h.tx_time_ms;
        d.seq        = h.seq;
        d.rx_seq     = h.rx_seq;
        d.state      = static_cast<FlightState>(h.state);
        d.flags      = h.flags;
        d.rx_rssi    = h.rx_rssi;
        d.rx_snr_dB  = Convert::snr_from_wire(h.rx_snr_q2);
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, HeartbeatData& out) {
        Wire::Heartbeat h;
        if (!SIGMA::deserialize(buf, buf_len, PacketType::HEARTBEAT, h)) return false;
        out = from_wire(h);
        return true;
    }
};

} // namespace SIGMA

// ===============================================================================
// Backwards-compatibility aliases
// Remove these once all call sites are updated to use SIGMA:: names.
// ===============================================================================

using SigmaPacketType         = SIGMA::PacketType;
using FlightState             = SIGMA::FlightState;
using SigmaLoRaPayload        = SIGMA::Wire::LoRa;
using SigmaWiFiPayload        = SIGMA::Wire::WiFi;
using SigmaStorageFullRecord  = SIGMA::Wire::StorageFull;
using SigmaStorageIMURecord   = SIGMA::Wire::StorageIMU;
using SigmaHeartbeatPayload   = SIGMA::Wire::Heartbeat;
using SigmaInterPicoPayload   = SIGMA::Wire::InterPico;
using SigmaLoRaData           = SIGMA::LoRaData;
using SigmaWiFiData           = SIGMA::WiFiData;
using SigmaIMUData            = SIGMA::IMUData;
static constexpr uint8_t SIGMA_FLAG_GPS_VALID  = SIGMA::FLAG_GPS_VALID;
static constexpr uint8_t SIGMA_FLAG_BARO_VALID = SIGMA::FLAG_BARO_VALID;
static constexpr uint8_t SIGMA_FLAG_IMU_VALID  = SIGMA::FLAG_IMU_VALID;
static constexpr uint8_t SIGMA_FLAG_MAG_VALID  = SIGMA::FLAG_MAG_VALID;
static constexpr uint8_t SIGMA_FLAG_TIME_VALID = SIGMA::FLAG_TIME_VALID;
