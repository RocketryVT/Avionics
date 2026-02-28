#pragma once

// SIGMA.hpp — Rocket Telemetry Packet Protocol
// Staticically Improbable Giga Muffin Aggregator
//
// Packet definitions for LoRa, WiFi, on-board flash, and SD card storage.
//
// Design rules
// ────────────
//   • No __packed__. Wire structs use naturally-aligned fixed-width types
//     ordered largest-to-smallest; the compiler adds zero implicit padding.
//     static_assert guards verify sizes at compile time.
//   • memcpy is used for all serialisation/deserialisation to satisfy
//     strict-aliasing rules and to work on any architecture.
//   • All multi-byte fields are little-endian (RP2350, ESP32, STM32 are all
//     little-endian; adjust sigma_serialize/deserialize if targeting BE).
//   • Scaled integers minimise wire size while retaining useful resolution.
//     See SigmaConvert:: helpers for physical ↔ wire conversions.
//
// Frame layout (all transports)
// ──────────────────────────────
//   [A5][5A]         start magic        2 bytes
//   [type]           SigmaPacketType    1 byte
//   [len_lo][len_hi] payload length     2 bytes  (little-endian)
//   [... payload ...]                   plen bytes
//   [crc_lo][crc_hi] CRC-16/CCITT       2 bytes  (covers type+len+payload)
//   [5A][A5]         end magic          2 bytes
//   ─────────────────────────────────────────────
//   Total overhead: 9 bytes
//
// LoRa note: the radio PHY already provides a CRC; the protocol CRC is
// redundant but enables receiver-side validation before application use.

#include <stddef.h>
#include <stdint.h>
#include <string.h>   // memcpy

// ─── Frame constants ──────────────────────────────────────────────────────────

static constexpr uint8_t  SIGMA_FRAME_START_0  = 0xA5;
static constexpr uint8_t  SIGMA_FRAME_START_1  = 0x5A;
static constexpr uint8_t  SIGMA_FRAME_END_0    = 0x5A;
static constexpr uint8_t  SIGMA_FRAME_END_1    = 0xA5;

static constexpr size_t   SIGMA_FRAME_OVERHEAD = 9;   // start(2)+type(1)+len(2)+crc(2)+end(2)
static constexpr size_t   SIGMA_MAX_PAYLOAD    = 246;
static constexpr size_t   SIGMA_MAX_FRAME      = SIGMA_MAX_PAYLOAD + SIGMA_FRAME_OVERHEAD;

// ─── Packet type ─────────────────────────────────────────────────────────────

enum class SigmaPacketType : uint8_t {
    LORA_TELEMETRY   = 0x01,   ///< Compact LoRa uplink  (~41 bytes on-air)
    WIFI_TELEMETRY   = 0x02,   ///< Full float telemetry (WiFi / USB)
    STORAGE_FULL     = 0x10,   ///< Maximum-fidelity SD / flash record
    STORAGE_IMU      = 0x11,   ///< High-rate IMU burst  (flash ring-buffer)
    COMMAND          = 0x20,   ///< Ground → rocket command
    ACK              = 0x21,   ///< Rocket → ground acknowledgement
    HEARTBEAT        = 0xFE,   ///< Keepalive / link-quality beacon
    INVALID          = 0xFF,
};

// ─── Flight state ─────────────────────────────────────────────────────────────

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

// ─── Data-validity flags (bitmask in flags field) ─────────────────────────────

static constexpr uint8_t SIGMA_FLAG_GPS_VALID   = (1u << 0);
static constexpr uint8_t SIGMA_FLAG_BARO_VALID  = (1u << 1);
static constexpr uint8_t SIGMA_FLAG_IMU_VALID   = (1u << 2);
static constexpr uint8_t SIGMA_FLAG_MAG_VALID   = (1u << 3);
static constexpr uint8_t SIGMA_FLAG_TIME_VALID  = (1u << 4);

// ─── Wire scaling constants ───────────────────────────────────────────────────
//
// All scaled-integer wire fields follow:
//   stored = physical × SCALE   (encode)
//   physical = stored ÷ SCALE   (decode)

/// GPS lat/lon: degrees × 10^7 → int32_t  (~1.1 cm resolution at equator)
static constexpr int32_t  SIGMA_LATLON_SCALE    = 10'000'000;

/// GPS altitude: metres → centimetres → int32_t
static constexpr int32_t  SIGMA_GPS_ALT_SCALE   = 100;

/// Euler angles: degrees × 10 → int16_t  (±3276.7°, res 0.1°)
static constexpr float    SIGMA_ANGLE_SCALE     = 10.0f;

/// Quaternion: Q1.15 fixed-point → int16_t  (range [-1, +1], res ~3×10⁻⁵)
static constexpr float    SIGMA_QUAT_SCALE      = 32767.0f;

/// Speed: m/s × 100 → uint16_t  (0–655.35 m/s, res 0.01 m/s)
/// Used in LoRa packet.  Covers supersonic rocket speeds (~400 m/s max).
static constexpr float    SIGMA_SPEED_SCALE     = 100.0f;

/// Mach: × 1000 → uint16_t  (0–65.535 Mach, res 0.001)
/// Used in WiFi / storage packets where the float field is unavailable.
static constexpr float    SIGMA_MACH_SCALE      = 1000.0f;

/// Acceleration: g → milli-g → int16_t  (±32.767 g, res 1 mg)
static constexpr float    SIGMA_ACCEL_SCALE     = 1000.0f;

/// Angular rate: deg/s × 10 → int16_t  (±3276.7 °/s, res 0.1 °/s)
static constexpr float    SIGMA_GYRO_SCALE      = 10.0f;

// ═══════════════════════════════════════════════════════════════════════════════
// Wire-format payload structs
//
// Fields are ordered largest-alignment-first so the compiler inserts no
// implicit padding.  Explicit _pad fields make the intent visible.
// static_assert lines catch any future reordering that would break the layout.
// ═══════════════════════════════════════════════════════════════════════════════

// ── LoRa Telemetry Payload ────────────────────────────────────────────────────
// Total on-air frame = 36 + 9 = 45 bytes.
// Sent at TX_PERIOD_MS; received by ground station.
// Attitude is a compressed quaternion (Q1.15); no redundant RPY angles.
// Speed is scalar magnitude in cm/s — direct, no Mach conversion needed.
//
// Offset  Size  Field
//      0     4  boot_ms
//      4     4  lat_deg_e7
//      8     4  lon_deg_e7
//     12     4  alt_baro_dm
//     16     4  alt_gps_cm
//     20     8  q[4]         (int16_t Q1.15 × 4)
//     28     2  speed_cms
//     30     1  state
//     31     1  satellites
//     32     1  flags
//     33     3  _pad[3]
//    ────    ──
//     36 bytes total
struct SigmaLoRaPayload {
    uint32_t boot_ms;       ///< ms since power-on
    int32_t  lat_deg_e7;    ///< latitude  × 10^7  (÷10^7 → °, WGS-84)
    int32_t  lon_deg_e7;    ///< longitude × 10^7  (÷10^7 → °, WGS-84)
    int32_t  alt_baro_dm;   ///< baro altitude, decimetres  (÷10 → m)
    int32_t  alt_gps_cm;    ///< GPS  altitude, centimetres (÷100 → m)
    int16_t  q[4];          ///< quaternion [w,x,y,z] Q1.15  (÷32767 → [-1,+1])
    uint16_t speed_cms;     ///< scalar speed, cm/s  (÷100 → m/s)
    uint8_t  state;         ///< FlightState
    uint8_t  satellites;    ///< GPS satellites in use
    uint8_t  flags;         ///< SIGMA_FLAG_* validity bits
    uint8_t  _pad[3];       ///< reserved — set to 0
};
static_assert(sizeof(SigmaLoRaPayload) == 36, "SigmaLoRaPayload layout changed");

// ── WiFi / Full Telemetry Payload ─────────────────────────────────────────────
// Sent at full rate over WiFi or USB.  Uses float/double for math convenience;
// no scaled-integer conversion needed before use in Kalman filters, etc.
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
//    ────   ──
//    120 bytes total
struct SigmaWiFiPayload {
    double   lat;           ///< latitude,  degrees (WGS-84)
    double   lon;           ///< longitude, degrees (WGS-84)
    uint64_t utc_unix_ms;   ///< UTC milliseconds since Unix epoch
    uint32_t boot_ms;       ///< ms since power-on
    float    alt_gps_m;     ///< GPS altitude, metres
    float    alt_baro_m;    ///< baro altitude, metres
    float    pressure_pa;   ///< static pressure, Pa
    float    temp_c;        ///< temperature, °C
    float    mach;          ///< Mach number
    float    q[4];          ///< quaternion [w, x, y, z]
    float    rpy_deg[3];    ///< roll / pitch / yaw, degrees
    float    accel_g[3];    ///< body-frame acceleration, g
    float    gyro_dps[3];   ///< angular rate, °/s
    float    vel_ned_ms[3]; ///< NED velocity, m/s
    uint8_t  state;         ///< FlightState
    uint8_t  satellites;    ///< GPS satellites in use
    uint8_t  flags;         ///< SIGMA_FLAG_* validity bits
    uint8_t  _pad[5];       ///< reserved — set to 0
};
static_assert(sizeof(SigmaWiFiPayload) == 120, "SigmaWiFiPayload layout changed");

// ── Full Storage Record (SD card / flash) ─────────────────────────────────────
// Maximum-fidelity record; written at full telemetry rate.
// Adds magnetometer and uses float for all dynamics.
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
//    ────   ──
//    128 bytes total (convenient power-of-2 for flash page alignment)
struct SigmaStorageFullRecord {
    double   lat;           ///< degrees
    double   lon;           ///< degrees
    uint64_t utc_unix_ms;   ///< UTC ms since Unix epoch
    uint32_t boot_ms;       ///< ms since power-on
    float    alt_gps_m;     ///< metres
    float    alt_baro_m;    ///< metres
    float    pressure_pa;   ///< Pa
    float    temp_c;        ///< °C
    float    mach;
    float    q[4];          ///< quaternion [w, x, y, z]
    float    rpy_deg[3];    ///< roll / pitch / yaw, degrees
    float    accel_g[3];    ///< body-frame acceleration, g
    float    gyro_dps[3];   ///< angular rate, °/s
    float    mag_uT[3];     ///< magnetometer, µT
    float    vel_ned_ms[3]; ///< NED velocity, m/s
    uint8_t  state;         ///< FlightState
    uint8_t  satellites;
    uint8_t  flags;         ///< SIGMA_FLAG_* validity bits
    uint8_t  _pad;          ///< reserved — set to 0
};
static_assert(sizeof(SigmaStorageFullRecord) == 128, "SigmaStorageFullRecord layout changed");

// ── High-rate IMU Storage Record ──────────────────────────────────────────────
// Written at IMU sample rate (500 Hz–4 kHz) to a flash ring-buffer.
// Compact: scaled integers only — no GPS/baro fields.
//
// Offset  Size  Field
//      0     4  boot_us
//      4     8  q[4]      (int16_t × 4)
//     12     6  accel_mg[3]
//     18     6  gyro_ddps[3]
//     24     1  state
//     25     1  flags
//     26     2  _pad[2]
//    ────   ──
//     28 bytes total
struct SigmaStorageIMURecord {
    uint32_t boot_us;       ///< µs since power-on  (rolls over at ~71 min; use boot_ms for epoch)
    int16_t  q[4];          ///< quaternion Q1.15   (÷32767 → [-1, +1])
    int16_t  accel_mg[3];   ///< acceleration, mg   (÷1000 → g)
    int16_t  gyro_ddps[3];  ///< angular rate, 0.1°/s (÷10 → °/s)
    uint8_t  state;         ///< FlightState
    uint8_t  flags;         ///< SIGMA_FLAG_* validity bits
    uint8_t  _pad[2];       ///< reserved — set to 0
};
static_assert(sizeof(SigmaStorageIMURecord) == 28, "SigmaStorageIMURecord layout changed");

// ── Heartbeat Payload ─────────────────────────────────────────────────────────

struct SigmaHeartbeatPayload {
    uint32_t boot_ms;
    uint8_t  state;         ///< FlightState
    uint8_t  flags;         ///< SIGMA_FLAG_* validity bits
    uint8_t  _pad[2];       ///< reserved — set to 0
};
static_assert(sizeof(SigmaHeartbeatPayload) == 8, "SigmaHeartbeatPayload layout changed");

// ═══════════════════════════════════════════════════════════════════════════════
// CRC-16/CCITT  (polynomial 0x1021, init value 0xFFFF)
// ═══════════════════════════════════════════════════════════════════════════════

inline uint16_t sigma_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < 8; ++i)
        crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                               : static_cast<uint16_t>(crc << 1);
    return crc;
}

inline uint16_t sigma_crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
        crc = sigma_crc16_update(crc, data[i]);
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Serialise / Deserialise
//
// sigma_serialize<Payload>  — encodes a payload struct into a framed byte buffer.
//   Returns total frame size on success, 0 if buf_len is too small.
//
// sigma_deserialize<Payload> — validates framing, CRC, and type, then copies the
//   payload into payload_out via memcpy.
//   Returns true on success, false on any validation failure.
// ═══════════════════════════════════════════════════════════════════════════════

template <typename Payload>
inline size_t sigma_serialize(SigmaPacketType  type,
                               const Payload&   payload,
                               uint8_t*         buf,
                               size_t           buf_len)
{
    const size_t plen      = sizeof(Payload);
    const size_t frame_len = plen + SIGMA_FRAME_OVERHEAD;
    if (buf_len < frame_len) return 0;

    size_t i = 0;
    buf[i++] = SIGMA_FRAME_START_0;
    buf[i++] = SIGMA_FRAME_START_1;
    buf[i++] = static_cast<uint8_t>(type);
    buf[i++] = static_cast<uint8_t>(plen & 0xFFu);
    buf[i++] = static_cast<uint8_t>((plen >> 8) & 0xFFu);
    memcpy(buf + i, &payload, plen);
    i += plen;

    // CRC covers: type(1) + len(2) + payload(plen) = buf[2..5+plen-1]
    const uint16_t crc = sigma_crc16(buf + 2, 3 + plen);
    buf[i++] = static_cast<uint8_t>(crc & 0xFFu);
    buf[i++] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
    buf[i++] = SIGMA_FRAME_END_0;
    buf[i++] = SIGMA_FRAME_END_1;
    return i;
}

template <typename Payload>
inline bool sigma_deserialize(const uint8_t*  buf,
                               size_t          buf_len,
                               SigmaPacketType expected_type,
                               Payload&        payload_out)
{
    const size_t expected_plen  = sizeof(Payload);
    const size_t expected_frame = expected_plen + SIGMA_FRAME_OVERHEAD;

    if (buf_len < expected_frame)                                     return false;
    if (buf[0] != SIGMA_FRAME_START_0 || buf[1] != SIGMA_FRAME_START_1) return false;
    if (static_cast<SigmaPacketType>(buf[2]) != expected_type)        return false;

    const uint16_t plen = static_cast<uint16_t>(buf[3]) |
                          (static_cast<uint16_t>(buf[4]) << 8);
    if (plen != expected_plen)                                        return false;

    const uint16_t expected_crc = sigma_crc16(buf + 2, 3 + plen);
    const uint16_t actual_crc   = static_cast<uint16_t>(buf[5 + plen]) |
                                   (static_cast<uint16_t>(buf[6 + plen]) << 8);
    if (actual_crc != expected_crc)                                   return false;
    if (buf[7 + plen] != SIGMA_FRAME_END_0)                          return false;
    if (buf[8 + plen] != SIGMA_FRAME_END_1)                          return false;

    memcpy(&payload_out, buf + 5, plen);
    return true;
}

// sigma_find_frame — scan a stream buffer for a valid start magic sequence.
// Returns the offset of the first SIGMA_FRAME_START_0 byte, or buf_len if
// not found.  Use when receiving over UART or TCP where framing can desync.
inline size_t sigma_find_frame(const uint8_t* buf, size_t buf_len)
{
    for (size_t i = 0; i + 1 < buf_len; ++i)
        if (buf[i] == SIGMA_FRAME_START_0 && buf[i + 1] == SIGMA_FRAME_START_1)
            return i;
    return buf_len;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Unit-conversion helpers   (physical value ↔ scaled wire integer)
//
// Usage:
//   SigmaLoRaPayload p;
//   p.lat_deg_e7  = SigmaConvert::lat_to_wire( gps.lat );
//   p.speed_cms   = SigmaConvert::speed_to_wire( vel_mag_ms );
//   for (int i = 0; i < 4; ++i) p.q[i] = SigmaConvert::quat_to_wire( q_float[i] );
//   // ... fill remaining fields ...
//   uint8_t frame[64];
//   size_t n = sigma_serialize(SigmaPacketType::LORA_TELEMETRY, p, frame, sizeof(frame));
// ═══════════════════════════════════════════════════════════════════════════════

namespace SigmaConvert {

// GPS lat/lon (degrees ↔ int32_t × 10^7)
inline int32_t lat_to_wire(double deg)    { return static_cast<int32_t>(deg * SIGMA_LATLON_SCALE); }
inline int32_t lon_to_wire(double deg)    { return static_cast<int32_t>(deg * SIGMA_LATLON_SCALE); }
inline double  lat_from_wire(int32_t v)   { return static_cast<double>(v)  / SIGMA_LATLON_SCALE; }
inline double  lon_from_wire(int32_t v)   { return static_cast<double>(v)  / SIGMA_LATLON_SCALE; }

// GPS altitude (metres ↔ centimetres)
inline int32_t gps_alt_to_wire(float m)   { return static_cast<int32_t>(m  * SIGMA_GPS_ALT_SCALE); }
inline float   gps_alt_from_wire(int32_t v){ return static_cast<float>(v)  / static_cast<float>(SIGMA_GPS_ALT_SCALE); }

// Baro altitude — MS5607 native unit is decimetres (ALTITUDE_SCALE = 10)
// Stored directly in alt_baro_dm; no additional scaling needed.
inline float   baro_dm_to_m(int32_t dm)  { return static_cast<float>(dm) * 0.1f; }
inline int32_t baro_m_to_dm(float m)     { return static_cast<int32_t>(m  * 10.0f); }

// Euler angles (degrees ↔ int16_t decidegrees)
inline int16_t angle_to_wire(float deg)   { return static_cast<int16_t>(deg * SIGMA_ANGLE_SCALE); }
inline float   angle_from_wire(int16_t v) { return static_cast<float>(v)  / SIGMA_ANGLE_SCALE; }

// Quaternion (float [-1,+1] ↔ int16_t Q1.15)
inline int16_t quat_to_wire(float q)      { return static_cast<int16_t>(q  * SIGMA_QUAT_SCALE); }
inline float   quat_from_wire(int16_t v)  { return static_cast<float>(v)  / SIGMA_QUAT_SCALE; }

// Scalar speed (m/s ↔ uint16_t cm/s) — used in SigmaLoRaPayload
inline uint16_t speed_to_wire(float ms)   { return static_cast<uint16_t>(ms  * SIGMA_SPEED_SCALE); }
inline float    speed_from_wire(uint16_t v){ return static_cast<float>(v)  / SIGMA_SPEED_SCALE; }

// Mach (Mach ↔ uint16_t × 1000) — used in WiFi / storage packets
inline uint16_t mach_to_wire(float mach)  { return static_cast<uint16_t>(mach * SIGMA_MACH_SCALE); }
inline float    mach_from_wire(uint16_t v){ return static_cast<float>(v)  / SIGMA_MACH_SCALE; }

// Acceleration (g ↔ int16_t milli-g)
inline int16_t accel_to_wire(float g)     { return static_cast<int16_t>(g  * SIGMA_ACCEL_SCALE); }
inline float   accel_from_wire(int16_t v) { return static_cast<float>(v)  / SIGMA_ACCEL_SCALE; }

// Angular rate (deg/s ↔ int16_t × 0.1 deg/s)
inline int16_t gyro_to_wire(float dps)    { return static_cast<int16_t>(dps * SIGMA_GYRO_SCALE); }
inline float   gyro_from_wire(int16_t v)  { return static_cast<float>(v)  / SIGMA_GYRO_SCALE; }

} // namespace SigmaConvert

// ═══════════════════════════════════════════════════════════════════════════════
// High-level builder structs
//
// Fill physical-unit fields, call .convert() to get the wire struct, then call
// .serialize() to produce a framed byte buffer.  The static from_wire() and
// deserialize() methods cover the receive path.
//
// Example (transmit):
//   SigmaLoRaData d;
//   d.boot_ms    = to_ms_since_boot(get_absolute_time());
//   d.lat        = gps.lat;
//   d.lon        = gps.lon;
//   d.alt_baro_m = SigmaConvert::baro_dm_to_m(baro_dm);
//   d.alt_gps_m  = gps.alt_m;
//   d.q[0] = qw; d.q[1] = qx; d.q[2] = qy; d.q[3] = qz;
//   d.speed_ms   = vel_mag;
//   d.state      = FlightState::POWERED_ASCENT;
//   d.satellites = gps.sats;
//   d.flags      = SIGMA_FLAG_GPS_VALID | SIGMA_FLAG_BARO_VALID;
//   uint8_t frame[64];
//   size_t n = d.serialize(frame, sizeof(frame));
//
// Example (receive):
//   SigmaLoRaData d;
//   if (SigmaLoRaData::deserialize(buf, len, d)) { /* use d */ }
// ═══════════════════════════════════════════════════════════════════════════════

// ── LoRa builder ─────────────────────────────────────────────────────────────

struct SigmaLoRaData {
    // Physical-unit fields — fill these directly
    uint32_t   boot_ms    = 0;
    double     lat        = 0.0;        ///< degrees
    double     lon        = 0.0;        ///< degrees
    float      alt_baro_m = 0.0f;       ///< metres (converted from MS5607 decimetres)
    float      alt_gps_m  = 0.0f;       ///< metres
    float      q[4]       = {1,0,0,0};  ///< quaternion [w,x,y,z], unit float
    float      speed_ms   = 0.0f;       ///< scalar speed, m/s
    FlightState state     = FlightState::GROUND_IDLE;
    uint8_t    satellites = 0;
    uint8_t    flags      = 0;          ///< SIGMA_FLAG_* bitmask

    /// Convert physical fields → wire struct (no framing).
    SigmaLoRaPayload convert() const {
        SigmaLoRaPayload p;
        p.boot_ms     = boot_ms;
        p.lat_deg_e7  = SigmaConvert::lat_to_wire(lat);
        p.lon_deg_e7  = SigmaConvert::lon_to_wire(lon);
        p.alt_baro_dm = SigmaConvert::baro_m_to_dm(alt_baro_m);
        p.alt_gps_cm  = SigmaConvert::gps_alt_to_wire(alt_gps_m);
        for (int i = 0; i < 4; ++i)
            p.q[i]    = SigmaConvert::quat_to_wire(q[i]);
        p.speed_cms   = SigmaConvert::speed_to_wire(speed_ms);
        p.state       = static_cast<uint8_t>(state);
        p.satellites  = satellites;
        p.flags       = flags;
        p._pad[0] = p._pad[1] = p._pad[2] = 0;
        return p;
    }

    /// Convert + serialize into a framed byte buffer.
    /// Returns frame length on success, 0 if buf is too small.
    size_t serialize(uint8_t* buf, size_t buf_len) const {
        SigmaLoRaPayload p = convert();
        return sigma_serialize(SigmaPacketType::LORA_TELEMETRY, p, buf, buf_len);
    }

    /// Populate from a wire struct (receive path, no framing).
    static SigmaLoRaData from_wire(const SigmaLoRaPayload& p) {
        SigmaLoRaData d;
        d.boot_ms    = p.boot_ms;
        d.lat        = SigmaConvert::lat_from_wire(p.lat_deg_e7);
        d.lon        = SigmaConvert::lon_from_wire(p.lon_deg_e7);
        d.alt_baro_m = SigmaConvert::baro_dm_to_m(p.alt_baro_dm);
        d.alt_gps_m  = SigmaConvert::gps_alt_from_wire(p.alt_gps_cm);
        for (int i = 0; i < 4; ++i)
            d.q[i]   = SigmaConvert::quat_from_wire(p.q[i]);
        d.speed_ms   = SigmaConvert::speed_from_wire(p.speed_cms);
        d.state      = static_cast<FlightState>(p.state);
        d.satellites = p.satellites;
        d.flags      = p.flags;
        return d;
    }

    /// Deserialize a framed byte buffer into a SigmaLoRaData.
    /// Returns true on success.
    static bool deserialize(const uint8_t* buf, size_t buf_len, SigmaLoRaData& out) {
        SigmaLoRaPayload p;
        if (!sigma_deserialize(buf, buf_len, SigmaPacketType::LORA_TELEMETRY, p))
            return false;
        out = from_wire(p);
        return true;
    }
};

// ── WiFi / full-rate builder ──────────────────────────────────────────────────
// SigmaWiFiPayload already uses float/double, so convert() is mostly a
// pass-through.  The builder exists for API symmetry and to apply the same
// serialize/deserialize pattern.

struct SigmaWiFiData {
    // Physical-unit fields (same units as SigmaWiFiPayload)
    double     lat          = 0.0;
    double     lon          = 0.0;
    uint64_t   utc_unix_ms  = 0;
    uint32_t   boot_ms      = 0;
    float      alt_gps_m    = 0.0f;
    float      alt_baro_m   = 0.0f;
    float      pressure_pa  = 0.0f;
    float      temp_c       = 0.0f;
    float      mach         = 0.0f;
    float      q[4]         = {1,0,0,0};
    float      rpy_deg[3]   = {0,0,0};
    float      accel_g[3]   = {0,0,0};
    float      gyro_dps[3]  = {0,0,0};
    float      vel_ned_ms[3]= {0,0,0};
    FlightState state       = FlightState::GROUND_IDLE;
    uint8_t    satellites   = 0;
    uint8_t    flags        = 0;

    SigmaWiFiPayload convert() const {
        SigmaWiFiPayload p;
        p.lat         = lat;
        p.lon         = lon;
        p.utc_unix_ms = utc_unix_ms;
        p.boot_ms     = boot_ms;
        p.alt_gps_m   = alt_gps_m;
        p.alt_baro_m  = alt_baro_m;
        p.pressure_pa = pressure_pa;
        p.temp_c      = temp_c;
        p.mach        = mach;
        for (int i = 0; i < 4; ++i) p.q[i]          = q[i];
        for (int i = 0; i < 3; ++i) p.rpy_deg[i]    = rpy_deg[i];
        for (int i = 0; i < 3; ++i) p.accel_g[i]    = accel_g[i];
        for (int i = 0; i < 3; ++i) p.gyro_dps[i]   = gyro_dps[i];
        for (int i = 0; i < 3; ++i) p.vel_ned_ms[i] = vel_ned_ms[i];
        p.state       = static_cast<uint8_t>(state);
        p.satellites  = satellites;
        p.flags       = flags;
        for (int i = 0; i < 5; ++i) p._pad[i] = 0;
        return p;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        SigmaWiFiPayload p = convert();
        return sigma_serialize(SigmaPacketType::WIFI_TELEMETRY, p, buf, buf_len);
    }

    static SigmaWiFiData from_wire(const SigmaWiFiPayload& p) {
        SigmaWiFiData d;
        d.lat         = p.lat;
        d.lon         = p.lon;
        d.utc_unix_ms = p.utc_unix_ms;
        d.boot_ms     = p.boot_ms;
        d.alt_gps_m   = p.alt_gps_m;
        d.alt_baro_m  = p.alt_baro_m;
        d.pressure_pa = p.pressure_pa;
        d.temp_c      = p.temp_c;
        d.mach        = p.mach;
        for (int i = 0; i < 4; ++i) d.q[i]          = p.q[i];
        for (int i = 0; i < 3; ++i) d.rpy_deg[i]    = p.rpy_deg[i];
        for (int i = 0; i < 3; ++i) d.accel_g[i]    = p.accel_g[i];
        for (int i = 0; i < 3; ++i) d.gyro_dps[i]   = p.gyro_dps[i];
        for (int i = 0; i < 3; ++i) d.vel_ned_ms[i] = p.vel_ned_ms[i];
        d.state       = static_cast<FlightState>(p.state);
        d.satellites  = p.satellites;
        d.flags       = p.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, SigmaWiFiData& out) {
        SigmaWiFiPayload p;
        if (!sigma_deserialize(buf, buf_len, SigmaPacketType::WIFI_TELEMETRY, p))
            return false;
        out = from_wire(p);
        return true;
    }
};

// ── IMU storage builder ───────────────────────────────────────────────────────

struct SigmaIMUData {
    // Physical-unit fields
    uint32_t   boot_us    = 0;
    float      q[4]       = {1,0,0,0};  ///< quaternion [w,x,y,z], unit float
    float      accel_g[3] = {0,0,0};    ///< body-frame acceleration, g
    float      gyro_dps[3]= {0,0,0};    ///< angular rate, °/s
    FlightState state     = FlightState::GROUND_IDLE;
    uint8_t    flags      = 0;

    SigmaStorageIMURecord convert() const {
        SigmaStorageIMURecord r;
        r.boot_us = boot_us;
        for (int i = 0; i < 4; ++i)
            r.q[i]         = SigmaConvert::quat_to_wire(q[i]);
        for (int i = 0; i < 3; ++i)
            r.accel_mg[i]  = SigmaConvert::accel_to_wire(accel_g[i]);
        for (int i = 0; i < 3; ++i)
            r.gyro_ddps[i] = SigmaConvert::gyro_to_wire(gyro_dps[i]);
        r.state       = static_cast<uint8_t>(state);
        r.flags       = flags;
        r._pad[0] = r._pad[1] = 0;
        return r;
    }

    size_t serialize(uint8_t* buf, size_t buf_len) const {
        SigmaStorageIMURecord r = convert();
        return sigma_serialize(SigmaPacketType::STORAGE_IMU, r, buf, buf_len);
    }

    static SigmaIMUData from_wire(const SigmaStorageIMURecord& r) {
        SigmaIMUData d;
        d.boot_us = r.boot_us;
        for (int i = 0; i < 4; ++i)
            d.q[i]        = SigmaConvert::quat_from_wire(r.q[i]);
        for (int i = 0; i < 3; ++i)
            d.accel_g[i]  = SigmaConvert::accel_from_wire(r.accel_mg[i]);
        for (int i = 0; i < 3; ++i)
            d.gyro_dps[i] = SigmaConvert::gyro_from_wire(r.gyro_ddps[i]);
        d.state = static_cast<FlightState>(r.state);
        d.flags = r.flags;
        return d;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, SigmaIMUData& out) {
        SigmaStorageIMURecord r;
        if (!sigma_deserialize(buf, buf_len, SigmaPacketType::STORAGE_IMU, r))
            return false;
        out = from_wire(r);
        return true;
    }
};
