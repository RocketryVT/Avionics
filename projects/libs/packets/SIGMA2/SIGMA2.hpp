#pragma once

// This is SIGMA2 which is a simplified version of SIGMA. It is designed to accomdate all forms of packets.
// Wheter its over WIFI with MQTT or over LoRa, Bluetooth, flash storage, or other radio modulation types.
// We support a single unified packet format with a common header and CRC.

// We don't use packed structs but instead serialze/deserialize to/from byte buffers.
// This ensures we don't have to worry about compiler-inserted padding bytes or field reordering.
// All fields will use little-endian byte order for consistency across platforms.
// WiFi/Ethernet handles the big-endian conversion for us

#include <stddef.h>
#include <stdint.h>
#include <string.h>   // memcpy

#include "crc.hpp"
#include "packets.hpp"


static constexpr uint8_t PROTOCOL_VERSION = 2;

// ===============================================================================
// Frame constants
// ===============================================================================

static constexpr uint8_t FRAME_START_0  = 0xAA;
static constexpr uint8_t FRAME_START_1  = 0x55;
static constexpr uint8_t FRAME_END_0    = 0xBB;
static constexpr uint8_t FRAME_END_1    = 0x66;

static constexpr size_t  FRAME_OVERHEAD = 19;
static constexpr size_t  MAX_FRAME      = 255;
static constexpr size_t  MAX_PAYLOAD    = MAX_FRAME - FRAME_OVERHEAD;

namespace SIGMA2 {

inline constexpr uint8_t PROTOCOL_VERSION = ::PROTOCOL_VERSION;
inline constexpr uint8_t FRAME_START_0    = ::FRAME_START_0;
inline constexpr uint8_t FRAME_START_1    = ::FRAME_START_1;
inline constexpr uint8_t FRAME_END_0      = ::FRAME_END_0;
inline constexpr uint8_t FRAME_END_1      = ::FRAME_END_1;
inline constexpr size_t  FRAME_OVERHEAD   = ::FRAME_OVERHEAD;
inline constexpr size_t  MAX_PAYLOAD      = ::MAX_PAYLOAD;
inline constexpr size_t  MAX_FRAME        = ::MAX_FRAME;
inline constexpr uint8_t DEFAULT_TTL      = 4;

// Forward declarations for utils functions
namespace utils {
    uint16_t read_u16_le(const uint8_t* buf, size_t& i);
    void     write_u16_le(uint8_t* buf, size_t& i, uint16_t value);
    uint32_t read_u32_le(const uint8_t* buf, size_t& i);
    void     write_u32_le(uint8_t* buf, size_t& i, uint32_t value);
    uint16_t crc16(const uint8_t* data, size_t len);
}
using namespace utils;

/**
 * @brief PacketType is a 255 value enum that 
 * 
 */
enum class PacketType : uint8_t {
    UNKNOWN         = 0,       // Unknown packet type
    // TELEMETRY       = 1,    // Telemetry data from trackers (e.g. position, velocity, attitude)
    // COMMAND         = 2,    // Commands to trackers (e.g. change mode, set target)
    // ACK             = 3,    // Acknowledgment packets for reliable delivery
    // NACK            = 4,    // Negative acknowledgment for failed packets
    // RADIO_CONFIG    = 5,    // Radio configuration packets (e.g. set frequency, power, channel)

    // --------- Telemetry Types --------- //
    GPS             = 10,   // GPS data (lat, lon, alt, speed, heading)
    IMU             = 11,   // IMU data (accel, gyro, orientation)
    BARO            = 12,   // Barometer data (pressure, temperature)
    FLIGHT_STATE    = 13,   // Flight state data (e.g. IDLE, ARMED, BOOST, COAST, APOGEE, DESCENT, LANDED)
    HEALTH          = 14,   // Health and status data (e.g. battery voltage, temperature, errors)
    NAV_STATE       = 15,   // Fused navigation state used for antenna tracking and flight telemetry

    // ---------  Control Types --------- //
    PING            = 20,   // Ping packet for latency measurement and link testing

    // --------- Mesh Network Types --------- //
    ROUTING          = 30,   // Routing information for mesh network (e.g. neighbor discovery, link quality)
    TIMESYNC         = 31,   // Time synchronization packets for mesh network (e.g. timestamp exchange)
    ROUTE_UPDATE     = 32,   // Route update packets for mesh network (e.g. new route advertisement, route error)
    NODE_ANNOUNCE    = 33,   // Node announcement packets for mesh network (e.g. new node joined)

    // --------- Ground Station Types --------- //
    TRACKER_STATUS    = 40,   // Status of trackers at ground station (e.g. last seen, signal strength)
    LINK_STATUS       = 41,   // Status of links at ground station (e.g. connected nodes, link quality, RSSI/SNR per Radio path)


    // --------- Mobile Tracker Types --------- //
    // These are the Radio packets, paired devices with internet can use the MQTT Backbone using the protobuf packets
    MOBILE_TELEMETRY     = 50,   // Telemetry data from mobile nodes (e.g. GPS, IMU, battery)
    MOBILE_HEALTH        = 51,   // Health and status data from mobile nodes (e.g. battery voltage, temperature, errors)
    MOBILE_LINK_STATUS   = 51,   // Link status of mobile nodes (e.g. connected nodes, link quality, RSSI/SNR)
    MOBILE_GPS           = 51,   // GPS data from mobile nodes (lat, lon, alt, speed, heading)
    MOBILE_BARO          = 52,   // Barometer data from mobile nodes (pressure, temperature)
    MOBILE_COMPANION_GPS = 52,   // GPS data from companion devices paired with mobile nodes (e.g. phone GPS)

};

enum class NodeID : uint8_t {
    UNDEFINED       = 0,    // 0 is unknown for source but is broadcast for destination
    NOSE_CONE       = 1,    // Bareman Tracker
    PAYLOAD         = 2,    // Payload Tracker
    EBAY            = 3,    // Unused for now
    ADS             = 4,    // Active Drag System in Thrust Structure
    ANTENNA_TRACKER = 5,    // Antenna Tracker Ground Station
    STATIONARY_GS   = 6,    // Stationary Ground Station (Low Gain + GPS)
    MOBILE_NODE1    = 7,    // Mobile Node 1 (e.g. Phone Paird with Mobile Tracker)
    MOBILE_NODE2    = 8,    // Mobile Node 2 (e.g. Laptop Paird with Mobile Tracker)
    MOBILE_NODE3    = 9,    // Mobile Node 3 (e.g. Tablet Paird with Mobile Tracker)
    MOBILE_NODE4    = 10,   // Mobile Node 4 (e.g. Something Paird with Mobile Tracker)
};

inline bool is_broadcast(NodeID node)
{
    return node == NodeID::UNDEFINED;
}

enum class DeviceType : uint8_t {
    Unknown = 0,
    Ads,
    Bareman,
    AntennaTracker,
    LowGainGroundStation,
    MobileTracker,
};

struct RadioConfig {
    float freq_mhz = 904.5f;
    float bandwidth_khz = 125.0f;
    uint8_t spreading_factor = 7;
    uint8_t coding_rate = 7;
    uint8_t sync_word = 0x12;
    int8_t tx_dbm = 20;
    uint16_t preamble_len = 8;
};

struct RadioRx {
    uint8_t data[MAX_FRAME] = {};
    size_t len = 0;
    uint8_t radio_id = 0;
    int16_t rssi_dbm = 0;
    int8_t snr_q4 = 0;
    uint32_t received_ms = 0;
};

class Radio {
public:
    virtual ~Radio() = default;
    virtual const char* name() const = 0;
    virtual int begin(const RadioConfig& config) = 0;
    virtual int transmit(const uint8_t* data, size_t len) = 0;
    virtual bool receive(RadioRx&) { return false; }
};

struct GpsFix {
    double lat = 0.0;
    double lon = 0.0;
    double alt_m = 0.0;
    uint8_t satellites = 0;
    float vel_ned_ms[3] = {0.0f, 0.0f, 0.0f};
    uint32_t utc_ms = 0;
    uint16_t utc_year = 0;
    uint8_t utc_month = 0;
    uint8_t utc_day = 0;
    uint8_t fix_type = 0;
    uint16_t h_acc_cm = 0;
    uint16_t v_acc_cm = 0;
    uint16_t s_acc_cms = 0;
    uint8_t flags = 0;
};

struct NavSnapshot {
    double lat = 0.0;
    double lon = 0.0;
    float alt_baro_m = 0.0f;
    float alt_fused_m = 0.0f;
    float vel_ned_ms[3] = {0.0f, 0.0f, 0.0f};
    float acc_ned_mss[3] = {0.0f, 0.0f, 0.0f};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    FLIGHT_STATE state = FLIGHT_STATE::UNKOWN;
    CoordinateFrame frame = CoordinateFrame::NED;
    uint8_t nav_source = 0;
    uint8_t flags = 0;
};

struct BaroSample {
    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;
    float altitude_m = 0.0f;
    uint8_t flags = 0;
};

struct MeshSnapshot {
    bool have_gps = false;
    GpsFix gps = {};
    bool have_nav = false;
    NavSnapshot nav = {};
    bool have_baro = false;
    BaroSample baro = {};
    uint32_t boot_ms = 0;
};

struct MeshConfig {
    DeviceType device_type = DeviceType::Unknown;
    NodeID node_id = NodeID::UNDEFINED;
    NodeID default_destination = NodeID::ANTENNA_TRACKER;
    RadioConfig radio = {};
    uint8_t time_sync_burst_count = 5;
    uint8_t time_sync_period_ticks = 5;
    bool controlled_flooding_enabled = true;
};

struct MeshTxStats {
    uint32_t ok = 0;
    uint32_t err = 0;
    uint32_t dropped = 0;
    uint32_t rx_ok = 0;
    uint32_t rx_err = 0;
    uint32_t rx_duplicate = 0;
    uint32_t rx_self = 0;
    uint32_t rx_ttl_expired = 0;
    uint32_t delivered = 0;
    uint32_t delivery_dropped = 0;
    uint32_t forwarded = 0;
    uint32_t forward_dropped = 0;
};

struct COMMAND_FLAG {
    uint8_t value;

    static constexpr uint8_t ACK_REQUESTED = 0x01; // Bit 0: If set, the sender requests an acknowledgment from the receiver
    static constexpr uint8_t ACK_RESPONSE  = 0x02; // Bit 1: If set, the packet is an acknowledgment response to a previous packet (only valid if ACK_REQUESTED was set in the original packet)
    static constexpr uint8_t RETRANSMIT    = 0x04; // Bit 2: If set, the packet is a retransmission of a previously sent packet (only valid if ACK_REQUESTED was set in the original packet)
    static constexpr uint8_t RETAINED      = 0x08; // Bit 3: If set, the packet should be retained by intermediate nodes for later delivery (e.g. when destination is offline)
    static constexpr uint8_t PRIORITY      = 0x10; // Bit 4: If set, the packet is high priority and should be transmitted before non-priority packets (e.g. for real-time telemetry)

    bool is_ack_requested() const { return value & ACK_REQUESTED; }
    bool is_ack_response() const { return value & ACK_RESPONSE; }
    bool is_retransmit() const { return value & RETRANSMIT; }
    bool is_retained() const { return value & RETAINED; }
    bool is_priority() const { return value & PRIORITY; }
};

/**
 * @brief The Header consists of a Start Frame (2 bytes), Packet Type (1 byte), Node ID (1 byte), Per Type Sequence Number (1 byte), and Payload Length (2 bytes).
 * 
 */
struct HEADER {
    uint8_t         start[2]     = {FRAME_START_0, FRAME_START_1};
    PacketType      type         = {};
    NodeID          node_src     = {};   // 0 for unknown
    NodeID          node_dst     = {};   // 0 for broadcast
    COMMAND_FLAG    cmd_flags    = {0};  // Command and control flags for reliability, priority, and routing
    uint8_t         ttl          = 0;    // Time to Live for routing, decremented by each hop, 0 means drop packet
    uint32_t        timestamp_ms = 0;    // time since boot in milliseconds
    uint16_t        seq          = 0;    // uint16_t sequence number per packet type, wraps around at 65535, used for duplicate detection and ordering
    uint16_t        len          = 0;    // payload length in bytes, max MAX_PAYLOAD

    static constexpr size_t WIRE_SIZE = sizeof(start) + sizeof(type) + sizeof(node_src) + sizeof(node_dst) + sizeof(cmd_flags) + sizeof(ttl) + sizeof(timestamp_ms) + sizeof(seq) + sizeof(len);

    size_t serialize(uint8_t* buf) const {
        size_t i = 0;
        buf[i++] = start[0];
        buf[i++] = start[1];
        buf[i++] = static_cast<uint8_t>(type);
        buf[i++] = static_cast<uint8_t>(node_src);
        buf[i++] = static_cast<uint8_t>(node_dst);
        buf[i++] = cmd_flags.value;
        buf[i++] = ttl;
        write_u32_le(buf, i, timestamp_ms);
        write_u16_le(buf, i, seq);
        write_u16_le(buf, i, len);
        return i; // always == WIRE_SIZE
    }

    bool valid_start() const {
        return start[0] == FRAME_START_0 && start[1] == FRAME_START_1;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, HEADER& header) {
        if (!buf || buf_len < WIRE_SIZE) {
            return false;
        }

        size_t i = 0;
        header.start[0] = buf[i++];
        header.start[1] = buf[i++];
        header.type     = static_cast<PacketType>(buf[i++]);
        header.node_src  = static_cast<NodeID>(buf[i++]);
        header.node_dst  = static_cast<NodeID>(buf[i++]);
        header.cmd_flags.value = buf[i++];
        header.ttl = buf[i++];
        header.timestamp_ms = read_u32_le(buf, i);
        header.seq = read_u16_le(buf, i);
        header.len = read_u16_le(buf, i);
        return true;
    }
};
static_assert(HEADER::WIRE_SIZE == 15);

struct FOOTER {
    uint16_t crc;           // 2 bytes
    uint8_t  end[2] = {FRAME_END_0, FRAME_END_1};  // 2 bytes

    static constexpr size_t WIRE_SIZE = sizeof(crc) + sizeof(end);

    size_t serialize(uint8_t* buf) const {
        size_t i = 0;
        buf[i++] = static_cast<uint8_t>(crc & 0xFF);       // Little-endian
        buf[i++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
        buf[i++] = end[0];
        buf[i++] = end[1];
        return i; // always == WIRE_SIZE
    }

    bool valid_end() const {
        return end[0] == FRAME_END_0 && end[1] == FRAME_END_1;
    }

    static bool deserialize(const uint8_t* buf, size_t buf_len, FOOTER& footer) {
        if (!buf || buf_len < WIRE_SIZE) {
            return false;
        }

        size_t i = 0;
        footer.crc = read_u16_le(buf, i);
        footer.end[0] = buf[i++];
        footer.end[1] = buf[i++];
        return true;
    }
};
static_assert(FOOTER::WIRE_SIZE == 4);

/**
 * @brief The Overall packet consists of a Header: (Start Frame, Node ID, Packet Type, Per Type Seqence Number, Payload Length), Payload, and a Footer (CRC, End Frame).
 * 
 */
struct PACKET {
    HEADER header = {};               // 6 bytes
    uint8_t payload[MAX_PAYLOAD] = {0};
    FOOTER footer = {};               // 4 bytes

    static constexpr size_t WIRE_SIZE = HEADER::WIRE_SIZE + MAX_PAYLOAD + FOOTER::WIRE_SIZE;
};
static_assert(PACKET::WIRE_SIZE == MAX_FRAME);

enum class DecodeStatus : uint8_t {
    Ok = 0,
    NullBuffer,
    TooShort,
    BadStart,
    PayloadTooLarge,
    LengthMismatch,
    BadEnd,
    BadCrc,
};

struct DecodedFrame {
    HEADER header = {};
    FOOTER footer = {};
    const uint8_t* payload = nullptr;
    size_t payload_len = 0;
    size_t frame_len = 0;
};

struct MeshReceivedFrame {
    RadioRx rx = {};
    HEADER header = {};

    const uint8_t* payload() const {
        return rx.data + HEADER::WIRE_SIZE;
    }

    size_t payload_len() const {
        return header.len;
    }
};

class PacketCounters {
public:
    uint16_t next(PacketType type) {
        const uint8_t idx = static_cast<uint8_t>(type);
        const uint16_t seq = counters_[idx];
        counters_[idx] = static_cast<uint16_t>(seq + 1u);
        return seq;
    }

    uint16_t current(PacketType type) const {
        return counters_[static_cast<uint8_t>(type)];
    }

    void reset(PacketType type) {
        counters_[static_cast<uint8_t>(type)] = 0;
    }

    void reset_all() {
        memset(counters_, 0, sizeof(counters_));
    }

private:
    uint16_t counters_[256] = {};
};

inline uint16_t frame_crc(const uint8_t* frame, size_t payload_len) {
    return crc16(frame + sizeof(uint8_t) + sizeof(uint8_t),
                 (HEADER::WIRE_SIZE - 2u) + payload_len);
}

inline bool serialize_frame(const HEADER& header,
                            const uint8_t* payload,
                            size_t payload_len,
                            uint8_t* out,
                            size_t out_len,
                            size_t& written)
{
    written = 0;
    if (!out || payload_len > MAX_PAYLOAD) {
        return false;
    }
    if (payload_len != 0u && !payload) {
        return false;
    }

    const size_t total = HEADER::WIRE_SIZE + payload_len + FOOTER::WIRE_SIZE;
    if (out_len < total || total > MAX_FRAME) {
        return false;
    }

    HEADER h = header;
    h.start[0] = FRAME_START_0;
    h.start[1] = FRAME_START_1;
    h.len = static_cast<uint16_t>(payload_len);

    size_t i = h.serialize(out);
    if (payload_len != 0u) {
        memcpy(out + i, payload, payload_len);
        i += payload_len;
    }

    FOOTER footer = {};
    footer.crc = frame_crc(out, payload_len);
    i += footer.serialize(out + i);
    written = i;
    return true;
}

inline DecodeStatus deserialize_frame(const uint8_t* frame,
                                      size_t len,
                                      DecodedFrame& out)
{
    out = {};
    if (!frame) {
        return DecodeStatus::NullBuffer;
    }
    if (len < HEADER::WIRE_SIZE + FOOTER::WIRE_SIZE) {
        return DecodeStatus::TooShort;
    }

    if (!HEADER::deserialize(frame, len, out.header)) {
        return DecodeStatus::TooShort;
    }
    if (!out.header.valid_start()) {
        return DecodeStatus::BadStart;
    }
    if (out.header.len > MAX_PAYLOAD) {
        return DecodeStatus::PayloadTooLarge;
    }

    const size_t total = HEADER::WIRE_SIZE + out.header.len + FOOTER::WIRE_SIZE;
    if (len != total) {
        return DecodeStatus::LengthMismatch;
    }

    const uint8_t* footer_ptr = frame + HEADER::WIRE_SIZE + out.header.len;
    if (!FOOTER::deserialize(footer_ptr, FOOTER::WIRE_SIZE, out.footer)) {
        return DecodeStatus::TooShort;
    }
    if (!out.footer.valid_end()) {
        return DecodeStatus::BadEnd;
    }

    const uint16_t expected_crc = frame_crc(frame, out.header.len);
    if (out.footer.crc != expected_crc) {
        return DecodeStatus::BadCrc;
    }

    out.payload = frame + HEADER::WIRE_SIZE;
    out.payload_len = out.header.len;
    out.frame_len = total;
    return DecodeStatus::Ok;
}

class Serializer {
public:
    explicit Serializer(NodeID node_src = NodeID::UNDEFINED)
        : node_src_(node_src)
    {}

    void set_node_src(NodeID node_src) { node_src_ = node_src; }
    NodeID node_src() const { return node_src_; }

    size_t serialize(PacketType type,
                     NodeID node_dst,
                     const uint8_t* payload,
                     size_t payload_len,
                     uint8_t* out,
                     size_t out_len,
                     uint32_t timestamp_ms,
                     COMMAND_FLAG cmd_flags = {0},
                     uint8_t ttl = DEFAULT_TTL)
    {
        if (!out || payload_len > MAX_PAYLOAD ||
            (payload_len != 0u && !payload)) {
            return 0;
        }
        const size_t total = HEADER::WIRE_SIZE + payload_len + FOOTER::WIRE_SIZE;
        if (out_len < total || total > MAX_FRAME) {
            return 0;
        }

        HEADER header = {};
        header.type = type;
        header.node_src = node_src_;
        header.node_dst = node_dst;
        header.cmd_flags = cmd_flags;
        header.ttl = ttl;
        header.timestamp_ms = timestamp_ms;
        header.seq = counters_.next(type);
        header.len = static_cast<uint16_t>(payload_len);

        size_t written = 0;
        if (!serialize_frame(header, payload, payload_len, out, out_len, written)) {
            return 0;
        }
        return written;
    }

    template <typename PacketT>
    size_t serialize_packet(const PacketT& packet,
                            NodeID node_dst,
                            uint8_t* out,
                            size_t out_len,
                            uint32_t timestamp_ms,
                            COMMAND_FLAG cmd_flags = {0},
                            uint8_t ttl = DEFAULT_TTL)
    {
        uint8_t payload[PacketT::WIRE_SIZE] = {};
        if (!packet.serialize(payload, sizeof(payload))) {
            return 0;
        }

        return serialize(PacketT::TYPE,
                         node_dst,
                         payload,
                         sizeof(payload),
                         out,
                         out_len,
                         timestamp_ms,
                         cmd_flags,
                         ttl);
    }

    const PacketCounters& counters() const { return counters_; }

private:
    NodeID node_src_;
    PacketCounters counters_ = {};
};

template <typename PacketT>
inline bool deserialize_packet_payload(const DecodedFrame& frame, PacketT& out)
{
    if (frame.header.type != PacketT::TYPE ||
        frame.payload_len != PacketT::WIRE_SIZE) {
        return false;
    }
    return PacketT::deserialize(frame.payload, frame.payload_len, out);
}



namespace utils {

    inline uint16_t read_u16_le(const uint8_t* buf, size_t& i) {
        uint8_t lo = buf[i++];
        uint8_t hi = buf[i++];
        return static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    }

    inline void write_u16_le(uint8_t* buf, size_t& i, uint16_t value) {
        buf[i++] = static_cast<uint8_t>(value & 0xFF);
        buf[i++] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }

    inline uint32_t read_u32_le(const uint8_t* buf, size_t& i) {
        uint32_t b0 = buf[i++];
        uint32_t b1 = buf[i++];
        uint32_t b2 = buf[i++];
        uint32_t b3 = buf[i++];
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
    }

    inline void write_u32_le(uint8_t* buf, size_t& i, uint32_t value) {
        buf[i++] = static_cast<uint8_t>(value & 0xFF);
        buf[i++] = static_cast<uint8_t>((value >> 8) & 0xFF);
        buf[i++] = static_cast<uint8_t>((value >> 16) & 0xFF);
        buf[i++] = static_cast<uint8_t>((value >> 24) & 0xFF);
    }

    /**
     * @brief CRC16 - CCITT - 0x1021
     * 
     * @param data 
     * @param len 
     * @return uint16_t 
     */
    inline uint16_t crc16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF; // Initial Value
        for (size_t i = 0; i < len; i++) {
            crc = (crc << 8) ^ CRC16_0x1021_CCIT_TABLE[((crc >> 8) ^ data[i]) & 0xFF];
        }
        return crc;
    }

};

};
