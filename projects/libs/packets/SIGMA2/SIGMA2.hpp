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
static constexpr size_t  MAX_PAYLOAD    = 256 - FRAME_OVERHEAD; // 244 Bytes
static constexpr size_t  MAX_FRAME      = MAX_PAYLOAD + FRAME_OVERHEAD; // 256 Bytes

namespace SIGMA2 {

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
    uint16_t        len          = 0;    // payload length in bytes, max 244

    static constexpr size_t WIRE_SIZE = sizeof(start) + sizeof(type) + sizeof(node_src) + sizeof(node_dst) + sizeof(cmd_flags) + sizeof(ttl) + sizeof(timestamp_ms) + sizeof(seq) + sizeof(len);

    size_t serialize(uint8_t* buf) const {
        size_t i = 0;
        buf[i++] = start[0];
        buf[i++] = start[1];
        buf[i++] = static_cast<uint8_t>(type);
        buf[i++] = static_cast<uint8_t>(node_src);
        buf[i++] = static_cast<uint8_t>(node_dst);
        write_u32_le(buf, i, timestamp_ms);
        write_u16_le(buf, i, seq);
        write_u16_le(buf, i, len);
        return i; // always == WIRE_SIZE
    }

    HEADER deserialize(const uint8_t* buf) {
        HEADER header;
        size_t i = 0;
        header.start[0] = buf[i++];
        header.start[1] = buf[i++];
        header.type     = static_cast<PacketType>(buf[i++]);
        header.node_src  = static_cast<NodeID>(buf[i++]);
        header.node_dst  = static_cast<NodeID>(buf[i++]);
        header.seq      = static_cast<uint16_t>(buf[i++]);
        header.len      = read_u16_le(buf, i);
        return header;
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
};
static_assert(FOOTER::WIRE_SIZE == 4);

/**
 * @brief The Overall packet consists of a Header: (Start Frame, Node ID, Packet Type, Per Type Seqence Number, Payload Length), Payload, and a Footer (CRC, End Frame).
 * 
 */
struct PACKET {
    HEADER header = {};               // 6 bytes
    uint8_t payload[MAX_PAYLOAD] = {0}; // 244 bytes
    FOOTER footer = {};               // 4 bytes

    static constexpr size_t WIRE_SIZE = HEADER::WIRE_SIZE + MAX_PAYLOAD + FOOTER::WIRE_SIZE;
};
static_assert(PACKET::WIRE_SIZE == 256);



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