#pragma once

#include <cstddef>
#include <cstdint>

#include "../packets.hpp"

namespace SIGMA2 {
namespace TRANSMIT_PACKETS {

struct TimeSync {
    static constexpr PacketType TYPE = PacketType::TIMESYNC;

    struct TimePoint {
        uint32_t timestamp_ms = 0; // time since boot in milliseconds
        uint32_t utc_ms = 0;       // UTC ms in the sender's chosen epoch/window
        uint16_t accuracy_ms = 0;  // estimated timestamp accuracy
        uint8_t flags = 0;         // DATA_VALID_FLAG bitmask
    };

    TimePoint t1 = {}; // sender timestamp at transmit
    TimePoint t2 = {}; // receiver timestamp when replying

    static constexpr size_t TIME_POINT_WIRE_SIZE =
        sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint8_t);
    static constexpr size_t WIRE_SIZE = 2u * TIME_POINT_WIRE_SIZE;

    bool serialize(uint8_t* buf, size_t len) const {
        if (!buf || len < WIRE_SIZE) {
            return false;
        }

        size_t i = 0;
        write_one(buf, i, t1);
        write_one(buf, i, t2);
        return true;
    }

    static bool deserialize(const uint8_t* buf, size_t len, TimeSync& out) {
        if (!buf || len < WIRE_SIZE) {
            return false;
        }

        size_t i = 0;
        out.t1 = read_one(buf, i);
        out.t2 = read_one(buf, i);
        return true;
    }

private:
    static void write_one(uint8_t* buf, size_t& i, const TimePoint& t) {
        wire::write_u32_le(buf, i, t.timestamp_ms);
        wire::write_u32_le(buf, i, t.utc_ms);
        wire::write_u16_le(buf, i, t.accuracy_ms);
        buf[i++] = t.flags;
    }

    static TimePoint read_one(const uint8_t* buf, size_t& i) {
        TimePoint t;
        t.timestamp_ms = wire::read_u32_le(buf, i);
        t.utc_ms = wire::read_u32_le(buf, i);
        t.accuracy_ms = wire::read_u16_le(buf, i);
        t.flags = buf[i++];
        return t;
    }
};

} // namespace TRANSMIT_PACKETS
} // namespace SIGMA2
