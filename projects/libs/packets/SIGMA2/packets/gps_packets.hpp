#pragma once

#include <cstdint>

#include "../packets.hpp"

namespace SIGMA2 {
    namespace GENERAL_PACKETS {

        struct GPS_NAV_PVT {
            uint32_t iTOW;             // GPS Millisecond time of week [ms]
            uint16_t year;             // Year (UTC)
            uint8_t  month;            // Month, range 1..12 (UTC)
            uint8_t  day;              // Day of month, range 1..31 (UTC)
            uint8_t  hour;             // Hour of day, range 0..23 (UTC)
            uint8_t  min;              // Minute of hour, range 0..59 (UTC)
            uint8_t  sec;              // Seconds of minute, range 0..60 (UTC)

            uint8_t  valid;            // Validity flags
            uint32_t tAcc;             // time accuracy estimate [ns] (UTC)
            int32_t  nano;             // fraction of a second [ns], range -1e9 .. 1e9 (UTC)

            uint8_t  fixType;          // GNSS fix Type, range 0..5
            uint8_t  flags;            // Fix Status Flags
            uint8_t  flags2;           // Additional Flags
            uint8_t  numSV;            // Number of SVs used in Nav Solution
            int32_t  lon;              // Longitude [deg / 1e-7]
            int32_t  lat;              // Latitude [deg / 1e-7]
            int32_t  height;           // Height above Ellipsoid [mm]
            int32_t  hMSL;             // Height above mean sea level [mm]
            uint32_t hAcc;             // Horizontal Accuracy Estimate [mm]
            uint32_t vAcc;             // Vertical Accuracy Estimate [mm]

            int32_t velN;              // NED north velocity [mm/s]
            int32_t velE;              // NED east velocity [mm/s]
            int32_t velD;              // NED down velocity [mm/s]
            int32_t gSpeed;            // Ground Speed (2-D) [mm/s]
            int32_t heading;           // Heading of motion 2-D [deg / 1e-5]
            uint32_t sAcc;             // Speed Accuracy Estimate [mm/s]
            uint32_t headAcc;          // Heading Accuracy Estimate (both motion & vehicle) [deg / 1e-5]
        };
    }

    namespace TRANSMIT_PACKETS {

        // --------- GNSS Packets --------- //

        struct GPSNav {
            static constexpr PacketType TYPE = PacketType::GPS;

            uint32_t gps_tow_ms;     // UBX iTOW
            int32_t  lat_deg_e7;     // degrees * 1e7
            int32_t  lon_deg_e7;     // degrees * 1e7
            int32_t  alt_msl_cm;     // hMSL in cm
            int16_t  vel_n_cms;      // NED north cm/s
            int16_t  vel_e_cms;      // NED east cm/s
            int16_t  vel_d_cms;      // NED down cm/s
            uint16_t h_acc_cm;       // horizontal accuracy
            uint16_t v_acc_cm;       // vertical accuracy
            uint16_t s_acc_cms;      // speed accuracy
            uint8_t  fix_type;       // UBX fixType
            uint8_t  num_sv;
            uint8_t  flags;          // valid bits

            static constexpr size_t WIRE_SIZE =
                sizeof(uint32_t) + sizeof(int32_t) + sizeof(int32_t) + sizeof(int32_t) +
                sizeof(int16_t) + sizeof(int16_t) + sizeof(int16_t) +
                sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t) +
                sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);

            bool serialize(uint8_t* buf, size_t len) const {
                if (!buf || len < WIRE_SIZE) {
                    return false;
                }

                size_t i = 0;
                wire::write_u32_le(buf, i, gps_tow_ms);
                wire::write_u32_le(buf, i, static_cast<uint32_t>(lat_deg_e7));
                wire::write_u32_le(buf, i, static_cast<uint32_t>(lon_deg_e7));
                wire::write_u32_le(buf, i, static_cast<uint32_t>(alt_msl_cm));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(vel_n_cms));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(vel_e_cms));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(vel_d_cms));
                wire::write_u16_le(buf, i, h_acc_cm);
                wire::write_u16_le(buf, i, v_acc_cm);
                wire::write_u16_le(buf, i, s_acc_cms);
                buf[i++] = fix_type;
                buf[i++] = num_sv;
                buf[i++] = flags;
                return true;
            }

            static bool deserialize(const uint8_t* buf, size_t len, GPSNav& out) {
                if (!buf || len < WIRE_SIZE) {
                    return false;
                }

                size_t i = 0;
                out.gps_tow_ms = wire::read_u32_le(buf, i);
                out.lat_deg_e7 = static_cast<int32_t>(wire::read_u32_le(buf, i));
                out.lon_deg_e7 = static_cast<int32_t>(wire::read_u32_le(buf, i));
                out.alt_msl_cm = static_cast<int32_t>(wire::read_u32_le(buf, i));
                out.vel_n_cms = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.vel_e_cms = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.vel_d_cms = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.h_acc_cm = wire::read_u16_le(buf, i);
                out.v_acc_cm = wire::read_u16_le(buf, i);
                out.s_acc_cms = wire::read_u16_le(buf, i);
                out.fix_type = buf[i++];
                out.num_sv = buf[i++];
                out.flags = buf[i++];
                return true;
            }
        };

        struct NavSource {
            static constexpr uint8_t GPS_POS      = 1u << 0;
            static constexpr uint8_t GPS_VEL      = 1u << 1;
            static constexpr uint8_t BARO_ALT     = 1u << 2;
            static constexpr uint8_t BARO_VEL     = 1u << 3;
            static constexpr uint8_t IMU_ACCEL    = 1u << 4;
            static constexpr uint8_t AHRS_ATT     = 1u << 5;
            static constexpr uint8_t KALMAN_STATE = 1u << 6;
            static constexpr uint8_t PREDICTED    = 1u << 7;
        };

        struct NavState {
            static constexpr PacketType TYPE = PacketType::NAV_STATE;

            uint8_t nav_source;      // bitmask of NavSource
            SIGMA2::CoordinateFrame frame = SIGMA2::CoordinateFrame::NED;

            int32_t  lat_deg_e7;     // degrees * 1e7
            int32_t  lon_deg_e7;     // degrees * 1e7

            int32_t alt_fused_cm;    // Fused altitude in cm

            int16_t  vel_n_cms;      // NED north cm/s
            int16_t  vel_e_cms;      // NED east cm/s
            int16_t  vel_d_cms;      // NED down cm/s

            int16_t  acc_n_mg;
            int16_t  acc_e_mg;
            int16_t  acc_d_mg;

            int16_t  q[4];           // Q1.15 quaternion [w,x,y,z]
            SIGMA2::FLIGHT_STATE  state;          // flight state
            uint8_t  flags;

            static constexpr size_t WIRE_SIZE =
                sizeof(uint8_t) + sizeof(uint8_t) +
                sizeof(int32_t) + sizeof(int32_t) + sizeof(int32_t) +
                (3u * sizeof(int16_t)) +
                (3u * sizeof(int16_t)) +
                (4u * sizeof(int16_t)) +
                sizeof(uint8_t) + sizeof(uint8_t);

            bool serialize(uint8_t* buf, size_t len) const {
                if (!buf || len < WIRE_SIZE) {
                    return false;
                }

                size_t i = 0;
                buf[i++] = nav_source;
                buf[i++] = static_cast<uint8_t>(frame);
                wire::write_u32_le(buf, i, static_cast<uint32_t>(lat_deg_e7));
                wire::write_u32_le(buf, i, static_cast<uint32_t>(lon_deg_e7));
                wire::write_u32_le(buf, i, static_cast<uint32_t>(alt_fused_cm));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(vel_n_cms));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(vel_e_cms));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(vel_d_cms));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(acc_n_mg));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(acc_e_mg));
                wire::write_u16_le(buf, i, static_cast<uint16_t>(acc_d_mg));
                for (int k = 0; k < 4; ++k) {
                    wire::write_u16_le(buf, i, static_cast<uint16_t>(q[k]));
                }
                buf[i++] = static_cast<uint8_t>(state);
                buf[i++] = flags;
                return true;
            }

            static bool deserialize(const uint8_t* buf, size_t len, NavState& out) {
                if (!buf || len < WIRE_SIZE) {
                    return false;
                }

                size_t i = 0;
                out.nav_source = buf[i++];
                out.frame = static_cast<SIGMA2::CoordinateFrame>(buf[i++]);
                out.lat_deg_e7 = static_cast<int32_t>(wire::read_u32_le(buf, i));
                out.lon_deg_e7 = static_cast<int32_t>(wire::read_u32_le(buf, i));
                out.alt_fused_cm = static_cast<int32_t>(wire::read_u32_le(buf, i));
                out.vel_n_cms = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.vel_e_cms = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.vel_d_cms = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.acc_n_mg = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.acc_e_mg = static_cast<int16_t>(wire::read_u16_le(buf, i));
                out.acc_d_mg = static_cast<int16_t>(wire::read_u16_le(buf, i));
                for (int k = 0; k < 4; ++k) {
                    out.q[k] = static_cast<int16_t>(wire::read_u16_le(buf, i));
                }
                out.state = static_cast<SIGMA2::FLIGHT_STATE>(buf[i++]);
                out.flags = buf[i++];
                return true;
            }
        };

    };
};
