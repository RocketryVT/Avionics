#pragma once

#include <cstddef>
#include <cstdint>

// This file implments all the actual packet structs that are used in SIGMA. 
// Every PacketType has a corresponding struct that defines the payload format.

/*

We have a couple different types of packets:
- Generic Sensor Packets, using these you must conform to the specificed unit types, 
yes some math or bit shifts are required but it is needed to ensure other parts of the library or system can use these interchangably.
- Log/Storage Specific Packets, these are more flexible and can be used for logging or storage of arbitrary data. They have a more flexible payload format but are not intended for real-time communication.
- Command Packets, these are used for sending commands to the antenna tracker. They have a specific format that includes a command ID and parameters.
- Telemetry Packets, these are used for sending telemetry data to/from the tracker. They have a specific format that includes the telemetry type and data.
    - Telemetry Packets are further divided into subtypes based on usecase:
        - NavState: Fused navigation state including position, velocity, attitude, and flight state, used for Antenna Tracking
        - Time Sync: Used for synchronizing time between nodes, contains timestamp and accuracy information
        - GPS: Used for sending raw GPS data from the tracker to the ground station, contains GPS fix information, position, velocity, and accuracy
        - IMU: Used for sending raw IMU data from the tracker to the ground station, contains accelerometer, gyroscope, and magnetometer data
        - Barometer: Used for sending raw barometer data from the tracker to the ground station, contains pressure and temperature data
        - Link Status: Used for sending link status information from the tracker to the ground station, contains information about connected nodes, link quality, RSSI/SNR, etc.
        - Tracker Status: Used for sending status information about the tracker itself to the ground station, contains information about battery voltage, temperature, errors, etc.
        - Mobile Telemetry: Used for sending telemetry data from mobile nodes (e.g. phones) to the ground station, contains GPS, IMU, battery, and link status information
        - System Status: Used for sending overall system status information from the trackers to the ground station, contains information about overall health, errors, flight state, etc.

*/

/*

2026 Theory of Operation:
1. Ground Station is Turned on and waiting
    - WiFi AP is active and waiting for connections from the Antenna Tracker, Low Gain GS, and Laptop, and any Mobile Nodes (e.g. phones)
    - Antenna Tracker boots up and we calibrate it on the ground
        - Antenna Tracker hopefully has a GPS lock and can send position information to the Ground Station
    - Low Gain GS is setup and connected over USB to the Ground Station Laptop
    - Laptop is running the Ground Station Software and has a GPS plugged in providing time and location information (GPS is a backup to the Antenna Tracker GPS and is also used for time synchronization across the system)
2. Rocket is on Pad
    - We turn on the Rocket
    - We connect over bluetooth to the Live Camera PicoW and tell it to go to 4W and make sure we can see it on our VRX/monitor, then we set it back to pad mode/reset and disconnect bluetooth to save power
    - We turn on the bottom ADS board on 433Mhz GFSK and the top Bareman board on 915mhz Lora and they send out a ready signal to the Ground Station
    - Because the bottom and top are on different frequencies, we can split the time system such that the first 666ms of every second is used for the top and bottom to transmit to GS or Mobile Trackers, 
        and the last 334ms of every second is used for Mobile trackers to transmit relayed/stored messages to GS or iPhone GPS data 
        (to track the mobile trackers themselves) [Mobile Trackers don't need an iPhone/Bluetooth connection, but when 
        they do they can use the GPS phone data and MQTT over cellular to transmit messages in adition to the radio packets]
        - Mobile Trackers use a simpler form of Contention Resolution Diversity Slotted ALOHA (CRDSA) where they randomly pick a slot in the last 334ms of the second to transmit, and if they don't get an ACK they retry with exponential backoff
    - Once the ready signals are received, we then from the GS AT or Low Gain send out TimeSync/Pings to determine initial latency and time offset between the GS and the Trackers,
        and then we exchange GPS data such that the Bottom ADS knows where the GS is so it can appropriately select 1 of the 4 antennas to use based on the relative position of the GS and the rocket (each antenna is on one side of the rocket split into 4 quadrants, front/back and left/right)
3. Rocket Launches
    - During the boost phase, the trackers are sending out high priority boost packets with Fused Nav State (GPS + IMU + Baro fused together with a Kalman Filter) to the Ground Station and Mobile Trackers at 10Hz, and the Ground Station is using this information to do predictive tracking of the rocket and to point the directional antennas appropriately
    - During the coast phase, the trackers are sending out lower priority coast packets with GPS and IMU data to the Ground Station and Mobile Trackers at 1Hz, and the Ground Station is using this information to continue tracking the rocket and to update the predicted landing location
    - During the descent phase, the trackers are sending out high priority descent packets with GPS and Barometer data to the Ground Station and Mobile Trackers at 5Hz, and the Ground Station is using this information to track the rocket and to point the directional antennas appropriately, and to update the predicted landing location
    - During the landing phase, the trackers are sending out high priority landing packets with GPS and Barometer data to the Ground Station and Mobile Trackers at 1Hz, and the Ground Station is using this information to track the rocket and to point the directional antennas appropriately, and to update the predicted landing location
4. Post Flight
    - After landing, the trackers are sending out low priority status packets with GPS, Barometer, and overall system health information to the Ground Station and Mobile Trackers at 0.1Hz
*/
namespace SIGMA2 {

namespace wire {
    inline uint16_t read_u16_le(const uint8_t* buf, size_t& i) {
        const uint8_t lo = buf[i++];
        const uint8_t hi = buf[i++];
        return static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    }

    inline void write_u16_le(uint8_t* buf, size_t& i, uint16_t value) {
        buf[i++] = static_cast<uint8_t>(value & 0xFFu);
        buf[i++] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    }

    inline uint32_t read_u32_le(const uint8_t* buf, size_t& i) {
        const uint32_t b0 = buf[i++];
        const uint32_t b1 = buf[i++];
        const uint32_t b2 = buf[i++];
        const uint32_t b3 = buf[i++];
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
    }

    inline void write_u32_le(uint8_t* buf, size_t& i, uint32_t value) {
        buf[i++] = static_cast<uint8_t>(value & 0xFFu);
        buf[i++] = static_cast<uint8_t>((value >> 8) & 0xFFu);
        buf[i++] = static_cast<uint8_t>((value >> 16) & 0xFFu);
        buf[i++] = static_cast<uint8_t>((value >> 24) & 0xFFu);
    }
}

enum class FLIGHT_STATE : uint8_t {
    UNKOWN = 0,
    PAD    = 1,
    BOOST  = 2,
    COAST  = 3,
    APOGEE = 4,
    DESCENT= 5,
    LANDED = 6,
};

/**
 * @brief Flight Events bitfield, this represents significant events in the flight like that we have detected liftoff, drouge deployment, main deployment, apogee, etc. These are used for triggering certain actions in the system (e.g. switching antennas, sending notifications, etc.)
 * 
 */
struct FLIGHT_EVENT {
    static constexpr uint8_t LIFTOFF_DETECTED   = 0x01; // Bit 0: Set when liftoff is detected based on accel threshold
    static constexpr uint8_t BOOST_END_DETECTED = 0x02; // Bit 1: Set when boost phase is detected to be over based on accel/gyro patterns
    static constexpr uint8_t APOGEE_DETECTED   = 0x04; // Bit 2: Set when apogee is detected based on baro altitude and vertical velocity
    static constexpr uint8_t DROGUE_DEPLOYED   = 0x08; // Bit 3: Set when drogue parachute deployment is detected based on accel/gyro patterns
    static constexpr uint8_t MAIN_DEPLOYED     = 0x10; // Bit 4: Set when main parachute deployment is detected based on accel/gyro patterns
    static constexpr uint8_t LANDED_DETECTED   = 0x20; // Bit 5: Set when landing is detected based on accel patterns and low altitude
    static constexpr uint8_t ERROR             = 0x40; // Bit 6: Set when an error condition is detected (e.g. sensor failure, GPS loss, etc.)
    static constexpr uint8_t RESERVED          = 0x80; // Bit 7: Reserved for future use
};

struct DATA_VALID_FLAG {
    static constexpr uint8_t GPS_VALID  = 1u << 0;
    static constexpr uint8_t BARO_VALID = 1u << 1;
    static constexpr uint8_t IMU_VALID  = 1u << 2;
    static constexpr uint8_t MAG_VALID  = 1u << 3;
    static constexpr uint8_t TIME_VALID = 1u << 4;
};


enum class SYSTEM_STATE: uint8_t {
    UNKOWN = 0,
    READY_FOR_LAUNCH = 1,
    NOT_READY_FOR_LAUNCH = 2,
    IN_FLIGHT = 3,
    LANDED = 4,
    BATTERY_LOW = 5,
    RADIO_ERROR = 6,
    LINK_ERROR = 7,
    GPS_ERROR = 8,
    IMU_ERROR = 9,
    BARO_ERROR = 10,
    OTHER_ERROR = 11,
    GENERAL_ERROR = 255,
};

enum class CoordinateFrame : uint8_t {
    UNKNOWN  = 0,
    NED      = 1, // North, East, Down local tangent frame
    ENU      = 2, // East, North, Up local tangent frame
    BODY_FRD = 3, // Body forward, right, down
    BODY_FLU = 4, // Body forward, left, up
};

// --------- Generic Packet Structures --------- //
/*

These packets aren't necessarily used for real-time communication but are more for logging, storage, or flexible command formats. They have a more flexible payload format but are not intended for real-time communication.

*/

// milli-g
struct Accelerometer {
    int16_t x_mg          = 0; // milli-G (1 G = 9.81 m/s^2)
    int16_t y_mg          = 0; // milli-G
    int16_t z_mg          = 0; // milli-G
    int16_t sensor_temp_c = 0; // Deg C
};

// milli-degrees per second
struct Gyroscope {
    int16_t x_mdps        = 0; // milli-degrees per second
    int16_t y_mdps        = 0; // milli-degrees per second
    int16_t z_mdps        = 0; // milli-degrees per second
    int16_t sensor_temp_c = 0; // Deg C
};

// milli-Gauss
struct Magnetometer {
    int16_t x_mG          = 0; // milli-Gauss
    int16_t y_mG          = 0; // milli-Gauss
    int16_t z_mG          = 0; // milli-Gauss
    int16_t sensor_temp_c = 0; // Deg C
};

struct Barometer {
    int32_t altitude_cm   = 0; // Altitude in centimeters (derived from pressure and temperature)
    int32_t pressure_pa   = 0; // Pressure in Pascals
    int16_t temperature_c = 0; // Temperature in Celsius * 100 (e.g. 2534 = 25.34 C)
};

struct IMU_6Axis {
    Accelerometer accel = {}; // milli-G
    Gyroscope gyro      = {}; // milli-degrees per second
};

struct IMU_9Axis {
    Accelerometer accel = {}; // milli-G
    Gyroscope gyro      = {}; // milli-degrees per second
    Magnetometer mag    = {}; // milli-Gauss
};

// Q1.15 fixed point quaternion
struct Attitude {
    int16_t w = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct RPY {
    int16_t roll_deg;  // Roll in degrees * 100 (e.g. 1234 = 12.34 degrees)
    int16_t pitch_deg; // Pitch in degrees * 100
    int16_t yaw_deg;   // Yaw in degrees * 100
};

// Forward Declarations for Packet Structs
namespace TRANSMIT_PACKETS {
    struct GPSNav;
    struct NavState;
    struct TimeSync;
}

namespace GENERAL_PACKETS {
    struct GPS_NAV_PVT;
};


};
