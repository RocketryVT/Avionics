#pragma once

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
    - WiFi AP is active and waiting for connections from the Antenna Tracker, Low Gain GS, and Laptop
    - Antenna Tracker boots up and we calibrate it on the ground
        - Antenna Tracker hopefully has a GPS lock and can send position information to the Ground Station
    - Low Gain GS is setup and connected over USB to the Ground Station Laptop
    - Laptop is running the Ground Station Software and has a GPS plugged in providing time and location information
2. Rocket is on Pad
    - We turn on the Rocket and the bottom ADS board on 433Mhz GFSK and the top Bareman board on 915mhz Lora send out a ready signal to the Ground Station
    - We split transmitions into 1 second chunks, bottom ADS gets first 333ms, top Bareman gets next 333ms, and then 333ms of silence/mobile telemetry from the trackers to the ground station giving either relayed data from the rocket or mobile tracker/iPhone data

*/

enum class FLIGHT_STATE : uint8_t {
    UNKOWN = 0,
    PAD    = 1,
    BOOST  = 2,
    COAST  = 3,
    APOGEE = 4,
    DESCENT= 5,
    LANDED = 6,
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

// --------- Generic Packet Structures --------- //

// milli-g
struct Accelerometer {
    int16_t x_mg; // milli-G (1 G = 9.81 m/s^2)
    int16_t y_mg; // milli-G
    int16_t z_mg; // milli-G
};

// milli-degrees per second
struct Gyroscope {
    int16_t x_mdps; // milli-degrees per second
    int16_t y_mdps; // milli-degrees per second
    int16_t z_mdps; // milli-degrees per second
};

// milli-Gauss
struct Magnetometer {
    int16_t x_mG; // milli-Gauss
    int16_t y_mG; // milli-Gauss
    int16_t z_mG; // milli-Gauss
};

struct Barometer {
    int32_t pressure_pa;   // Pressure in Pascals
    int16_t temperature_c; // Temperature in Celsius * 100 (e.g. 2534 = 25.34 C)
};

struct IMU_6Axis {
    Accelerometer accel; // milli-G
    Gyroscope gyro;      // milli-degrees per second
};

struct IMU_9Axis {
    Accelerometer accel; // milli-G
    Gyroscope gyro;      // milli-degrees per second
    Magnetometer mag;    // milli-Gauss
};

// Q1.15 fixed point quaternion
struct Attitude {
    int16_t q[4]; // Q1.15 quaternion [w,x,y,z]
};

struct RPY {
    int16_t roll_deg;  // Roll in degrees * 100 (e.g. 1234 = 12.34 degrees)
    int16_t pitch_deg; // Pitch in degrees * 100
    int16_t yaw_deg;   // Yaw in degrees * 100
};

// --------- Time Sync Packets --------- //

struct TimeSync {
    uint32_t timestamp_ms; // time since boot in milliseconds
    uint32_t utc_ms;       // UTC ms since Unix epoch
    uint16_t accuracy_ms;  // estimated accuracy of the timestamp in milliseconds
    uint8_t  flags;       // bitmask of flags (e.g. valid, source, etc.)
};


// --------- GNSS Packets --------- //

struct GPSNav {
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
};

struct NavSource {
    static constexpr int8_t GPS_POS      = 1u << 0;
    static constexpr int8_t GPS_VEL      = 1u << 1;
    static constexpr int8_t BARO_ALT     = 1u << 2;
    static constexpr int8_t BARO_VEL     = 1u << 3;
    static constexpr int8_t IMU_ACCEL    = 1u << 4;
    static constexpr int8_t AHRS_ATT     = 1u << 5;
    static constexpr int8_t KALMAN_STATE = 1u << 6;
    static constexpr int8_t PREDICTED    = 1u << 7;
};

struct NavState {
    uint8_t nav_source;      // bitmask of NavSource

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
    FLIGHT_STATE  state;          // flight state
    uint8_t  flags;
};