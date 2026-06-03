// radio/Radio.hpp — core abstractions shared by all radio drivers.
//
// Defines:
//   radio::Packet   — received-packet descriptor
//   radio::IRadio   — pure interface every driver implements
#pragma once

#include <cstdint>

namespace radio {

// -- Packet --------------------------------------------------------------------
// Filled by IRadio::read_packet().  Fields unused by a given modulation are
// left at their zero-initialised defaults (e.g. snr for FSK radios).

struct Packet {
    uint8_t data[256] = {};
    uint8_t len       = 0;
    float   rssi      = 0.0f;  // dBm
    float   snr       = 0.0f;  // dB — LoRa only; 0 for FSK
};

// -- IRadio --------------------------------------------------------------------
// Minimal, polling-oriented interface for receive-focused radio drivers.
// All drivers are constructed with their config; begin() takes no arguments.

class IRadio {
public:
    virtual ~IRadio() = default;

    // Initialise hardware (SPI, GPIO) and apply stored configuration.
    // Returns 0 on success, a negative driver-specific error code on failure.
    virtual int  begin()                       = 0;

    // Enter continuous receive mode.  Must be called once after begin() and
    // again after every read_packet() to re-arm the receiver.
    virtual void start_receive()               = 0;

    // Non-blocking poll.  Returns true when a packet is waiting in the FIFO.
    virtual bool packet_available() const      = 0;

    // Dequeue the waiting packet into pkt.  Call only when packet_available()
    // is true.  Returns 0 on success, negative on error.
    virtual int  read_packet( Packet& pkt )    = 0;
};

} // namespace radio
