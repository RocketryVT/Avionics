#pragma once

// SX123x FSK/OOK transceiver driver stub (SX1231, SX1233, etc.).

#include "../lora_interface.hpp"
#include "../hal/hal_interface.hpp"

#include <cstdint>
#include <span>

template <typename Hal>
    requires HalImpl<Hal>
struct SX123x
{
    struct Config {
        uint8_t  pin_reset;
        uint8_t  pin_dio0;
        uint32_t frequency_hz     = 915'000'000;
        uint8_t  bandwidth        = 7;
        uint8_t  spreading_factor = 9;
        uint8_t  coding_rate      = 1;
        int8_t   tx_power_dbm     = 14;
    };

    SX123x(Hal& hal, Config cfg) : m_hal{hal}, m_cfg{cfg} {}

    bool initialize()                        { /* TODO */ return false; }
    bool send(std::span<const uint8_t> data) { /* TODO */ return false; }
    int  receive(std::span<uint8_t> buf)     { /* TODO */ return -1; }

    void set_frequency(uint32_t hz)          { m_cfg.frequency_hz = hz; }
    void set_bandwidth(uint8_t bw)           { m_cfg.bandwidth = bw; }
    void set_spreading_factor(uint8_t sf)    { m_cfg.spreading_factor = sf; }
    void set_coding_rate(uint8_t cr)         { m_cfg.coding_rate = cr; }
    void set_tx_power(int8_t dbm)            { m_cfg.tx_power_dbm = dbm; }

    int16_t get_rssi() { return 0; }
    int8_t  get_snr()  { return 0; }
    void    sleep()    { /* TODO */ }
    void    standby()  { /* TODO */ }

private:
    Hal&   m_hal;
    Config m_cfg;
};
