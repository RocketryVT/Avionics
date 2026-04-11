#pragma once

// gps/ubx_commands.hpp — stateless UBX command frame builders
//
// Every function returns a UbxFrame (stack-allocated, no heap) ready to write
// to a transport.  The Fletcher-8 checksum is computed and appended automatically.
//
// Usage:
//   auto frame = gps::Ubx::enable_nav_pvt(gps::Port::UART1);
//   transport.write(frame.bytes(), frame.size);
//
// Or via GpsDriver:
//   driver.send_ubx(gps::Ubx::cfg_rate(100));   // 10 Hz
//
// UBX frame wire format:
//   0xB5 0x62        sync chars
//   class  id        message class and id (1 byte each)
//   len_lo len_hi    payload length, little-endian uint16
//   [ payload ]      0..N bytes
//   ck_a  ck_b       Fletcher-8 checksum over class+id+len+payload

#include <array>
#include <cstddef>
#include <cstdint>

#include "types.hpp"

namespace gps {

// -- UbxFrame -----------------------------------------------------------------
// Fixed-size buffer for one outgoing UBX command.
// 64 bytes covers all CFG messages (largest CFG-PRT payload = 20 bytes).

static constexpr std::size_t UBX_FRAME_MAX = 64;

struct UbxFrame {
    std::array<uint8_t, UBX_FRAME_MAX> data{};
    std::size_t size = 0;

    [[nodiscard]] const uint8_t* bytes() const noexcept { return data.data(); }
};

// -- Ubx command builders -��---------------------------------------------------

namespace Ubx {
namespace detail {

inline void append(UbxFrame& f, uint8_t b) noexcept {
    f.data[f.size++] = b;
}
inline void append_u16le(UbxFrame& f, uint16_t v) noexcept {
    append(f, static_cast<uint8_t>(v & 0xFF));
    append(f, static_cast<uint8_t>(v >> 8));
}
inline void append_u32le(UbxFrame& f, uint32_t v) noexcept {
    append(f, static_cast<uint8_t>( v        & 0xFF));
    append(f, static_cast<uint8_t>((v >>  8) & 0xFF));
    append(f, static_cast<uint8_t>((v >> 16) & 0xFF));
    append(f, static_cast<uint8_t>((v >> 24) & 0xFF));
}

// Write sync + class + id + length placeholder.  Returns index of len_lo.
inline std::size_t begin(UbxFrame& f, uint8_t cls, uint8_t id) noexcept {
    f.size = 0;
    append(f, 0xB5); append(f, 0x62);
    append(f, cls);  append(f, id);
    append(f, 0x00); append(f, 0x00);   // length placeholder
    return 4;
}

// Patch length field, compute and append Fletcher-8 checksum.
inline void end(UbxFrame& f, std::size_t len_idx) noexcept {
    const std::size_t plen = f.size - len_idx - 2;
    f.data[len_idx]     = static_cast<uint8_t>(plen & 0xFF);
    f.data[len_idx + 1] = static_cast<uint8_t>(plen >> 8);
    uint8_t a = 0, b = 0;
    for (std::size_t i = 2; i < f.size; ++i) { a += f.data[i]; b += a; }
    append(f, a); append(f, b);
}

} // namespace detail

// --- CFG-PRT (0x06 0x00) -----------------------------------------------------
// Set input/output protocol masks for a port.  For UART, also sets the baud
// rate and hard-codes 8N1 framing.  For I2C / SPI / USB, pass baud = 0.
[[nodiscard]] inline UbxFrame cfg_prt(Port     port,
                                      uint32_t baud,
                                      InProto  in_proto,
                                      OutProto out_proto) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x00);
    detail::append(f, static_cast<uint8_t>(port));  // portID
    detail::append(f, 0x00);                         // reserved0
    detail::append_u16le(f, 0x0000);                 // txReady — disabled
    // UART mode: 8N1 → DataBits::Eight(0b11<<6) | Parity::None(0b100<<9) = 0x8C0
    const uint32_t mode = (port == Port::UART1 || port == Port::UART2)
                          ? 0x000008C0u : 0x00000000u;
    detail::append_u32le(f, mode);
    detail::append_u32le(f, baud);
    detail::append_u16le(f, static_cast<uint16_t>(in_proto));
    detail::append_u16le(f, static_cast<uint16_t>(out_proto));
    detail::append_u16le(f, 0x0000);                 // flags
    detail::append_u16le(f, 0x0000);                 // reserved1
    detail::end(f, li);
    return f;
}

// --- CFG-MSG (0x06 0x01) — per-port rate -------------------------------------
// Enable or disable a specific UBX message on one port.
// rate = 0 → disabled; rate = 1 → once per navigation solution.
[[nodiscard]] inline UbxFrame cfg_msg(uint8_t msg_class,
                                      uint8_t msg_id,
                                      Port    port,
                                      uint8_t rate) noexcept
{
    // 8-byte payload: class, id, then one rate slot per port (I2C/U1/U2/USB/SPI/reserved).
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x01);
    detail::append(f, msg_class);
    detail::append(f, msg_id);
    for (uint8_t p = 0; p < 6; ++p)
        detail::append(f, (p == static_cast<uint8_t>(port)) ? rate : 0x00u);
    detail::end(f, li);
    return f;
}

// --- CFG-MSG (0x06 0x01) — same rate on all ports ----------------------------
[[nodiscard]] inline UbxFrame cfg_msg_all(uint8_t msg_class,
                                          uint8_t msg_id,
                                          uint8_t rate) noexcept
{
    // 3-byte payload: class, id, rate (applied to the current/all ports)
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x01);
    detail::append(f, msg_class);
    detail::append(f, msg_id);
    detail::append(f, rate);
    detail::end(f, li);
    return f;
}

// --- CFG-RATE (0x06 0x08) ----------------------------------------------------
// Set the navigation measurement rate.
//   meas_rate_ms : period in ms  (100 = 10 Hz, 200 = 5 Hz, 1000 = 1 Hz)
//   nav_rate     : measurements per navigation solution (usually 1)
[[nodiscard]] inline UbxFrame cfg_rate(uint16_t meas_rate_ms,
                                       uint16_t nav_rate = 1) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x08);
    detail::append_u16le(f, meas_rate_ms);
    detail::append_u16le(f, nav_rate);
    detail::append_u16le(f, 0x0001);   // timeRef = GPS
    detail::end(f, li);
    return f;
}

// --- CFG-RST (0x06 0x04) -----------------------------------------------------
// Reset the receiver.
//   nav_bb_mask : 0x0000 = hot start, 0x0001 = warm start, 0xFFFF = cold start
//   reset_mode  : 0x00 = hardware reset, 0x01 = controlled software reset,
//                 0x02 = GNSS-only software reset, 0x04 = hardware after shutdown
[[nodiscard]] inline UbxFrame cfg_rst(uint16_t nav_bb_mask = 0x0000,
                                      uint8_t  reset_mode  = 0x01) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x04);
    detail::append_u16le(f, nav_bb_mask);
    detail::append(f, reset_mode);
    detail::append(f, 0x00);   // reserved
    detail::end(f, li);
    return f;
}

// --- CFG-CFG (0x06 0x09) — save configuration --------------------------------
// Persist all current settings to battery-backed RAM and flash.
[[nodiscard]] inline UbxFrame cfg_save() noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x09);
    detail::append_u32le(f, 0x00000000);   // clearMask — nothing to clear
    detail::append_u32le(f, 0x0000FFFF);   // saveMask  — all sections
    detail::append_u32le(f, 0x00000000);   // loadMask  — nothing to load
    detail::append(f, 0x17);               // deviceMask — BBR + Flash + EEPROM + SPI flash
    detail::end(f, li);
    return f;
}

// --- Poll request -------------------------------------------------------------
// Send a zero-payload poll for any message.  The module responds with the
// current value of that message.
[[nodiscard]] inline UbxFrame poll(uint8_t msg_class, uint8_t msg_id) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, msg_class, msg_id);
    detail::end(f, li);
    return f;
}

// --- Named NAV message helpers ------------------------------------------------

[[nodiscard]] inline UbxFrame enable_nav_pvt(Port port, uint8_t rate = 1) noexcept {
    return cfg_msg(0x01, 0x07, port, rate);
}
[[nodiscard]] inline UbxFrame disable_nav_pvt(Port port) noexcept {
    return cfg_msg(0x01, 0x07, port, 0);
}
[[nodiscard]] inline UbxFrame enable_nav_hpposllh(Port port, uint8_t rate = 1) noexcept {
    return cfg_msg(0x01, 0x14, port, rate);
}
[[nodiscard]] inline UbxFrame enable_nav_sat(Port port, uint8_t rate = 1) noexcept {
    return cfg_msg(0x01, 0x35, port, rate);
}
[[nodiscard]] inline UbxFrame enable_nav_dop(Port port, uint8_t rate = 1) noexcept {
    return cfg_msg(0x01, 0x04, port, rate);
}

// Disable all standard NMEA output sentences on a port.
// Returns an array of 6 frames — send them all.
[[nodiscard]] inline std::array<UbxFrame, 6> disable_all_nmea(Port port) noexcept {
    return {{
        cfg_msg(0xF0, 0x00, port, 0),   // GGA
        cfg_msg(0xF0, 0x04, port, 0),   // RMC
        cfg_msg(0xF0, 0x02, port, 0),   // GSA
        cfg_msg(0xF0, 0x03, port, 0),   // GSV
        cfg_msg(0xF0, 0x01, port, 0),   // GLL
        cfg_msg(0xF0, 0x05, port, 0),   // VTG
    }};
}

} // namespace Ubx
} // namespace gps
