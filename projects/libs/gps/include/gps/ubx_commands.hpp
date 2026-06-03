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
//
// Generation 9+ (NEO-M9N, ZED-F9P, …) uses CFG-VALSET / CFG-VALGET for all
// configuration.  Use the raw_valset_u8 / raw_valset_u16 / raw_valget builders
// or the named wrappers that call them.
//
// CFG-VALSET (0x06 0x8A) payload:
//   version(1)  layers(1)  reserved(2)  key(4 LE)  value(N)
//   layers byte: 0x01 = RAM, 0x02 = BBR, 0x04 = Flash
//
// CFG-VALGET (0x06 0x8B) payload:
//   version(1)  layer(1)  position(2)  key(4 LE)
//   layer byte: 0x00 = RAM, 0x01 = BBR, 0x02 = Flash

#include <array>
#include <cstddef>
#include <cstdint>

#include "types.hpp"

namespace gps {

// -- CFG-VALSET / CFG-VALGET layer selectors ----------------------------------
// Defined in namespace gps (not gps::Ubx) so call sites write gps::ValLayer::RAM.

// Bitmask for CFG-VALSET layers (may be OR'd):
//   RAM   = 0x01  — takes effect immediately, lost on power-cycle
//   BBR   = 0x02  — battery-backed RAM, survives soft reset
//   Flash = 0x04  — non-volatile, survives power-cycle
enum class ValLayer : uint8_t {
    RAM   = 0x01,
    BBR   = 0x02,
    Flash = 0x04,
};
constexpr ValLayer operator|(ValLayer a, ValLayer b) noexcept {
    return static_cast<ValLayer>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

// Layer selector for CFG-VALGET:
//   RAM   = 0x00  BBR = 0x01  Flash = 0x02
enum class ValGetLayer : uint8_t {
    RAM   = 0x00,
    BBR   = 0x01,
    Flash = 0x02,
};

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

// --- CFG-NAV5 (0x06 0x24) — navigation engine settings ----------------------
// Sets the dynamic platform model.
// Only the dynModel byte and its mask bit are set; all other fields are left
// at their current module values (mask = 0x0001 → apply dynModel only).
//
// Common dynModel values:
//   0 = portable  2 = stationary  3 = pedestrian  4 = automotive
//   5 = sea       6 = airborne <1g  7 = airborne <2g  8 = airborne <4g
[[nodiscard]] inline UbxFrame cfg_nav5_dyn_model(uint8_t dyn_model) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x24);
    detail::append_u16le(f, 0x0001);    // mask — apply dynModel only
    detail::append(f, dyn_model);       // dynModel
    // remaining 34 payload bytes zeroed (ignored due to mask)
    for (int i = 0; i < 34; ++i) detail::append(f, 0x00);
    detail::end(f, li);
    return f;
}

// --- CFG-GNSS (0x06 0x3E) — GNSS constellation configuration ----------------
// Enables all major constellations: GPS, SBAS, Galileo, BeiDou, QZSS, GLONASS.
// Channel counts are representative defaults for the NEO-M9N (72 channels).
// flags bits[16:0] = 0x01 (enable) | sigCfgMask shifted to bits[28:16].
//
// sigCfgMask per constellation (bits 28:16 of flags field):
//   GPS     0x01 → L1C/A
//   SBAS    0x01 → L1C/A
//   Galileo 0x01 → E1
//   BeiDou  0x01 → B1I
//   QZSS    0x05 → L1C/A + L1S
//   GLONASS 0x01 → L1OF
[[nodiscard]] inline UbxFrame cfg_gnss_all() noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x3E);

    // Header: msgVer=0, numTrkChHw=0(read-only, ignored), numTrkChUse=0xFF(use all), numConfigBlocks=6
    detail::append(f, 0x00);   // msgVer
    detail::append(f, 0x00);   // numTrkChHw (ignored on write)
    detail::append(f, 0xFF);   // numTrkChUse — use all available channels
    detail::append(f, 0x06);   // numConfigBlocks

    // Helper lambda: one 8-byte config block.
    // gnssId, resTrkCh (min channels reserved), maxTrkCh, reserved, flags(u32le)
    auto block = [&](uint8_t gnss_id, uint8_t res_ch, uint8_t max_ch,
                     uint32_t flags) {
        detail::append(f, gnss_id);
        detail::append(f, res_ch);
        detail::append(f, max_ch);
        detail::append(f, 0x00);   // reserved1
        detail::append_u32le(f, flags);
    };

    // flags = enable(bit0) | sigCfgMask(bits 28:16)
    // sigCfgMask is OR'd into the upper 16 bits shifted left 16 + bit 16 offset
    // Encoding: bits[28:16] hold the signal config; bit 0 = enable.
    // Use (sigCfgMask << 16) | 0x01.
    block(0, 8,  16, (0x01u << 16) | 0x01u);   // GPS      — L1C/A
    block(1, 1,   3, (0x01u << 16) | 0x01u);   // SBAS     — L1C/A
    block(2, 4,   8, (0x01u << 16) | 0x01u);   // Galileo  — E1
    block(3, 8,  16, (0x01u << 16) | 0x01u);   // BeiDou   — B1I
    block(5, 0,   3, (0x05u << 16) | 0x01u);   // QZSS     — L1C/A + L1S
    block(6, 8,  14, (0x01u << 16) | 0x01u);   // GLONASS  — L1OF

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

// =============================================================================
// CFG-VALSET / CFG-VALGET builders (Generation 9+)
// ValLayer / ValGetLayer are defined in namespace gps above.
// =============================================================================

// --- CFG-VALSET (0x06 0x8A) — set one 1-byte key/value pair ------------------
[[nodiscard]] inline UbxFrame raw_valset_u8(uint32_t key,
                                            uint8_t  value,
                                            ValLayer layers) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x8A);
    detail::append(f, 0x01);                              // version
    detail::append(f, static_cast<uint8_t>(layers));     // layers bitmask
    detail::append(f, 0x00); detail::append(f, 0x00);    // reserved
    detail::append_u32le(f, key);
    detail::append(f, value);
    detail::end(f, li);
    return f;
}

// --- CFG-VALSET (0x06 0x8A) — set one 2-byte (U2) key/value pair -------------
[[nodiscard]] inline UbxFrame raw_valset_u16(uint32_t key,
                                             uint16_t value,
                                             ValLayer layers) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x8A);
    detail::append(f, 0x01);
    detail::append(f, static_cast<uint8_t>(layers));
    detail::append(f, 0x00); detail::append(f, 0x00);
    detail::append_u32le(f, key);
    detail::append_u16le(f, value);
    detail::end(f, li);
    return f;
}

// --- CFG-VALGET (0x06 0x8B) — query one key ----------------------------------
[[nodiscard]] inline UbxFrame raw_valget(uint32_t     key,
                                         ValGetLayer  layer = ValGetLayer::RAM) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x8B);
    detail::append(f, 0x00);                              // version
    detail::append(f, static_cast<uint8_t>(layer));      // layer
    detail::append(f, 0x00); detail::append(f, 0x00);    // position
    detail::append_u32le(f, key);
    detail::end(f, li);
    return f;
}

// =============================================================================
// Named CFG-VALSET wrappers — NEO-M9N / ZED-F9P configuration keys
// All default to RAM|BBR|Flash so one call persists across power cycles.
// Pass a different `layers` to target only RAM (for runtime-only changes).
// =============================================================================

// --- CFG-MSGOUT-UBX_NAV_PVT_UART1 (key 0x20910007) --------------------------
// rate=1 → one NAV-PVT per navigation solution on UART1
[[nodiscard]] inline UbxFrame valset_nav_pvt_uart1(
    uint8_t rate = 1,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x20910007u, rate, layers);
}

// --- CFG-MSGOUT-UBX_NAV_ODO_UART1 (key 0x2091007F) --------------------------
// rate=1 → one NAV-ODO per navigation solution on UART1
[[nodiscard]] inline UbxFrame valset_nav_odo_uart1(
    uint8_t rate = 1,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x2091007Fu, rate, layers);
}

// --- CFG-MSGOUT-UBX_NAV_DOP_UART1 (key 0x20910039) --------------------------
// rate=1 → one NAV-DOP per navigation solution on UART1
[[nodiscard]] inline UbxFrame valset_nav_dop_uart1(
    uint8_t rate = 1,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x20910039u, rate, layers);
}

// --- CFG-MSGOUT-UBX_NAV_SAT_UART1 (key 0x20910016) --------------------------
// rate=1 → one NAV-SAT per navigation solution on UART1
// NAV-SAT is variable-length (~8 + numSvs×12 bytes); can be large at 25 Hz.
[[nodiscard]] inline UbxFrame valset_nav_sat_uart1(
    uint8_t rate = 1,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x20910016u, rate, layers);
}

// --- CFG-NAVSPG-FIXMODE (key 0x20110011) -------------------------------------
// 1=2D only  2=3D only  3=auto (default)
[[nodiscard]] inline UbxFrame valset_fix_mode(
    uint8_t mode = 3,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x20110011u, mode, layers);
}

// --- CFG-NAVSPG-DYNMODEL (key 0x20110021) ------------------------------------
// 0=portable 2=stationary 3=pedestrian 4=automotive 5=sea
// 6=air<1g  7=air<2g  8=air<4g (AIR4 — use for rockets)
[[nodiscard]] inline UbxFrame valset_dyn_model(
    uint8_t model = 8,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x20110021u, model, layers);
}

// --- CFG-UART1INPROT-UBX (key 0x10730001) ------------------------------------
[[nodiscard]] inline UbxFrame valset_uart1_inprot_ubx(
    bool enable = true,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x10730001u, enable ? 1u : 0u, layers);
}

// --- CFG-UART1INPROT-NMEA (key 0x10730002) ------------------------------------
[[nodiscard]] inline UbxFrame valset_uart1_inprot_nmea(
    bool enable = true,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x10730002u, enable ? 1u : 0u, layers);
}

// --- CFG-UART1OUTPROT-UBX (key 0x10740001) ------------------------------------
[[nodiscard]] inline UbxFrame valset_uart1_outprot_ubx(
    bool enable = true,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x10740001u, enable ? 1u : 0u, layers);
}

// --- CFG-UART1OUTPROT-NMEA (key 0x10740002) -----------------------------------
[[nodiscard]] inline UbxFrame valset_uart1_outprot_nmea(
    bool enable = false,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u8(0x10740002u, enable ? 1u : 0u, layers);
}

// --- CFG-RATE-MEAS (key 0x30210001) — measurement period in ms ---------------
// 40 ms = 25 Hz, 100 ms = 10 Hz, 200 ms = 5 Hz, 1000 ms = 1 Hz
[[nodiscard]] inline UbxFrame valset_rate_meas(
    uint16_t meas_ms = 40,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u16(0x30210001u, meas_ms, layers);
}

// --- CFG-RATE-NAV (key 0x30210002) — measurements per nav solution -----------
// Minimum 1, maximum 127.  Normally 1 (nav solution every measurement).
[[nodiscard]] inline UbxFrame valset_rate_nav(
    uint16_t ratio = 1,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    return raw_valset_u16(0x30210002u, ratio, layers);
}

// --- CFG-UART1-BAUDRATE (key 0x40520001) — UART1 baud rate ------------------
// Send at the CURRENT baud rate; re-init the host UART immediately after.
// Common values: 9600, 38400, 115200, 230400, 460800, 921600.
[[nodiscard]] inline UbxFrame valset_uart1_baud(
    uint32_t baud,
    ValLayer layers = ValLayer::RAM | ValLayer::BBR | ValLayer::Flash) noexcept
{
    UbxFrame f;
    const auto li = detail::begin(f, 0x06, 0x8A);
    detail::append(f, 0x01);
    detail::append(f, static_cast<uint8_t>(layers));
    detail::append(f, 0x00); detail::append(f, 0x00);
    detail::append_u32le(f, 0x40520001u);   // key: CFG-UART1-BAUDRATE (U4)
    detail::append_u32le(f, baud);
    detail::end(f, li);
    return f;
}

} // namespace Ubx
} // namespace gps
