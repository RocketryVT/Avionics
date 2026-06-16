#pragma once

// gps/gps_driver.hpp — u-blox GPS driver (C++23)
//
// Single user-facing GPS API. Owns the transport, packet routing, protocol
// parsers, and unified Coordinate output.
//
// Typical usage (UART, UBX-only output, NAV-PVT at 5 Hz)
// --------------------------------------------------------
//   gps::UartTransport uart(uart0, GPS_TX, GPS_RX, 38400);
//   gps::GpsDriver     driver(uart);
//
//   driver.configure({
//       .port         = gps::Port::UART1,
//       .out_proto    = gps::OutProto::UBX,
//       .meas_rate_ms = 200,          // 5 Hz
//   });
//   vTaskDelay(pdMS_TO_TICKS(100));   // let module apply config
//
//   for (;;) {
//       driver.poll();
//       if (driver.has_fix()) {
//           auto& c = driver.coordinate();
//           // use c.latitude, c.longitude, c.altitude, c.vel_*_mms …
//       }
//       vTaskDelay(pdMS_TO_TICKS(10));
//   }
//
// Transport concept (any struct with read/write satisfies it)
// ------------------------------------------------------------
//   struct MyTransport {
//       std::size_t read (uint8_t* buf, std::size_t len);  // returns bytes read
//       std::size_t write(const uint8_t* buf, std::size_t len);
//   };

#include <concepts>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "types.hpp"
#include "nmea_parser.hpp"
#include "ubx_parser.hpp"
#include "ubx_commands.hpp"

namespace gps {

namespace detail {

class GpsStreamRouter {
public:
    GpsStreamRouter(NmeaParser& nmea, UbxParser& ubx) noexcept
        : nmea_(nmea), ubx_(ubx) {}

    void feed(const uint8_t* data, std::size_t len) noexcept {
        for (std::size_t i = 0; i < len; ++i)
            feed_byte(data[i]);
    }

    void feed_ubx_only(const uint8_t* data, std::size_t len) noexcept {
        if (state_ == State::Nmea)
            reset();
        for (std::size_t i = 0; i < len; ++i) {
            if (state_ == State::Idle && data[i] != 0xB5u)
                continue;
            feed_byte(data[i]);
        }
    }

    void reset() noexcept {
        state_ = State::Idle;
        nmea_len_ = 0;
        ubx_len_ = 0;
        ubx_expected_len_ = 0;
    }

private:
    enum class State : uint8_t {
        Idle,
        Nmea,
        Ubx,
    };

    static constexpr std::size_t NMEA_BUF_SIZE = 128;
    static constexpr std::size_t UBX_FRAME_BUF_SIZE = 1024;

    NmeaParser& nmea_;
    UbxParser& ubx_;
    State state_ = State::Idle;

    std::array<uint8_t, NMEA_BUF_SIZE> nmea_buf_{};
    std::size_t nmea_len_ = 0;

    std::array<uint8_t, UBX_FRAME_BUF_SIZE> ubx_buf_{};
    std::size_t ubx_len_ = 0;
    std::size_t ubx_expected_len_ = 0;

    void feed_byte(uint8_t b) noexcept {
        switch (state_) {
        case State::Idle:
            if (b == '$') {
                start_nmea(b);
            } else if (b == 0xB5u) {
                start_ubx(b);
            }
            break;

        case State::Nmea:
            append_nmea(b);
            break;

        case State::Ubx:
            append_ubx(b);
            break;
        }
    }

    void start_nmea(uint8_t b) noexcept {
        state_ = State::Nmea;
        nmea_len_ = 0;
        nmea_buf_[nmea_len_++] = b;
    }

    void append_nmea(uint8_t b) noexcept {
        if (b == '$') {
            start_nmea(b);
            return;
        }

        if (nmea_len_ >= nmea_buf_.size()) {
            reset();
            return;
        }

        nmea_buf_[nmea_len_++] = b;
        if (b == '\n' || b == '\r') {
            feed_nmea_packet(nmea_buf_.data(), nmea_len_);
            reset();
        }
    }

    void start_ubx(uint8_t b) noexcept {
        state_ = State::Ubx;
        ubx_len_ = 0;
        ubx_expected_len_ = 0;
        ubx_buf_[ubx_len_++] = b;
    }

    void append_ubx(uint8_t b) noexcept {
        if (ubx_len_ >= ubx_buf_.size()) {
            reset();
            return;
        }

        ubx_buf_[ubx_len_++] = b;

        if (ubx_len_ == 2u) {
            if (b == 0x62u) {
                return;
            }
            if (b == 0xB5u) {
                start_ubx(b);
                return;
            }
            reset();
            return;
        }

        if (ubx_len_ == 6u) {
            const uint16_t payload_len =
                static_cast<uint16_t>(ubx_buf_[4]) |
                (static_cast<uint16_t>(ubx_buf_[5]) << 8);
            ubx_expected_len_ = static_cast<std::size_t>(payload_len) + 8u;
            if (ubx_expected_len_ > ubx_buf_.size()) {
                reset();
                return;
            }
        }

        if (ubx_expected_len_ != 0u && ubx_len_ >= ubx_expected_len_) {
            feed_ubx_packet(ubx_buf_.data(), ubx_len_);
            reset();
        }
    }

    void feed_nmea_packet(const uint8_t* data, std::size_t len) noexcept {
        for (std::size_t i = 0; i < len; ++i)
            nmea_.feed(data[i]);
    }

    void feed_ubx_packet(const uint8_t* data, std::size_t len) noexcept {
        for (std::size_t i = 0; i < len; ++i)
            ubx_.feed(data[i]);
    }
};

} // namespace detail

// ===============================================================================
// Transport concept
// ===============================================================================

template<typename T>
concept Transport = requires(T t, uint8_t* rbuf, const uint8_t* wbuf, std::size_t n) {
    { t.read (rbuf, n) } -> std::convertible_to<std::size_t>;
    { t.write(wbuf, n) } -> std::convertible_to<std::size_t>;
};

// ===============================================================================
// GpsDriver<T>
// ===============================================================================

template<Transport T>
class GpsDriver {
public:
    explicit GpsDriver(T& transport) noexcept
        : transport_(transport)
        , nmea_(coord_, diag_)
        , ubx_(coord_,  diag_)
        , router_(nmea_, ubx_)
    {}

    // -- Configuration ---------------------------------------------------------
    //
    // configure() sends the typical startup sequence in one call.  After calling
    // it, wait ~100 ms before expecting valid messages from the module.
    //
    // Steps performed (in order):
    //   1. CFG-PRT   — set port protocol masks (and baud for UART)
    //   2. CFG-MSG×6 — silence all standard NMEA output sentences (if NMEA out
    //                  not requested)
    //   3. CFG-MSG   — enable NAV-PVT at pvt_rate
    //   4. CFG-MSG   — optionally enable NAV-HPPOSLLH, NAV-SAT, NAV-DOP
    //   5. CFG-RATE  — set navigation measurement period
    //   6. CFG-CFG   — optionally save to flash

    struct ConfigOptions {
        Port     port           = Port::UART1;
        uint32_t baud           = 38400;
        InProto  in_proto       = InProto::UBX | InProto::NMEA;
        OutProto out_proto      = OutProto::UBX;
        uint8_t  pvt_rate       = 1;      // NAV-PVT messages per navigation solution
        bool     enable_hp      = false;  // NAV-HPPOSLLH — ~1 cm position accuracy
        bool     enable_nav_sat = false;  // NAV-SAT — per-SV C/N0 and used-in-fix
        bool     enable_nav_dop = false;  // NAV-DOP — hDOP / vDOP
        uint16_t meas_rate_ms   = 1000;  // navigation period in ms (1000 = 1 Hz)
        bool     save_config    = false;  // persist settings to flash via CFG-CFG
    };

    void configure(const ConfigOptions& opt = {}) noexcept {
        send_ubx(Ubx::cfg_prt(opt.port, opt.baud, opt.in_proto, opt.out_proto));

        if (!(static_cast<uint16_t>(opt.out_proto) &
              static_cast<uint16_t>(OutProto::NMEA))) {
            for (const auto& f : Ubx::disable_all_nmea(opt.port))
                send_ubx(f);
        }

        send_ubx(Ubx::enable_nav_pvt(opt.port, opt.pvt_rate));

        if (opt.enable_hp)       send_ubx(Ubx::enable_nav_hpposllh(opt.port));
        if (opt.enable_nav_sat)  send_ubx(Ubx::enable_nav_sat(opt.port));
        if (opt.enable_nav_dop)  send_ubx(Ubx::enable_nav_dop(opt.port));

        send_ubx(Ubx::cfg_rate(opt.meas_rate_ms));

        if (opt.save_config) send_ubx(Ubx::cfg_save());
    }

    // Convenience overload for the common UART case.
    void configure(Port port, InProto in_proto, OutProto out_proto,
                   uint16_t meas_rate_ms = 1000) noexcept
    {
        configure(ConfigOptions{
            .port         = port,
            .in_proto     = in_proto,
            .out_proto    = out_proto,
            .meas_rate_ms = meas_rate_ms,
        });
    }

    // -- Send a pre-built UBX command ------------------------------------------
    void send_ubx(const UbxFrame& frame) noexcept {
        transport_.write(frame.bytes(), frame.size);
    }

    // -- Poll transport — call as often as possible ----------------------------
    // Reads chunks and routes complete NMEA sentences or UBX frames to the
    // matching internal parser.
    static constexpr std::size_t READ_CHUNK = 128;

    std::size_t poll() noexcept {
        uint8_t buf[READ_CHUNK];
        std::size_t total = 0;
        for (;;) {
            const std::size_t n = transport_.read(buf, sizeof(buf));
            if (n == 0) break;
            feed(buf, n);
            total += n;
            if (n < sizeof(buf)) break;
        }
        return total;
    }

    // Poll transport and route bytes to the UBX parser only.
    // Use this when the module is configured for UBX output only.
    std::size_t poll_ubx_only() noexcept {
        uint8_t buf[READ_CHUNK];
        std::size_t total = 0;
        for (;;) {
            const std::size_t n = transport_.read(buf, sizeof(buf));
            if (n == 0) break;
            feed_ubx_only(buf, n);
            total += n;
            if (n < sizeof(buf)) break;
        }
        return total;
    }

    // Read raw bytes from the transport into caller-supplied buffer.
    // Returns the number of bytes read.  Does not feed any parser.
    std::size_t read_raw(uint8_t* buf, std::size_t len) noexcept {
        return transport_.read(buf, len);
    }

    // Feed bytes from an external transport through the same packet router.
    void feed(uint8_t b) noexcept {
        feed(&b, 1);
    }

    void feed(const uint8_t* data, std::size_t len) noexcept {
        router_.feed(data, len);
    }

    void feed_ubx_only(const uint8_t* data, std::size_t len) noexcept {
        router_.feed_ubx_only(data, len);
    }

    void reset_stream() noexcept {
        router_.reset();
    }

    // -- Accessors -------------------------------------------------------------
    [[nodiscard]] const Coordinate&  coordinate()   const noexcept { return coord_; }
    [[nodiscard]] bool               has_fix()       const noexcept { return coord_.valid; }
    [[nodiscard]] const Diagnostics& diagnostics()   const noexcept { return diag_; }

    // Monotonic count of fresh valid solutions. If this stops advancing between
    // polls, coordinate() is stale — the position fields still hold their last
    // value but no longer reflect a live fix. Consumers with a clock can record
    // the time this last changed to compute an age (see PicoGpsDriver::is_stale).
    [[nodiscard]] uint32_t           fix_seq()       const noexcept { return coord_.fix_seq; }

    [[nodiscard]] std::string_view   fix_label()     const noexcept {
        return gps::fix_label(coord_);
    }

    // -- Static navigation helpers ---------------------------------------------
    [[nodiscard]] static double distance_between(double lat1, double lon1,
                                                  double lat2, double lon2) noexcept
    {
        return gps::distance_between(lat1, lon1, lat2, lon2);
    }
    [[nodiscard]] static double course_to(double lat1, double lon1,
                                           double lat2, double lon2) noexcept
    {
        return gps::course_to(lat1, lon1, lat2, lon2);
    }
    [[nodiscard]] static std::string_view cardinal(float course) noexcept {
        return gps::cardinal(course);
    }

private:
    T&          transport_;
    Coordinate  coord_{};
    Diagnostics diag_{};
    NmeaParser  nmea_;
    UbxParser   ubx_;
    detail::GpsStreamRouter router_;
};

} // namespace gps

// ===============================================================================
// Platform transport wrappers
// ===============================================================================
// Pico SDK headers must be included outside any namespace — they declare types
// (uart_inst_t, i2c_inst_t, time_us_64, …) that belong in the global namespace.

#if defined(PICO_SDK_VERSION_MAJOR)

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "pico/time.h"

namespace gps {

inline constexpr std::array<uint32_t, 9> GPS_UART_BAUDS = {
    4800u, 9600u, 19200u, 38400u, 57600u, 115200u, 230400u, 460800u, 921600u,
};

enum class UartRxMode : uint8_t {
    Polling,
    DmaRing,
};

enum class DmaRingBufferSize : uint8_t {
    Bytes1K = 10,
    Bytes2K = 11,
    Bytes4K = 12,
};

[[nodiscard]] constexpr uint32_t dma_ring_buffer_bytes(DmaRingBufferSize size) noexcept {
    return 1u << static_cast<uint8_t>(size);
}

struct PicoGpsConfig {
    uart_inst_t* uart;
    uint tx_pin;
    uint rx_pin;
    uint32_t desired_baud;
    UartRxMode rx_mode = UartRxMode::Polling;
    DmaRingBufferSize dma_ring_size = DmaRingBufferSize::Bytes1K;
};

namespace detail {

class GpsWireDetector {
public:
    bool feed(uint8_t b) noexcept {
        feed_nmea(b);
        feed_ubx(b);
        return valid_;
    }

    [[nodiscard]] bool valid() const noexcept { return valid_; }

private:
    enum class NmeaState : uint8_t { Idle, Data, Ck1, Ck2 };
    NmeaState nmea_state_ = NmeaState::Idle;
    uint8_t nmea_ck_ = 0;
    uint8_t nmea_hi_ = 0;

    enum class UbxState : uint8_t {
        Idle, Sync2, Class, Id, Len1, Len2, Payload, CkA, CkB,
    };
    UbxState ubx_state_ = UbxState::Idle;
    uint16_t ubx_len_ = 0;
    uint16_t ubx_idx_ = 0;
    uint8_t ubx_ck_a_ = 0;
    uint8_t ubx_ck_b_ = 0;

    bool valid_ = false;

    static bool is_hex(char c) noexcept {
        return (c >= '0' && c <= '9') ||
               (c >= 'A' && c <= 'F') ||
               (c >= 'a' && c <= 'f');
    }

    static uint8_t hex_val(char c) noexcept {
        if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
        if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
        if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
        return 0;
    }

    void feed_nmea(uint8_t b) noexcept {
        const char c = static_cast<char>(b);
        switch (nmea_state_) {
        case NmeaState::Idle:
            if (c == '$') {
                nmea_ck_ = 0;
                nmea_state_ = NmeaState::Data;
            }
            break;
        case NmeaState::Data:
            if (c == '$') {
                nmea_ck_ = 0;
            } else if (c == '*') {
                nmea_state_ = NmeaState::Ck1;
            } else if (c == '\r' || c == '\n') {
                nmea_state_ = NmeaState::Idle;
            } else {
                nmea_ck_ ^= b;
            }
            break;
        case NmeaState::Ck1:
            if (!is_hex(c)) {
                nmea_state_ = NmeaState::Idle;
                break;
            }
            nmea_hi_ = static_cast<uint8_t>(hex_val(c) << 4);
            nmea_state_ = NmeaState::Ck2;
            break;
        case NmeaState::Ck2:
            if (is_hex(c) && static_cast<uint8_t>(nmea_hi_ | hex_val(c)) == nmea_ck_) {
                valid_ = true;
            }
            nmea_state_ = NmeaState::Idle;
            break;
        }
    }

    void feed_ubx(uint8_t b) noexcept {
        switch (ubx_state_) {
        case UbxState::Idle:
            if (b == 0xB5u) ubx_state_ = UbxState::Sync2;
            break;
        case UbxState::Sync2:
            ubx_state_ = (b == 0x62u) ? UbxState::Class :
                         (b == 0xB5u) ? UbxState::Sync2 : UbxState::Idle;
            break;
        case UbxState::Class:
            ubx_ck_a_ = b;
            ubx_ck_b_ = b;
            ubx_state_ = UbxState::Id;
            break;
        case UbxState::Id:
            ubx_ck_a_ += b;
            ubx_ck_b_ += ubx_ck_a_;
            ubx_state_ = UbxState::Len1;
            break;
        case UbxState::Len1:
            ubx_len_ = b;
            ubx_ck_a_ += b;
            ubx_ck_b_ += ubx_ck_a_;
            ubx_state_ = UbxState::Len2;
            break;
        case UbxState::Len2:
            ubx_len_ |= static_cast<uint16_t>(b) << 8;
            ubx_ck_a_ += b;
            ubx_ck_b_ += ubx_ck_a_;
            ubx_idx_ = 0;
            if (ubx_len_ > 1024u) {
                ubx_state_ = UbxState::Idle;
            } else {
                ubx_state_ = (ubx_len_ == 0u) ? UbxState::CkA : UbxState::Payload;
            }
            break;
        case UbxState::Payload:
            ubx_ck_a_ += b;
            ubx_ck_b_ += ubx_ck_a_;
            if (++ubx_idx_ >= ubx_len_) ubx_state_ = UbxState::CkA;
            break;
        case UbxState::CkA:
            ubx_state_ = (b == ubx_ck_a_) ? UbxState::CkB : UbxState::Idle;
            break;
        case UbxState::CkB:
            if (b == ubx_ck_b_) valid_ = true;
            ubx_state_ = UbxState::Idle;
            break;
        }
    }
};

} // namespace detail

// -- UART transport ---------------------------------------------------------
class UartTransport {
public:
    static constexpr uint32_t MAX_DMA_BUF_SIZE = 4096u;

    // tx_pin / rx_pin: GPIO numbers.  Pass 0xFF to skip a pin (e.g. RX-only).
    UartTransport(uart_inst_t* uart, uint tx_pin, uint rx_pin,
                  uint32_t baud) noexcept
        : uart_(uart)
    {
        uart_init(uart_, baud);
        uart_set_hw_flow(uart_, false, false);
        uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(uart_, true);
        if (tx_pin != 0xFFu) gpio_set_function(tx_pin, UART_FUNCSEL_NUM(uart_, tx_pin));
        if (rx_pin != 0xFFu) gpio_set_function(rx_pin, UART_FUNCSEL_NUM(uart_, rx_pin));
    }

    void set_baudrate(uint32_t baud) noexcept {
        uart_set_baudrate(uart_, baud);
    }

    void drain_rx() noexcept {
        while (uart_is_readable(uart_))
            (void)uart_getc(uart_);
    }

    void wait_tx_idle() noexcept {
        uart_tx_wait_blocking(uart_);
    }

    bool start_rx(UartRxMode mode,
                  DmaRingBufferSize dma_ring_size = DmaRingBufferSize::Bytes1K) noexcept {
        rx_mode_ = mode;
        dma_ring_size_ = dma_ring_size;
        if (rx_mode_ == UartRxMode::Polling) {
            stop_dma();
            drain_rx();
            return true;
        }
        return start_dma();
    }

    void restart_rx() noexcept {
        if (rx_mode_ == UartRxMode::DmaRing) {
            (void)start_dma();
        } else {
            drain_rx();
        }
    }

    void stop_dma() noexcept {
        if (dma_chan_ >= 0)
            dma_channel_abort(dma_chan_);
    }

    std::size_t read(uint8_t* buf, std::size_t len) noexcept {
        if (rx_mode_ == UartRxMode::DmaRing && dma_chan_ >= 0)
            return dma_read(buf, len);

        std::size_t n = 0;
        while (n < len && uart_is_readable(uart_))
            buf[n++] = static_cast<uint8_t>(uart_getc(uart_));
        return n;
    }

    std::size_t write(const uint8_t* buf, std::size_t len) noexcept {
        uart_write_blocking(uart_, buf, len);
        return len;
    }

    [[nodiscard]] uart_inst_t* uart() const noexcept { return uart_; }
    [[nodiscard]] UartRxMode rx_mode() const noexcept { return rx_mode_; }
    [[nodiscard]] int dma_channel() const noexcept { return dma_chan_; }
    [[nodiscard]] uint32_t dma_ring_size_bytes() const noexcept {
        return dma_ring_buffer_bytes(dma_ring_size_);
    }

    [[nodiscard]] std::size_t rx_available() const noexcept {
        if (rx_mode_ == UartRxMode::DmaRing && dma_chan_ >= 0)
            return dma_available();
        return uart_is_readable(uart_) ? 1u : 0u;
    }

private:
    uart_inst_t* uart_;
    UartRxMode rx_mode_ = UartRxMode::Polling;
    DmaRingBufferSize dma_ring_size_ = DmaRingBufferSize::Bytes1K;
    int dma_chan_ = -1;
    uint32_t dma_read_idx_ = 0;
    alignas(MAX_DMA_BUF_SIZE) uint8_t dma_buf_[MAX_DMA_BUF_SIZE]{};

    bool start_dma() noexcept {
        stop_dma();

        if (dma_chan_ < 0) {
            dma_chan_ = dma_claim_unused_channel(false);
            if (dma_chan_ < 0) {
                rx_mode_ = UartRxMode::Polling;
                return false;
            }
        }

        dma_read_idx_ = 0;
        const uint32_t ring_bytes = dma_ring_size_bytes();
        for (uint32_t i = 0; i < ring_bytes; ++i)
            dma_buf_[i] = 0;

        dma_channel_config cfg = dma_channel_get_default_config(dma_chan_);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_dreq(&cfg, uart_get_dreq(uart_, false));
        channel_config_set_ring(&cfg, true, static_cast<uint8_t>(dma_ring_size_));

        dma_channel_configure(
            dma_chan_,
            &cfg,
            dma_buf_,
            &uart_get_hw(uart_)->dr,
            0xFFFFFFFFu,
            true
        );

        return true;
    }

    [[nodiscard]] std::size_t dma_available() const noexcept {
        const uintptr_t write_ptr =
            static_cast<uintptr_t>(dma_channel_hw_addr(dma_chan_)->write_addr);
        const uintptr_t base = reinterpret_cast<uintptr_t>(dma_buf_);
        const uint32_t mask = dma_ring_size_bytes() - 1u;
        const uint32_t write_idx =
            static_cast<uint32_t>(write_ptr - base) & mask;
        return static_cast<std::size_t>((write_idx - dma_read_idx_) & mask);
    }

    std::size_t dma_read(uint8_t* dst, std::size_t len) noexcept {
        const std::size_t avail = dma_available();
        const std::size_t n = (avail < len) ? avail : len;
        const uint32_t mask = dma_ring_size_bytes() - 1u;

        for (std::size_t i = 0; i < n; ++i) {
            dst[i] = dma_buf_[dma_read_idx_ & mask];
            dma_read_idx_ = (dma_read_idx_ + 1u) & mask;
        }
        return n;
    }
};

class PicoGpsDriver {
public:
    explicit PicoGpsDriver(const PicoGpsConfig& config) noexcept
        : PicoGpsDriver(config.uart, config.tx_pin, config.rx_pin,
                        config.desired_baud, config.rx_mode, config.dma_ring_size)
    {}

    PicoGpsDriver(uart_inst_t* uart, uint tx_pin, uint rx_pin,
                  uint32_t desired_baud,
                  UartRxMode rx_mode = UartRxMode::Polling,
                  DmaRingBufferSize dma_ring_size = DmaRingBufferSize::Bytes1K) noexcept
        : transport_(uart, tx_pin, rx_pin, desired_baud)
        , driver_(transport_)
        , desired_baud_(desired_baud)
        , configured_rx_mode_(rx_mode)
        , configured_dma_ring_size_(dma_ring_size)
    {
        // Arm the configured RX path (DMA ring) *before* autobaud detection so
        // detection can keep up at high bauds — polling the 32-byte UART FIFO
        // overruns above ~230400 and corrupts every frame, breaking detection.
        rx_mode_ok_ = transport_.start_rx(configured_rx_mode_, configured_dma_ring_size_);
        initialized_ = detect_current_baud();
        if (initialized_ && desired_baud_ != 0u && current_baud_ != desired_baud_) {
            baud_change_ok_ = set_baudrate(desired_baud_);
        } else {
            baud_change_ok_ = initialized_;
        }
    }

    [[nodiscard]] bool initialized() const noexcept { return initialized_; }
    [[nodiscard]] bool baud_change_ok() const noexcept { return baud_change_ok_; }
    [[nodiscard]] bool rx_mode_ok() const noexcept { return rx_mode_ok_; }
    [[nodiscard]] uint32_t detected_baud() const noexcept { return detected_baud_; }
    [[nodiscard]] uint32_t current_baud() const noexcept { return current_baud_; }
    [[nodiscard]] uint32_t desired_baud() const noexcept { return desired_baud_; }
    [[nodiscard]] uart_inst_t* uart() const noexcept { return transport_.uart(); }
    [[nodiscard]] UartRxMode rx_mode() const noexcept { return transport_.rx_mode(); }
    [[nodiscard]] int dma_channel() const noexcept { return transport_.dma_channel(); }
    [[nodiscard]] uint32_t dma_ring_size_bytes() const noexcept { return transport_.dma_ring_size_bytes(); }
    [[nodiscard]] std::size_t rx_available() const noexcept { return transport_.rx_available(); }

    bool detect_current_baud(uint32_t timeout_per_baud_ms = 1200u) noexcept {
        for (uint32_t baud : GPS_UART_BAUDS) {
            set_host_baudrate(baud);
            transport_.drain_rx();
            if (listen_for_valid_wire_data(timeout_per_baud_ms)) {
                detected_baud_ = baud;
                current_baud_ = baud;
                initialized_ = true;
                return true;
            }
        }
        return false;
    }

    bool set_baudrate(uint32_t baud, uint32_t verify_timeout_ms = 1500u) noexcept {
        if (baud == 0u) return false;
        if (current_baud_ == baud) return true;

        const uint32_t old_baud = current_baud_;
        send_ubx(Ubx::valset_uart1_baud(baud, ValLayer::RAM));
        transport_.wait_tx_idle();
        sleep_ms(20);

        set_host_baudrate(baud);
        if (listen_for_valid_wire_data(verify_timeout_ms)) {
            current_baud_ = baud;
            baud_change_ok_ = true;
            return true;
        }

        // Generation 8 and older receivers may not support CFG-VALSET. If the
        // old baud is still alive, retry with legacy CFG-PRT as a fallback.
        set_host_baudrate(old_baud);
        if (listen_for_valid_wire_data(verify_timeout_ms)) {
            send_ubx(Ubx::cfg_prt(Port::UART1, baud,
                                  InProto::UBX | InProto::NMEA,
                                  OutProto::UBX | OutProto::NMEA));
            transport_.wait_tx_idle();
            sleep_ms(20);
            set_host_baudrate(baud);
            if (listen_for_valid_wire_data(verify_timeout_ms)) {
                current_baud_ = baud;
                baud_change_ok_ = true;
                return true;
            }
        }

        set_host_baudrate(old_baud);
        current_baud_ = old_baud;
        baud_change_ok_ = false;
        return false;
    }

    void configure(const GpsDriver<UartTransport>::ConfigOptions& opt = {}) noexcept {
        driver_.configure(opt);
    }

    void configure(Port port, InProto in_proto, OutProto out_proto,
                   uint16_t meas_rate_ms = 1000) noexcept
    {
        driver_.configure(port, in_proto, out_proto, meas_rate_ms);
    }

    void send_ubx(const UbxFrame& frame) noexcept { driver_.send_ubx(frame); }
    std::size_t poll() noexcept { const std::size_t n = driver_.poll(); note_freshness(); return n; }
    std::size_t poll_ubx_only() noexcept { const std::size_t n = driver_.poll_ubx_only(); note_freshness(); return n; }
    std::size_t read_raw(uint8_t* buf, std::size_t len) noexcept { return driver_.read_raw(buf, len); }
    void feed(uint8_t b) noexcept { driver_.feed(b); note_freshness(); }
    void feed(const uint8_t* data, std::size_t len) noexcept { driver_.feed(data, len); note_freshness(); }
    void feed_ubx_only(const uint8_t* data, std::size_t len) noexcept { driver_.feed_ubx_only(data, len); note_freshness(); }

    [[nodiscard]] const Coordinate& coordinate() const noexcept { return driver_.coordinate(); }
    [[nodiscard]] bool has_fix() const noexcept { return driver_.has_fix(); }
    [[nodiscard]] const Diagnostics& diagnostics() const noexcept { return driver_.diagnostics(); }
    [[nodiscard]] std::string_view fix_label() const noexcept { return driver_.fix_label(); }

    // -- Freshness / staleness -------------------------------------------------
    // A fix is "fresh" while new valid solutions keep arriving; coordinate()
    // always returns the last-known position, so use these to decide whether it
    // can still be trusted as live before acting on or transmitting it.

    [[nodiscard]] uint32_t fix_seq() const noexcept { return driver_.fix_seq(); }

    // Whether at least one valid fix has been seen since construction.
    [[nodiscard]] bool ever_had_fix() const noexcept { return ever_fix_; }

    // Milliseconds since the last fresh valid solution. UINT32_MAX if no fix yet.
    [[nodiscard]] uint32_t fix_age_ms() const noexcept {
        if (!ever_fix_) return UINT32_MAX;
        const uint64_t dt_ms = (time_us_64() - last_fix_us_) / 1000u;
        return (dt_ms > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(dt_ms);
    }

    // True when there is no fix yet or the last one is older than max_age_ms.
    // Default 2 s ≈ two missed 1 Hz solutions.
    [[nodiscard]] bool is_stale(uint32_t max_age_ms = 2000u) const noexcept {
        return fix_age_ms() > max_age_ms;
    }

private:
    UartTransport transport_;
    GpsDriver<UartTransport> driver_;
    uint32_t desired_baud_ = 0;
    uint32_t detected_baud_ = 0;
    uint32_t current_baud_ = 0;
    bool initialized_ = false;
    bool baud_change_ok_ = false;
    bool rx_mode_ok_ = false;
    UartRxMode configured_rx_mode_ = UartRxMode::Polling;
    DmaRingBufferSize configured_dma_ring_size_ = DmaRingBufferSize::Bytes1K;

    // Freshness tracking — timestamp the wall clock whenever a new valid fix
    // (advancing fix_seq) is committed by a poll/feed call.
    uint32_t last_seen_seq_ = 0;
    uint64_t last_fix_us_   = 0;
    bool     ever_fix_      = false;

    void note_freshness() noexcept {
        const uint32_t seq = driver_.fix_seq();
        if (seq != last_seen_seq_) {
            last_seen_seq_ = seq;
            last_fix_us_   = time_us_64();
            ever_fix_      = true;
        }
    }

    void set_host_baudrate(uint32_t baud) noexcept {
        transport_.set_baudrate(baud);
        current_baud_ = baud;
        transport_.restart_rx();
        driver_.reset_stream();
    }

    bool listen_for_valid_wire_data(uint32_t timeout_ms) noexcept {
        detail::GpsWireDetector detector;
        uint8_t buf[64];
        const uint64_t deadline = time_us_64() + static_cast<uint64_t>(timeout_ms) * 1000u;

        while (time_us_64() < deadline) {
            const std::size_t n = transport_.read(buf, sizeof(buf));
            if (n == 0) {
                sleep_us(1000);
                continue;
            }

            feed(buf, n);
            for (std::size_t i = 0; i < n; ++i) {
                if (detector.feed(buf[i])) return true;
            }
        }
        return detector.valid();
    }
};

// -- I2C transport (u-blox DDC, default address 0x42) ----------------------
class I2cTransport {
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x42;

    I2cTransport(i2c_inst_t* i2c, uint sda_pin, uint scl_pin,
                 uint32_t freq_hz = 400'000,
                 uint8_t  addr    = DEFAULT_ADDR) noexcept
        : i2c_(i2c), addr_(addr)
    {
        i2c_init(i2c_, freq_hz);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
    }

    std::size_t read(uint8_t* buf, std::size_t len) noexcept {
        const int r = i2c_read_blocking(i2c_, addr_, buf, static_cast<uint>(len), false);
        if (r < 0) return 0;
        // u-blox DDC returns 0xFF when no data is available — strip trailing padding
        std::size_t n = static_cast<std::size_t>(r);
        while (n > 0 && buf[n - 1] == 0xFF) --n;
        return n;
    }

    std::size_t write(const uint8_t* buf, std::size_t len) noexcept {
        const int r = i2c_write_blocking(i2c_, addr_, buf, static_cast<uint>(len), false);
        return (r < 0) ? 0u : static_cast<std::size_t>(r);
    }

private:
    i2c_inst_t* i2c_;
    uint8_t     addr_;
};

} // namespace gps

#endif // PICO_SDK_VERSION_MAJOR
