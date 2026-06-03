#pragma once

// gps/gps_driver.hpp — u-blox GPS driver (C++23)
//
// Combines NmeaParser + UbxParser into a single class that owns a Transport
// and a unified Coordinate.  Automatically selects the right parser for each
// byte — NMEA ('$') and UBX (0xB5 0x62) can be interleaved on the same stream.
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
// Using parsers standalone (without a transport)
// ------------------------------------------------
//   gps::Coordinate  coord;
//   gps::Diagnostics diag;
//   gps::UbxParser   ubx(coord, diag);   // #include "gps/ubx_parser.hpp"
//   gps::NmeaParser  nmea(coord, diag);  // #include "gps/nmea_parser.hpp"
//
//   while (uart_is_readable(uart0)) {
//       uint8_t b = uart_getc(uart0);
//       ubx.feed(b);    // or nmea.feed(b), or both
//   }
//
// Transport concept (any struct with read/write satisfies it)
// ------------------------------------------------------------
//   struct MyTransport {
//       std::size_t read (uint8_t* buf, std::size_t len);  // returns bytes read
//       std::size_t write(const uint8_t* buf, std::size_t len);
//   };

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "types.hpp"
#include "nmea_parser.hpp"
#include "ubx_parser.hpp"
#include "ubx_commands.hpp"

namespace gps {

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
    // Reads up to READ_CHUNK bytes and routes each one to the correct parser.
    // NMEA bytes (stream starting with '$') → NmeaParser
    // UBX bytes  (stream starting with 0xB5) → UbxParser
    // Unknown bytes are fed to both (they self-synchronise).
    static constexpr std::size_t READ_CHUNK = 128;

    std::size_t poll() noexcept {
        uint8_t buf[READ_CHUNK];
        const std::size_t n = transport_.read(buf, sizeof(buf));
        for (std::size_t i = 0; i < n; ++i)
            feed(buf[i]);
        return n;
    }

    // Poll transport and route bytes to the UBX parser only.
    // Use this when the module is configured for UBX output only.
    std::size_t poll_ubx_only() noexcept {
        uint8_t buf[READ_CHUNK];
        const std::size_t n = transport_.read(buf, sizeof(buf));
        for (std::size_t i = 0; i < n; ++i)
            ubx_.feed(buf[i]);
        return n;
    }

    // Read raw bytes from the transport into caller-supplied buffer.
    // Returns the number of bytes read.  Does not feed any parser.
    std::size_t read_raw(uint8_t* buf, std::size_t len) noexcept {
        return transport_.read(buf, len);
    }

    // Feed a single byte — route to both parsers; each discards what it doesn't own.
    void feed(uint8_t b) noexcept {
        nmea_.feed(b);
        ubx_.feed(b);
    }

    // -- Accessors -------------------------------------------------------------
    [[nodiscard]] const Coordinate&  coordinate()   const noexcept { return coord_; }
    [[nodiscard]] bool               has_fix()       const noexcept { return coord_.valid; }
    [[nodiscard]] const Diagnostics& diagnostics()   const noexcept { return diag_; }

    [[nodiscard]] std::string_view   fix_label()     const noexcept {
        return gps::fix_label(coord_);
    }

    // Direct parser access — use if you need to feed bytes yourself without poll()
    [[nodiscard]] NmeaParser& nmea_parser() noexcept { return nmea_; }
    [[nodiscard]] UbxParser&  ubx_parser()  noexcept { return ubx_;  }

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
};

// ===============================================================================
// Platform transport wrappers
// ===============================================================================

#if defined(PICO_SDK_VERSION_MAJOR)

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// -- UART transport ---------------------------------------------------------
class UartTransport {
public:
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

    std::size_t read(uint8_t* buf, std::size_t len) noexcept {
        std::size_t n = 0;
        while (n < len && uart_is_readable(uart_))
            buf[n++] = static_cast<uint8_t>(uart_getc(uart_));
        return n;
    }

    std::size_t write(const uint8_t* buf, std::size_t len) noexcept {
        uart_write_blocking(uart_, buf, len);
        return len;
    }

private:
    uart_inst_t* uart_;
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

#endif // PICO_SDK_VERSION_MAJOR

} // namespace gps
