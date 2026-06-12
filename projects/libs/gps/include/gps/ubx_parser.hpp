#pragma once

// gps/ubx_parser.hpp — UBX binary protocol parser
//
// Parses the UBX binary stream from a u-blox module and writes decoded fields
// into a Coordinate reference supplied at construction.
//
// Can be used standalone:
//
//   gps::Coordinate  coord;
//   gps::Diagnostics diag;
//   gps::UbxParser   ubx(coord, diag);
//   while (uart_is_readable(uart0))
//       ubx.feed(uart_getc(uart0));
//   if (coord.valid) { ... }
//
// Or let GpsDriver own it alongside NmeaParser — see gps_driver.hpp.
//
// Messages decoded
// ----------------
//   NAV-PVT       (0x01 0x07)  position, velocity, time, fix status
//   NAV-HPPOSLLH  (0x01 0x14)  high-precision position (~1 cm), merged into PVT
//   NAV-DOP       (0x01 0x04)  hDOP, vDOP
//   NAV-SAT       (0x01 0x35)  per-SV C/N0 and used-in-fix flags (streamed inline)
//   ACK-ACK/NAK   (0x05 0x01/0x00)  command acknowledgement counters
//
// HP merge behaviour
// -------------------
// NAV-HPPOSLLH and NAV-PVT may arrive in either order within a navigation
// epoch.  The parser buffers a pending HP frame and merges it on the next PVT
// (if PVT arrives second) or merges it immediately into an existing valid fix
// (if HP arrives second).  Both orderings produce correct output.

#include <array>
#include <cstdint>
#include <cstring>

#include "types.hpp"

namespace gps {

class UbxParser {
public:
    explicit UbxParser(Coordinate& coord, Diagnostics& diag) noexcept
        : coord_(coord), diag_(diag) {}

    // Feed one byte from the transport stream.
    void feed(uint8_t b) noexcept { parse_byte(b); }

private:
    Coordinate&  coord_;
    Diagnostics& diag_;

    // -- State machine ---------------------------------------------------------
    enum class State : uint8_t {
        Idle,
        Sync2,
        Class, Id,
        Len1, Len2,
        Payload,
        CkA, CkB,
    };
    State state_ = State::Idle;

    // -- Frame fields ---------------------------------------------------------
    uint8_t  cls_        = 0;
    uint8_t  id_         = 0;
    uint16_t length_     = 0;   // payload byte count
    uint16_t pay_idx_    = 0;   // bytes received so far
    uint8_t  ck_a_       = 0;
    uint8_t  ck_b_       = 0;

    // -- Payload buffer --------------------------------------------------------
    // Sized for the largest accepted message (NavPvt = 92 bytes).
    static constexpr uint16_t BUF = 92;
    std::array<uint8_t, BUF> buf_{};

    // -- NAV-SAT streaming accumulators ----------------------------------------
    // NAV-SAT is variable-length and too large to buffer fully.  We extract the
    // two fields we need (cno, svUsed) byte-by-byte as the payload arrives.
    uint8_t sat_best_cno_  = 0;
    uint8_t sat_sv_used_   = 0;

    // -- High-precision pending frame -----------------------------------------
    bool        hp_pending_ = false;
    NavHpPosLlh hp_frame_{};

    // -- Byte pump -------------------------------------------------------------
    void parse_byte(uint8_t b) noexcept {
        switch (state_) {

        case State::Idle:
            if (b == 0xB5u) state_ = State::Sync2;
            break;

        case State::Sync2:
            if (b == 0x62u) {
                state_ = State::Class;
            } else if (b == 0xB5u) {
                // Repeated sync byte; stay armed for 0x62 instead of losing
                // the next frame in streams like B5 62 B5 62 01 07...
                state_ = State::Sync2;
            } else {
                state_ = State::Idle;
            }
            break;

        case State::Class:
            if (b == 0xB5u) {
                state_ = State::Sync2;
                break;
            }
            cls_  = b;
            ck_a_ = b; ck_b_ = b;
            state_ = State::Id;
            break;

        case State::Id:
            id_    = b;
            ck_a_ += b; ck_b_ += ck_a_;
            state_ = State::Len1;
            break;

        case State::Len1:
            length_  = b;
            ck_a_ += b; ck_b_ += ck_a_;
            state_ = State::Len2;
            break;

        case State::Len2:
            length_ |= static_cast<uint16_t>(b) << 8;
            ck_a_ += b; ck_b_ += ck_a_;
            pay_idx_ = 0;
            if (!accepted()) { state_ = State::Idle; break; }
            // Reset NAV-SAT streaming accumulators for this new frame.
            if (cls_ == 0x01u && id_ == 0x35u) {
                sat_best_cno_ = 0;
                sat_sv_used_  = 0;
            }
            state_ = (length_ > 0) ? State::Payload : State::CkA;
            break;

        case State::Payload:
            ck_a_ += b; ck_b_ += ck_a_;
            if (pay_idx_ < BUF) buf_[pay_idx_] = b;
            if (cls_ == 0x01u && id_ == 0x35u) stream_nav_sat_byte(b);
            if (++pay_idx_ >= length_) state_ = State::CkA;
            break;

        case State::CkA:
            state_ = (b == ck_a_) ? State::CkB : State::Idle;
            break;

        case State::CkB:
            if (b == ck_b_) {
                diag_.ubx_frames++;
                dispatch();
            }
            state_ = State::Idle;
            break;
        }
    }

    // -- Message filter --------------------------------------------------------
    // Returns false for unknown/oversized messages so the payload is not buffered.
    [[nodiscard]] bool accepted() const noexcept {
        if (cls_ == 0x05u) return length_ == 2u;            // ACK-ACK / ACK-NAK
        if (cls_ != 0x01u) return false;
        switch (id_) {
        case 0x07u: return length_ == sizeof(NavPvt);        // NAV-PVT
        case 0x14u: return length_ == sizeof(NavHpPosLlh);   // NAV-HPPOSLLH
        case 0x04u: return length_ == 18u;                   // NAV-DOP
        case 0x09u: return length_ == 20u;                   // NAV-ODO
        case 0x35u: return true;                             // NAV-SAT (variable)
        default:    return false;
        }
    }

    // -- Dispatcher -----------------------------------------------------------
    void dispatch() noexcept {
        if (cls_ == 0x05u) {
            if (id_ == 0x01u) diag_.ubx_ack++;
            if (id_ == 0x00u) diag_.ubx_nak++;
            return;
        }
        if (cls_ != 0x01u) return;
        switch (id_) {
        case 0x07u: diag_.ubx_pvt++; decode_nav_pvt();      break;
        case 0x14u: diag_.ubx_hp++;  decode_nav_hpposllh(); break;
        case 0x04u: diag_.ubx_dop++; decode_nav_dop();      break;
        case 0x09u: diag_.ubx_odo++;                         break;
        case 0x35u:                   decode_nav_sat();       break;
        }
    }

    // -- Little-endian payload readers ----------------------------------------
    [[nodiscard]] uint16_t u16(int off) const noexcept {
        return static_cast<uint16_t>(buf_[off]) |
               static_cast<uint16_t>(static_cast<uint16_t>(buf_[off+1]) << 8);
    }
    [[nodiscard]] uint32_t u32(int off) const noexcept {
        return static_cast<uint32_t>(buf_[off+0])        |
               static_cast<uint32_t>(buf_[off+1]) <<  8  |
               static_cast<uint32_t>(buf_[off+2]) << 16  |
               static_cast<uint32_t>(buf_[off+3]) << 24;
    }

    // -- NAV-PVT --------------------------------------------------------------
    void decode_nav_pvt() noexcept {
        NavPvt pvt;
        std::memcpy(&pvt, buf_.data(), sizeof(NavPvt));

        coord_.gps_tow_ms = pvt.iTOW;

        // UTC date (validDate bit)
        if ((pvt.valid & 0x01u) && pvt.year > 0) {
            coord_.utc_year  = pvt.year;
            coord_.utc_month = pvt.month;
            coord_.utc_day   = pvt.day;
        }
        // UTC time (validTime bit) — clamp negative nano to 0
        if (pvt.valid & 0x02u) {
            const int32_t  nano  = (pvt.nano < 0) ? 0 : pvt.nano;
            const uint32_t ms    = static_cast<uint32_t>(nano / 1'000'000);
            coord_.utc_ms = (static_cast<uint32_t>(pvt.hour) * 3600u +
                             static_cast<uint32_t>(pvt.min)  *   60u +
                             static_cast<uint32_t>(pvt.sec)) * 1000u + ms;
        }

        coord_.fix_type   = static_cast<FixType>(pvt.fixType);
        coord_.carr_soln  = static_cast<CarrierSolution>((pvt.flags >> 5u) & 0x03u);
        coord_.h_acc_mm   = pvt.hAcc;
        coord_.v_acc_mm   = pvt.vAcc;
        coord_.pdop       = static_cast<float>(pvt.pDOP) * 0.01f;
        coord_.satellites = pvt.numSV;

        if (!(pvt.flags & 0x01u)) {   // gnssFixOK clear — no valid fix
            coord_.valid = false;
            return;
        }

        // Position: merge high-precision frame if one arrived before this PVT
        if (hp_pending_) {
            if (!(hp_frame_.flags & 0x02u)) {   // invalidLlh bit clear
                coord_.latitude  = (hp_frame_.lat  * 1e-7) + (hp_frame_.latHp  * 1e-9);
                coord_.longitude = (hp_frame_.lon  * 1e-7) + (hp_frame_.lonHp  * 1e-9);
                coord_.altitude  = static_cast<float>((hp_frame_.hMSL + hp_frame_.hMSLHp * 0.1) * 1e-3);
                coord_.hp_valid  = true;
            }
            hp_pending_ = false;
        } else {
            coord_.latitude  = pvt.lat  * 1e-7;
            coord_.longitude = pvt.lon  * 1e-7;
            coord_.altitude  = static_cast<float>(pvt.hMSL * 0.001);
            coord_.hp_valid  = false;
        }

        coord_.vel_north_mms = pvt.velN;
        coord_.vel_east_mms  = pvt.velE;
        coord_.vel_down_mms  = pvt.velD;
        coord_.speed_mps     = static_cast<float>(pvt.gSpeed)  * 0.001f;
        coord_.course_deg    = static_cast<float>(pvt.headMot) * 1e-5f;
        coord_.valid         = true;
    }

    // -- NAV-HPPOSLLH ---------------------------------------------------------
    void decode_nav_hpposllh() noexcept {
        std::memcpy(&hp_frame_, buf_.data(), sizeof(NavHpPosLlh));

        if (hp_frame_.flags & 0x02u) return;   // invalidLlh — discard

        hp_pending_ = true;

        // If a valid fix already exists (PVT arrived first in this epoch),
        // apply the HP correction immediately without waiting for the next PVT.
        if (coord_.valid) {
            coord_.latitude  = (hp_frame_.lat  * 1e-7) + (hp_frame_.latHp  * 1e-9);
            coord_.longitude = (hp_frame_.lon  * 1e-7) + (hp_frame_.lonHp  * 1e-9);
            coord_.altitude  = static_cast<float>((hp_frame_.hMSL + hp_frame_.hMSLHp * 0.1) * 1e-3);
            coord_.hp_valid  = true;
            hp_pending_      = false;
        }
    }

    // -- NAV-DOP --------------------------------------------------------------
    // Payload layout: iTOW(4) gDOP(2) pDOP(2) tDOP(2) vDOP(2) hDOP(2) nDOP(2) eDOP(2)
    void decode_nav_dop() noexcept {
        coord_.hdop = u16(12) * 0.01f;   // hDOP at byte offset 12
        coord_.vdop = u16(10) * 0.01f;   // vDOP at byte offset 10
    }

    // -- NAV-SAT (inline streaming) --------------------------------------------
    // Header: iTOW(4) version(1) numSvs(1) reserved(2) = 8 bytes
    // Then numSvs × 12-byte records:
    //   [0] gnssId  [1] svId  [2] cno (dBHz)  [3] elev  [4-5] azim
    //   [6-7] prRes  [8-11] flags   (flags bit3 = svUsed)
    void stream_nav_sat_byte(uint8_t b) noexcept {
        if (pay_idx_ < 8u) return;
        const uint8_t byte_in_rec = static_cast<uint8_t>((pay_idx_ - 8u) % 12u);
        if (byte_in_rec == 2u) {
            if (b > sat_best_cno_) sat_best_cno_ = b;
        } else if (byte_in_rec == 8u) {
            if (b & 0x08u) sat_sv_used_++;
        }
    }

    void decode_nav_sat() noexcept {
        coord_.best_cno    = sat_best_cno_;
        coord_.num_sv_used = sat_sv_used_;
    }
};

} // namespace gps
