#!/usr/bin/env python3
"""
sim_replay.py — Replay an OpenRocket CSV as SIGMA INTER_PICO UDP packets.

Sends SIGMA-framed Wire::InterPico packets to the primary Pico's UDP port 5005,
matching the exact path used by the secondary Pico during flight.  The ground
station's udp_recv_task decodes these and drives antenna tracking normally.

Usage:
    python sim_replay.py OpenRocket/2026/IREC2026SIM.csv --pico-ip 192.168.x.x
    python sim_replay.py <csv> --pico-ip <ip> --speed 2.0 --loop

Arguments:
    csv          Path to OpenRocket export CSV
    --pico-ip    Primary Pico WiFi IP (required)
    --port       UDP port (default 5005)
    --speed      Playback speed multiplier (default 1.0 = real time)
    --loop       Replay indefinitely
    --dry-run    Parse and print rows without sending UDP packets

Column order expected (standard OpenRocket export):
    Time (s), Altitude (m), Altitude above sea level (m),
    Vertical velocity (m/s), Total velocity (m/s),
    Vertical acceleration (m/s²), Total acceleration (m/s²),
    Latitude (° N), Longitude (° E),
    Roll rate (°/s), Pitch rate (°/s), Yaw rate (°/s),
    ...
"""

import argparse
import math
import socket
import struct
import sys
import time

# ── SIGMA protocol constants ──────────────────────────────────────────────────

FRAME_START     = bytes([0xAA, 0x55])
FRAME_END       = bytes([0xBB, 0x66])
PKT_INTER_PICO  = 0x03
FLAG_GPS_VALID  = 0x01
INTER_PICO_PORT = 5005

# FlightState enum (SIGMA.hpp)
FS_GROUND_IDLE    = 0
FS_POWERED_ASCENT = 2
FS_COAST_ASCENT   = 3
FS_APOGEE         = 4
FS_DESCENT_DROGUE = 5
FS_DESCENT_MAIN   = 6
FS_LANDED         = 7

# Wire::InterPico struct (40 bytes, all little-endian):
#   uint32 boot_ms, int32 lat_e7, int32 lon_e7,
#   int32 alt_baro_dm, int32 alt_gps_cm,
#   int16 q[4], uint16 speed_cms,
#   uint8 state, uint8 sats, uint8 flags, uint8 pad[3],
#   int8 rssi, int8 snr_q2, uint8 pad[2]
INTERPICO_FMT  = "<IiiiihhhhHBBBBBBbbBB"
INTERPICO_SIZE = struct.calcsize(INTERPICO_FMT)
assert INTERPICO_SIZE == 40, f"InterPico size mismatch: {INTERPICO_SIZE}"

# Simulated radio values — fake but plausible
SIM_RSSI   = -75   # dBm
SIM_SNR_Q2 =  20   # 5 dB × 4
SIM_SATS   =  12

# ── CRC-16/CCITT (poly 0x1021, init 0xFFFF) ──────────────────────────────────

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(seq: int, payload: bytes) -> bytes:
    plen   = len(payload)
    header = bytes([PKT_INTER_PICO, seq & 0xFF, plen & 0xFF, (plen >> 8) & 0xFF])
    c      = crc16(header + payload)
    return FRAME_START + header + payload + bytes([c & 0xFF, (c >> 8) & 0xFF]) + FRAME_END


# ── Quaternion helpers ────────────────────────────────────────────────────────

def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)

def _qnorm(q):
    n = math.sqrt(sum(x*x for x in q))
    return tuple(x/n for x in q) if n > 1e-10 else (1.0, 0.0, 0.0, 0.0)

def quat_integrate(q, roll_dps, pitch_dps, yaw_dps, dt):
    """First-order quaternion integration from body-frame angular rates."""
    wx = math.radians(roll_dps)
    wy = math.radians(pitch_dps)
    wz = math.radians(yaw_dps)
    dq = _qmul(q, (0.0, wx, wy, wz))
    half_dt = 0.5 * dt
    return _qnorm(tuple(q[i] + dq[i] * half_dt for i in range(4)))

def quat_to_wire(q):
    """Float [w,x,y,z] → int16 Q1.15."""
    return [max(-32767, min(32767, int(v * 32767.0))) for v in q]


# ── OpenRocket CSV parser ─────────────────────────────────────────────────────

def parse_csv(path: str):
    """Return list of float rows, skipping all comment/event lines."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 9:
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue
    return rows


# ── Flight state from simulation event times ──────────────────────────────────

def flight_state_at(t: float) -> int:
    if t < 0.05:    return FS_GROUND_IDLE
    if t < 3.616:   return FS_POWERED_ASCENT
    if t < 25.156:  return FS_COAST_ASCENT
    if t < 25.157:  return FS_APOGEE
    if t < 145.81:  return FS_DESCENT_DROGUE
    if t < 193.593: return FS_DESCENT_MAIN
    return FS_LANDED

FS_NAMES = {
    FS_GROUND_IDLE: "GROUND_IDLE",
    FS_POWERED_ASCENT: "POWERED_ASCENT",
    FS_COAST_ASCENT: "COAST_ASCENT",
    FS_APOGEE: "APOGEE",
    FS_DESCENT_DROGUE: "DESCENT_DROGUE",
    FS_DESCENT_MAIN: "DESCENT_MAIN",
    FS_LANDED: "LANDED",
}

# ── Packet builder ────────────────────────────────────────────────────────────

def pack_inter_pico(boot_ms, lat, lon, alt_m, speed_ms, q_float, state):
    q_w16 = quat_to_wire(q_float)
    payload = struct.pack(INTERPICO_FMT,
        boot_ms & 0xFFFFFFFF,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt_m * 10),    # alt_baro_dm
        int(alt_m * 100),   # alt_gps_cm
        q_w16[0], q_w16[1], q_w16[2], q_w16[3],
        max(0, min(65535, int(speed_ms * 100))),  # speed_cms
        state & 0xFF, SIM_SATS, FLAG_GPS_VALID,
        0, 0, 0,             # _pad0[3]
        SIM_RSSI,            # int8 rssi (-128..127)
        SIM_SNR_Q2,          # int8 snr_q2
        0, 0,                # _pad1[2]
    )
    assert len(payload) == INTERPICO_SIZE
    return payload


# ── Initial quaternion: nose pointing straight up ─────────────────────────────
#
# Body frame convention (matches ICM-42688-P / Fusion AHRS):
#   +X = nose (roll axis), +Y = right, +Z = down
# At launch on the pad, nose points up = NED -Z direction.
# Rotation from NED to body-nose-up: rotate 180° around Y.
# q = [cos(90°), 0, sin(90°), 0] = [0, 0, 1, 0]  (w,x,y,z)
#
# This is approximate; the quaternion is logged by the laptop but does NOT
# drive antenna pointing — only lat/lon/alt matter for step_ctrl.
INITIAL_QUAT = (0.0, 0.0, 1.0, 0.0)


# ── Main replay loop ──────────────────────────────────────────────────────────

def replay(rows, sock, dest, speed, dry_run):
    q        = INITIAL_QUAT
    seq      = 0
    n_sent   = 0
    prev_t   = None
    wall_t0  = None
    sim_t0   = rows[0][0]
    last_state = -1

    print(f"  Rows: {len(rows)}  Duration: {rows[-1][0]:.1f}s  "
          f"Speed: {speed}x  Dest: {dest[0]}:{dest[1]}")
    print()

    for i, row in enumerate(rows):
        t            = row[0]   # time (s)
        alt_asl      = row[2]   # altitude ASL (m)
        vel_v        = row[3]   # vertical velocity (m/s), positive = up
        vel_total    = row[4]   # total speed (m/s)
        lat          = row[7]
        lon          = row[8]
        roll_rate    = row[9]   # deg/s
        pitch_rate   = row[10]
        yaw_rate     = row[11]

        dt = (t - prev_t) if prev_t is not None else 0.0
        prev_t = t

        # Integrate quaternion
        if dt > 0:
            q = quat_integrate(q, roll_rate, pitch_rate, yaw_rate, dt)

        state = flight_state_at(t)
        if state != last_state:
            print(f"  t={t:7.3f}s  → {FS_NAMES.get(state, str(state))}")
            last_state = state

        boot_ms = int(t * 1000)
        payload = pack_inter_pico(boot_ms, lat, lon, alt_asl, vel_total, q, state)
        frame   = build_frame(seq, payload)
        seq     = (seq + 1) & 0xFF

        if not dry_run:
            sock.sendto(frame, dest)
        n_sent += 1

        # Real-time pacing
        sim_elapsed  = t - sim_t0
        if wall_t0 is None:
            wall_t0 = time.monotonic()
        target_wall  = wall_t0 + sim_elapsed / speed
        sleep_needed = target_wall - time.monotonic()
        if sleep_needed > 0:
            time.sleep(sleep_needed)

    print(f"\n  Replay complete. {n_sent} packets sent.")


def main():
    ap = argparse.ArgumentParser(description="Replay OpenRocket CSV as SIGMA UDP packets")
    ap.add_argument("csv",       help="OpenRocket CSV file")
    ap.add_argument("--pico-ip", required=True, help="Primary Pico IP address")
    ap.add_argument("--port",    type=int, default=INTER_PICO_PORT, help="UDP port (default 5005)")
    ap.add_argument("--speed",   type=float, default=1.0, help="Playback speed multiplier")
    ap.add_argument("--loop",    action="store_true", help="Replay indefinitely")
    ap.add_argument("--dry-run", action="store_true", help="Parse only, no UDP output")
    args = ap.parse_args()

    print(f"Loading {args.csv}...")
    rows = parse_csv(args.csv)
    if not rows:
        print("ERROR: no data rows found", file=sys.stderr)
        sys.exit(1)
    print(f"Loaded {len(rows)} rows, t=[{rows[0][0]:.3f}..{rows[-1][0]:.3f}]s")

    dest = (args.pico_ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    run = 0
    try:
        while True:
            run += 1
            print(f"\n── Run {run} ──────────────────────────────────────────")
            replay(rows, sock, dest, args.speed, args.dry_run)
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
