"""rf_log.py — loader for the RF bench-test logs.

Parses the common CSV schema emitted by the lora915_* / rfm69_433_* Pico tools
(see projects/pico_examples/radio_examples/common/rf_csv.h) into a DataFrame.
As a convenience it
also understands the *old* human-readable log format (the "[rx] PACKET ... RSSI
... SNR ..." lines) so historical captures still load.

The loader is tolerant of the junk a serial terminal interleaves:
  - "# ..." comment / status lines
  - "---- Opened the serial port ... ----" terminal banners
  - repeated CSV header lines (when several runs are concatenated)
  - partial / truncated lines at the start or end of a capture

Returned DataFrame columns (always present, NaN/empty where N/A):
  timestamp_ms, role, freq_mhz, modulation, event, seq, len_bytes,
  rssi_dbm, snr_db, ferr_hz, good, lost, crc, per_pct, air_ms,
  gps_lat, gps_lon, gps_alt_m, utc, source

`utc` is the GPS UTC timestamp ("YYYY-MM-DDTHH:MM:SSZ", empty until the GPS
resolves time). Older logs without the column load fine — it comes through NaN.

`source` is the input file name, so several logs can be concatenated and still
told apart.
"""

from __future__ import annotations

import io
import re
from pathlib import Path

import pandas as pd

CSV_COLUMNS = [
    "timestamp_ms", "role", "freq_mhz", "modulation", "event", "seq",
    "len_bytes", "rssi_dbm", "snr_db", "ferr_hz", "good", "lost", "crc",
    "per_pct", "air_ms", "gps_lat", "gps_lon", "gps_alt_m", "utc",
]

NUMERIC_COLUMNS = [
    "timestamp_ms", "freq_mhz", "seq", "len_bytes", "rssi_dbm", "snr_db",
    "ferr_hz", "good", "lost", "crc", "per_pct", "air_ms",
    "gps_lat", "gps_lon", "gps_alt_m",
]

_HEADER_FIRST = "timestamp_ms,role,freq_mhz"

# -- Legacy (human-readable) log patterns --------------------------------------
_RE_PKT = re.compile(
    r"\[rx\]\s+PACKET\s+(\d+)\s*B\s+RSSI\s+(-?[\d.]+)\s*dBm"
    r"(?:\s+SNR\s+(-?[\d.]+)\s*dB)?(?:\s+ferr\s+(-?[\d.]+)\s*Hz)?"
)
_RE_CRC = re.compile(
    r"\[rx\]\s+CRC ERROR\s*\((\d+)\s*B\)\s+RSSI\s+(-?[\d.]+)\s*dBm"
    r"(?:\s+SNR\s+(-?[\d.]+)\s*dB)?(?:\s+ferr\s+(-?[\d.]+)\s*Hz)?"
)
_RE_FLOOR = re.compile(r"\[rx\]\s+noise floor\s+(-?[\d.]+)\s*dBm")
_RE_TEXT = re.compile(r'text:\s*"[^"]*#(\d+)"')
_RE_TX = re.compile(r"\[tx\]\s+sent\s+#(\d+)\s*\((\d+)\s*B\)\s+in\s+(\d+)\s*ms")


def _looks_like_csv(text: str) -> bool:
    return _HEADER_FIRST in text


def _load_csv(text: str, source: str) -> pd.DataFrame:
    # Drop terminal banners and comment lines; keep header + data rows.
    keep = []
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#") or s.startswith("----") or s.startswith("****"):
            continue
        keep.append(line)
    if not keep:
        return pd.DataFrame(columns=CSV_COLUMNS + ["source"])

    df = pd.read_csv(io.StringIO("\n".join(keep)), header=None, names=CSV_COLUMNS,
                     on_bad_lines="skip", dtype=str, engine="python")
    # Remove any repeated header rows that slipped in as data.
    df = df[df["timestamp_ms"] != "timestamp_ms"]
    df["source"] = source
    return df


def _load_legacy(text: str, source: str) -> pd.DataFrame:
    """Parse the old human-readable format. No real timestamps exist there, so
    we synthesize a monotonic index in milliseconds (1 ms per event) purely to
    keep ordering; treat legacy timestamps as relative, not absolute."""
    rows = []
    # The only legacy (pre-CSV) logs are the 915 MHz LoRa captures, but detect
    # robustly: LoRa frames carry an SNR field, FSK frames never do. Filename
    # hints ("433"/"915") win if present.
    low = source.lower()
    if "433" in source or "rfm69" in low or "gfsk" in text.lower():
        freq, mod = 433.0, "gfsk"
    elif "915" in source or "SX1276" in text or "SNR" in text:
        freq, mod = 915.0, "lora"
    else:
        freq, mod = 915.0, "lora"   # legacy default
    pending_seq = None  # seq parsed from the following text: line
    i = 0
    for line in text.splitlines():
        m = _RE_PKT.search(line)
        if m:
            length, rssi, snr, ferr = m.groups()
            rows.append(dict(timestamp_ms=i, role="rx", freq_mhz=freq,
                             modulation=mod, event="packet", seq=None,
                             len_bytes=length, rssi_dbm=rssi, snr_db=snr,
                             ferr_hz=ferr))
            i += 1
            continue
        m = _RE_TEXT.search(line)
        if m and rows and rows[-1]["event"] == "packet" and rows[-1]["seq"] is None:
            rows[-1]["seq"] = m.group(1)
            continue
        m = _RE_CRC.search(line)
        if m:
            length, rssi, snr, ferr = m.groups()
            rows.append(dict(timestamp_ms=i, role="rx", freq_mhz=freq,
                             modulation=mod, event="crc_error", seq=None,
                             len_bytes=length, rssi_dbm=rssi, snr_db=snr,
                             ferr_hz=ferr))
            i += 1
            continue
        m = _RE_FLOOR.search(line)
        if m:
            rows.append(dict(timestamp_ms=i, role="rx", freq_mhz=freq,
                             modulation=mod, event="noise_floor",
                             rssi_dbm=m.group(1)))
            i += 1
            continue
        m = _RE_TX.search(line)
        if m:
            seq, length, air = m.groups()
            rows.append(dict(timestamp_ms=i, role="tx", freq_mhz=freq,
                             modulation=mod, event="tx_ok", seq=seq,
                             len_bytes=length, air_ms=air))
            i += 1
            continue

    df = pd.DataFrame(rows)
    for c in CSV_COLUMNS:
        if c not in df.columns:
            df[c] = None
    df = df[CSV_COLUMNS]
    df["source"] = source
    return df


def load(path: str | Path) -> pd.DataFrame:
    """Load one log file (CSV or legacy) into a normalized DataFrame."""
    path = Path(path)
    text = path.read_text(errors="replace")
    if _looks_like_csv(text):
        df = _load_csv(text, path.name)
    else:
        df = _load_legacy(text, path.name)

    for c in NUMERIC_COLUMNS:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df.reset_index(drop=True)


def load_many(paths) -> pd.DataFrame:
    """Load and concatenate several logs (each tagged by `source`)."""
    frames = [load(p) for p in paths]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def seq_stats(df: pd.DataFrame) -> dict:
    """Recompute loss/PER from the "#N" sequence numbers of good RX packets.

    More robust than the firmware's running counter because it ignores TX
    restarts within a capture (expected = max-min+1 over the seqs seen)."""
    good = df[(df["role"] == "rx") & (df["event"] == "packet")].dropna(subset=["seq"])
    if good.empty:
        return dict(received=0, expected=0, lost=0, per=float("nan"))
    seqs = good["seq"].astype(int)
    lo, hi = int(seqs.min()), int(seqs.max())
    expected = hi - lo + 1
    received = int(seqs.nunique())
    lost = max(expected - received, 0)
    per = 100.0 * lost / expected if expected else float("nan")
    return dict(received=received, expected=expected, lost=lost, per=per)


def summary_row(df: pd.DataFrame, name: str) -> dict:
    """One flat dict of link metrics for a single capture (for summary CSVs)."""
    rx = df[df["role"] == "rx"]
    tx = df[df["role"] == "tx"]
    pkts = rx[rx["event"] == "packet"]
    rssi = pkts["rssi_dbm"].dropna()
    snr = pkts["snr_db"].dropna()
    floor = rx[rx["event"] == "noise_floor"]["rssi_dbm"].dropna()
    ts = df["timestamp_ms"].dropna()
    s = seq_stats(df)

    def stat(series, fn):
        return round(float(fn(series)), 2) if not series.empty else ""

    return {
        "file": name,
        "freq_mhz": (sorted(df["freq_mhz"].dropna().unique()) or [""])[0],
        "modulation": (sorted(df["modulation"].dropna().unique()) or [""])[0],
        "duration_s": round((ts.max() - ts.min()) / 1000.0, 1) if len(ts) > 1 else 0.0,
        "rx_packets": len(pkts),
        "crc_errors": int((rx["event"] == "crc_error").sum()),
        "read_errors": int((rx["event"] == "read_error").sum()),
        "expected": s["expected"],
        "received": s["received"],
        "lost": s["lost"],
        "per_pct": round(s["per"], 2) if s["per"] == s["per"] else "",  # NaN check
        "rssi_mean": stat(rssi, pd.Series.mean),
        "rssi_median": stat(rssi, pd.Series.median),
        "rssi_min": stat(rssi, pd.Series.min),
        "rssi_max": stat(rssi, pd.Series.max),
        "snr_mean": stat(snr, pd.Series.mean),
        "snr_median": stat(snr, pd.Series.median),
        "noise_floor_median": stat(floor, pd.Series.median),
        "tx_sent": int((tx["event"] == "tx_ok").sum()),
        "tx_fail": int((tx["event"] == "tx_fail").sum()),
        "air_ms_mean": stat(tx["air_ms"].dropna(), pd.Series.mean),
    }
