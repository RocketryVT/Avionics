#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pandas", "matplotlib"]
# ///
"""run_radio_tests.py — batch-compare RF captures organized into test folders.

Layout expected under scripts/radio_tests/ (one subfolder per test, each holding
the captures you want to compare against each other):

    radio_tests/
      test1/
        cots.csv
        diy.csv
      test2/
        antenna_a.csv
        antenna_b.csv
        ...

For every test folder this script:
  * loads all capture files in it (CSV from the Pico tools, or legacy .txt logs),
  * writes  <test>/summary.csv      — one metrics row per capture,
  * writes comparison plots into <test>/:
        rssi_over_time.png   — signal strength over time (the main comparison)
        rssi_over_seq.png    — RSSI vs packet sequence number
        snr_over_seq.png     — SNR vs seq (LoRa captures only)
        rssi_over_distance.png — RSSI vs TX/RX GPS distance when available
        per_bar.png          — packet-error-rate per capture
        noise_floor.png      — channel noise floor over time

Usage:
    python run_radio_tests.py                 # process scripts/radio_tests/*
    python run_radio_tests.py path/to/tests   # custom root
    python run_radio_tests.py --test test1    # just one test folder
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

import rf_log

# Files we generate — never treat them as input captures.
_OUTPUT_NAMES = {"summary.csv"}
_OUTPUT_SUFFIXES = {".png"}
_CAPTURE_GLOBS = ("*.csv", "*.txt", "*.log")

DEFAULT_ROOT = Path(__file__).resolve().parent / "radio_tests"


def find_captures(test_dir: Path) -> list[Path]:
    files: set[Path] = set()
    for pat in _CAPTURE_GLOBS:
        files.update(test_dir.glob(pat))
    return sorted(
        f for f in files
        if f.name not in _OUTPUT_NAMES and f.suffix.lower() not in _OUTPUT_SUFFIXES
    )


def _rel_seconds(df: pd.DataFrame) -> pd.Series:
    """Timestamp in seconds relative to each capture's first event."""
    ts = df["timestamp_ms"]
    base = ts.dropna().min()
    return (ts - base) / 1000.0


def process_test(test_dir: Path, plt) -> bool:
    captures = find_captures(test_dir)
    if not captures:
        print(f"  (no capture files in {test_dir.name}, skipping)")
        return False

    loaded: list[tuple[str, pd.DataFrame]] = []
    rows = []
    for cap in captures:
        df = rf_log.load(cap)
        loaded.append((cap.stem, df))
        rows.append(rf_log.summary_row(df, cap.name))

    # -- summary.csv -----------------------------------------------------------
    summary = pd.DataFrame(rows)
    summary_path = test_dir / "summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"  {len(captures)} capture(s) -> {summary_path.name}")
    for _, r in summary.iterrows():
        per = r["per_pct"]
        print(f"    {r['file']:<24} {r['freq_mhz']} MHz {r['modulation']:<4} "
              f"pkts={r['rx_packets']:<5} PER={per}%  "
              f"RSSI med={r['rssi_median']} dBm")

    if plt is None:
        return True

    # -- plots -----------------------------------------------------------------
    _plot_over_time(loaded, test_dir, plt)
    _plot_over_seq(loaded, test_dir, plt)
    _plot_snr(loaded, test_dir, plt)
    _plot_distance(loaded, test_dir, plt)
    _plot_per(summary, test_dir, plt)
    _plot_noise_floor(loaded, test_dir, plt)
    return True


def _packets(df: pd.DataFrame) -> pd.DataFrame:
    return df[(df["role"] == "rx") & (df["event"] == "packet")]


def _plot_over_time(loaded, test_dir: Path, plt) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    plotted = False
    for name, df in loaded:
        pkts = _packets(df)
        if pkts.empty or pkts["rssi_dbm"].dropna().empty:
            continue
        ax.plot(_rel_seconds(pkts), pkts["rssi_dbm"], ".", ms=4, alpha=0.7, label=name)
        plotted = True
    if not plotted:
        plt.close(fig)
        return
    ax.set_xlabel("time since capture start (s)")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title(f"{test_dir.name} — signal strength over time")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(test_dir / "rssi_over_time.png", dpi=110)
    plt.close(fig)


def _plot_over_seq(loaded, test_dir: Path, plt) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    plotted = False
    for name, df in loaded:
        pkts = _packets(df)
        if pkts.empty or pkts["rssi_dbm"].dropna().empty:
            continue
        x = pkts["seq"] if not pkts["seq"].dropna().empty else range(len(pkts))
        ax.plot(x, pkts["rssi_dbm"], ".", ms=4, alpha=0.7, label=name)
        plotted = True
    if not plotted:
        plt.close(fig)
        return
    ax.set_xlabel("packet seq")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title(f"{test_dir.name} — RSSI vs sequence")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(test_dir / "rssi_over_seq.png", dpi=110)
    plt.close(fig)


def _plot_snr(loaded, test_dir: Path, plt) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    plotted = False
    for name, df in loaded:
        pkts = _packets(df)
        if pkts.empty or pkts["snr_db"].dropna().empty:
            continue
        x = pkts["seq"] if not pkts["seq"].dropna().empty else range(len(pkts))
        ax.plot(x, pkts["snr_db"], ".", ms=4, alpha=0.7, label=name)
        plotted = True
    if not plotted:
        plt.close(fig)  # FSK-only test, nothing to plot
        return
    ax.set_xlabel("packet seq")
    ax.set_ylabel("SNR (dB)")
    ax.set_title(f"{test_dir.name} — SNR vs sequence (LoRa)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(test_dir / "snr_over_seq.png", dpi=110)
    plt.close(fig)


def _plot_distance(loaded, test_dir: Path, plt) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    plotted = False
    for name, df in loaded:
        pkts = _packets(df)
        if ("distance_m" not in pkts or pkts.empty or
                pkts["distance_m"].dropna().empty or
                pkts["rssi_dbm"].dropna().empty):
            continue
        ax.plot(pkts["distance_m"], pkts["rssi_dbm"], ".", ms=4, alpha=0.7, label=name)
        plotted = True
    if not plotted:
        plt.close(fig)
        return
    ax.set_xlabel("TX/RX GPS distance (m)")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title(f"{test_dir.name} — RSSI vs distance")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(test_dir / "rssi_over_distance.png", dpi=110)
    plt.close(fig)


def _plot_per(summary: pd.DataFrame, test_dir: Path, plt) -> None:
    per = pd.to_numeric(summary["per_pct"], errors="coerce")
    if per.dropna().empty:
        return
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar(summary["file"], per.fillna(0))
    ax.set_ylabel("PER (%)")
    ax.set_title(f"{test_dir.name} — packet error rate")
    ax.grid(True, axis="y", alpha=0.3)
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")
    fig.tight_layout()
    fig.savefig(test_dir / "per_bar.png", dpi=110)
    plt.close(fig)


def _plot_noise_floor(loaded, test_dir: Path, plt) -> None:
    fig, ax = plt.subplots(figsize=(11, 4))
    plotted = False
    for name, df in loaded:
        nf = df[df["event"] == "noise_floor"]
        s = nf["rssi_dbm"].dropna()
        if s.empty:
            continue
        ax.plot(_rel_seconds(nf), nf["rssi_dbm"], ".", ms=3, alpha=0.6, label=name)
        plotted = True
    if not plotted:
        plt.close(fig)
        return
    ax.set_xlabel("time since capture start (s)")
    ax.set_ylabel("noise floor (dBm)")
    ax.set_title(f"{test_dir.name} — channel noise floor")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(test_dir / "noise_floor.png", dpi=110)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", nargs="?", default=str(DEFAULT_ROOT),
                    help="root folder containing test subfolders "
                         "(default: scripts/radio_tests)")
    ap.add_argument("--test", default=None,
                    help="only process this one test subfolder by name")
    ap.add_argument("--no-plots", action="store_true",
                    help="write summary.csv only, skip the PNG plots")
    args = ap.parse_args(argv)

    root = Path(args.root)
    if not root.is_dir():
        print(f"root folder not found: {root}\n"
              f"Create it and drop one subfolder per test, each with the CSVs "
              f"to compare.", file=sys.stderr)
        return 1

    plt = None
    if not args.no_plots:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt  # noqa: F811
        except ImportError:
            print("matplotlib not installed; writing CSV only "
                  "(`pip install -r requirements.txt` to enable plots).",
                  file=sys.stderr)

    tests = ([root / args.test] if args.test
             else sorted(d for d in root.iterdir() if d.is_dir()))
    if not tests:
        print(f"no test subfolders found under {root}", file=sys.stderr)
        return 1

    any_done = False
    for test_dir in tests:
        if not test_dir.is_dir():
            print(f"  (test folder not found: {test_dir})")
            continue
        print(f"\n[{test_dir.name}]")
        any_done |= process_test(test_dir, plt)

    return 0 if any_done else 1


if __name__ == "__main__":
    raise SystemExit(main())
