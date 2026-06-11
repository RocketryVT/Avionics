#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pandas", "matplotlib"]
# ///
"""analyze_rf.py — summarize and plot RF bench-test logs.

Loads one or more capture files (the common CSV format from the lora915_* /
rfm69_433_* Pico tools, or the older human-readable logs) and prints a per-file
link summary. Optionally plots RSSI / SNR / PER over the run, and can overlay
several files for a head-to-head antenna comparison.

Examples
--------
  # Summaries for two captures
  python analyze_rf.py 915cots.txt 915diy.txt

  # Summary + plots written next to each input
  python analyze_rf.py 915diy.csv --plot

  # Overlay several files on shared axes for comparison
  python analyze_rf.py 915cots.txt 915diy.txt --compare --plot

PER here is recomputed from the payload "#N" sequence numbers (expected =
max-min+1 over good packets), which is more robust than the firmware's running
counter because it ignores TX restarts within a capture.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

import rf_log


def _seq_stats(rx: pd.DataFrame) -> dict:
    """Recompute loss/PER from the sequence numbers of good packets."""
    good = rx[rx["event"] == "packet"].dropna(subset=["seq"])
    if good.empty:
        return dict(received=0, expected=0, lost=0, per=float("nan"))
    seqs = good["seq"].astype(int)
    lo, hi = int(seqs.min()), int(seqs.max())
    expected = hi - lo + 1
    received = int(seqs.nunique())
    lost = max(expected - received, 0)
    per = 100.0 * lost / expected if expected else float("nan")
    return dict(received=received, expected=expected, lost=lost, per=per)


def summarize(df: pd.DataFrame, name: str) -> None:
    rx = df[df["role"] == "rx"]
    tx = df[df["role"] == "tx"]

    print(f"\n=== {name} ===")
    if not df.empty:
        freqs = sorted(df["freq_mhz"].dropna().unique())
        mods = sorted(df["modulation"].dropna().unique())
        dur = df["timestamp_ms"].dropna()
        span = (dur.max() - dur.min()) / 1000.0 if len(dur) > 1 else 0.0
        print(f"  freq: {', '.join(f'{f:.1f}' for f in freqs)} MHz   "
              f"mod: {', '.join(mods)}   span: {span:.0f} s")

    if not rx.empty:
        pkts = rx[rx["event"] == "packet"]
        crc = int((rx["event"] == "crc_error").sum())
        rerr = int((rx["event"] == "read_error").sum())
        s = _seq_stats(rx)
        print(f"  RX packets: {len(pkts)} good, {crc} CRC errors, "
              f"{rerr} read errors")
        if s["expected"]:
            print(f"  seq span: {s['expected']} expected, {s['received']} got, "
                  f"{s['lost']} lost  ->  PER {s['per']:.1f}%")
        if not pkts["rssi_dbm"].dropna().empty:
            r = pkts["rssi_dbm"].dropna()
            print(f"  RSSI dBm: mean {r.mean():.1f}  median {r.median():.1f}  "
                  f"min {r.min():.1f}  max {r.max():.1f}")
        if not pkts["snr_db"].dropna().empty:
            n = pkts["snr_db"].dropna()
            print(f"  SNR  dB : mean {n.mean():.1f}  median {n.median():.1f}  "
                  f"min {n.min():.1f}  max {n.max():.1f}")
        floor = rx[rx["event"] == "noise_floor"]["rssi_dbm"].dropna()
        if not floor.empty:
            print(f"  noise floor dBm: median {floor.median():.1f}  "
                  f"min {floor.min():.1f}  (n={len(floor)})")

    if not tx.empty:
        ok = int((tx["event"] == "tx_ok").sum())
        fail = int((tx["event"] == "tx_fail").sum())
        air = tx["air_ms"].dropna()
        air_str = f"  air_ms: mean {air.mean():.0f}" if not air.empty else ""
        print(f"  TX: {ok} sent, {fail} failed{air_str}")


def _plot_single(df: pd.DataFrame, name: str, out: Path, plt) -> None:
    pkts = df[(df["role"] == "rx") & (df["event"] == "packet")]
    if pkts.empty:
        print(f"  (no rx packets to plot for {name})")
        return
    has_snr = not pkts["snr_db"].dropna().empty
    fig, axes = plt.subplots(2 if has_snr else 1, 1, figsize=(11, 7), sharex=True)
    axes = axes if has_snr else [axes]

    x = pkts["seq"] if not pkts["seq"].dropna().empty else range(len(pkts))
    axes[0].plot(x, pkts["rssi_dbm"], ".", ms=4)
    floor = df[df["event"] == "noise_floor"]["rssi_dbm"].dropna()
    if not floor.empty:
        axes[0].axhline(floor.median(), color="gray", ls="--", lw=1,
                        label=f"noise floor {floor.median():.0f} dBm")
        axes[0].legend(loc="lower left")
    axes[0].set_ylabel("RSSI (dBm)")
    axes[0].set_title(f"{name} — RX link")
    axes[0].grid(True, alpha=0.3)
    if has_snr:
        axes[1].plot(x, pkts["snr_db"], ".", ms=4, color="tab:orange")
        axes[1].set_ylabel("SNR (dB)")
        axes[1].grid(True, alpha=0.3)
    axes[-1].set_xlabel("packet seq")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"  wrote {out}")
    plt.close(fig)


def _plot_compare(frames: list[tuple[str, pd.DataFrame]], out: Path, plt) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    for name, df in frames:
        pkts = df[(df["role"] == "rx") & (df["event"] == "packet")]
        if pkts.empty:
            continue
        x = pkts["seq"] if not pkts["seq"].dropna().empty else range(len(pkts))
        ax.plot(x, pkts["rssi_dbm"], ".", ms=4, alpha=0.7, label=name)
    ax.set_xlabel("packet seq")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title("RSSI comparison")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\nwrote {out}")
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="capture file(s) to analyze")
    ap.add_argument("--plot", action="store_true", help="write PNG plots")
    ap.add_argument("--compare", action="store_true",
                    help="overlay all inputs on one RSSI plot")
    ap.add_argument("--outdir", default=None,
                    help="directory for plots (default: next to each input)")
    args = ap.parse_args(argv)

    frames = []
    for p in args.logs:
        df = rf_log.load(p)
        summarize(df, Path(p).name)
        frames.append((Path(p).name, df, Path(p)))

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\nmatplotlib not installed; `pip install matplotlib` to plot.",
                  file=sys.stderr)
            return 1
        outdir = Path(args.outdir) if args.outdir else None
        if args.compare:
            target = (outdir or frames[0][2].parent) / "rf_compare_rssi.png"
            _plot_compare([(n, d) for n, d, _ in frames], target, plt)
        else:
            for name, df, path in frames:
                target = (outdir or path.parent) / f"{path.stem}_rssi.png"
                _plot_single(df, name, target, plt)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
