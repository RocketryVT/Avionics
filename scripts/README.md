# scripts — RF bench-test analysis

Python tooling for the standalone radio test firmware
(`projects/pico_examples/radio_examples/*`). Loads their serial-log
captures, computes link metrics (PER, RSSI/SNR, noise floor), and plots
antenna-vs-antenna comparisons.

## Quick start (with `uv`)

The scripts carry inline PEP 723 dependency metadata, so `uv` installs what they
need automatically — no venv setup:

```sh
# Batch-compare every test folder under radio_tests/
uv run run_radio_tests.py

# Just one test folder
uv run run_radio_tests.py --test test1

# Ad-hoc: summarize / plot specific files
uv run analyze_rf.py ../915cots.txt ../915diy.txt --compare --plot
```

Without `uv`: `pip install -r requirements.txt`, then `python run_radio_tests.py`.

## Files

| File | What it does |
|------|--------------|
| `run_radio_tests.py` | Loops over `radio_tests/<test>/`, writes `summary.csv` + comparison plots into each test folder. **Main entry point.** |
| `analyze_rf.py` | Ad-hoc summary / plots for one or more files given on the command line. |
| `rf_log.py` | Loader module: parses the CSV (and legacy `.txt`) logs into a pandas DataFrame; shared by the above. |
| `radio_tests/` | Your captures, one subfolder per test. See its README. |
| `generate_crc_tables.py` | Unrelated existing helper (SIGMA CRC tables). |

## CSV format

The firmware emits a common 18-column schema defined in
`projects/pico_examples/radio_examples/common/rf_csv.h`. One row per event
(`packet` / `crc_error` / `read_error` / `noise_floor` / `tx_ok` / `tx_fail`);
status lines are prefixed `#` and skipped by the loader. Columns:

```
timestamp_ms, role, freq_mhz, modulation, event, seq, len_bytes,
rssi_dbm, snr_db, ferr_hz, good, lost, crc, per_pct, air_ms,
gps_lat, gps_lon, gps_alt_m
```

`freq_mhz` / `modulation` identify the band (915 lora / 433 gfsk). FSK captures
leave `snr_db` / `ferr_hz` blank. The three `gps_*` columns are reserved for
future GPS logging and are blank for now.

## Notes

- PER in the summaries is **recomputed from the `#N` sequence numbers**
  (expected = max−min+1 over good packets), which is robust to TX restarts
  within a capture.
- Capturing a log = save the USB-serial output to a file (any serial terminal
  with capture-to-file works).
