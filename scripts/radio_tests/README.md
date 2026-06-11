# radio_tests

Drop your RF captures here, **one subfolder per test**. Each subfolder holds the
captures you want to compare against each other (e.g. one file per antenna):

```
radio_tests/
  test1/
    cots.csv        # capture with the COTS antenna
    diy.csv         # capture with the DIY antenna
  rooftop_range/
    yagi.csv
    omni.csv
```

Capture a log by saving the USB-serial output of a `*_rx_test` / `*_tx_test`
Pico tool to a file in the test folder. The new tools emit the common CSV
format directly (`projects/common/rf_csv.h`); old human-readable `.txt` logs
also load.

Then from `scripts/`:

```sh
uv run run_radio_tests.py            # process every test folder
uv run run_radio_tests.py --test test1
```

For each test folder this writes (and overwrites on re-run):

- `summary.csv` — one metrics row per capture (PER, RSSI/SNR stats, noise floor)
- `rssi_over_time.png` — signal strength over time (the main comparison)
- `rssi_over_seq.png` — RSSI vs packet sequence number
- `snr_over_seq.png` — SNR vs seq (LoRa captures only)
- `per_bar.png` — packet-error-rate per capture
- `noise_floor.png` — channel noise floor over time

These generated files are git-ignored; only your raw captures are tracked.

`test1/` ships as a worked example using two real outdoor 915 MHz captures.
