# SDRplay API Odin bindings

Complete Odin language bindings for the SDRplay API v3.15, controlling
SDRplay RSP original software-defined radio hardware on Linux.

## Supported Hardware

RSP1, RSP1A, RSP1B, RSP2, RSPduo, RSPdx, RSPdx-R2 <br>
Tested on RSPdx-R2.

## Features

- **API bindings** — All 19 SDRplay API functions, every type, enum, and struct
- **Hardware tests** — 10-test integration suite verifying streaming and runtime updates
- **FM stereo receiver** — Real-time FM broadcast demodulation with pilot PLL, stereo decoding, deemphasis, AGC, and audio normalization
- **Sample rates up to 10 MHz** — Constants for 2/8/10 MHz (max RSPdx rate ~10.66 MHz)

## Requirements

- Odin compiler (dev-2025-12)
- SDRplay API v3.15 (`libsdrplay_api.so` installed at `/usr/lib/`)
- `sdrplay_apiService` systemd service running
- SDRplay RSP device connected via USB
- Audio output device (for FM receiver mode)

See the license of SDRplay API, because it can only be used with original SDRplay hardware and that is my case.
The site of SRDPlay that has the SDRplay API from https://www.sdrplay.com/downloads/

## Build

```bash
make
```

## Usage

### Hardware tests

```bash
make run_tests
# or
./sdrplay_bindings_test.exe
```

Runs 10 integration tests: API open, device enumeration, select, configure,
streaming, runtime updates (freq/gain/antenna/bandwidth), error strings,
cleanup. Expects all to pass.

### FM radio receiver

```bash
make run_fm
# or
./sdrplay_bindings_test.exe -fm 97.4
```

Tunes to 97.4 MHz (Radio Comercial, Lisbon). Controls while running:

- `+` / `=` — Increase volume
- `-` — Decrease volume
- `Enter` — Quit

Features real-time FM stereo demodulation with 19 kHz pilot PLL, 38 kHz
subcarrier recovery, 75 us deemphasis, and automatic audio normalization.

## Project Structure

```
sdrplay_api_bindings/
  sdrplay_api_bindings/
    sdrplay_api.odin   — API bindings (745 lines)
    sdrplay_test.odin  — Hardware integration tests (656 lines)
    fm_receiver.odin   — FM stereo receiver (648 lines)
  main.odin             — Entry point with CLI argument parsing (35 lines)
  Makefile              — Build/run/clean targets (12 lines)
```

## Documentation

- `doc_api_bindings.txt` — API bindings reference
- `doc_tests.txt` — Test suite documentation
- `doc_fm_receiver.txt` — FM receiver architecture and code documentation
- From the SDRplay site `SDRplay_API_Specification_v3.15.pdf` — Official SDRplay API spec

## Sample Rate Support

The SDRplay API distinguishes between bandwidth (analog IF filter, max 8 MHz)
and sample rate (ADC output, up to ~10.66 MHz on RSPdx). Constants provided:

- `SDRPLAY_FS_2MHZ` — 2 MHz (default, FM radio)
- `SDRPLAY_FS_8MHZ` — 8 MHz
- `SDRPLAY_FS_10MHZ` — 10 MHz (wideband capture)
- `SDRPLAY_FS_MAX` — 10.66 MHz (RSPdx maximum)

For 10 MHz sample rate, set `bwType = .BW_8_000` (maximum analog bandwidth)
and `fsFreq.fsHz = SDRPLAY_FS_10MHZ`.

## License

MIT Open Source License for these bindings only.

The SDRplay API itself has its own license — check and see the SDRplay website for terms. An original SDRplay device is required to use the API.

## Have fun
Best regards, <br>
Joao Carvalho
