# ESP32 Decoder Performance Test

This test runs on an ESP32 and measures decoder CPU time vs. audio duration.

Protocol (optional host streaming):
- Host sends `DECODE <num_samples>` followed by raw 16-bit LE PCM mono at 48 kHz.
- ESP32 replies with: `DONE packets=<n> decode_us=<us> audio_sec=<sec> rt_factor=<x> cpu_pct=<pct>`.

Run on ESP32:
- `pio test -e esp32dev -f test_demodulator_esp32`

Embedded clip:
- Firmware currently runs the embedded 20s clip from `embedded_audio.h` and prints the same `DONE` line.

Host streaming helper (optional):
- `python3 scripts/stream_flac_to_esp32.py --port /dev/tty.usbserial-XXXX test/fixtures/01_40-Mins-Traffic-on-144.39.flac`

Notes:
- Requires `ffmpeg` and `pyserial` on the host.
- `cpu_pct` is based on decoder time only (serial I/O excluded).
