# esp32-afsk

ESP32 AFSK modem helper library for Arduino.

## Install

Use PlatformIO Library Manager or add as a Git dependency.

## Usage

See `examples/basic/basic.ino`.

## Testing

Native decoder tests:
- `pio test -e native`

ESP32 embedded benchmark test (requires board connected):
- `pio test -e esp32dev -v`
  - The test runs an embedded 20s audio clip that is compiled into flash as a `int16_t` PCM array in `test/test_decoder_esp32/embedded_audio.h`.
  - The clip is derived from `test/fixtures/01_40-Mins-Traffic-on-144.39.flac`, trimmed to 20s, resampled to 48 kHz mono, and converted to raw PCM before being converted into the header.
  - During the test, the decoder processes the entire embedded clip and reports:
    - `packets`: number of deduplicated frames decoded
    - `decode_sec`: CPU time spent in the decode loop (seconds)
    - `audio_sec`: clip duration (seconds)
    - `rt_factor`: `audio_sec / decode_sec` (values > 1.0 mean faster than real time)
    - `cpu_pct`: `decode_sec / audio_sec * 100` (values < 100% mean real-time or better)
  - The test currently asserts an expected packet count and an upper CPU usage bound.

If you only want to build (no upload), add `--without-uploading`.
