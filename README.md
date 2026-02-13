# esp32-afsk

ESP32 AFSK modem helper library for Arduino.

## Install

Use PlatformIO Library Manager or add as a Git dependency.

## Usage

See `examples/basic/basic.ino`.

Common usage patterns:

- Demodulate `int16_t` PCM chunks (e.g. ADC/I2S input):
```cpp
#include "AfskDemodulator.h"

static void onPacket(const uint8_t *frame, size_t len) {
  // handle AX.25 payload
}

AfskDemodulator demod(48000, 2, onPacket);

// call for each incoming chunk
demod.processSamples(samples_i16, sample_count);

// call at end of stream / mode switch
demod.flush();
```

- Demodulate `float` samples:
```cpp
AfskDemodulator demod(48000, 1, onPacket);
demod.processSamples(samples_f32, sample_count);
demod.flush();
```

- Modulate packets to audio via callback:
```cpp
#include "AfskModulator.h"

static void onTxSamples(const float *samples, size_t count) {
  // send to DAC / I2S
}

AfskModulator mod(48000, onTxSamples);
float chunk[256];
mod.modulate(payload, payload_len, chunk, 256);
```

  - `chunk` is a caller-provided scratch/output buffer.
  - The modulator fills this buffer with PCM float samples in blocks.
  - When a block is ready (or a final partial block remains), it invokes `onTxSamples(samples, count)`.
  - The callback must consume/copy samples immediately if it needs them later.

- Modulate with lead/tail silence (PTT friendly):
```cpp
mod.modulate(payload, payload_len, chunk, 256, 1000.0f, 1000.0f);
```

- Stream-style single-sample demod (less efficient, useful for harnesses):
```cpp
demod.processSample(sample_f32);
```

See also:
- `examples/basic/basic.ino`
- `examples/kiss_tnc/src/main.cpp`
  - Simple KISS TNC terminal example for KV4P-style hardware wiring/pinout (SA818/DRA818 + ESP32 audio/PTT switching).

## Testing

Native decoder tests:
- `pio test -e native`

ESP32 embedded benchmark test (requires board connected):
- `pio test -e esp32dev -v`
  - The test runs an embedded 20s audio clip that is compiled into flash as a `int16_t` PCM array in `test/test_demodulator_esp32/embedded_audio.h`.
  - The clip is derived from `test/fixtures/01_40-Mins-Traffic-on-144.39.flac`, trimmed to 20s, resampled to 48 kHz mono, and converted to raw PCM before being converted into the header.
  - During the test, the decoder processes the entire embedded clip and reports:
    - `packets`: number of deduplicated frames decoded
    - `decode_sec`: CPU time spent in the decode loop (seconds)
    - `audio_sec`: clip duration (seconds)
    - `rt_factor`: `audio_sec / decode_sec` (values > 1.0 mean faster than real time)
    - `cpu_pct`: `decode_sec / audio_sec * 100` (values < 100% mean real-time or better)
  - The test currently asserts `packets == 13` for all decimation factors, with per-decim CPU limits.

Current metrics (latest run):
- Native decoder packet counts:

| Decim | Track 1 (`01_40-Mins-Traffic-on-144.39.flac`) | Track 2 (`02_100-Mic-E-Bursts-DE-emphasized.flac`) |
|---|---:|---:|
| 1 | 1005 | 947 |
| 2 | 1005 | 946 |
| 3 | 1005 | 945 |
| 4 | 1000 | 948 |

  - These are the full-length WA8LMF tracks.
  - Source reference: http://www.wa8lmf.net/TNCtest/
- ESP32 benchmark CPU usage (`cpu_pct`):

| Decim | CPU usage |
|---|---:|
| 1 | 17.96% |
| 2 | 9.58% |
| 3 | 6.70% |
| 4 | 5.50% |

  - Measurement method:
    - The benchmark measures wall-clock time spent inside the decode loop (`decode_sec`) for a fixed embedded clip.
    - `audio_sec` is the real-time duration of that clip.
    - `cpu_pct = (decode_sec / audio_sec) * 100`.

If you only want to build (no upload), add `--without-uploading`.
