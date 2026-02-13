# KISS TNC Example (ESP32 + arduino-audio-tools)

This example implements a simple KISS TNC over USB serial using the kv4p pinout.
It uses the built-in ESP32 ADC for RX audio and the internal DAC (I2S) for TX audio.
It also configures a SA818 module via UART using the `arduino-dra818` library.

## Build

```
pio run -d examples/kiss_tnc
```

## Serial

- KISS over USB serial at `115200` baud
- Sends decoded AX.25 frames as KISS DATA (`0x00`) frames
- Accepts KISS DATA frames and transmits via AFSK

## Pinout (kv4p)

```cpp
#define DEFAULT_PIN_RF_RXD    16
#define DEFAULT_PIN_RF_TXD    17
#define DEFAULT_PIN_AUDIO_OUT 25  // I2S DAC (GPIO25 implied)
#define DEFAULT_PIN_AUDIO_IN  34  // ADC
#define DEFAULT_PIN_PTT       18
#define DEFAULT_PIN_PD        19
#define DEFAULT_PIN_LED        2
#define DEFAULT_VOLUME         8
```

Notes:
- `DEFAULT_PIN_AUDIO_OUT` is implied by the internal DAC (GPIO25).
- If you change `DEFAULT_PIN_AUDIO_IN`, you may need to adjust ADC attenuation.
- RX mode applies ADC attenuation and injects ADC bias using DAC2 (`GPIO26`), plus a software DC remover in the RX path.

## Audio Tools

Dependencies are pinned in `examples/kiss_tnc/platformio.ini`:

```
https://github.com/pschatzmann/arduino-audio-tools/archive/refs/tags/v1.0.1.zip
https://github.com/fatpat/arduino-dra818.git#89582e3ef7bf3f31f1af149e32cec16c4b9e4cf2
```

## SA818 Defaults

The example configures the SA818 on startup. Defaults are defined in `examples/kiss_tnc/src/main.cpp`:

- `DEFAULT_RF_FREQ_RX` / `DEFAULT_RF_FREQ_TX` (`445.000` MHz)
- `DEFAULT_RF_SQUELCH` (`0`)
- `DEFAULT_RF_BANDWIDTH` (`DRA818_12K5`)
- `DEFAULT_RF_CTCSS_RX` / `DEFAULT_RF_CTCSS_TX` (0)
- `DEFAULT_RF_PREEMPH`, `DEFAULT_RF_HIGHPASS`, `DEFAULT_RF_LOWPASS` (`false`)
- Handshake retry count: `RF_HANDSHAKE_RETRIES = 3`

## Runtime Defaults

- Demodulator: `AfskDemodulator(48000, 2, on_rx_packet)` (decimation factor `2`)
- TX modulation gain: `0.8`
- Lead/tail TX silence: `1000 ms` / `1000 ms`
