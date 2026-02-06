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

If you only want to build (no upload), add `--without-uploading`.
