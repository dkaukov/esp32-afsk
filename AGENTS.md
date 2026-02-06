# Repository Guidelines

## Project Structure & Module Organization
- `src/` contains the core AFSK modem implementation (headers and implementation are colocated as `*.h`/`*.cpp`).
- `include/` exposes the public Arduino-facing API (`ESP32Afsk.h`).
- `examples/basic/` is a minimal Arduino/PlatformIO example with its own `platformio.ini` and sketch.
- `test/` is reserved for PlatformIO unit tests (currently empty aside from the template README).
- `lib/` is available for private/third‑party libraries if needed.

## Build, Test, and Development Commands
This is a PlatformIO/Arduino library. Typical commands:
- `pio run` builds the current project (for the active environment in `platformio.ini`).
- `pio run -t upload` builds and uploads to the configured board.
- `pio test` runs PlatformIO unit tests in `test/` (no tests are defined yet).
- Example build: `pio run -d examples/basic` builds the `examples/basic` sketch.

## Coding Style & Naming Conventions
- C++/Arduino style is used throughout. Prefer 2‑space indentation (see `examples/basic/basic.ino`).
- Public Arduino API lives in `include/` and follows `CamelCase` types with `lower_snake_case` members where practical (see `ESP32Afsk.h`).
- Use `*.h`/`*.cpp` for library code and `*.ino` for sketches.

## Testing Guidelines
- Tests should live under `test/` and be executed with `pio test`.
- Follow PlatformIO’s unit testing layout (one suite per folder). Example naming: `test/test_afsk_encoder/`.

## Commit & Pull Request Guidelines
- No Git history is present in this workspace, so there are no established commit message conventions. Use short, imperative subjects (e.g., "Fix CRC check").
- PRs should include: a brief summary, test commands run (or “not run”), and any hardware/board assumptions.

## Agent-Specific Notes
- Prefer `rg` for searching.
- Keep changes local to this repository; do not modify generated or cached files in `.pio_home/`.
