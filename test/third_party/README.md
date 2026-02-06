# Third-Party Test Dependencies

This test harness uses `dr_flac.h` (single-header FLAC decoder) for native
unit tests. Arduino compatibility is provided by the `FakeArduino` library
declared in `platformio.ini` under the `native` environment.

Place the file here:
- `test/third_party/dr_flac.h`

You can obtain `dr_flac.h` from the `dr_libs` project by David Reid.
