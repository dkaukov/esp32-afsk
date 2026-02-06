#pragma once

// Minimal Arduino compatibility for native unit tests.
// Keep this small: only what the decoder headers need.

#include <stdint.h>

#ifndef PROGMEM
#define PROGMEM
#endif
