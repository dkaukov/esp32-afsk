/*
 * test_globals.h - Minimal stubs for AFSK test harness
 *
 * Provides only the constants that AfskDemodulator.h and afsk_encoder.h
 * need from the full globals.h, without any hardware dependencies.
 */
#pragma once

#include <Arduino.h>

// Audio sampling rate — must match AfskDemodulator.h expectations
#ifndef AUDIO_SAMPLE_RATE
#define AUDIO_SAMPLE_RATE 48000
#endif
