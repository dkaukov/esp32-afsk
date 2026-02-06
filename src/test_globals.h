/*
 * test_globals.h - Minimal stubs for AFSK test harness
 *
 * Provides only the constants that afsk_demod.h and afsk_encoder.h
 * need from the full globals.h, without any hardware dependencies.
 */
#pragma once

#include <Arduino.h>

// Audio sampling rate — must match afsk_demod.h expectations
#define AUDIO_SAMPLE_RATE 48000
