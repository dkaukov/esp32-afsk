/*
AFSK 1200 Multi-Demodulator for ESP32
Port of javAX25 Afsk1200MultiDemodulator.java by Sivan Toledo (2012)
Adapted for KV4P-HT ESP32 firmware.

Runs two parallel Afsk1200Demodulator instances:
  d0: flat (no de-emphasis) — best for flat discriminator audio
  d6: 6dB de-emphasis — best for de-emphasized audio
Deduplicates identical packets decoded by both instances.
*/

#pragma once

#include "afsk_demod.h"

// ============================================================================
// Section 1: Multi-Demodulator State
// ============================================================================

// Callback type for deduplicated decoded packets
typedef void (*AfskMultiPacketCallback)(const uint8_t *frame, size_t len);

typedef struct {
    // Two parallel demodulator instances
    AfskDemodulator d0;                  // flat / no de-emphasis
    AfskDemodulator d6;                  // 6dB de-emphasis

    // Deduplication state
    uint8_t last_packet[AFSK_MAX_FRAME_SIZE];
    size_t  last_packet_len;
    int     last_demod;                  // emphasis value of last packet source (0 or 6)

    // Statistics
    uint32_t packet_count;               // unique packets forwarded
    uint32_t dup_count;                  // duplicates suppressed

    // Outer callback for unique decoded packets
    AfskMultiPacketCallback callback;
} AfskMultiDemodulator;

// Forward declaration — the inner callback that handles dedup
static void afsk_multi_inner_callback(const uint8_t *frame, size_t len, int emphasis);

// Global pointer for the inner callback to find the multi-demod instance.
// Only one multi-demodulator instance is expected per firmware.
static AfskMultiDemodulator *g_multi_demod = NULL;

// ============================================================================
// Section 2: Inner Callback (Deduplication Logic)
// ============================================================================

// Called by each AfskDemodulator when it decodes a valid packet.
// Mirrors Afsk1200MultiDemodulator.InnerHandler.handlePacket() from Java.
static void afsk_multi_inner_callback(const uint8_t *frame, size_t len, int emphasis) {
    AfskMultiDemodulator *md = g_multi_demod;
    if (md == NULL) return;

    // Check for duplicate: same packet from the other demodulator
    if (md->last_packet_len > 0
        && emphasis != md->last_demod
        && len == md->last_packet_len
        && memcmp(frame, md->last_packet, len) == 0) {
        // Duplicate — suppress
        md->dup_count++;
        return;
    }

    // New unique packet — store for future dedup comparison
    if (len <= AFSK_MAX_FRAME_SIZE) {
        memcpy(md->last_packet, frame, len);
        md->last_packet_len = len;
        md->last_demod = emphasis;
    }

    md->packet_count++;

    // Forward to outer callback
    if (md->callback != NULL) {
        md->callback(frame, len);
    }
}

// ============================================================================
// Section 3: Initialization
// ============================================================================

static void afsk_multi_init(AfskMultiDemodulator *md, AfskMultiPacketCallback callback) {
    memset(md, 0, sizeof(AfskMultiDemodulator));

    md->callback = callback;
    md->last_demod = -1;  // no previous packet

    // Set global pointer for inner callback
    g_multi_demod = md;

    // Initialize both demodulators with the shared inner callback
    afsk_demod_init(&md->d0, 0, afsk_multi_inner_callback);
    afsk_demod_init(&md->d6, 6, afsk_multi_inner_callback);
}

// ============================================================================
// Section 4: Per-Sample Processing
// ============================================================================

// Feed one sample to both demodulators.
static inline void afsk_multi_process_sample(AfskMultiDemodulator *md, float sample) {
    afsk_demod_process_sample(&md->d0, sample);
    afsk_demod_process_sample(&md->d6, sample);
}

// Data carrier detect: true if either demodulator has lock
static inline bool afsk_multi_dcd(const AfskMultiDemodulator *md) {
    return md->d0.data_carrier || md->d6.data_carrier;
}
