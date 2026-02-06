/*
 * afsk_encoder.h - Buffer-Output AFSK 1200 Encoder for Test Harness
 * ===================================================================
 * Derived from aprs.h in the KV4P-HT production code.
 *
 * All I2S, GPIO, PTT, and hardware dependencies have been removed.
 * Instead of writing audio samples to I2S DMA buffers, this version
 * writes 16-bit signed PCM samples into a caller-provided memory buffer.
 *
 * This allows the encoder output to be:
 *   - Fed directly into the AFSK decoder for loopback testing
 *   - Streamed over Serial to the host for analysis
 *   - Compared sample-by-sample against reference implementations
 *
 * Copyright (C) 2025 - Contributed to KV4P-HT project
 * Licensed under GNU General Public License v3 (same as parent project)
 */

#pragma once

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "test_globals.h"
#include "test_debug.h"

// ============================================================================
// SECTION 1: AFSK MODEM CONSTANTS (identical to aprs.h)
// ============================================================================

static const uint16_t ENC_AFSK_MARK_FREQ  = 1200;
static const uint16_t ENC_AFSK_SPACE_FREQ = 2200;
static const uint16_t ENC_AFSK_BAUD_RATE  = 1200;
static const uint32_t ENC_AFSK_SAMPLE_RATE = AUDIO_SAMPLE_RATE;  // 48000
static const uint16_t ENC_SAMPLES_PER_SYMBOL = ENC_AFSK_SAMPLE_RATE / ENC_AFSK_BAUD_RATE;

// ============================================================================
// SECTION 2: DDS CONSTANTS
// ============================================================================

static const uint32_t ENC_DDS_PHASE_INC_MARK =
    (uint32_t)((double)ENC_AFSK_MARK_FREQ / ENC_AFSK_SAMPLE_RATE * 4294967296.0);

static const uint32_t ENC_DDS_PHASE_INC_SPACE =
    (uint32_t)((double)ENC_AFSK_SPACE_FREQ / ENC_AFSK_SAMPLE_RATE * 4294967296.0);

// ============================================================================
// SECTION 3: SINE LOOKUP TABLE
// ============================================================================

static const uint16_t ENC_SINE_TABLE_SIZE = 256;
static int16_t encSineTable[ENC_SINE_TABLE_SIZE];

void initEncoderSineTable() {
    for (int i = 0; i < ENC_SINE_TABLE_SIZE; i++) {
        encSineTable[i] = (int16_t)(sin(2.0 * M_PI * i / ENC_SINE_TABLE_SIZE) * 22937.0);
    }
}

// ============================================================================
// SECTION 4: AX.25 PROTOCOL CONSTANTS
// ============================================================================

static const uint8_t ENC_AX25_FLAG = 0x7E;
static const uint8_t ENC_PREAMBLE_FLAGS = 45;
static const uint8_t ENC_TRAILER_FLAGS  = 3;

// ============================================================================
// SECTION 5: ENCODER STATE (per-encode, reset before each packet)
// ============================================================================

typedef struct {
    // DDS state
    uint32_t ddsPhase;
    uint32_t ddsPhaseInc;

    // NRZI state
    bool nrziState;

    // Bit stuffing
    uint8_t stuffCount;

    // Output buffer
    int16_t *outputBuf;
    size_t   outputBufLen;   // capacity in samples
    size_t   outputIndex;    // current write position
    bool     overflow;       // set if buffer was too small
} AfskEncoderState;

// ============================================================================
// SECTION 6: SAMPLE OUTPUT (buffer instead of I2S)
// ============================================================================

static inline void encWriteSample(AfskEncoderState *st, int16_t sample) {
    if (st->outputIndex < st->outputBufLen) {
        st->outputBuf[st->outputIndex++] = sample;
    } else {
        st->overflow = true;
    }
}

// ============================================================================
// SECTION 7: TONE GENERATION
// ============================================================================

static void encGenerateSymbol(AfskEncoderState *st) {
    for (uint16_t i = 0; i < ENC_SAMPLES_PER_SYMBOL; i++) {
        uint8_t tableIndex = (uint8_t)(st->ddsPhase >> 24);
        int16_t sample = encSineTable[tableIndex];
        encWriteSample(st, sample);
        st->ddsPhase += st->ddsPhaseInc;
    }
}

static void encGenerateSilence(AfskEncoderState *st, uint16_t durationMs) {
    uint32_t totalSamples = (uint32_t)ENC_AFSK_SAMPLE_RATE * durationMs / 1000;
    for (uint32_t i = 0; i < totalSamples; i++) {
        encWriteSample(st, 0);
    }
}

// ============================================================================
// SECTION 8: NRZI ENCODING AND BIT STUFFING
// ============================================================================

static void encTransmitBit(AfskEncoderState *st, uint8_t bit) {
    if (bit == 0) {
        st->nrziState = !st->nrziState;
    }
    st->ddsPhaseInc = st->nrziState ? ENC_DDS_PHASE_INC_MARK : ENC_DDS_PHASE_INC_SPACE;
    encGenerateSymbol(st);
}

static void encTransmitByte(AfskEncoderState *st, uint8_t byteVal, bool stuff) {
    for (int i = 0; i < 8; i++) {
        uint8_t bit = (byteVal >> i) & 0x01;
        encTransmitBit(st, bit);

        if (stuff) {
            if (bit == 1) {
                st->stuffCount++;
                if (st->stuffCount == 5) {
                    encTransmitBit(st, 0);
                    st->stuffCount = 0;
                }
            } else {
                st->stuffCount = 0;
            }
        }
    }
}

// ============================================================================
// SECTION 9: MAIN ENCODE FUNCTION
// ============================================================================

/*
 * afskEncodePacket()
 *
 * Encodes a complete AX.25 packet (with CRC already appended) into
 * AFSK 1200 audio samples written to the provided output buffer.
 *
 * Parameters:
 *   packetData   - AX.25 frame bytes WITH CRC (from Packet.bytesWithCRC())
 *   packetLen    - Number of bytes in packetData (18-512)
 *   outputBuf    - Caller-provided buffer for 16-bit PCM samples
 *   outputBufLen - Capacity of outputBuf in samples
 *   preambleFlags - Number of 0x7E flag bytes before data (default 45)
 *   trailerFlags  - Number of 0x7E flag bytes after data (default 3)
 *   preSilenceMs  - Milliseconds of silence before preamble (0 for loopback)
 *   postSilenceMs - Milliseconds of silence after trailer (0 for loopback)
 *
 * Returns:
 *   Number of samples written to outputBuf, or 0 on error.
 *   If the buffer is too small, returns the number written before overflow
 *   and logs a warning.
 *
 * Estimated buffer size needed (no silence):
 *   preamble:  45 flags * 8 bits * 40 samples = 14,400
 *   data:      80 bytes * ~9 bits * 40 samples = 28,800 (with stuffing)
 *   trailer:   3 flags * 8 bits * 40 samples   = 960
 *   Total:     ~44,160 samples for a typical 80-byte packet
 *   With 1100ms pre + 700ms post silence: +86,400 = ~130,560 total
 */
size_t afskEncodePacket(const uint8_t *packetData, size_t packetLen,
                        int16_t *outputBuf, size_t outputBufLen,
                        uint8_t preambleFlags, uint8_t trailerFlags,
                        uint16_t preSilenceMs, uint16_t postSilenceMs) {

    // Validate
    if (packetLen < 18 || packetLen > 512) {
        _LOGW("AFSK ENC: Invalid packet length %d (expected 18-512)", packetLen);
        return 0;
    }
    if (outputBuf == NULL || outputBufLen == 0) {
        return 0;
    }

    // Initialize encoder state
    AfskEncoderState st;
    st.ddsPhase    = 0;
    st.ddsPhaseInc = ENC_DDS_PHASE_INC_MARK;
    st.nrziState   = true;
    st.stuffCount  = 0;
    st.outputBuf   = outputBuf;
    st.outputBufLen = outputBufLen;
    st.outputIndex = 0;
    st.overflow    = false;

    // Pre-silence
    if (preSilenceMs > 0) {
        encGenerateSilence(&st, preSilenceMs);
    }

    // Preamble flags (no bit stuffing)
    for (uint8_t i = 0; i < preambleFlags; i++) {
        encTransmitByte(&st, ENC_AX25_FLAG, false);
    }

    // Packet data (with bit stuffing)
    st.stuffCount = 0;
    for (size_t i = 0; i < packetLen; i++) {
        encTransmitByte(&st, packetData[i], true);
    }

    // Trailer flags (no bit stuffing)
    for (uint8_t i = 0; i < trailerFlags; i++) {
        encTransmitByte(&st, ENC_AX25_FLAG, false);
    }

    // Post-silence
    if (postSilenceMs > 0) {
        encGenerateSilence(&st, postSilenceMs);
    }

    if (st.overflow) {
        _LOGW("AFSK ENC: Output buffer overflow (%d samples capacity, needed more)", outputBufLen);
    }

    return st.outputIndex;
}

/*
 * afskEncodePacketDefaults()
 *
 * Convenience wrapper using default preamble/trailer/silence values.
 * For loopback testing, use afskEncodePacket() with 0ms silence.
 */
size_t afskEncodePacketDefaults(const uint8_t *packetData, size_t packetLen,
                                int16_t *outputBuf, size_t outputBufLen) {
    return afskEncodePacket(packetData, packetLen, outputBuf, outputBufLen,
                            ENC_PREAMBLE_FLAGS, ENC_TRAILER_FLAGS,
                            0, 0);  // No silence for test harness
}
