/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This file is part of esp32-afsk.
 *
 * esp32-afsk is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * esp32-afsk is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*
AFSK 1200 Modulator for ESP32
Ported from https://github.com/dkaukov/afsk-java (Afsk1200Modulator + Modulator).
*/

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "AfskCrc.h"

#ifndef AFSK_SAMPLE_RATE
#define AFSK_SAMPLE_RATE 48000
#endif

#ifndef AFSK_BAUD_RATE
#define AFSK_BAUD_RATE 1200.0f
#endif

#ifndef AFSK_MARK_FREQ
#define AFSK_MARK_FREQ 1200.0f
#endif

#ifndef AFSK_SPACE_FREQ
#define AFSK_SPACE_FREQ 2200.0f
#endif

typedef void (*AfskSampleCallback)(const float *samples, size_t count);

namespace afsk {
namespace mod {

// DDS oscillator for modulation (table-based, cosine output).
class ModDdsOsc {
public:
    void init(float sample_rate, float freq_hz) {
        phase = 0;
        setFrequency(sample_rate, freq_hz);
    }

    void setFrequency(float sample_rate, float freq_hz) {
        phase_step = (uint32_t)((freq_hz * (1ull << 32)) / sample_rate);
    }

    inline float nextCos() {
        phase += phase_step;
        int index = (int)((phase >> (32 - TABLE_BITS)) & TABLE_MASK);
        const float *tbl = table();
        return tbl[(index + COS_SHIFT) & TABLE_MASK];
    }

    inline void reset() {
        phase = 0;
    }

private:
    static constexpr int TABLE_BITS = 9;
    static constexpr int TABLE_SIZE = 1 << TABLE_BITS;
    static constexpr int TABLE_MASK = TABLE_SIZE - 1;
    static constexpr int COS_SHIFT = TABLE_SIZE / 4;
    static const float *table() {
        static float tbl[TABLE_SIZE];
        static bool init = false;
        if (!init) {
            for (int i = 0; i < TABLE_SIZE; i++) {
                tbl[i] = sinf(2.0f * (float)M_PI * i / (float)TABLE_SIZE);
            }
            init = true;
        }
        return tbl;
    }

    uint32_t phase = 0;
    uint32_t phase_step = 0;
};

}  // namespace mod
}  // namespace afsk

class AfskModulator {
public:
    AfskModulator(int sample_rate = AFSK_SAMPLE_RATE, AfskSampleCallback callback = nullptr, int preamble_flags = -1, int postamble_flags = 3)
        : sample_rate((float)sample_rate),
          callback(callback),
          postamble_flags(postamble_flags) {
        if (preamble_flags < 0) {
            preamble_flags = (int)lroundf(0.25f * AFSK_BAUD_RATE / 8.0f);  // 250 ms preamble
        }
        this->preamble_flags = preamble_flags;
        samples_per_bit = sample_rate / AFSK_BAUD_RATE;
        osc.init(sample_rate, AFSK_MARK_FREQ);
    }

    void reset() {
        nrzi_state = 0;
        one_count = 0;
        bit_phase = 0.0f;
        bit_end = 0.0f;
        osc.reset();
    }

    void modulate(const uint8_t *payload, size_t len, float *buffer, size_t chunk_size) {
        if (!callback || !buffer || chunk_size == 0) return;
        chunk_index = 0;
        internalModulate(payload, len, buffer, chunk_size);
        if (chunk_index > 0) {
            callback(buffer, chunk_index);
        }
    }

    void modulate(const uint8_t *payload, size_t len, float *buffer, size_t chunk_size, float lead_silence_ms, float tail_silence_ms) {
        if (!callback || !buffer || chunk_size == 0) return;
        chunk_index = 0;
        emitSilence(buffer, chunk_size, lead_silence_ms);
        internalModulate(payload, len, buffer, chunk_size);
        emitSilence(buffer, chunk_size, tail_silence_ms);
        if (chunk_index > 0) {
            callback(buffer, chunk_index);
        }
    }

private:
    void emitSilence(float *buffer, size_t chunk_size, float ms) {
        if (ms <= 0.0f) return;
        int samples = (int)(ms * 1e-3f * sample_rate);
        for (int i = 0; i < samples; i++) {
            buffer[chunk_index++] = 0.0f;
            if (chunk_index % chunk_size == 0) {
                callback(buffer, chunk_index);
                chunk_index = 0;
            }
        }
    }

    void internalModulate(const uint8_t *payload, size_t len,
                          float *buffer, size_t chunk_size) {
        reset();
        // Lead flags (raw, no stuffing)
        for (int i = 0; i < preamble_flags; i++) {
            writeRawByte(FLAG, buffer, chunk_size);
        }
        // Payload with bit stuffing
        for (size_t i = 0; i < len; i++) {
            stuffByte(payload[i], buffer, chunk_size);
        }
        // CRC (little-endian, inverted)
        uint16_t crc = afsk::crc::calc(payload, len) ^ 0xFFFF;
        stuffByte((uint8_t)(crc & 0xFF), buffer, chunk_size);
        stuffByte((uint8_t)((crc >> 8) & 0xFF), buffer, chunk_size);
        // Tail flags (raw, no stuffing)
        for (int i = 0; i < postamble_flags; i++) {
            writeRawByte(FLAG, buffer, chunk_size);
        }
    }

    void writeRawByte(uint8_t byteVal, float *buffer, size_t chunk_size) {
        for (int i = 0; i < 8; i++) {
            int bit = (byteVal >> i) & 1;
            emitBit(bit, buffer, chunk_size);
        }
    }

    void stuffByte(uint8_t byteVal, float *buffer, size_t chunk_size) {
        for (int i = 0; i < 8; i++) {
            int bit = (byteVal >> i) & 1;
            emitBit(bit, buffer, chunk_size);
            if (bit == 1) {
                one_count++;
                if (one_count == 5) {
                    emitBit(0, buffer, chunk_size);
                    one_count = 0;
                }
            } else {
                one_count = 0;
            }
        }
    }

    void emitBit(int bit, float *buffer, size_t chunk_size) {
        if (bit == 0) {
            nrzi_state ^= 1;
        }
        osc.setFrequency(sample_rate, nrzi_state ? AFSK_MARK_FREQ : AFSK_SPACE_FREQ);
        bit_end += samples_per_bit;
        while (bit_phase < bit_end) {
            buffer[chunk_index++] = osc.nextCos();
            bit_phase += 1.0f;
            if (chunk_index % chunk_size == 0) {
                callback(buffer, chunk_index);
                chunk_index = 0;
            }
        }
    }

private:
    static constexpr uint8_t FLAG = 0x7E;

    float sample_rate;
    AfskSampleCallback callback;
    float samples_per_bit;
    int preamble_flags;
    int postamble_flags;

    int nrzi_state = 0;
    int one_count = 0;
    float bit_phase = 0.0f;
    float bit_end = 0.0f;

    afsk::mod::ModDdsOsc osc;
    size_t chunk_index = 0;
};
