/*
AFSK 1200 Demodulator for ESP32
Port of javAX25 Afsk1200Demodulator.java by Sivan Toledo (2012)
Adapted for KV4P-HT ESP32 firmware.

Original copyright:
  Copyright (C) Sivan Toledo, 2012
  Released under GNU General Public License v2 or later.

This implementation targets the ESP32 WROOM-32 with single-precision FPU.
It uses correlation-based mark/space detection, zero-crossing bit timing
recovery, and AX.25 HDLC framing with CRC-CCITT validation.
*/

#pragma once

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Section 1: Constants
// ============================================================================

// Audio parameters (must match rxAudio.h I2S config)
#define AFSK_SAMPLE_RATE       48000
#define AFSK_ACTUAL_SAMPLE_RATE (AFSK_SAMPLE_RATE * 1.02f)  // 48960 Hz (2% oversample)
#define AFSK_BAUD_RATE         1200
#define AFSK_MARK_FREQ         1200.0f
#define AFSK_SPACE_FREQ        2200.0f

// Derived constants
#define AFSK_SAMPLES_PER_BIT   (AFSK_ACTUAL_SAMPLE_RATE / (float)AFSK_BAUD_RATE)  // ~40.8
#define AFSK_CORR_LEN          ((int)AFSK_SAMPLES_PER_BIT)  // 40 (correlation window)

// Phase increments for correlator
#define AFSK_PHASE_INC_F0      (2.0f * (float)M_PI * AFSK_MARK_FREQ  / AFSK_ACTUAL_SAMPLE_RATE)
#define AFSK_PHASE_INC_F1      (2.0f * (float)M_PI * AFSK_SPACE_FREQ / AFSK_ACTUAL_SAMPLE_RATE)

// AX.25 frame limits
#define AFSK_MAX_FRAME_SIZE    330  // 7+7+(8*7)+1+1+256+2
#define AFSK_MIN_FRAME_SIZE    18   // dest(7)+src(7)+ctrl(1)+pid(1)+fcs(2)

// CRC constants
#define AX25_CRC_CORRECT       0xF0B8
#define CRC_CCITT_INIT_VAL     0xFFFF

// FIR filter tap count (matching javAX25 filter_index=1, rate_index=7 for 48kHz)
#define AFSK_FILTER_TAPS       80

// ============================================================================
// Section 2: CRC-CCITT Lookup Table (flash)
// ============================================================================

static const uint16_t crc_ccitt_tab[256] PROGMEM = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

// ============================================================================
// Section 3: FIR Filter Coefficients (flash)
// Source: Afsk1200Filters.java, indices [1][7] (filter_index=1, rate_index=7)
// ============================================================================

// Time-domain filter for flat/no-emphasis demodulator (d0)
// From: time_domain_filter_none[1][7]
static const float td_filter_none[AFSK_FILTER_TAPS] PROGMEM = {
    1.090725e-003f,  2.145359e-004f, -3.671281e-004f, -5.168203e-004f, -1.324122e-004f,
    8.390482e-004f,  2.391043e-003f,  4.452077e-003f,  6.885825e-003f,  9.497335e-003f,
    1.204518e-002f,  1.425887e-002f,  1.586042e-002f,  1.658843e-002f,  1.622265e-002f,
    1.460720e-002f,  1.167002e-002f,  7.436988e-003f,  2.038835e-003f, -4.289859e-003f,
   -1.122021e-002f, -1.834677e-002f, -2.521240e-002f, -3.133890e-002f, -3.626095e-002f,
   -3.956110e-002f, -4.090303e-002f, -4.006032e-002f, -3.693852e-002f, -3.158836e-002f,
   -2.420895e-002f, -1.514021e-002f, -4.844976e-003f,  6.118407e-003f,  1.713096e-002f,
    2.755318e-002f,  3.676743e-002f,  4.421957e-002f,  4.945669e-002f,  5.215826e-002f,
    5.215826e-002f,  4.945669e-002f,  4.421957e-002f,  3.676743e-002f,  2.755318e-002f,
    1.713096e-002f,  6.118407e-003f, -4.844976e-003f, -1.514021e-002f, -2.420895e-002f,
   -3.158836e-002f, -3.693852e-002f, -4.006032e-002f, -4.090303e-002f, -3.956110e-002f,
   -3.626095e-002f, -3.133890e-002f, -2.521240e-002f, -1.834677e-002f, -1.122021e-002f,
   -4.289859e-003f,  2.038835e-003f,  7.436988e-003f,  1.167002e-002f,  1.460720e-002f,
    1.622265e-002f,  1.658843e-002f,  1.586042e-002f,  1.425887e-002f,  1.204518e-002f,
    9.497335e-003f,  6.885825e-003f,  4.452077e-003f,  2.391043e-003f,  8.390482e-004f,
   -1.324122e-004f, -5.168203e-004f, -3.671281e-004f,  2.145359e-004f,  1.090725e-003f
};

// Time-domain filter for 6dB de-emphasis demodulator (d6)
// From: time_domain_filter_full[1][7]
static const float td_filter_full[AFSK_FILTER_TAPS] PROGMEM = {
   -1.555936e-003f, -3.074922e-003f, -4.298532e-003f, -5.055988e-003f, -5.207867e-003f,
   -4.661980e-003f, -3.386024e-003f, -1.415645e-003f,  1.143128e-003f,  4.117646e-003f,
    7.278662e-003f,  1.035612e-002f,  1.305964e-002f,  1.510224e-002f,  1.622537e-002f,
    1.622320e-002f,  1.496402e-002f,  1.240675e-002f,  8.611013e-003f,  3.739340e-003f,
   -1.948784e-003f, -8.111153e-003f, -1.434449e-002f, -2.021209e-002f, -2.527549e-002f,
   -2.912772e-002f, -3.142564e-002f, -3.191868e-002f, -3.047148e-002f, -2.707872e-002f,
   -2.187043e-002f, -1.510729e-002f, -7.165972e-003f,  1.484604e-003f,  1.031443e-002f,
    1.876872e-002f,  2.630674e-002f,  3.244019e-002f,  3.676812e-002f,  3.900592e-002f,
    3.900592e-002f,  3.676812e-002f,  3.244019e-002f,  2.630674e-002f,  1.876872e-002f,
    1.031443e-002f,  1.484604e-003f, -7.165972e-003f, -1.510729e-002f, -2.187043e-002f,
   -2.707872e-002f, -3.047148e-002f, -3.191868e-002f, -3.142564e-002f, -2.912772e-002f,
   -2.527549e-002f, -2.021209e-002f, -1.434449e-002f, -8.111153e-003f, -1.948784e-003f,
    3.739340e-003f,  8.611013e-003f,  1.240675e-002f,  1.496402e-002f,  1.622320e-002f,
    1.622537e-002f,  1.510224e-002f,  1.305964e-002f,  1.035612e-002f,  7.278662e-003f,
    4.117646e-003f,  1.143128e-003f, -1.415645e-003f, -3.386024e-003f, -4.661980e-003f,
   -5.207867e-003f, -5.055988e-003f, -4.298532e-003f, -3.074922e-003f, -1.555936e-003f
};

// Correlation difference filter (used by both demodulators)
// From: corr_diff_filter[1][7]
static const float cd_filter_coeff[AFSK_FILTER_TAPS] PROGMEM = {
   -5.038663e-005f, -1.566090e-004f, -2.776591e-004f, -4.222836e-004f, -5.979724e-004f,
   -8.099774e-004f, -1.060382e-003f, -1.347281e-003f, -1.664124e-003f, -1.999269e-003f,
   -2.335792e-003f, -2.651573e-003f, -2.919681e-003f, -3.109052e-003f, -3.185454e-003f,
   -3.112703e-003f, -2.854091e-003f, -2.373975e-003f, -1.639451e-003f, -6.220534e-004f,
    7.006086e-004f,  2.343343e-003f,  4.312078e-003f,  6.602848e-003f,  9.201100e-003f,
    1.208140e-002f,  1.520752e-002f,  1.853299e-002f,  2.200202e-002f,  2.555088e-002f,
    2.910955e-002f,  3.260375e-002f,  3.595715e-002f,  3.909375e-002f,  4.194031e-002f,
    4.442872e-002f,  4.649828e-002f,  4.809774e-002f,  4.918705e-002f,  4.973870e-002f,
    4.973870e-002f,  4.918705e-002f,  4.809774e-002f,  4.649828e-002f,  4.442872e-002f,
    4.194031e-002f,  3.909375e-002f,  3.595715e-002f,  3.260375e-002f,  2.910955e-002f,
    2.555088e-002f,  2.200202e-002f,  1.853299e-002f,  1.520752e-002f,  1.208140e-002f,
    9.201100e-003f,  6.602848e-003f,  4.312078e-003f,  2.343343e-003f,  7.006086e-004f,
   -6.220534e-004f, -1.639451e-003f, -2.373975e-003f, -2.854091e-003f, -3.112703e-003f,
   -3.185454e-003f, -3.109052e-003f, -2.919681e-003f, -2.651573e-003f, -2.335792e-003f,
   -1.999269e-003f, -1.664124e-003f, -1.347281e-003f, -1.060382e-003f, -8.099774e-004f,
   -5.979724e-004f, -4.222836e-004f, -2.776591e-004f, -1.566090e-004f, -5.038663e-005f
};

// ============================================================================
// Section 4: Demodulator State
// ============================================================================

// Callback type for decoded packets
typedef void (*AfskPacketCallback)(const uint8_t *frame, size_t len, int emphasis);

typedef enum {
    DEMOD_WAITING,
    DEMOD_JUST_SEEN_FLAG,
    DEMOD_DECODING
} AfskDemodState;

typedef struct {
    // Configuration
    int emphasis;                        // 0 = flat, 6 = de-emphasis
    float samples_per_bit;               // ~40.8
    const float *td_filter;              // pointer to time-domain filter coefficients
    const float *cd_filter;              // pointer to corr-diff filter coefficients

    // Phase tracking for correlator
    float phase_f0;                      // 1200 Hz correlator phase
    float phase_f1;                      // 2200 Hz correlator phase

    // Circular buffers for time-domain filtering
    float u1[AFSK_FILTER_TAPS];          // raw input samples
    float x[AFSK_FILTER_TAPS];           // filtered input samples

    // Circular buffers for frequency correlation (sized to one symbol period)
    float c0_real[AFSK_CORR_LEN];        // 1200 Hz correlator real part
    float c0_imag[AFSK_CORR_LEN];        // 1200 Hz correlator imaginary part
    float c1_real[AFSK_CORR_LEN];        // 2200 Hz correlator real part
    float c1_imag[AFSK_CORR_LEN];        // 2200 Hz correlator imaginary part

    // Circular buffer for correlation difference
    float diff[AFSK_FILTER_TAPS];        // c0-c1 difference signal

    // Circular buffer indices
    int j_td;                            // time-domain filter index
    int j_cd;                            // correlation-difference filter index
    int j_corr;                          // correlator index

    // Running sample counter
    int t;

    // Zero-crossing detection
    float previous_fdiff;
    int last_transition;

    // Bit assembly
    AfskDemodState state;
    int data;                            // current byte being assembled
    int bitcount;                        // bits accumulated in current byte
    int flag_count;
    bool flag_separator_seen;

    // Packet assembly
    uint8_t packet_buf[AFSK_MAX_FRAME_SIZE];
    int packet_size;
    uint16_t packet_crc;
    bool packet_active;                  // true when packet_buf is in use

    // Data carrier detect
    volatile bool data_carrier;

    // Callback
    AfskPacketCallback callback;
} AfskDemodulator;

// ============================================================================
// Section 5: Helper Functions
// ============================================================================

// FIR filter: convolve circular buffer x[] at position j with filter f[]
// Matches Filter.filter() from Filter.java
static inline float afsk_fir_filter(const float *x, int j, const float *f, int f_len, int buf_len) {
    float c = 0.0f;
    for (int i = 0; i < f_len; i++) {
        c += x[j] * f[i];
        j--;
        if (j < 0) j = buf_len - 1;
    }
    return c;
}

// Sum all elements in circular buffer x[] of length len, starting at position j
// Matches sum() from Afsk1200Demodulator.java
static inline float afsk_circular_sum(const float *x, int j, int len) {
    float c = 0.0f;
    for (int i = 0; i < len; i++) {
        c += x[j];
        j--;
        if (j < 0) j = len - 1;
    }
    return c;
}

// CRC-CCITT update
static inline void afsk_crc_update(uint16_t *crc, uint8_t b) {
    *crc = (*crc >> 8) ^ crc_ccitt_tab[(*crc ^ b) & 0xFF];
}

// ============================================================================
// Section 6: Demodulator Initialization
// ============================================================================

static void afsk_demod_init(AfskDemodulator *d, int emphasis, AfskPacketCallback callback) {
    memset(d, 0, sizeof(AfskDemodulator));

    d->emphasis = emphasis;
    d->samples_per_bit = AFSK_SAMPLES_PER_BIT;
    d->callback = callback;

    // Select time-domain filter based on emphasis
    if (emphasis == 0) {
        d->td_filter = td_filter_none;
    } else {
        d->td_filter = td_filter_full;
    }
    d->cd_filter = cd_filter_coeff;

    // State machine starts in WAITING
    d->state = DEMOD_WAITING;
    d->data_carrier = false;

    // Initialize packet CRC
    d->packet_crc = CRC_CCITT_INIT_VAL;
    d->packet_active = false;
}

// ============================================================================
// Section 7: Packet Assembly Helpers
// ============================================================================

// Start a new packet
static inline void afsk_packet_reset(AfskDemodulator *d) {
    d->packet_size = 0;
    d->packet_crc = CRC_CCITT_INIT_VAL;
    d->packet_active = true;
}

// Add a byte to the current packet, updating CRC
// Returns false if packet buffer is full
static inline bool afsk_packet_add_byte(AfskDemodulator *d, uint8_t b) {
    if (d->packet_size >= AFSK_MAX_FRAME_SIZE) return false;

    afsk_crc_update(&d->packet_crc, b);
    d->packet_buf[d->packet_size] = b;
    d->packet_size++;
    return true;
}

// Check if the packet is valid (minimum size + correct CRC)
static inline bool afsk_packet_terminate(AfskDemodulator *d) {
    if (d->packet_size < AFSK_MIN_FRAME_SIZE) return false;
    return (d->packet_crc == AX25_CRC_CORRECT);
}

// ============================================================================
// Section 8: Per-Sample Processing
// ============================================================================

// Process one audio sample through the demodulator.
// This is the core inner loop — called once per sample (~48,960 times/second).
// Ported from Afsk1200Demodulator.addSamplesPrivate() lines 248-466.
static void afsk_demod_process_sample(AfskDemodulator *d, float sample) {

    // --- Step 1: Time-domain filtering ---
    d->u1[d->j_td] = sample;
    d->x[d->j_td] = afsk_fir_filter(d->u1, d->j_td, d->td_filter,
                                      AFSK_FILTER_TAPS, AFSK_FILTER_TAPS);

    // --- Step 2: Correlator products ---
    float cos_f0 = cosf(d->phase_f0);
    float sin_f0 = sinf(d->phase_f0);
    float cos_f1 = cosf(d->phase_f1);
    float sin_f1 = sinf(d->phase_f1);

    float filtered = d->x[d->j_td];
    d->c0_real[d->j_corr] = filtered * cos_f0;
    d->c0_imag[d->j_corr] = filtered * sin_f0;
    d->c1_real[d->j_corr] = filtered * cos_f1;
    d->c1_imag[d->j_corr] = filtered * sin_f1;

    // Advance phase accumulators
    d->phase_f0 += AFSK_PHASE_INC_F0;
    if (d->phase_f0 > 2.0f * (float)M_PI) d->phase_f0 -= 2.0f * (float)M_PI;
    d->phase_f1 += AFSK_PHASE_INC_F1;
    if (d->phase_f1 > 2.0f * (float)M_PI) d->phase_f1 -= 2.0f * (float)M_PI;

    // --- Step 3: Magnitude computation ---
    float cr, ci;

    cr = afsk_circular_sum(d->c0_real, d->j_corr, AFSK_CORR_LEN);
    ci = afsk_circular_sum(d->c0_imag, d->j_corr, AFSK_CORR_LEN);
    float c0 = sqrtf(cr * cr + ci * ci);

    cr = afsk_circular_sum(d->c1_real, d->j_corr, AFSK_CORR_LEN);
    ci = afsk_circular_sum(d->c1_imag, d->j_corr, AFSK_CORR_LEN);
    float c1 = sqrtf(cr * cr + ci * ci);

    // --- Step 4: Difference signal + filtering ---
    d->diff[d->j_cd] = c0 - c1;
    float fdiff = afsk_fir_filter(d->diff, d->j_cd, d->cd_filter,
                                   AFSK_FILTER_TAPS, AFSK_FILTER_TAPS);

    // --- Step 5: Zero-crossing detection ---
    if (d->previous_fdiff * fdiff < 0 || d->previous_fdiff == 0) {

        // Transition detected — measure bit period
        int p = d->t - d->last_transition;
        d->last_transition = d->t;

        int bits = (int)roundf((float)p / d->samples_per_bit);

        // --- Step 6: State machine ---
        if (bits == 0 || bits > 7) {
            // Invalid period — reset
            d->state = DEMOD_WAITING;
            d->data_carrier = false;
            d->flag_count = 0;
        } else if (bits == 7) {
            // Flag byte (0x7E) detected
            d->flag_count++;
            d->flag_separator_seen = false;
            d->data = 0;
            d->bitcount = 0;

            switch (d->state) {
                case DEMOD_WAITING:
                    d->state = DEMOD_JUST_SEEN_FLAG;
                    d->data_carrier = true;
                    break;

                case DEMOD_JUST_SEEN_FLAG:
                    // Stay in this state (consecutive flags)
                    break;

                case DEMOD_DECODING:
                    // End of packet — check CRC
                    if (d->packet_active && afsk_packet_terminate(d)) {
                        // Valid packet! Send without CRC (last 2 bytes)
                        if (d->callback != NULL && d->packet_size > 2) {
                            d->callback(d->packet_buf, d->packet_size - 2, d->emphasis);
                        }
                    }
                    d->packet_active = false;
                    d->state = DEMOD_JUST_SEEN_FLAG;
                    break;
            }
        } else {
            // Data bits (1-6)
            switch (d->state) {
                case DEMOD_WAITING:
                    break;
                case DEMOD_JUST_SEEN_FLAG:
                    d->state = DEMOD_DECODING;
                    break;
                case DEMOD_DECODING:
                    break;
            }

            if (d->state == DEMOD_DECODING) {
                // Handle flag separator tracking
                if (bits != 1) {
                    d->flag_count = 0;
                } else {
                    if (d->flag_count > 0 && !d->flag_separator_seen) {
                        d->flag_separator_seen = true;
                    } else {
                        d->flag_count = 0;
                    }
                }

                // --- Step 7: NRZ bit extraction with bit-stuffing removal ---
                // Each "no transition" period = consecutive 1-bits
                for (int k = 0; k < bits - 1; k++) {
                    d->bitcount++;
                    d->data >>= 1;
                    d->data += 128;  // set MSB (bit = 1)

                    if (d->bitcount == 8) {
                        if (!d->packet_active) {
                            afsk_packet_reset(d);
                        }
                        if (!afsk_packet_add_byte(d, (uint8_t)d->data)) {
                            d->state = DEMOD_WAITING;
                            d->data_carrier = false;
                        }
                        d->data = 0;
                        d->bitcount = 0;
                    }
                }

                // The zero-crossing itself represents a 0-bit,
                // UNLESS it's a stuffed zero (after 5 consecutive 1s)
                if (bits - 1 != 5) {
                    d->bitcount++;
                    d->data >>= 1;
                    // MSB stays 0 (bit = 0)

                    if (d->bitcount == 8) {
                        if (!d->packet_active) {
                            afsk_packet_reset(d);
                        }
                        if (!afsk_packet_add_byte(d, (uint8_t)d->data)) {
                            d->state = DEMOD_WAITING;
                            d->data_carrier = false;
                        }
                        d->data = 0;
                        d->bitcount = 0;
                    }
                }
            }
        }
    }

    // Save for next zero-crossing comparison
    d->previous_fdiff = fdiff;

    // Advance counters
    d->t++;

    d->j_td++;
    if (d->j_td >= AFSK_FILTER_TAPS) d->j_td = 0;

    d->j_cd++;
    if (d->j_cd >= AFSK_FILTER_TAPS) d->j_cd = 0;

    d->j_corr++;
    if (d->j_corr >= AFSK_CORR_LEN) d->j_corr = 0;
}
