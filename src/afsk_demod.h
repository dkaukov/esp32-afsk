/*
AFSK 1200 Demodulator for ESP32
Port of https://github.com/dkaukov/afsk-java (Demodulator + SymbolSlicerPll + NRZI + HDLC).

Pipeline:
  - Band-pass filter
  - Mix to baseband (DDS oscillator at center frequency)
  - Low-pass I/Q filters
  - FM demod via phase difference (deltaQ)
  - Symbol slicer PLL
  - NRZI decode
  - HDLC deframe + CRC verify
*/

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#if defined(AFSK_DEMOD_PROFILE) && defined(ESP32)
#include "xtensa/core-macros.h"
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

#if defined(ESP32)
#include <esp_dsp.h>
#include <dsps_fir.h>
#define AFSK_USE_ESP_DSP 1
#define AFSK_ALIGN16 __attribute__((aligned(16)))
#else
#define AFSK_USE_ESP_DSP 0
#define AFSK_ALIGN16
#endif

#if !AFSK_USE_ESP_DSP
#if defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type-c-linkage"
#include <liquid.h>
#pragma GCC diagnostic pop
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <liquid.h>
#pragma GCC diagnostic pop
#endif
#endif

#ifndef AFSK_SAMPLE_RATE
#define AFSK_SAMPLE_RATE 48000
#endif

#define AFSK_BAUD_RATE  1200.0f
#define AFSK_MARK_FREQ  1200.0f
#define AFSK_SPACE_FREQ 2200.0f

#define AFSK_MAX_FRAME_SIZE 360
#define AFSK_MIN_FRAME_SIZE 9

#define AX25_CRC_CORRECT 0xF0B8
#define CRC_CCITT_INIT_VAL 0xFFFF


#define AFSK_DEMOD_MAGIC 0x4146534BU

// Filter limits (static buffers sized for 48 kHz / 1200 baud)
#define AFSK_MAX_BPF_TAPS 61
#define AFSK_MAX_LPF_TAPS 41

// CRC-CCITT lookup table (same as ax25-java)
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
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

// ============================================================================
// FIR designer (ported from afsk-java)
// ============================================================================

static inline float afsk_bessel_i0(float x) {
    double sum = 1.0;
    double y = (x * x) / 4.0;
    double term = y;
    for (int k = 1; k < 30; k++) {
        sum += term;
        term *= y / (k * k);
    }
    return (float)sum;
}

static inline float afsk_kaiser_beta(float attenuation_db) {
    if (attenuation_db > 50.0f) {
        return 0.1102f * (attenuation_db - 8.7f);
    } else if (attenuation_db >= 21.0f) {
        return 0.5842f * powf(attenuation_db - 21.0f, 0.4f) + 0.07886f * (attenuation_db - 21.0f);
    }
    return 0.0f;
}

static inline void afsk_normalize_unity_gain(float *taps, int len) {
    float sum = 0.0f;
    for (int i = 0; i < len; i++) sum += taps[i];
    if (sum == 0.0f) return;
    float inv = 1.0f / sum;
    for (int i = 0; i < len; i++) taps[i] *= inv;
}

static void afsk_design_lowpass_hamming(float *out, int num_taps, float cutoff_hz, float sample_rate) {
    if ((num_taps & 1) == 0) num_taps++;
    float fc = cutoff_hz / sample_rate;
    int mid = num_taps / 2;
    for (int i = 0; i < num_taps; i++) {
        int n = i - mid;
        float sinc = (n == 0) ? 2.0f * fc : sinf(2.0f * (float)M_PI * fc * n) / ((float)M_PI * n);
        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / (num_taps - 1));
        out[i] = sinc * w;
    }
    afsk_normalize_unity_gain(out, num_taps);
}

static void afsk_design_bandpass_kaiser(float *out, int num_taps, float low_hz, float high_hz, float sample_rate, float attenuation_db) {
    if ((num_taps & 1) == 0) num_taps++;
    float fl = low_hz / sample_rate;
    float fh = high_hz / sample_rate;
    int mid = num_taps / 2;
    float beta = afsk_kaiser_beta(attenuation_db);
    float denom = afsk_bessel_i0(beta);
    for (int i = 0; i < num_taps; i++) {
        int n = i - mid;
        float sinc_h = (n == 0) ? 2.0f * fh : sinf(2.0f * (float)M_PI * fh * n) / ((float)M_PI * n);
        float sinc_l = (n == 0) ? 2.0f * fl : sinf(2.0f * (float)M_PI * fl * n) / ((float)M_PI * n);
        float r = (2.0f * i) / (num_taps - 1) - 1.0f;
        float w = afsk_bessel_i0(beta * sqrtf(1.0f - r * r)) / denom;
        out[i] = (sinc_h - sinc_l) * w;
    }
    // Band-pass is normalized downstream to avoid excessive tap magnitudes.
}

// ============================================================================
// FIR (liquidDSP) - float
// ============================================================================

#if AFSK_USE_ESP_DSP
typedef struct {
    fir_f32_t fir;
    float *coeffs;
    float *delay;
    int len;
} AfskFastFIR;

static void afsk_fir_init(AfskFastFIR *f, const float *taps, int len, float *taps_store, float *state_store, int state_len) {
    if (state_len < (len + 4)) return;
    f->len = len;
    f->coeffs = taps_store;
    f->delay = state_store;
    for (int i = 0; i < len; i++) taps_store[i] = taps[i];
    memset(f->delay, 0, sizeof(float) * (size_t)(len + 4));
    dsps_fir_init_f32(&f->fir, f->coeffs, f->delay, len);
}

static inline float afsk_fir_filter(AfskFastFIR *f, float x) {
    float y = 0.0f;
    dsps_fir_f32(&f->fir, &x, &y, 1);
    return y;
}
#else
typedef struct {
    firfilt_rrrf q;
    int len;
} AfskFastFIR;

static void afsk_fir_init(AfskFastFIR *f, const float *taps, int len, float *taps_store, float *state_store, int state_len) {
    (void)state_store;
    (void)state_len;
    if (f->q) {
        firfilt_rrrf_destroy(f->q);
        f->q = NULL;
    }
    f->len = len;
    for (int i = 0; i < len; i++) taps_store[i] = taps[i];
    f->q = firfilt_rrrf_create(taps_store, len);
}

static inline float afsk_fir_filter(AfskFastFIR *f, float x) {
    float y = 0.0f;
    firfilt_rrrf_push(f->q, x);
    firfilt_rrrf_execute(f->q, &y);
    return y;
}
#endif

// ============================================================================
// IIR LPF (single pole) - float
// ============================================================================

typedef struct {
    float alpha;
    float y;
} AfskIIR1;

static inline void afsk_iir_init(AfskIIR1 *f, float sample_rate, float cutoff_hz) {
    f->alpha = 1.0f - expf(-2.0f * (float)M_PI * cutoff_hz / sample_rate);
    f->y = 0.0f;
}

static inline float afsk_iir_filter(AfskIIR1 *f, float x) {
    f->y += f->alpha * (x - f->y);
    return f->y;
}

// ============================================================================
// DDS Oscillator (table-based) - float
// ============================================================================

typedef struct {
    uint32_t phase;
    uint32_t phase_step;
    int index;
} AfskDdsOsc;

#define AFSK_DDS_TABLE_BITS 9
#define AFSK_DDS_TABLE_SIZE (1 << AFSK_DDS_TABLE_BITS)
#define AFSK_DDS_TABLE_MASK (AFSK_DDS_TABLE_SIZE - 1)
#define AFSK_DDS_COS_SHIFT (AFSK_DDS_TABLE_SIZE / 4)

static float afsk_dds_table[AFSK_DDS_TABLE_SIZE];
static bool afsk_dds_table_init = false;

static void afsk_dds_init(AfskDdsOsc *osc, float sample_rate, float freq) {
    if (!afsk_dds_table_init) {
        for (int i = 0; i < AFSK_DDS_TABLE_SIZE; i++) {
            afsk_dds_table[i] = sinf(2.0f * (float)M_PI * i / (float)AFSK_DDS_TABLE_SIZE);
        }
        afsk_dds_table_init = true;
    }
    osc->phase = 0;
    osc->index = 0;
    osc->phase_step = (uint32_t)((freq * (1ull << 32)) / sample_rate);
}

static inline void afsk_dds_next(AfskDdsOsc *osc) {
    osc->phase += osc->phase_step;
    osc->index = (int)((osc->phase >> (32 - AFSK_DDS_TABLE_BITS)) & AFSK_DDS_TABLE_MASK);
}

static inline float afsk_dds_sin(const AfskDdsOsc *osc) {
    return afsk_dds_table[osc->index];
}

static inline float afsk_dds_cos(const AfskDdsOsc *osc) {
    return afsk_dds_table[(osc->index + AFSK_DDS_COS_SHIFT) & AFSK_DDS_TABLE_MASK];
}

// ============================================================================
// Symbol slicer PLL (ported from afsk-java)
// ============================================================================

typedef struct {
    float nominal_step;
    float step_min;
    float step_max;
    float kp;
    float ki;
    float phase_gain_acq;
    float lock_error_window;
    float unlock_error_window;
    int lock_consecutive;
    int unlock_bad_limit;
    float step;
    float phase;
    float integ;
    float integ_clamp;
    int prev_symbol;
    bool locked;
    int good_trans;
    int bad_trans;
} AfskSlicerPll;

#ifdef AFSK_DEMOD_STATS
typedef struct {
    float demod_min;
    float demod_max;
    float demod_sum;
    uint64_t samples;
} AfskDemodStats;
#endif

static inline float afsk_clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline int afsk_clamp_int(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline int afsk_make_odd(int v) {
    return (v & 1) ? v : (v + 1);
}

static void afsk_slicer_init(AfskSlicerPll *pll, float sample_rate, float baud_rate) {
    float nominal = baud_rate / sample_rate;
    float ppm = 5100.0f * 1e-6f;
    pll->nominal_step = nominal;
    pll->step_min = nominal * (1.0f - ppm);
    pll->step_max = nominal * (1.0f + ppm);
    pll->kp = 1.0e-4f;
    pll->ki = 8.0e-6f;
    pll->phase_gain_acq = 0.218f;
    pll->lock_error_window = 0.30f;
    pll->unlock_error_window = fminf(0.49f, pll->lock_error_window * 1.5f);
    pll->lock_consecutive = 6;
    pll->unlock_bad_limit = 3;
    pll->step = nominal;
    pll->phase = 0.5f;
    pll->integ = 0.0f;
    pll->integ_clamp = 1000.0f;
    pll->prev_symbol = 0;
    pll->locked = false;
    pll->good_trans = 0;
    pll->bad_trans = 0;
}

// ============================================================================
// NRZI + HDLC deframer + CRC verify
// ============================================================================

typedef void (*AfskPacketCallback)(const uint8_t *frame, size_t len, int emphasis);

typedef struct {
    uint8_t flag_window;
    bool in_frame;
    int one_run;
    int bit_pos;
    int current_byte;
    uint8_t frame[AFSK_MAX_FRAME_SIZE];
    int frame_size;
} AfskHdlcDeframer;

static inline void afsk_hdlc_reset(AfskHdlcDeframer *d) {
    d->flag_window = 0;
    d->in_frame = false;
    d->one_run = 0;
    d->bit_pos = 0;
    d->current_byte = 0;
    d->frame_size = 0;
}

static inline void afsk_hdlc_start(AfskHdlcDeframer *d) {
    d->in_frame = true;
    d->one_run = 0;
    d->bit_pos = 0;
    d->current_byte = 0;
    d->frame_size = 0;
}

static inline void afsk_hdlc_drop(AfskHdlcDeframer *d) {
    d->in_frame = false;
    d->one_run = 0;
    d->bit_pos = 0;
    d->current_byte = 0;
    d->frame_size = 0;
}

static inline void afsk_hdlc_add_bit(AfskHdlcDeframer *d, int bit) {
    d->current_byte |= (bit << d->bit_pos);
    d->bit_pos++;
    if (d->bit_pos == 8) {
        if (d->frame_size < AFSK_MAX_FRAME_SIZE) {
            d->frame[d->frame_size++] = (uint8_t)d->current_byte;
        }
        d->current_byte = 0;
        d->bit_pos = 0;
    }
}

static inline uint16_t afsk_crc_calc(const uint8_t *data, size_t len) {
    uint16_t crc = CRC_CCITT_INIT_VAL;
    for (size_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc_ccitt_tab[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

#ifdef AFSK_DEMOD_PROFILE
typedef struct {
    uint64_t samples;
    uint64_t cycles_total;
    uint64_t cycles_bpf;
    uint64_t cycles_mix;
    uint64_t cycles_lpf_i;
    uint64_t cycles_lpf_q;
    uint64_t cycles_demod;
    uint64_t cycles_slicer;
} AfskDemodProfile;

static inline uint32_t afsk_ccount(void) {
#if defined(ESP32)
    return XTHAL_GET_CCOUNT();
#else
    return 0;
#endif
}
#endif


// ============================================================================
// Demodulator state
// ============================================================================

typedef struct {
    uint32_t magic;
    int emphasis;
    AfskPacketCallback callback;

    AfskDdsOsc osc;
    AfskFastFIR bpf;
    AfskFastFIR i_filt;
    AfskFastFIR q_filt;
    float bpf_taps[AFSK_MAX_BPF_TAPS] AFSK_ALIGN16;
    float bpf_state[AFSK_MAX_BPF_TAPS + 4] AFSK_ALIGN16;
    float lpf_taps[AFSK_MAX_LPF_TAPS] AFSK_ALIGN16;
    float i_state[AFSK_MAX_LPF_TAPS + 4] AFSK_ALIGN16;
    float q_state[AFSK_MAX_LPF_TAPS + 4] AFSK_ALIGN16;
    AfskIIR1 out_lp;
    float norm_gain;
    float prev_i;
    float prev_q;

    AfskSlicerPll slicer;
    int nrzi_last;

    AfskHdlcDeframer hdlc;

#ifdef AFSK_DEMOD_STATS
    AfskDemodStats stats;
#endif
#ifdef AFSK_DEMOD_PROFILE
    AfskDemodProfile profile;
#endif
} AfskDemodulator;

// ============================================================================
// Init
// ============================================================================

static void afsk_demod_deinit(AfskDemodulator *d) {
    if (d == NULL) return;
    if (d->magic != AFSK_DEMOD_MAGIC) return;
#if !AFSK_USE_ESP_DSP
    if (d->bpf.q) {
        firfilt_rrrf_destroy(d->bpf.q);
        d->bpf.q = NULL;
    }
    if (d->i_filt.q) {
        firfilt_rrrf_destroy(d->i_filt.q);
        d->i_filt.q = NULL;
    }
    if (d->q_filt.q) {
        firfilt_rrrf_destroy(d->q_filt.q);
        d->q_filt.q = NULL;
    }
#endif
    d->magic = 0;
}

static void afsk_demod_init(AfskDemodulator *d, int emphasis, AfskPacketCallback callback) {
    memset(d, 0, sizeof(AfskDemodulator));
    d->magic = AFSK_DEMOD_MAGIC;
    d->emphasis = emphasis;
    d->callback = callback;
    float sample_rate = (float)AFSK_SAMPLE_RATE;
    float baud = AFSK_BAUD_RATE;
    float center = (AFSK_MARK_FREQ + AFSK_SPACE_FREQ) * 0.5f;
    float dev = 0.5f * (AFSK_SPACE_FREQ - AFSK_MARK_FREQ);

    afsk_dds_init(&d->osc, sample_rate, center);

    const float bpf_scale = 0.95f;
    int bpf_len = (int)lroundf(sample_rate / baud * 1.425f * bpf_scale);
    bpf_len = afsk_make_odd(bpf_len);
    bpf_len = afsk_clamp_int(bpf_len, 9, AFSK_MAX_BPF_TAPS | 1);
    afsk_design_bandpass_kaiser(d->bpf_taps, bpf_len, AFSK_MARK_FREQ, AFSK_SPACE_FREQ, sample_rate, 30.0f);
    afsk_fir_init(&d->bpf, d->bpf_taps, bpf_len, d->bpf_taps, d->bpf_state, AFSK_MAX_BPF_TAPS + 4);

    const float lpf_scale = 0.95f;
    int lpf_len = (int)lroundf(sample_rate / baud * 0.875f * lpf_scale);
    lpf_len = afsk_make_odd(lpf_len);
    lpf_len = afsk_clamp_int(lpf_len, 7, AFSK_MAX_LPF_TAPS | 1);
    afsk_design_lowpass_hamming(d->lpf_taps, lpf_len, dev, sample_rate);
    afsk_fir_init(&d->i_filt, d->lpf_taps, lpf_len, d->lpf_taps, d->i_state, AFSK_MAX_LPF_TAPS + 4);
    afsk_fir_init(&d->q_filt, d->lpf_taps, lpf_len, d->lpf_taps, d->q_state, AFSK_MAX_LPF_TAPS + 4);

    d->norm_gain = 1.0f / (2.0f * (float)M_PI * (dev / sample_rate));
    afsk_iir_init(&d->out_lp, sample_rate, baud * 3.0f);
    d->prev_i = 0.0f;
    d->prev_q = 0.0f;

    afsk_slicer_init(&d->slicer, sample_rate, baud);
    d->nrzi_last = 0;
    afsk_hdlc_reset(&d->hdlc);

#ifdef AFSK_DEMOD_STATS
    d->stats.demod_min = 1e9f;
    d->stats.demod_max = -1e9f;
    d->stats.demod_sum = 0.0f;
    d->stats.samples = 0;
#endif
}

// ============================================================================
// Per-sample processing
// ============================================================================

static inline int afsk_nrzi_decode(AfskDemodulator *d, int bit) {
    int out = (bit == d->nrzi_last) ? 1 : 0;
    d->nrzi_last = bit;
    return out;
}

static void afsk_hdlc_process_bit(AfskDemodulator *d, int bit) {
    const uint8_t FLAG = 0x7E;
    AfskHdlcDeframer *h = &d->hdlc;

    h->flag_window = (uint8_t)((h->flag_window << 1) | (bit & 1));
    if (h->flag_window == FLAG) {
        if (h->in_frame && h->bit_pos == 7 && h->frame_size >= AFSK_MIN_FRAME_SIZE) {
            if (afsk_crc_calc(h->frame, (size_t)h->frame_size) == AX25_CRC_CORRECT) {
                if (d->callback && h->frame_size > 2) {
                    d->callback(h->frame, (size_t)h->frame_size - 2, d->emphasis);
                }
            }
        }
        afsk_hdlc_start(h);
        return;
    }

    if (!h->in_frame) return;

    if (bit == 1) {
        if (++h->one_run >= 7) {
            afsk_hdlc_drop(h);
            return;
        }
    } else if (h->one_run == 5) {
        h->one_run = 0;
        return;
    } else {
        h->one_run = 0;
    }

    afsk_hdlc_add_bit(h, bit);
}

static void afsk_slicer_process(AfskDemodulator *d, float sample) {
    AfskSlicerPll *pll = &d->slicer;
    const int symbol = (sample > 0.0f) ? 1 : 0;

    pll->phase += pll->step;
    while (pll->phase >= 1.0f) {
        pll->phase -= 1.0f;
        int decoded = afsk_nrzi_decode(d, symbol);
        afsk_hdlc_process_bit(d, decoded);
    }

    if (symbol != pll->prev_symbol) {
        float error = pll->phase - 0.5f;
        error = afsk_clamp(error, -0.5f, 0.5f);

        if (fabsf(error) <= pll->lock_error_window) {
            pll->good_trans++;
            pll->bad_trans = 0;
            if (!pll->locked && pll->good_trans >= pll->lock_consecutive) {
                pll->locked = true;
                pll->integ *= 0.5f;
            }
        } else {
            pll->good_trans = 0;
            pll->bad_trans++;
            if (pll->locked && fabsf(error) >= pll->unlock_error_window && pll->bad_trans >= pll->unlock_bad_limit) {
                pll->locked = false;
                pll->integ = 0.0f;
            }
        }

        pll->integ = afsk_clamp(pll->integ + error, -pll->integ_clamp, pll->integ_clamp);
        float delta = pll->kp * error + pll->ki * pll->integ;
        pll->step = afsk_clamp(pll->step - delta, pll->step_min, pll->step_max);
        pll->phase -= pll->phase_gain_acq * error;
    }

    pll->prev_symbol = symbol;
}

static void afsk_demod_process_sample(AfskDemodulator *d, float sample) {
#ifdef AFSK_DEMOD_PROFILE
    uint32_t t_start = afsk_ccount();
    uint32_t t0 = t_start;
    uint32_t t1 = t_start;
#endif
    float s = afsk_fir_filter(&d->bpf, sample);
#ifdef AFSK_DEMOD_PROFILE
    t1 = afsk_ccount();
    d->profile.cycles_bpf += (uint32_t)(t1 - t0);
    t0 = t1;
#endif
    float mixed_i = s * afsk_dds_cos(&d->osc);
    float mixed_q = -s * afsk_dds_sin(&d->osc);
#ifdef AFSK_DEMOD_PROFILE
    t1 = afsk_ccount();
    d->profile.cycles_mix += (uint32_t)(t1 - t0);
    t0 = t1;
#endif
    float fi = afsk_fir_filter(&d->i_filt, mixed_i);
#ifdef AFSK_DEMOD_PROFILE
    t1 = afsk_ccount();
    d->profile.cycles_lpf_i += (uint32_t)(t1 - t0);
    t0 = t1;
#endif
    float fq = afsk_fir_filter(&d->q_filt, mixed_q);
#ifdef AFSK_DEMOD_PROFILE
    t1 = afsk_ccount();
    d->profile.cycles_lpf_q += (uint32_t)(t1 - t0);
    t0 = t1;
#endif
    float delta_q = (fq * d->prev_i) - (fi * d->prev_q);
    float mag_sq = (fi * fi) + (fq * fq);
    float demod = 0.0f;
    if (mag_sq > 0.0f) {
        demod = (delta_q / mag_sq) * d->norm_gain;
    }
    demod = afsk_iir_filter(&d->out_lp, demod);
#ifdef AFSK_DEMOD_STATS
    if (demod < d->stats.demod_min) d->stats.demod_min = demod;
    if (demod > d->stats.demod_max) d->stats.demod_max = demod;
    d->stats.demod_sum += demod;
    d->stats.samples++;
#endif
    if (demod > -0.006f && demod < 0.006f) {
        demod = 1.0f;
    }
    d->prev_i = fi;
    d->prev_q = fq;
    afsk_dds_next(&d->osc);
#ifdef AFSK_DEMOD_PROFILE
    t1 = afsk_ccount();
    d->profile.cycles_demod += (uint32_t)(t1 - t0);
    t0 = t1;
#endif
    afsk_slicer_process(d, demod);
#ifdef AFSK_DEMOD_PROFILE
    t1 = afsk_ccount();
    d->profile.cycles_slicer += (uint32_t)(t1 - t0);
    d->profile.cycles_total += (uint32_t)(t1 - t_start);
    d->profile.samples++;
#endif
}

static inline void afsk_demod_process_samples_f32(AfskDemodulator *d, const float *samples, size_t count) {
    if (!samples || count == 0) return;
    for (size_t i = 0; i < count; i++) {
        afsk_demod_process_sample(d, samples[i]);
    }
}

static inline void afsk_demod_process_samples_i16(AfskDemodulator *d, const int16_t *samples, size_t count) {
    if (!samples || count == 0) return;
    for (size_t i = 0; i < count; i++) {
        float s = (float)samples[i] / 32768.0f;
        afsk_demod_process_sample(d, s);
    }
}
