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

#include "AfskCrc.h"

#define AFSK_ALIGN16 __attribute__((aligned(16)))
#include <esp_dsp.h>
#include <dsps_fir.h>

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef AFSK_SAMPLE_RATE
#define AFSK_SAMPLE_RATE 48000
#endif

#ifndef AFSK_DECIM_FACTOR
#define AFSK_DECIM_FACTOR 4
#endif

#ifndef AFSK_DECIM_OUT_SAMPLES
#define AFSK_DECIM_OUT_SAMPLES 24
#endif




// ============================================================================
// FIR designer (ported from afsk-java)
// ============================================================================

namespace afsk {
namespace dsp {

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
// FIR - float
// ============================================================================

class Esp32Fir {
public:
    void init(const float *taps, int len_in, float *taps_store, float *state_store, int state_len) {
        if (state_len < len_in + 4) {
            len = 0;
            return;
        }
        len = len_in;
        coeffs = taps_store;
        delay = state_store;
        for (int i = 0; i < len; i++) coeffs[i] = taps[i];
        memset(delay, 0, sizeof(float) * (size_t)state_len);
        dsps_fir_init_f32(&fir, coeffs, delay, len);
    }

    void initDecim(const float *taps, int len_in, float *taps_store, float *state_store, int state_len, int decim) {
        if (state_len < len_in + 4 || decim < 1) {
            len = 0;
            return;
        }
        len = len_in;
        coeffs = taps_store;
        delay = state_store;
        for (int i = 0; i < len; i++) coeffs[i] = taps[i];
        memset(delay, 0, sizeof(float) * (size_t)state_len);
        dsps_fird_init_f32(&fir, coeffs, delay, len, decim);
    }

    inline float filter(float x) {
        float y = 0.0f;
        dsps_fir_f32(&fir, &x, &y, 1);
        return y;
    }

    fir_f32_t fir;
    float *coeffs;
    float *delay;
    int len;
};

// ============================================================================
// IIR LPF (single pole) - float
// ============================================================================

class Iir1 {
public:
    void init(float sample_rate, float cutoff_hz) {
        alpha = 1.0f - expf(-2.0f * (float)M_PI * cutoff_hz / sample_rate);
        y = 0.0f;
    }

    inline float filter(float x) {
        y += alpha * (x - y);
        return y;
    }

    float alpha;
    float y;
};

// ============================================================================
// DDS Oscillator (table-based) - float
// ============================================================================

static constexpr int AFSK_DDS_TABLE_BITS = 9;
static constexpr int AFSK_DDS_TABLE_SIZE = 1 << AFSK_DDS_TABLE_BITS;
static constexpr int AFSK_DDS_TABLE_MASK = AFSK_DDS_TABLE_SIZE - 1;
static constexpr int AFSK_DDS_COS_SHIFT = AFSK_DDS_TABLE_SIZE / 4;

static float afsk_dds_table[AFSK_DDS_TABLE_SIZE];
static bool afsk_dds_table_init = false;

class DdsOsc {
public:
    void init(float sample_rate, float freq) {
        if (!afsk_dds_table_init) {
            for (int i = 0; i < AFSK_DDS_TABLE_SIZE; i++) {
                afsk_dds_table[i] = sinf(2.0f * (float)M_PI * i / (float)AFSK_DDS_TABLE_SIZE);
            }
            afsk_dds_table_init = true;
        }
        phase = 0;
        index = 0;
        phase_step = (uint32_t)((freq * (1ull << 32)) / sample_rate);
    }

    inline void next() {
        phase += phase_step;
        index = (int)((phase >> (32 - AFSK_DDS_TABLE_BITS)) & AFSK_DDS_TABLE_MASK);
    }

    inline float sin() const {
        return afsk_dds_table[index];
    }

    inline float cos() const {
        return afsk_dds_table[(index + AFSK_DDS_COS_SHIFT) & AFSK_DDS_TABLE_MASK];
    }

    uint32_t phase;
    uint32_t phase_step;
    int index;
};

}  // namespace dsp
}  // namespace afsk


// ============================================================================
// Symbol slicer PLL (ported from afsk-java)
// ============================================================================

#ifdef AFSK_DEMOD_STATS
typedef struct {
    float demod_min;
    float demod_max;
    float demod_sum;
    uint64_t samples;
} AfskDemodStats;
#endif

namespace afsk {
namespace detail {

static constexpr float AFSK_BAUD_RATE = 1200.0f;
static constexpr float AFSK_MARK_FREQ = 1200.0f;
static constexpr float AFSK_SPACE_FREQ = 2200.0f;

static constexpr int AFSK_MAX_FRAME_SIZE = 360;
static constexpr int AFSK_MIN_FRAME_SIZE = 9;

// Filter limits (static buffers sized for 48 kHz / 1200 baud)
static constexpr int AFSK_MAX_BPF_TAPS = 61;
static constexpr int AFSK_MAX_LPF_TAPS = 41;

static constexpr int AFSK_DECIM_IN_SAMPLES_MAX = AFSK_DECIM_OUT_SAMPLES * AFSK_DECIM_FACTOR;

class AfskSlicerPll {
public:
    void init(float sample_rate, float baud_rate) {
        float nominal = baud_rate / sample_rate;
        float ppm = 5100.0f * 1e-6f;
        float rate_scale = 48000.0f / sample_rate;
        nominal_step = nominal;
        step_min = nominal * (1.0f - ppm);
        step_max = nominal * (1.0f + ppm);
        kp = 1.0e-4f * rate_scale;
        ki = 8.0e-6f * rate_scale;
        phase_gain_acq = 0.218f;
        lock_error_window = 0.30f;
        unlock_error_window = fminf(0.49f, lock_error_window * 1.5f);
        lock_consecutive = 6;
        unlock_bad_limit = 3;
        step = nominal;
        phase = 0.5f;
        integ = 0.0f;
        integ_clamp = 1000.0f;
        prev_symbol = 0;
        locked = false;
        good_trans = 0;
        bad_trans = 0;
    }

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
};

static inline float afsk_clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline int afsk_clamp_int(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline int afsk_make_odd(int v) {
    return (v & 1) ? v : (v + 1);
}

// ============================================================================
// NRZI + HDLC deframer + CRC verify
// ============================================================================

typedef struct {
    uint8_t flag_window;
    bool in_frame;
    int one_run;
    int bit_pos;
    int current_byte;
    uint8_t frame[afsk::detail::AFSK_MAX_FRAME_SIZE];
    int frame_size;
} AfskHdlcDeframer;

static inline void afsk_hdlc_reset(AfskHdlcDeframer &d) {
    d.flag_window = 0;
    d.in_frame = false;
    d.one_run = 0;
    d.bit_pos = 0;
    d.current_byte = 0;
    d.frame_size = 0;
}

static inline void afsk_hdlc_start(AfskHdlcDeframer &d) {
    d.in_frame = true;
    d.one_run = 0;
    d.bit_pos = 0;
    d.current_byte = 0;
    d.frame_size = 0;
}

static inline void afsk_hdlc_drop(AfskHdlcDeframer &d) {
    d.in_frame = false;
    d.one_run = 0;
    d.bit_pos = 0;
    d.current_byte = 0;
    d.frame_size = 0;
}

static inline void afsk_hdlc_add_bit(AfskHdlcDeframer &d, int bit) {
    d.current_byte |= (bit << d.bit_pos);
    d.bit_pos++;
    if (d.bit_pos == 8) {
        if (d.frame_size < afsk::detail::AFSK_MAX_FRAME_SIZE) {
            d.frame[d.frame_size++] = (uint8_t)d.current_byte;
        }
        d.current_byte = 0;
        d.bit_pos = 0;
    }
}

}  // namespace detail
}  // namespace afsk

// ============================================================================
// NRZI + HDLC deframer + CRC verify (public callback type)
// ============================================================================

typedef void (*AfskPacketCallback)(const uint8_t *frame, size_t len);


// ============================================================================
// Demodulator class
// ============================================================================

class AfskDemodulator {
public:
    AfskDemodulator(int sample_rate = AFSK_SAMPLE_RATE,
                    int decim = AFSK_DECIM_FACTOR,
                    AfskPacketCallback callback = nullptr) {
        init((float)sample_rate, decim, callback);
    }

    void processSamples(const int16_t *samples, size_t count) {
        if (!samples || count == 0) return;
        if (decim > 1) {
            processSamplesI16Decim(samples, count);
            return;
        }
        processSamplesI16Block(samples, count);
    }

    void processSamples(const float *samples, size_t count) {
        if (!samples || count == 0) return;
        if (decim > 1) {
            processSamplesF32Decim(samples, count);
            return;
        }
        processSamplesF32Block(samples, count);
    }

    void processSample(float sample) {
        processSampleRaw(sample);
    }

    void flush() {
        if (decim > 1) {
            flushDecim();
        } else {
            flushBlock();
        }
    }

#ifdef AFSK_DEMOD_STATS
    const AfskDemodStats &getStats() const { return stats; }
#endif

private:
    void init(float sample_rate, int decim_factor, AfskPacketCallback callback_fn) {
        memset(this, 0, sizeof(AfskDemodulator));
        callback = callback_fn;
        if (decim_factor < 1) decim_factor = 1;
        decim = decim_factor;
        demod_sample_rate = sample_rate / (float)decim;
        dc_alpha = 1.0f - expf(-1.0f / (demod_sample_rate * 0.25f));
        dc_prev = 0.0f;
        float baud = afsk::detail::AFSK_BAUD_RATE;
        float center = (afsk::detail::AFSK_MARK_FREQ + afsk::detail::AFSK_SPACE_FREQ) * 0.5f;
        float dev = 0.5f * (afsk::detail::AFSK_SPACE_FREQ - afsk::detail::AFSK_MARK_FREQ);

        float osc_rate = (decim > 1) ? demod_sample_rate : sample_rate;
        osc.init(osc_rate, center);

        int bpf_len = (int)lroundf(sample_rate / baud * 1.425f);
        bpf_len = afsk::detail::afsk_make_odd(bpf_len);
        bpf_len = afsk::detail::afsk_clamp_int(bpf_len, 9, afsk::detail::AFSK_MAX_BPF_TAPS | 1);
        afsk::dsp::afsk_design_bandpass_kaiser(bpf_taps, bpf_len, afsk::detail::AFSK_MARK_FREQ, afsk::detail::AFSK_SPACE_FREQ, sample_rate, 30.0f);
        if (decim > 1) {
            bpf.initDecim(bpf_taps, bpf_len, bpf_taps, bpf_state, afsk::detail::AFSK_MAX_BPF_TAPS + 4, decim);
        } else {
            bpf.init(bpf_taps, bpf_len, bpf_taps, bpf_state, afsk::detail::AFSK_MAX_BPF_TAPS + 4);
        }

        int lpf_len = (int)lroundf(demod_sample_rate / baud * 0.875f);
        if (decim > 1 && lpf_len < 13) lpf_len = 13;
        lpf_len = afsk::detail::afsk_make_odd(lpf_len);
        lpf_len = afsk::detail::afsk_clamp_int(lpf_len, 7, afsk::detail::AFSK_MAX_LPF_TAPS | 1);
        afsk::dsp::afsk_design_lowpass_hamming(lpf_taps, lpf_len, dev, demod_sample_rate);
        i_filt.init(lpf_taps, lpf_len, lpf_taps, i_state, afsk::detail::AFSK_MAX_LPF_TAPS + 4);
        q_filt.init(lpf_taps, lpf_len, lpf_taps, q_state, afsk::detail::AFSK_MAX_LPF_TAPS + 4);
        i_decim.initDecim(lpf_taps, lpf_len, lpf_taps, i_decim_state, afsk::detail::AFSK_MAX_LPF_TAPS + 4, decim);
        q_decim.initDecim(lpf_taps, lpf_len, lpf_taps, q_decim_state, afsk::detail::AFSK_MAX_LPF_TAPS + 4, decim);

        norm_gain = 1.0f / (2.0f * (float)M_PI * (dev / demod_sample_rate));
        out_lp.init(demod_sample_rate, baud * 3.0f);
        prev_i = 0.0f;
        prev_q = 0.0f;

        slicer.init(demod_sample_rate, baud);
        nrzi_last = 0;
        afsk::detail::afsk_hdlc_reset(hdlc);

#ifdef AFSK_DEMOD_STATS
        stats.demod_min = 1e9f;
        stats.demod_max = -1e9f;
        stats.demod_sum = 0.0f;
        stats.samples = 0;
#endif
    }

    int nrziDecode(int bit) {
        int out = (bit == nrzi_last) ? 1 : 0;
        nrzi_last = bit;
        return out;
    }

    void hdlcProcessBit(int bit) {
        const uint8_t FLAG = 0x7E;
        afsk::detail::AfskHdlcDeframer &h = hdlc;

        h.flag_window = (uint8_t)((h.flag_window << 1) | (bit & 1));
        if (h.flag_window == FLAG) {
        if (h.in_frame && h.bit_pos == 7 && h.frame_size >= afsk::detail::AFSK_MIN_FRAME_SIZE) {
                if (afsk::crc::calc(h.frame, (size_t)h.frame_size) == afsk::crc::AX25_CRC_CORRECT) {
                    if (callback && h.frame_size > 2) {
                        callback(h.frame, (size_t)h.frame_size - 2);
                    }
                }
            }
            afsk::detail::afsk_hdlc_start(h);
            return;
        }

        if (!h.in_frame) return;

        if (bit == 1) {
            if (++h.one_run >= 7) {
                afsk::detail::afsk_hdlc_drop(h);
                return;
            }
        } else if (h.one_run == 5) {
            h.one_run = 0;
            return;
        } else {
            h.one_run = 0;
        }

        afsk::detail::afsk_hdlc_add_bit(h, bit);
    }

    void slicerProcess(float sample) {
        afsk::detail::AfskSlicerPll &pll = slicer;
        const int symbol = (sample > 0.0f) ? 1 : 0;

        pll.phase += pll.step;
        while (pll.phase >= 1.0f) {
            pll.phase -= 1.0f;
            int decoded = nrziDecode(symbol);
            hdlcProcessBit(decoded);
        }

        if (symbol != pll.prev_symbol) {
            float error = pll.phase - 0.5f;
            error = afsk::detail::afsk_clamp(error, -0.5f, 0.5f);

            if (fabsf(error) <= pll.lock_error_window) {
                pll.good_trans++;
                pll.bad_trans = 0;
                if (!pll.locked && pll.good_trans >= pll.lock_consecutive) {
                    pll.locked = true;
                    pll.integ *= 0.5f;
                }
            } else {
                pll.good_trans = 0;
                pll.bad_trans++;
                if (pll.locked && fabsf(error) >= pll.unlock_error_window && pll.bad_trans >= pll.unlock_bad_limit) {
                    pll.locked = false;
                    pll.integ = 0.0f;
                }
            }

            pll.integ = afsk::detail::afsk_clamp(pll.integ + error, -pll.integ_clamp, pll.integ_clamp);
            float delta = pll.kp * error + pll.ki * pll.integ;
            pll.step = afsk::detail::afsk_clamp(pll.step - delta, pll.step_min, pll.step_max);
            pll.phase -= pll.phase_gain_acq * error;
        }

        pll.prev_symbol = symbol;
    }

    void processIQ(float fi, float fq) {
        float delta_q = (fq * prev_i) - (fi * prev_q);
        float mag_sq = (fi * fi) + (fq * fq);
        float demod = 0.0f;
        if (mag_sq > 0.0f) {
            demod = (delta_q / mag_sq) * norm_gain;
        }
        demod = out_lp.filter(demod);
#ifdef AFSK_DEMOD_STATS
        if (demod < stats.demod_min) stats.demod_min = demod;
        if (demod > stats.demod_max) stats.demod_max = demod;
        stats.demod_sum += demod;
        stats.samples++;
#endif
        float dz = (decim > 1) ? 0.005f : 0.006f;
        if (demod > -dz && demod < dz) {
            demod = 1.0f;
        }
        prev_i = fi;
        prev_q = fq;
        slicerProcess(demod);
    }

    void processSampleRaw(float sample) {
        float s = bpf.filter(sample);
        s = dcBlock(s);
        float mixed_i = s * osc.cos();
        float mixed_q = -s * osc.sin();
        float fi = i_filt.filter(mixed_i);
        float fq = q_filt.filter(mixed_q);
        osc.next();
        processIQ(fi, fq);
    }

    void processBlockF32(size_t n) {
        if (n == 0) return;
        dsps_fir_f32(&bpf.fir, decim_in, decim_in, (int)n);
        uint32_t phase = osc.phase;
        const uint32_t step = osc.phase_step;
        for (size_t i = 0; i < n; i++) {
            int index = (int)((phase >> (32 - afsk::dsp::AFSK_DDS_TABLE_BITS)) & afsk::dsp::AFSK_DDS_TABLE_MASK);
            float s = dcBlock(decim_in[i]);
            decim_mix[i] = s * afsk::dsp::afsk_dds_table[(index + afsk::dsp::AFSK_DDS_COS_SHIFT) & afsk::dsp::AFSK_DDS_TABLE_MASK];
            decim_q_out[i] = -s * afsk::dsp::afsk_dds_table[index];
            phase += step;
        }
        osc.phase = phase;
        osc.index = (int)((osc.phase >> (32 - afsk::dsp::AFSK_DDS_TABLE_BITS)) & afsk::dsp::AFSK_DDS_TABLE_MASK);
        dsps_fir_f32(&i_filt.fir, decim_mix, decim_i_out, (int)n);
        dsps_fir_f32(&q_filt.fir, decim_q_out, decim_q_out, (int)n);
        for (size_t i = 0; i < n; i++) {
            processIQ(decim_i_out[i], decim_q_out[i]);
        }
    }

    void processSamplesI16Block(const int16_t *samples, size_t count) {
        size_t offset = 0;
        while (offset < count) {
            size_t need = afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX - decim_fill;
            size_t take = count - offset;
            if (take > need) take = need;
            for (size_t i = 0; i < take; i++) {
                decim_in[decim_fill + i] = (float)samples[offset + i] / 32768.0f;
            }
            decim_fill += take;
            offset += take;
            if (decim_fill < afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX) continue;
            processBlockF32(afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX);
            decim_fill = 0;
        }
    }

    void processSamplesF32Block(const float *samples, size_t count) {
        size_t offset = 0;
        while (offset < count) {
            size_t need = afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX - decim_fill;
            size_t take = count - offset;
            if (take > need) take = need;
            for (size_t i = 0; i < take; i++) {
                decim_in[decim_fill + i] = samples[offset + i];
            }
            decim_fill += take;
            offset += take;
            if (decim_fill < afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX) continue;
            processBlockF32(afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX);
            decim_fill = 0;
        }
    }

    void processSamplesI16Decim(const int16_t *samples, size_t count) {
        if (!samples || count == 0) return;
        size_t offset = 0;
        while (offset < count) {
            size_t need = afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX - decim_fill;
            size_t take = count - offset;
            if (take > need) take = need;
            for (size_t i = 0; i < take; i++) {
                decim_in[decim_fill + i] = (float)samples[offset + i] / 32768.0f;
            }
            decim_fill += take;
            offset += take;
            if (decim_fill < afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX) continue;

            int out_len = (int)(afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX / decim);
            dsps_fird_f32(&bpf.fir, decim_in, decim_i_out, out_len);
            for (int i = 0; i < out_len; i++) {
                float s = dcBlock(decim_i_out[i]);
                float mixed_i = s * osc.cos();
                float mixed_q = -s * osc.sin();
                float fi = i_filt.filter(mixed_i);
                float fq = q_filt.filter(mixed_q);
                osc.next();
                processIQ(fi, fq);
            }
            decim_fill = 0;
        }
    }

    void processSamplesF32Decim(const float *samples, size_t count) {
        if (!samples || count == 0) return;
        size_t offset = 0;
        while (offset < count) {
            size_t need = afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX - decim_fill;
            size_t take = count - offset;
            if (take > need) take = need;
            for (size_t i = 0; i < take; i++) {
                decim_in[decim_fill + i] = samples[offset + i];
            }
            decim_fill += take;
            offset += take;
            if (decim_fill < afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX) continue;

            int out_len = (int)(afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX / decim);
            dsps_fird_f32(&bpf.fir, decim_in, decim_i_out, out_len);
            for (int i = 0; i < out_len; i++) {
                float s = dcBlock(decim_i_out[i]);
                float mixed_i = s * osc.cos();
                float mixed_q = -s * osc.sin();
                float fi = i_filt.filter(mixed_i);
                float fq = q_filt.filter(mixed_q);
                osc.next();
                processIQ(fi, fq);
            }
            decim_fill = 0;
        }
    }

    void flushDecim() {
        if (decim_fill == 0) return;
        while (decim_fill < afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX) {
            decim_in[decim_fill++] = 0.0f;
        }

        int out_len = (int)(afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX / decim);
        dsps_fird_f32(&bpf.fir, decim_in, decim_i_out, out_len);
        for (int i = 0; i < out_len; i++) {
            float s = dcBlock(decim_i_out[i]);
            float mixed_i = s * osc.cos();
            float mixed_q = -s * osc.sin();
            float fi = i_filt.filter(mixed_i);
            float fq = q_filt.filter(mixed_q);
            osc.next();
            processIQ(fi, fq);
        }
        decim_fill = 0;
    }

    void flushBlock() {
        if (decim_fill == 0) return;
        processBlockF32(decim_fill);
        decim_fill = 0;
    }

    inline float dcBlock(float x) {
        dc_prev += dc_alpha * (x - dc_prev);
        return x - dc_prev;
    }
    AfskPacketCallback callback;
    int decim;
    float demod_sample_rate;
    float dc_alpha;
    float dc_prev;

    afsk::dsp::DdsOsc osc;
    afsk::dsp::Esp32Fir bpf;
    afsk::dsp::Esp32Fir i_filt;
    afsk::dsp::Esp32Fir q_filt;
    afsk::dsp::Esp32Fir i_decim;
    afsk::dsp::Esp32Fir q_decim;
    float bpf_taps[afsk::detail::AFSK_MAX_BPF_TAPS] AFSK_ALIGN16;
    float bpf_state[afsk::detail::AFSK_MAX_BPF_TAPS + 4] AFSK_ALIGN16;
    float lpf_taps[afsk::detail::AFSK_MAX_LPF_TAPS] AFSK_ALIGN16;
    float i_state[afsk::detail::AFSK_MAX_LPF_TAPS + 4] AFSK_ALIGN16;
    float q_state[afsk::detail::AFSK_MAX_LPF_TAPS + 4] AFSK_ALIGN16;
    float i_decim_state[afsk::detail::AFSK_MAX_LPF_TAPS + 4] AFSK_ALIGN16;
    float q_decim_state[afsk::detail::AFSK_MAX_LPF_TAPS + 4] AFSK_ALIGN16;
    afsk::dsp::Iir1 out_lp;
    float norm_gain;
    float prev_i;
    float prev_q;
    size_t decim_fill;
    float decim_in[afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX] AFSK_ALIGN16;
    float decim_mix[afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX] AFSK_ALIGN16;
    float decim_i_out[afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX] AFSK_ALIGN16;
    float decim_q_out[afsk::detail::AFSK_DECIM_IN_SAMPLES_MAX] AFSK_ALIGN16;

    afsk::detail::AfskSlicerPll slicer;
    int nrzi_last;
    afsk::detail::AfskHdlcDeframer hdlc;

#ifdef AFSK_DEMOD_STATS
    AfskDemodStats stats;
#endif
};
