#include <Arduino.h>
#include <esp_timer.h>

#include "unity.h"

#define AFSK_DEMOD_PROFILE 1

#include "afsk_demod.h"
#include "embedded_audio.h"

static AfskDemodulator g_demod;
static volatile uint32_t g_packets = 0;

static void on_packet_decoded(const uint8_t *, size_t, int) {
    g_packets++;
}

static void test_decoder_embedded(void) {
    TEST_MESSAGE("Starting embedded decoder benchmark...");
    g_packets = 0;
    afsk_demod_init(&g_demod, 0, on_packet_decoded);

    int64_t start = esp_timer_get_time();
    afsk_demod_process_samples_i16(&g_demod, EMBEDDED_AUDIO, EMBEDDED_AUDIO_SAMPLES);
    int64_t end = esp_timer_get_time();
    TEST_MESSAGE("DONE");

    const uint64_t decode_us = (uint64_t)(end - start);
    const double audio_sec = (double)EMBEDDED_AUDIO_SAMPLES / (double)AFSK_SAMPLE_RATE;
    const float decode_sec = (float)decode_us / 1000000.0f;
    const double rt_factor = decode_sec > 0.0 ? (audio_sec / decode_sec) : 0.0;
    const double cpu_pct = audio_sec > 0.0 ? (decode_sec / audio_sec * 100.0) : 0.0;

    char msg[256];
    snprintf(msg, sizeof(msg),
             "\n\nRESULTS\n"
             "metric        value\n"
             "---------------------------\n"
             "packets       %lu\n"
             "decode_sec    %.3f\n"
             "audio_sec     %.3f\n"
             "rt_factor     %.3f\n"
             "cpu_pct       %.2f\n",
             (unsigned long)g_packets,
             decode_sec,
             audio_sec,
             rt_factor,
             cpu_pct);
    //TEST_MESSAGE(msg);
    Serial.println(msg);

#ifdef AFSK_DEMOD_PROFILE
    const uint64_t cycles_total = g_demod.profile.cycles_total;
    const uint64_t samples = g_demod.profile.samples;
    if (cycles_total > 0 && samples > 0) {
        const float cps = (float)cycles_total / (float)samples;
        const float pct_bpf = (float)g_demod.profile.cycles_bpf * 100.0f / (float)cycles_total;
        const float pct_mix = (float)g_demod.profile.cycles_mix * 100.0f / (float)cycles_total;
        const float pct_lpf_i = (float)g_demod.profile.cycles_lpf_i * 100.0f / (float)cycles_total;
        const float pct_lpf_q = (float)g_demod.profile.cycles_lpf_q * 100.0f / (float)cycles_total;
        const float pct_demod = (float)g_demod.profile.cycles_demod * 100.0f / (float)cycles_total;
        const float pct_slicer = (float)g_demod.profile.cycles_slicer * 100.0f / (float)cycles_total;
        Serial.printf("\nPROFILE (cycles)\n"
                      "total     %llu\n"
                      "samples   %llu\n"
                      "cps       %.2f\n"
                      "bpf       %5.1f%%\n"
                      "mix       %5.1f%%\n"
                      "lpf_i     %5.1f%%\n"
                      "lpf_q     %5.1f%%\n"
                      "demod     %5.1f%%\n"
                      "slicer    %5.1f%%\n",
                      (unsigned long long)cycles_total,
                      (unsigned long long)samples,
                      cps,
                      pct_bpf,
                      pct_mix,
                      pct_lpf_i,
                      pct_lpf_q,
                      pct_demod,
                      pct_slicer);
    }
#endif

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(13, g_packets, "Unexpected packet count");
    TEST_ASSERT_TRUE_MESSAGE(cpu_pct < 50.0, "CPU usage exceeds 50%");
}

void setup() {
    Serial.begin(921600);
    UNITY_BEGIN();
    RUN_TEST(test_decoder_embedded);
    UNITY_END();
}

void loop() {
}
