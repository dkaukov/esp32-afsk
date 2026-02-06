#include <Arduino.h>
#include <esp_timer.h>

#include "unity.h"

#include "afsk_multi_demod.h"
#include "embedded_audio.h"

static AfskMultiDemodulator g_demod;

static void on_packet_decoded(const uint8_t *, size_t) {
    // Counted by g_demod.packet_count.
}

static void test_decoder_embedded(void) {
    TEST_MESSAGE("Starting embedded decoder benchmark...");
    afsk_multi_init(&g_demod, on_packet_decoded);

    int64_t start = esp_timer_get_time();
    for (uint32_t i = 0; i < EMBEDDED_AUDIO_SAMPLES; i++) {
        float s = (float)EMBEDDED_AUDIO[i] / 32768.0f;
        afsk_multi_process_sample(&g_demod, s);
    }
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
             (unsigned long)g_demod.packet_count,
             decode_sec,
             audio_sec,
             rt_factor,
             cpu_pct);
    //TEST_MESSAGE(msg);
    Serial.println(msg);

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(12, g_demod.packet_count, "Unexpected packet count");
    TEST_ASSERT_TRUE_MESSAGE(cpu_pct <= 40.0, "CPU usage exceeds 40%");
}

void setup() {
    Serial.begin(921600);
    UNITY_BEGIN();
    RUN_TEST(test_decoder_embedded);
    UNITY_END();
}

void loop() {
}
