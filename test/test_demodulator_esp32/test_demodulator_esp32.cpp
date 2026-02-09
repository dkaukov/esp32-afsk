#include <Arduino.h>
#include <esp_timer.h>

#include "unity.h"

#include "AfskDemodulator.h"
#include "embedded_audio.h"

static void on_packet_decoded(const uint8_t *, size_t, int);
static volatile uint32_t g_packets = 0;

static void on_packet_decoded(const uint8_t *, size_t, int) {
    g_packets++;
}

static void run_embedded_decoder(int decim) {
    AfskDemodulator demod((float)AFSK_SAMPLE_RATE, decim, 0, on_packet_decoded);
    TEST_MESSAGE("Starting embedded decoder benchmark...");
    g_packets = 0;

    int64_t start = esp_timer_get_time();
    demod.processSamples(EMBEDDED_AUDIO, EMBEDDED_AUDIO_SAMPLES);
    demod.flush();
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
             "decim         %d\n"
             "packets       %lu\n"
             "decode_sec    %.3f\n"
             "audio_sec     %.3f\n"
             "rt_factor     %.3f\n"
             "cpu_pct       %.2f\n",
             decim,
             (unsigned long)g_packets,
             decode_sec,
             audio_sec,
             rt_factor,
             cpu_pct);
    //TEST_MESSAGE(msg);
    Serial.println(msg);

    uint32_t expected_packets = 13;
    float max_cpu_pct = 50.0f;
    switch (decim) {
        case 1:
            expected_packets = 13;
            max_cpu_pct = 20.69f;
            break;
        case 2:
            expected_packets = 13;
            max_cpu_pct = 13.37f;
            break;
        case 3:
            expected_packets = 13;
            max_cpu_pct = 11.68f;
            break;
        case 4:
            expected_packets = 13;
            max_cpu_pct = 10.85f;
            break;
        default:
            expected_packets = 13;
            max_cpu_pct = 50.0f;
            break;
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected_packets, g_packets, "Unexpected packet count");
    char cpu_msg[128];
    snprintf(cpu_msg, sizeof(cpu_msg), "CPU usage %.2f exceeds limit %.2f (decim=%d)", cpu_pct, max_cpu_pct, decim);
    TEST_ASSERT_TRUE_MESSAGE(cpu_pct <= max_cpu_pct, cpu_msg);
}

static void test_decoder_embedded_decim1(void) { run_embedded_decoder(1); }
static void test_decoder_embedded_decim2(void) { run_embedded_decoder(2); }
static void test_decoder_embedded_decim3(void) { run_embedded_decoder(3); }
static void test_decoder_embedded_decim4(void) { run_embedded_decoder(4); }

void setup() {
    Serial.begin(921600);
    UNITY_BEGIN();
    RUN_TEST(test_decoder_embedded_decim1);
    RUN_TEST(test_decoder_embedded_decim2);
    RUN_TEST(test_decoder_embedded_decim3);
    RUN_TEST(test_decoder_embedded_decim4);
    UNITY_END();
}

void loop() {
}
