#include <Arduino.h>
#include <esp_timer.h>

#include "unity.h"

#include "AfskDemodulator.h"
#include "AfskModulator.h"
#include "embedded_audio.h"

static void on_packet_decoded(const uint8_t *, size_t);
static volatile uint32_t g_packets = 0;
static void on_loopback_packet(const uint8_t *frame, size_t len);
static void on_loopback_samples(const float *samples, size_t count);
static volatile uint32_t g_loopback_packets = 0;
static uint8_t g_loopback_frame[512];
static size_t g_loopback_frame_len = 0;
static AfskDemodulator *g_loopback_demod = nullptr;

static void on_packet_decoded(const uint8_t *, size_t) {
    g_packets++;
}

static void on_loopback_packet(const uint8_t *frame, size_t len) {
    if (g_loopback_packets == 0 && len <= sizeof(g_loopback_frame)) {
        memcpy(g_loopback_frame, frame, len);
        g_loopback_frame_len = len;
    }
    g_loopback_packets++;
}

static void on_loopback_samples(const float *samples, size_t count) {
    if (g_loopback_demod) {
        g_loopback_demod->processSamples(samples, count);
    }
}

static void test_modem_loopback_esp32(void) {
    const uint8_t payload[] = {
        0x82, 0xa0, 0xa4, 0xa6, 0x40, 0x40, 0x60,
        0x9a, 0x66, 0x66, 0x96, 0x40, 0x40, 0x61,
        0x03, 0xf0,
        'T', 'E', 'S', 'T'
    };

    g_loopback_packets = 0;
    g_loopback_frame_len = 0;
    memset(g_loopback_frame, 0, sizeof(g_loopback_frame));

    AfskDemodulator demod(AFSK_SAMPLE_RATE, 2, on_loopback_packet);
    g_loopback_demod = &demod;
    AfskModulator mod(AFSK_SAMPLE_RATE, on_loopback_samples);
    float chunk[256];
    mod.modulate(payload, sizeof(payload), chunk, sizeof(chunk) / sizeof(chunk[0]));
    demod.flush();
    g_loopback_demod = nullptr;

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, g_loopback_packets, "Expected one decoded loopback frame");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(sizeof(payload), g_loopback_frame_len, "Loopback frame length mismatch");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(payload, g_loopback_frame, sizeof(payload), "Loopback payload mismatch");
}

static void run_embedded_decoder(int decim) {
    AfskDemodulator demod(AFSK_SAMPLE_RATE, decim, on_packet_decoded);
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
    RUN_TEST(test_modem_loopback_esp32);
    RUN_TEST(test_decoder_embedded_decim1);
    RUN_TEST(test_decoder_embedded_decim2);
    RUN_TEST(test_decoder_embedded_decim3);
    RUN_TEST(test_decoder_embedded_decim4);
    UNITY_END();
}

void loop() {
}
