#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "unity.h"

#if __has_include("dr_flac.h")
#define DR_FLAC_IMPLEMENTATION
#include "dr_flac.h"
#else
#error "Missing test/third_party/dr_flac.h. See test/third_party/README.md."
#endif

#include "AfskDemodulator.h"

namespace {

struct TestCase {
    const char *path;
    uint32_t min_packets;
};

struct LinearResampler {
    double step;      // input samples per output sample
    double pos;       // next output position in input-sample units
    uint64_t in_index;
    float prev;
    bool has_prev;
};

static void resampler_init(LinearResampler *rs, uint32_t in_rate, uint32_t out_rate) {
    rs->step = (double)in_rate / (double)out_rate;
    rs->pos = 0.0;
    rs->in_index = 0;
    rs->prev = 0.0f;
    rs->has_prev = false;
}

static uint32_t g_packet_count = 0;
#ifdef AFSK_DEMOD_STATS
static AfskDemodStats g_last_stats;
#endif

static void on_packet_decoded(const uint8_t *, size_t) {
    g_packet_count++;
}

static uint32_t decode_flac_and_count_packets(const char *path, int decim) {
    drflac *flac = drflac_open_file(path, NULL);
    TEST_ASSERT_NOT_NULL_MESSAGE(flac, "Failed to open FLAC fixture.");

    const uint32_t channels = flac->channels;
    const uint32_t sample_rate = flac->sampleRate;

    const uint32_t target_rate = 48000;
    TEST_ASSERT_MESSAGE(sample_rate > 0, "Invalid FLAC sample rate.");

    AfskDemodulator demod((float)target_rate, decim, on_packet_decoded);
    g_packet_count = 0;

    LinearResampler rs;
    resampler_init(&rs, sample_rate, target_rate);

    const uint32_t frames_per_chunk = 4096;
    const uint64_t samples_per_chunk = (uint64_t)frames_per_chunk * channels;
    int16_t *buffer = (int16_t *)malloc(samples_per_chunk * sizeof(int16_t));
    TEST_ASSERT_NOT_NULL_MESSAGE(buffer, "Out of memory for FLAC buffer.");

    while (true) {
        const drflac_uint64 frames_read = drflac_read_pcm_frames_s16(flac, frames_per_chunk, buffer);
        if (frames_read == 0) {
            break;
        }

        for (drflac_uint64 i = 0; i < frames_read; i++) {
            int32_t acc = 0;
            for (uint32_t ch = 0; ch < channels; ch++) {
                acc += buffer[i * channels + ch];
            }
            const float sample = (float)acc / (float)(channels * 32768.0f);

            if (!rs.has_prev) {
                rs.prev = sample;
                rs.has_prev = true;
                rs.in_index = 1;
                continue;
            }

            while (rs.pos <= (double)rs.in_index) {
                const double t = rs.pos - (double)(rs.in_index - 1);
                const float out = rs.prev + (sample - rs.prev) * (float)t;
                demod.processSamples(&out, 1);
                rs.pos += rs.step;
            }

            rs.prev = sample;
            rs.in_index++;
        }
    }

#ifdef AFSK_DEMOD_STATS
    g_last_stats = demod.getStats();
#endif

    demod.flush();
    free(buffer);
    drflac_close(flac);

    return g_packet_count;
}

}  // namespace

void setUp() {}
void tearDown() {}

static const char *fixture_basename(const char *path) {
    const char *last = path;
    for (const char *p = path; *p; ++p) {
        if (*p == '/' || *p == '\\') last = p + 1;
    }
    return last;
}

static void print_results_header_once(void) {
    static bool printed = false;
    if (printed) return;
    printed = true;
    printf("\nRESULTS (native)\n");
    printf("%-40s %5s %8s %10s %10s %10s %12s %10s\n",
           "fixture", "decim", "packets", "min_req", "min", "max", "mean", "samples");
    printf("%-40s %5s %8s %10s %10s %10s %12s %10s\n",
           "----------------------------------------", "-----", "--------", "----------", "----------", "----------", "------------", "----------");
}

static void assertDecoded(uint32_t count, uint32_t min_packets, const char *path, int decim) {
    print_results_header_once();
#ifdef AFSK_DEMOD_STATS
    const float mean = g_last_stats.samples ? (g_last_stats.demod_sum / (float)g_last_stats.samples) : 0.0f;
    printf("%-40s %5d %8u %10u %10.3f %10.3f %12.6f %10llu\n",
           fixture_basename(path),
           decim,
           (unsigned)count,
           (unsigned)min_packets,
           (double)g_last_stats.demod_min,
           (double)g_last_stats.demod_max,
           (double)mean,
           (unsigned long long)g_last_stats.samples);
#else
    printf("%-40s %5d %8u %10u\n",
           fixture_basename(path),
           decim,
           (unsigned)count,
           (unsigned)min_packets);
#endif
    char msg[160];
    snprintf(msg, sizeof(msg), "%s: decoded %u packets (decim=%d)", path, (unsigned)count, decim);
    TEST_ASSERT_TRUE_MESSAGE(count >= min_packets, msg);
}

static const char kFixtureTrack1[] = "test/fixtures/01_40-Mins-Traffic-on-144.39.flac";
static const char kFixtureTrack2[] = "test/fixtures/01_40-Mins-Traffic-on-144.39_20s.flac";
static const char kFixtureTrack3[] = "test/fixtures/01_40-Mins-Traffic-on-144.39_60s.flac";
static const char kFixtureTrack4[] = "test/fixtures/02_100-Mic-E-Bursts-DE-emphasized.flac";

void test_decoder_decim1_track1(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack1, 1), 1005, kFixtureTrack1, 1); }
void test_decoder_decim1_track2(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack2, 1), 13, kFixtureTrack2, 1); }
void test_decoder_decim1_track3(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack3, 1), 51, kFixtureTrack3, 1); }
void test_decoder_decim1_track4(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack4, 1), 947, kFixtureTrack4, 1); }

void test_decoder_decim2_track1(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack1, 2), 1005, kFixtureTrack1, 2); }
void test_decoder_decim2_track2(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack2, 2), 13, kFixtureTrack2, 2); }
void test_decoder_decim2_track3(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack3, 2), 52, kFixtureTrack3, 2); }
void test_decoder_decim2_track4(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack4, 2), 946, kFixtureTrack4, 2); }

void test_decoder_decim3_track1(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack1, 3), 1005, kFixtureTrack1, 3); }
void test_decoder_decim3_track2(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack2, 3), 13, kFixtureTrack2, 3); }
void test_decoder_decim3_track3(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack3, 3), 52, kFixtureTrack3, 3); }
void test_decoder_decim3_track4(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack4, 3), 945, kFixtureTrack4, 3); }

void test_decoder_decim4_track1(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack1, 4), 1000, kFixtureTrack1, 4); }
void test_decoder_decim4_track2(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack2, 4), 13, kFixtureTrack2, 4); }
void test_decoder_decim4_track3(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack3, 4), 52, kFixtureTrack3, 4); }
void test_decoder_decim4_track4(void) { assertDecoded(decode_flac_and_count_packets(kFixtureTrack4, 4), 948, kFixtureTrack4, 4); }

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_decoder_decim1_track1);
    RUN_TEST(test_decoder_decim1_track2);
    RUN_TEST(test_decoder_decim1_track3);
    RUN_TEST(test_decoder_decim1_track4);
    RUN_TEST(test_decoder_decim2_track1);
    RUN_TEST(test_decoder_decim2_track2);
    RUN_TEST(test_decoder_decim2_track3);
    RUN_TEST(test_decoder_decim2_track4);
    RUN_TEST(test_decoder_decim3_track1);
    RUN_TEST(test_decoder_decim3_track2);
    RUN_TEST(test_decoder_decim3_track3);
    RUN_TEST(test_decoder_decim3_track4);
    RUN_TEST(test_decoder_decim4_track1);
    RUN_TEST(test_decoder_decim4_track2);
    RUN_TEST(test_decoder_decim4_track3);
    RUN_TEST(test_decoder_decim4_track4);
    return UNITY_END();
}
