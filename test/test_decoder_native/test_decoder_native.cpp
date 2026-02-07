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

#include "afsk_demod.h"

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

static void on_packet_decoded(const uint8_t *, size_t, int) {
    g_packet_count++;
}

static uint32_t decode_flac_and_count_packets(const char *path) {
    drflac *flac = drflac_open_file(path, NULL);
    TEST_ASSERT_NOT_NULL_MESSAGE(flac, "Failed to open FLAC fixture.");

    const uint32_t channels = flac->channels;
    const uint32_t sample_rate = flac->sampleRate;

    const uint32_t target_rate = 48000;
    TEST_ASSERT_MESSAGE(sample_rate > 0, "Invalid FLAC sample rate.");

    AfskDemodulator demod;
    g_packet_count = 0;
    afsk_demod_init(&demod, 0, on_packet_decoded);

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
                afsk_demod_process_sample(&demod, out);
                rs.pos += rs.step;
            }

            rs.prev = sample;
            rs.in_index++;
        }
    }

#ifdef AFSK_DEMOD_STATS
    if (demod.stats.samples > 0) {
        const float mean = demod.stats.demod_sum / (float)demod.stats.samples;
        printf("Demod stats for %s: min=%.6f max=%.6f mean=%.6f samples=%llu\n",
               path, demod.stats.demod_min, demod.stats.demod_max, mean,
               (unsigned long long)demod.stats.samples);
    }
#endif

    free(buffer);
    drflac_close(flac);

    return g_packet_count;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_decoder_fixtures_min_packets(void) {
    // TODO: Update min packet counts once fixtures are in place.
    const TestCase cases[] = {
        {"test/fixtures/01_40-Mins-Traffic-on-144.39.flac", 994},
        {"test/fixtures/01_40-Mins-Traffic-on-144.39_20s.flac", 13},
        {"test/fixtures/01_40-Mins-Traffic-on-144.39_60s.flac", 51},
        {"test/fixtures/02_100-Mic-E-Bursts-DE-emphasized.flac", 935},
    };

    for (const auto &tc : cases) {
        const uint32_t count = decode_flac_and_count_packets(tc.path);
        printf("Decoded %u packets from %s\n", (unsigned)count, tc.path);
        char msg[128];
        snprintf(msg, sizeof(msg), "%s: decoded %u packets", tc.path, (unsigned)count);
        TEST_ASSERT_TRUE_MESSAGE(count >= tc.min_packets, msg);
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_decoder_fixtures_min_packets);
    return UNITY_END();
}
