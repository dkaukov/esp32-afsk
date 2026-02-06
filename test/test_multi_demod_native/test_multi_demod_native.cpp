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
#include "afsk_multi_demod.h"

namespace {

struct TestCase {
    const char *path;
    uint32_t expected_multi;  // 0 means "print only"
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

static uint32_t g_d0_count = 0;
static uint32_t g_d6_count = 0;

static void on_packet_d0(const uint8_t *, size_t, int) {
    g_d0_count++;
}

static void on_packet_d6(const uint8_t *, size_t, int) {
    g_d6_count++;
}

static void on_packet_multi(const uint8_t *, size_t) {
    // Counted by AfskMultiDemodulator.packet_count.
}

struct Counts {
    uint32_t d0;
    uint32_t d6;
    uint32_t multi;
};

static Counts decode_flac_counts(const char *path) {
    drflac *flac = drflac_open_file(path, NULL);
    TEST_ASSERT_NOT_NULL_MESSAGE(flac, "Failed to open FLAC fixture.");

    const uint32_t channels = flac->channels;
    const uint32_t sample_rate = flac->sampleRate;
    TEST_ASSERT_MESSAGE(sample_rate > 0, "Invalid FLAC sample rate.");

    AfskDemodulator d0;
    AfskDemodulator d6;
    AfskMultiDemodulator multi;

    g_d0_count = 0;
    g_d6_count = 0;

    afsk_demod_init(&d0, 0, on_packet_d0);
    afsk_demod_init(&d6, 6, on_packet_d6);
    afsk_multi_init(&multi, on_packet_multi);

    const uint32_t frames_per_chunk = 4096;
    const uint64_t samples_per_chunk = (uint64_t)frames_per_chunk * channels;
    int16_t *buffer = (int16_t *)malloc(samples_per_chunk * sizeof(int16_t));
    TEST_ASSERT_NOT_NULL_MESSAGE(buffer, "Out of memory for FLAC buffer.");

    LinearResampler rs;
    resampler_init(&rs, sample_rate, 48000);

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

                afsk_demod_process_sample(&d0, out);
                afsk_demod_process_sample(&d6, out);
                afsk_multi_process_sample(&multi, out);

                rs.pos += rs.step;
            }

            rs.prev = sample;
            rs.in_index++;
        }
    }

    free(buffer);
    drflac_close(flac);

    return Counts{g_d0_count, g_d6_count, multi.packet_count};
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_multi_demod_improves_or_matches(void) {
    const TestCase cases[] = {
        {"test/fixtures/02_100-Mic-E-Bursts-DE-emphasized.flac", 957},
    };

    for (const auto &tc : cases) {
        Counts c = decode_flac_counts(tc.path);
        printf("%s: d0=%u d6=%u multi=%u\n", tc.path, c.d0, c.d6, c.multi);
        const uint32_t best_single = c.d0 > c.d6 ? c.d0 : c.d6;
        TEST_ASSERT_TRUE_MESSAGE(c.multi >= best_single, "Multi-demod did not improve packet count");
        if (tc.expected_multi > 0) {
            TEST_ASSERT_EQUAL_UINT32_MESSAGE(
                tc.expected_multi, c.multi, "Unexpected multi-demod packet count");
        }
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_multi_demod_improves_or_matches);
    return UNITY_END();
}
