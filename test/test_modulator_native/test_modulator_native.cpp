#include <stdint.h>
#include <string.h>

#include <vector>

#include "unity.h"

#include "AfskDemodulator.h"
#include "AfskModulator.h"

namespace {

struct DecodedFrame {
    std::vector<uint8_t> data;
};

static DecodedFrame g_decoded;
static int g_frame_count = 0;
static std::vector<float> g_samples;

static void on_frame(const uint8_t *frame, size_t len) {
    g_decoded.data.assign(frame, frame + len);
    g_frame_count++;
}

static void on_samples(const float *samples, size_t count) {
    g_samples.insert(g_samples.end(), samples, samples + count);
}

}  // namespace

void setUp() {
    g_decoded.data.clear();
    g_frame_count = 0;
    g_samples.clear();
}

void tearDown() {}

void test_modulator_native_loopback_basic(void) {
    const uint8_t payload[] = {
        0x82, 0xa0, 0xa4, 0xa6, 0x40, 0x40, 0x60,  // DEST
        0x9a, 0x66, 0x66, 0x96, 0x40, 0x40, 0x61,  // SRC
        0x03, 0xf0,                                // Control + PID
        'T', 'E', 'S', 'T'                         // Info
    };

    g_samples.reserve(48000);
    AfskModulator mod(AFSK_SAMPLE_RATE, on_samples);
    float chunk[256];

    mod.modulate(payload, sizeof(payload), chunk, sizeof(chunk) / sizeof(chunk[0]));

    AfskDemodulator demod(AFSK_SAMPLE_RATE, AFSK_DECIM_FACTOR, on_frame);
    demod.processSamples(g_samples.data(), g_samples.size());
    demod.flush();

    TEST_ASSERT_EQUAL_INT_MESSAGE(1, g_frame_count, "Expected one decoded frame");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(sizeof(payload), g_decoded.data.size(), "Decoded length mismatch");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(payload, g_decoded.data.data(), g_decoded.data.size(), "Decoded payload mismatch");
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_modulator_native_loopback_basic);
    return UNITY_END();
}
