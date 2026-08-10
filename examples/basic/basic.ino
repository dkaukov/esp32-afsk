#include <Arduino.h>
#include <AfskDemodulator.h>

static void onPacket(const uint8_t *frame, size_t length) {
  Serial.printf("Received AX.25 frame (%u bytes):", (unsigned)length);
  for (size_t i = 0; i < length; ++i) {
    Serial.printf(" %02X", frame[i]);
  }
  Serial.println();
}

AfskDemodulator demodulator(48000, 2, onPacket);

// Call this function with signed 16-bit mono PCM captured from ADC or I2S.
void processAudio(const int16_t *samples, size_t count) {
  demodulator.processSamples(samples, count);
}

void setup() {
  Serial.begin(115200);
  Serial.println("AFSK demodulator ready");
}

void loop() {
  // Capture an audio block and pass it to processAudio().
  delay(10);
}
