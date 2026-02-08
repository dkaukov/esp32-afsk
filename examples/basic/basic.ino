#include <Arduino.h>
#include "AfskDemodulator.h"

void setup() {
  Serial.begin(115200);
  Serial.println("AFSK demo");
}

void loop() {
  // Placeholder: process silence through the decoder.
  static AfskDemodulator demod((float)AFSK_SAMPLE_RATE, AFSK_DECIM_FACTOR, 0, nullptr);
  demod.processSample(0.0f);
  delay(1);
}
