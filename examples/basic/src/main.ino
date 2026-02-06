#include <Arduino.h>
#include "afsk_multi_demod.h"

void setup() {
  Serial.begin(115200);
  Serial.println("AFSK demo");
}

void loop() {
  // Placeholder: process silence through the decoder.
  static AfskMultiDemodulator demod;
  static bool initialized = false;
  if (!initialized) {
    afsk_multi_init(&demod, nullptr);
    initialized = true;
  }
  afsk_multi_process_sample(&demod, 0.0f);
  delay(1);
}
