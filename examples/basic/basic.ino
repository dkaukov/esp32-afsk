#include <Arduino.h>
#include "afsk_demod.h"

void setup() {
  Serial.begin(115200);
  Serial.println("AFSK demo");
}

void loop() {
  // Placeholder: process silence through the decoder.
  static AfskDemodulator demod;
  static bool initialized = false;
  if (!initialized) {
    afsk_demod_init(&demod, 0, nullptr);
    initialized = true;
  }
  afsk_demod_process_sample(&demod, 0.0f);
  delay(1);
}
