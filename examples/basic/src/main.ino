#include <ESP32Afsk.h>

ESP32Afsk afsk;

void setup() {
  Serial.begin(115200);

  if (!afsk.begin(8000)) {
    Serial.println("AFSK init failed");
  } else {
    Serial.println("AFSK init ok");
  }
}

void loop() {
  // Placeholder: generate a silence sample.
  afsk.writeSample(0);
  delay(1);
}
