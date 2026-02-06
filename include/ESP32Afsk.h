#pragma once

#include <Arduino.h>

class ESP32Afsk {
 public:
  ESP32Afsk();

  // Initialize the modem with a target sample rate in Hz.
  bool begin(uint32_t sample_rate_hz);

  // Write a single signed 16-bit sample to the output buffer.
  // Returns false if not initialized.
  bool writeSample(int16_t sample);

  // Stop output and release any resources.
  void end();

 private:
  uint32_t sample_rate_hz_ = 0;
  bool initialized_ = false;
};
