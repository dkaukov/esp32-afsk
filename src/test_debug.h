/*
 * test_debug.h - Lightweight debug logging for AFSK test harness
 *
 * Maps _LOGI/_LOGW/_LOGE/_LOGD/_LOGT macros to Serial.printf
 * instead of the production protocol-based debug_log_printf().
 */
#pragma once

#include <Arduino.h>

#define _LOGE(fmt, ...) Serial.printf("[E] " fmt "\n", ##__VA_ARGS__)
#define _LOGW(fmt, ...) Serial.printf("[W] " fmt "\n", ##__VA_ARGS__)
#define _LOGI(fmt, ...) Serial.printf("[I] " fmt "\n", ##__VA_ARGS__)
#define _LOGD(fmt, ...) Serial.printf("[D] " fmt "\n", ##__VA_ARGS__)
#define _LOGT(fmt, ...) /* trace disabled in test harness */
