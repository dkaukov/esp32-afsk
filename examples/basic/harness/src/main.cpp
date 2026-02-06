/*
 * main.cpp - AFSK 1200 Test Harness for ESP32
 * =============================================
 * Standalone firmware for validating the KV4P-HT AFSK encoder/decoder
 * against WA8LMF test tracks and javAX25 reference output.
 *
 * No radio hardware, audio codec, or DRA818 dependencies.
 * Communicates with host via Serial at 921600 baud.
 *
 * Commands (sent as text lines):
 *   DECODE <num_samples>  - Receive raw 16-bit LE PCM, decode, output packets
 *   ENCODE <hex_bytes>    - Encode given AX.25 frame, output audio samples
 *   LOOPBACK              - Encode test packet -> decode -> verify roundtrip
 *   STATUS                - Report free heap, uptime, packet stats
 *   HELP                  - List commands
 *
 * Protocol for DECODE:
 *   1. Host sends: DECODE 48000\n  (number of 16-bit samples)
 *   2. ESP32 sends: READY\n
 *   3. Host sends 1024-byte chunks of raw LE int16 PCM
 *   4. After each chunk, ESP32 sends: READY\n
 *   5. As packets are decoded: PKT <hex>\n
 *   6. When all samples processed: DONE <count>\n
 *
 * Protocol for ENCODE:
 *   1. Host sends: ENCODE DEADBEEF...\n  (hex-encoded AX.25 frame with CRC)
 *   2. ESP32 encodes and sends: ENCODED <num_samples>\n
 *   3. Then streams raw LE int16 samples in 1024-byte chunks
 *   4. Host reads until all samples received
 *   5. ESP32 sends: DONE\n
 *
 * Copyright (C) 2025 - Contributed to KV4P-HT project
 * Licensed under GNU General Public License v3
 */

#include <Arduino.h>
#include "test/stubs/test_globals.h"
#include "afsk_encoder.h"
#include "afsk_multi_demod.h"

// ============================================================================
// Constants
// ============================================================================

#define SERIAL_BAUD       921600
#define CHUNK_SIZE        16384     // bytes per Serial chunk (8192 int16 samples)
#define CMD_BUF_SIZE      1024      // max command line length
#define MAX_ENCODE_SAMPLES 55000    // ~1.1s at 48kHz (enough for any packet without silence)

// ============================================================================
// Global State
// ============================================================================

static AfskMultiDemodulator multiDemod;
static uint32_t totalPacketsDecoded = 0;
static uint32_t sessionPacketCount = 0;

// Command line buffer
static char cmdBuf[CMD_BUF_SIZE];
static int cmdBufPos = 0;

// Packet output buffer for decode sessions
#define MAX_PKT_QUEUE 32
static struct {
    uint8_t data[AFSK_MAX_FRAME_SIZE];
    size_t len;
} pktQueue[MAX_PKT_QUEUE];
static volatile int pktQueueHead = 0;
static volatile int pktQueueTail = 0;

// ============================================================================
// Packet Callback (called by multi-demodulator)
// ============================================================================

static void onPacketDecoded(const uint8_t *frame, size_t len) {
    int next = (pktQueueHead + 1) % MAX_PKT_QUEUE;
    if (next != pktQueueTail) {
        memcpy(pktQueue[pktQueueHead].data, frame, len);
        pktQueue[pktQueueHead].len = len;
        pktQueueHead = next;
    }
    totalPacketsDecoded++;
    sessionPacketCount++;
}

// ============================================================================
// Helper: Drain packet queue to Serial
// ============================================================================

static int drainPacketQueue() {
    int count = 0;
    while (pktQueueTail != pktQueueHead) {
        size_t len = pktQueue[pktQueueTail].len;
        Serial.print("PKT ");
        for (size_t i = 0; i < len; i++) {
            char hex[3];
            sprintf(hex, "%02X", pktQueue[pktQueueTail].data[i]);
            Serial.print(hex);
        }
        Serial.println();
        pktQueueTail = (pktQueueTail + 1) % MAX_PKT_QUEUE;
        count++;
    }
    return count;
}

// ============================================================================
// Helper: Parse hex string to bytes
// ============================================================================

static int hexToBytesInPlace(const char *hex, uint8_t *out, int maxLen) {
    int len = strlen(hex);
    if (len % 2 != 0) return -1;
    int byteCount = len / 2;
    if (byteCount > maxLen) return -1;

    for (int i = 0; i < byteCount; i++) {
        char hi = hex[i * 2];
        char lo = hex[i * 2 + 1];

        uint8_t nib_hi, nib_lo;
        if (hi >= '0' && hi <= '9') nib_hi = hi - '0';
        else if (hi >= 'A' && hi <= 'F') nib_hi = hi - 'A' + 10;
        else if (hi >= 'a' && hi <= 'f') nib_hi = hi - 'a' + 10;
        else return -1;

        if (lo >= '0' && lo <= '9') nib_lo = lo - '0';
        else if (lo >= 'A' && lo <= 'F') nib_lo = lo - 'A' + 10;
        else if (lo >= 'a' && lo <= 'f') nib_lo = lo - 'a' + 10;
        else return -1;

        out[i] = (nib_hi << 4) | nib_lo;
    }
    return byteCount;
}

// ============================================================================
// Command: DECODE
// ============================================================================

static void cmdDecode(const char *arg) {
    int numSamples = atoi(arg);
    if (numSamples <= 0 || numSamples > 100000000) {
        Serial.println("ERROR Invalid sample count");
        return;
    }

    // Reset decoder state
    afsk_multi_init(&multiDemod, onPacketDecoded);
    sessionPacketCount = 0;
    pktQueueHead = 0;
    pktQueueTail = 0;

    int samplesRemaining = numSamples;
    static uint8_t chunkBuf[CHUNK_SIZE];  // static to avoid stack overflow (16KB > 8KB task stack)

    Serial.println("READY");

    while (samplesRemaining > 0) {
        // Calculate how many bytes to expect this chunk
        int samplesToRead = min(samplesRemaining, (int)(CHUNK_SIZE / 2));
        int bytesToRead = samplesToRead * 2;

        // Read exactly bytesToRead from Serial
        int bytesRead = 0;
        unsigned long timeout = millis() + 5000;  // 5 second timeout per chunk
        while (bytesRead < bytesToRead) {
            if (Serial.available()) {
                int n = Serial.readBytes(chunkBuf + bytesRead, bytesToRead - bytesRead);
                bytesRead += n;
                timeout = millis() + 5000;  // reset timeout on data
            }
            if (millis() > timeout) {
                Serial.println("ERROR Timeout waiting for data");
                Serial.printf("DONE %d\n", sessionPacketCount);
                return;
            }
            yield();
        }

        // Process samples through demodulator
        int16_t *samples = (int16_t *)chunkBuf;
        for (int i = 0; i < samplesToRead; i++) {
            float sample = (float)samples[i] / 32768.0f;
            afsk_multi_process_sample(&multiDemod, sample);
        }

        samplesRemaining -= samplesToRead;

        // Drain any decoded packets
        drainPacketQueue();

        // Signal ready for next chunk
        if (samplesRemaining > 0) {
            Serial.println("READY");
        }
    }

    // Final drain
    drainPacketQueue();

    Serial.printf("DONE %d\n", sessionPacketCount);
}

// ============================================================================
// Command: ENCODE
// ============================================================================

static void cmdEncode(const char *arg) {
    // Parse hex packet data
    static uint8_t packetBuf[512];
    int packetLen = hexToBytesInPlace(arg, packetBuf, sizeof(packetBuf));
    if (packetLen < 0) {
        Serial.println("ERROR Invalid hex string");
        return;
    }
    if (packetLen < 18 || packetLen > 512) {
        Serial.println("ERROR Packet length out of range (18-512)");
        return;
    }

    // Allocate output buffer on PSRAM if available, else heap
    int16_t *audioBuf;
    if (ESP.getPsramSize() > 0) {
        audioBuf = (int16_t *)ps_malloc(MAX_ENCODE_SAMPLES * sizeof(int16_t));
    } else {
        audioBuf = (int16_t *)malloc(MAX_ENCODE_SAMPLES * sizeof(int16_t));
    }
    if (audioBuf == NULL) {
        Serial.println("ERROR Cannot allocate audio buffer");
        return;
    }

    // Encode (no silence for test harness - just flags + data)
    size_t numSamples = afskEncodePacketDefaults(
        packetBuf, packetLen, audioBuf, MAX_ENCODE_SAMPLES);

    if (numSamples == 0) {
        Serial.println("ERROR Encode failed");
        free(audioBuf);
        return;
    }

    Serial.printf("ENCODED %d\n", numSamples);

    // Stream audio samples to host in chunks
    size_t bytesTotal = numSamples * sizeof(int16_t);
    uint8_t *ptr = (uint8_t *)audioBuf;
    size_t bytesSent = 0;

    while (bytesSent < bytesTotal) {
        size_t chunkBytes = min((size_t)CHUNK_SIZE, bytesTotal - bytesSent);
        Serial.write(ptr + bytesSent, chunkBytes);
        bytesSent += chunkBytes;
        Serial.flush();
    }

    Serial.println("DONE");
    free(audioBuf);
}

// ============================================================================
// Command: LOOPBACK
// ============================================================================

static void cmdLoopback() {
    // A minimal but valid AX.25 UI frame:
    //   Dest: APRS   (7 bytes, shifted)
    //   Src:  TEST-1 (7 bytes, shifted, last bit set)
    //   Ctrl: 0x03 (UI)
    //   PID:  0xF0 (no L3)
    //   Info: "Test 1234"
    //   CRC:  computed below
    //
    // We build the frame and compute CRC-CCITT ourselves.

    static const uint8_t testFrame[] = {
        // Destination: "APRS  " (6 chars, space-padded, shifted left 1, SSID=0)
        'A' << 1, 'P' << 1, 'R' << 1, 'S' << 1, ' ' << 1, ' ' << 1, 0x60,
        // Source: "TEST  " (6 chars, shifted left 1, SSID=1, last address bit set)
        'T' << 1, 'E' << 1, 'S' << 1, 'T' << 1, ' ' << 1, ' ' << 1, 0x63,
        // Control: UI frame
        0x03,
        // PID: No layer 3
        0xF0,
        // Information field
        'T', 'e', 's', 't', ' ', '1', '2', '3', '4'
    };
    const size_t frameLen = sizeof(testFrame);

    // Compute CRC-CCITT (per AX.25 spec, same as afsk_demod.h)
    // CRC is computed over all bytes after the flag and before the CRC field
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < frameLen; i++) {
        crc = (crc >> 8) ^ pgm_read_word(&crc_ccitt_tab[(crc ^ testFrame[i]) & 0xFF]);
    }
    crc ^= 0xFFFF;  // Final XOR

    // Build complete frame with CRC appended (little-endian)
    uint8_t fullFrame[sizeof(testFrame) + 2];
    memcpy(fullFrame, testFrame, frameLen);
    fullFrame[frameLen]     = crc & 0xFF;        // CRC low byte
    fullFrame[frameLen + 1] = (crc >> 8) & 0xFF; // CRC high byte
    size_t fullLen = frameLen + 2;

    Serial.printf("LOOPBACK: Frame %d bytes (with CRC)\n", fullLen);

    // Print the frame hex
    Serial.print("FRAME ");
    for (size_t i = 0; i < fullLen; i++) {
        Serial.printf("%02X", fullFrame[i]);
    }
    Serial.println();

    // Encode to audio
    int16_t *audioBuf;
    if (ESP.getPsramSize() > 0) {
        audioBuf = (int16_t *)ps_malloc(MAX_ENCODE_SAMPLES * sizeof(int16_t));
    } else {
        audioBuf = (int16_t *)malloc(MAX_ENCODE_SAMPLES * sizeof(int16_t));
    }
    if (audioBuf == NULL) {
        Serial.println("ERROR Cannot allocate audio buffer");
        return;
    }

    initEncoderSineTable();

    size_t numSamples = afskEncodePacketDefaults(
        fullFrame, fullLen, audioBuf, MAX_ENCODE_SAMPLES);

    Serial.printf("ENCODED %d samples\n", numSamples);

    if (numSamples == 0) {
        Serial.println("ERROR Encode failed");
        free(audioBuf);
        return;
    }

    // Feed encoded audio into decoder
    afsk_multi_init(&multiDemod, onPacketDecoded);
    sessionPacketCount = 0;
    pktQueueHead = 0;
    pktQueueTail = 0;

    for (size_t i = 0; i < numSamples; i++) {
        float sample = (float)audioBuf[i] / 32768.0f;
        afsk_multi_process_sample(&multiDemod, sample);
    }

    free(audioBuf);

    // Drain decoded packets
    int decoded = drainPacketQueue();

    if (decoded > 0) {
        Serial.println("LOOPBACK: PASS");
    } else {
        Serial.println("LOOPBACK: FAIL (no packets decoded)");
    }

    Serial.printf("DONE %d\n", decoded);
}

// ============================================================================
// Command: STATUS
// ============================================================================

static void cmdStatus() {
    Serial.println("--- AFSK Test Harness Status ---");
    Serial.printf("Uptime: %lu ms\n", millis());
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
    Serial.printf("PSRAM free: %d bytes\n", ESP.getFreePsram());
    Serial.printf("Total packets decoded: %d\n", totalPacketsDecoded);
    Serial.printf("Unique packets forwarded: %d\n", multiDemod.packet_count);
    Serial.printf("Dedup suppressed: %d\n", multiDemod.dup_count);
    Serial.printf("CPU freq: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.println("---");
}

// ============================================================================
// Command: HELP
// ============================================================================

static void cmdHelp() {
    Serial.println("AFSK 1200 Test Harness Commands:");
    Serial.println("  DECODE <num_samples>  - Receive 16-bit LE PCM, decode AX.25 packets");
    Serial.println("  ENCODE <hex_bytes>    - Encode AX.25 frame to AFSK audio");
    Serial.println("  LOOPBACK              - Encode test packet, decode, verify roundtrip");
    Serial.println("  STATUS                - Show heap, uptime, stats");
    Serial.println("  HELP                  - This message");
    Serial.println();
    Serial.println("DECODE protocol: ESP32 sends READY, host sends 1024-byte chunks.");
    Serial.println("Decoded packets output as: PKT <hex>");
    Serial.println("Session ends with: DONE <count>");
}

// ============================================================================
// Command Dispatcher
// ============================================================================

static void processCommand(const char *line) {
    // Skip empty lines
    if (line[0] == '\0') return;

    // Parse command and argument
    char cmd[32] = {0};
    const char *arg = NULL;

    // Find first space
    const char *space = strchr(line, ' ');
    if (space) {
        int cmdLen = min((int)(space - line), 31);
        strncpy(cmd, line, cmdLen);
        cmd[cmdLen] = '\0';
        arg = space + 1;
        // Skip leading spaces in arg
        while (*arg == ' ') arg++;
    } else {
        strncpy(cmd, line, 31);
        cmd[31] = '\0';
    }

    // Convert command to uppercase
    for (int i = 0; cmd[i]; i++) {
        if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] -= 32;
    }

    if (strcmp(cmd, "DECODE") == 0) {
        if (arg == NULL || *arg == '\0') {
            Serial.println("ERROR Usage: DECODE <num_samples>");
            return;
        }
        cmdDecode(arg);
    } else if (strcmp(cmd, "ENCODE") == 0) {
        if (arg == NULL || *arg == '\0') {
            Serial.println("ERROR Usage: ENCODE <hex_bytes>");
            return;
        }
        cmdEncode(arg);
    } else if (strcmp(cmd, "LOOPBACK") == 0) {
        cmdLoopback();
    } else if (strcmp(cmd, "STATUS") == 0) {
        cmdStatus();
    } else if (strcmp(cmd, "HELP") == 0) {
        cmdHelp();
    } else {
        Serial.printf("ERROR Unknown command: %s\n", cmd);
        Serial.println("Type HELP for available commands.");
    }
}

// ============================================================================
// Arduino Setup / Loop
// ============================================================================

void setup() {
    Serial.setRxBufferSize(32768);  // Larger RX buffer for faster streaming
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { delay(10); }

    Serial.println();
    Serial.println("=== AFSK 1200 Test Harness ===");
    Serial.printf("Sample rate: %d Hz\n", AUDIO_SAMPLE_RATE);
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM: %d bytes\n", ESP.getPsramSize());
    Serial.println("Type HELP for commands.");
    Serial.println();

    // Initialize sine table for encoder
    initEncoderSineTable();

    // Initialize multi-demodulator
    afsk_multi_init(&multiDemod, onPacketDecoded);
}

void loop() {
    // Read Serial input line-by-line
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdBufPos > 0) {
                cmdBuf[cmdBufPos] = '\0';
                processCommand(cmdBuf);
                cmdBufPos = 0;
            }
        } else if (cmdBufPos < CMD_BUF_SIZE - 1) {
            cmdBuf[cmdBufPos++] = c;
        }
    }
}
