/*
KISS TNC terminal example for ESP32 using arduino-audio-tools and esp32-afsk.

- Audio in: ADC on DEFAULT_PIN_AUDIO_IN
- Audio out: I2S DAC (GPIO25 implied)
- KISS over USB serial
- PTT on DEFAULT_PIN_PTT
*/

#include <Arduino.h>
#include <AudioTools.h>
#include <DRA818.h>
#include <driver/adc.h>
#include <driver/dac.h>

#include "AfskDemodulator.h"
#include "AfskModulator.h"

using namespace audio_tools;

// Connections to radio module
#define DEFAULT_PIN_RF_RXD    16
#define DEFAULT_PIN_RF_TXD    17
#define DEFAULT_PIN_AUDIO_OUT 25
#define DEFAULT_PIN_AUDIO_IN  34
#define DEFAULT_PIN_PTT       18
#define DEFAULT_PIN_PD        19
#define DEFAULT_PIN_LED        2
#define DEFAULT_VOLUME         8

#define DEFAULT_ADC_BIAS_VOLTAGE     1.75
#define DEFAULT_ADC_ATTENUATION      ADC_ATTEN_DB_12
#define DEFAULT_RF_MODULE_TYPE       SA818_UHF

// SA818 defaults (adjust to your local channel plan)
static constexpr float DEFAULT_RF_FREQ_RX = 445.000f;
static constexpr float DEFAULT_RF_FREQ_TX = 445.000f;
static constexpr uint8_t DEFAULT_RF_SQUELCH = 0;
static constexpr uint8_t DEFAULT_RF_BANDWIDTH = DRA818_12K5;
static constexpr uint8_t DEFAULT_RF_CTCSS_RX = 0;
static constexpr uint8_t DEFAULT_RF_CTCSS_TX = 0;
static constexpr bool DEFAULT_RF_PREEMPH = false;
static constexpr bool DEFAULT_RF_HIGHPASS = false;
static constexpr bool DEFAULT_RF_LOWPASS = false;
static constexpr uint8_t RF_HANDSHAKE_RETRIES = 3;
static constexpr uint16_t RF_POWERUP_DELAY_MS = 350;

// KISS framing constants
static constexpr uint8_t KISS_FEND  = 0xC0;
static constexpr uint8_t KISS_FESC  = 0xDB;
static constexpr uint8_t KISS_TFEND = 0xDC;
static constexpr uint8_t KISS_TFESC = 0xDD;
static constexpr uint8_t KISS_CMD_DATA = 0x00;

static constexpr int AUDIO_SAMPLE_RATE_HZ = AFSK_SAMPLE_RATE;
static constexpr size_t KISS_MAX_FRAME = 512;
static constexpr size_t TX_BUFFER_SAMPLES = 256;
static constexpr float TX_GAIN = 0.8f;
static constexpr float TX_LEAD_SILENCE_MS = 1000.0f;
static constexpr float TX_TAIL_SILENCE_MS = 1000.0f;
static constexpr float DC_REMOVER_DECAY_SEC = 0.25f;
static constexpr adc1_channel_t AUDIO_IN_ADC1_CHANNEL = ADC1_CHANNEL_6;  // GPIO34

// Audio streams
I2SStream dac;
AudioInfo txInfo(AUDIO_SAMPLE_RATE_HZ, 1, 16);
AnalogAudioStream adc;
AudioInfo rxInfo(AUDIO_SAMPLE_RATE_HZ, 1, 16);
HardwareSerial radio_serial(2);
DRA818 radio(&radio_serial, DEFAULT_RF_MODULE_TYPE);
bool adc_active = false;
bool dac_active = false;
float dc_prev = 0.0f;

static int16_t rx_pcm_i16[TX_BUFFER_SAMPLES];
static float tx_mod_buffer[TX_BUFFER_SAMPLES];
static int16_t tx_pcm_i16[TX_BUFFER_SAMPLES];

// KISS RX state
static bool kiss_in_frame = false;
static bool kiss_escape = false;
static uint8_t kiss_port_cmd = 0;
static bool kiss_have_cmd = false;
static uint8_t kiss_payload_buf[KISS_MAX_FRAME];
static size_t kiss_payload_len = 0;

static void on_rx_packet(const uint8_t *frame, size_t len, int);
static void on_tx_samples(const float *samples, size_t count);
AfskDemodulator demod(AUDIO_SAMPLE_RATE_HZ, 2, 0, on_rx_packet);
AfskModulator mod(AUDIO_SAMPLE_RATE_HZ, on_tx_samples);

static void kiss_write_escaped(uint8_t b) {
    if (b == KISS_FEND) {
        Serial.write(KISS_FESC);
        Serial.write(KISS_TFEND);
    } else if (b == KISS_FESC) {
        Serial.write(KISS_FESC);
        Serial.write(KISS_TFESC);
    } else {
        Serial.write(b);
    }
}

static void kiss_send_frame(const uint8_t *data, size_t len) {
    Serial.write(KISS_FEND);
    Serial.write(KISS_CMD_DATA);
    for (size_t i = 0; i < len; i++) {
        kiss_write_escaped(data[i]);
    }
    Serial.write(KISS_FEND);
}

static void on_rx_packet(const uint8_t *frame, size_t len, int) {
    kiss_send_frame(frame, len);
}

static void on_tx_samples(const float *samples, size_t count) {
    if (!samples || count == 0) return;
    if (count > TX_BUFFER_SAMPLES) count = TX_BUFFER_SAMPLES;
    for (size_t i = 0; i < count; i++) {
        float s = samples[i] * TX_GAIN;
        if (s > 1.0f) s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        tx_pcm_i16[i] = (int16_t)lroundf(s * 32767.0f);
    }
    if (dac_active) {
        dac.write((uint8_t *)tx_pcm_i16, count * sizeof(int16_t));
    }
}

static void apply_adc_bias_and_attenuation() {
    const uint8_t bias_code = (uint8_t)lroundf((255.0f / 3.3f) * DEFAULT_ADC_BIAS_VOLTAGE);
    dac_output_enable(DAC_CHANNEL_2);
    dac_output_voltage(DAC_CHANNEL_2, bias_code);
    adc1_config_channel_atten(AUDIO_IN_ADC1_CHANNEL, DEFAULT_ADC_ATTENUATION);
}

static inline int16_t remove_dc(int16_t x) {
    const float alpha = 1.0f - expf(-1.0f / (AUDIO_SAMPLE_RATE_HZ * (DC_REMOVER_DECAY_SEC / logf(2.0f))));
    dc_prev = alpha * (float)x + (1.0f - alpha) * dc_prev;
    float y = (float)x - dc_prev;
    if (y > 32767.0f) y = 32767.0f;
    if (y < -32768.0f) y = -32768.0f;
    return (int16_t)y;
}

static void switch_to_rx_audio() {
    if (dac_active) {
        dac.end();
        dac_active = false;
        delay(2);
    }
    if (!adc_active) {
        apply_adc_bias_and_attenuation();
        delay(100);
        auto config = adc.defaultConfig(RX_MODE);
        config.copyFrom(rxInfo);
        config.is_auto_center_read = false;
        config.use_apll = true;
        config.auto_clear = false;
        config.adc_pin = DEFAULT_PIN_AUDIO_IN;
        config.sample_rate = AUDIO_SAMPLE_RATE_HZ * 1.02; // 2% over sample rate to avoid buffer underruns
        adc_active = adc.begin(config);
    }
}

static void switch_to_tx_audio() {
    if (adc_active) {
        adc.end();
        adc_active = false;
        delay(2);
    }
    dac_output_disable(DAC_CHANNEL_2);
    if (!dac_active) {
        auto config = dac.defaultConfig(TX_MODE);
        config.copyFrom(txInfo);
        config.pin_data = DEFAULT_PIN_AUDIO_OUT;
        config.pin_ws = 27;
        config.use_apll = true;
        config.auto_clear = false;
        config.signal_type = PDM;
        dac_active = dac.begin(config);
    }
}

static void begin_tx() {
    demod.flush();
    switch_to_tx_audio();
    digitalWrite(DEFAULT_PIN_PTT, LOW);
    digitalWrite(DEFAULT_PIN_LED, HIGH);
    delay(5);
}

static void end_tx() {
    digitalWrite(DEFAULT_PIN_PTT, HIGH);
    digitalWrite(DEFAULT_PIN_LED, LOW);
    delay(5);
    switch_to_rx_audio();
}

static void handle_kiss_rx() {
    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        if (b == KISS_FEND) {
            if (kiss_in_frame && kiss_payload_len > 0 && kiss_port_cmd == KISS_CMD_DATA) {
                begin_tx();
                mod.modulate(kiss_payload_buf, kiss_payload_len, tx_mod_buffer, TX_BUFFER_SAMPLES, TX_LEAD_SILENCE_MS, TX_TAIL_SILENCE_MS);
                end_tx();
            }
            kiss_in_frame = true;
            kiss_escape = false;
            kiss_payload_len = 0;
            kiss_port_cmd = 0;
            kiss_have_cmd = false;
            continue;
        }
        if (!kiss_in_frame) continue;
        if (kiss_escape) {
            if (b == KISS_TFEND) b = KISS_FEND;
            else if (b == KISS_TFESC) b = KISS_FESC;
            kiss_escape = false;
        } else if (b == KISS_FESC) {
            kiss_escape = true;
            continue;
        }
        if (!kiss_have_cmd) {
            kiss_port_cmd = b;
            kiss_have_cmd = true;
            continue;
        }
        if (kiss_payload_len < KISS_MAX_FRAME) {
            kiss_payload_buf[kiss_payload_len++] = b;
        }
    }
}

static void reboot_after_radio_init_failure(const char *reason) {
    Serial.print("RF init failed: ");
    Serial.println(reason);
    Serial.println("Rebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
}

static void init_radio_module() {
    radio_serial.begin(9600, SERIAL_8N1, DEFAULT_PIN_RF_RXD, DEFAULT_PIN_RF_TXD);
    bool hs_ok = false;
    for (uint8_t i = 0; i < RF_HANDSHAKE_RETRIES; i++) {
        if (radio.handshake()) {
            hs_ok = true;
            break;
        }
        Serial.print("RF handshake failed [");
        Serial.print((unsigned)i + 1);
        Serial.print("/");
        Serial.print((unsigned)RF_HANDSHAKE_RETRIES);
        Serial.println("]");
        delay(100);
    }
    if (!hs_ok) {
        reboot_after_radio_init_failure("no handshake");
        return;
    }
    bool grp_ok = radio.group(DEFAULT_RF_BANDWIDTH,
                            DEFAULT_RF_FREQ_TX,
                            DEFAULT_RF_FREQ_RX,
                            DEFAULT_RF_CTCSS_TX,
                            DEFAULT_RF_SQUELCH,
                            DEFAULT_RF_CTCSS_RX);
    bool vol_ok = radio.volume(DEFAULT_VOLUME);
    bool fil_ok = radio.filters(DEFAULT_RF_PREEMPH, DEFAULT_RF_HIGHPASS, DEFAULT_RF_LOWPASS);

    Serial.print("RF init result: group=");
    Serial.print(grp_ok ? "ok" : "fail");
    Serial.print(" volume=");
    Serial.print(vol_ok ? "ok" : "fail");
    Serial.print(" filters=");
    Serial.println(fil_ok ? "ok" : "fail");
    if (!(grp_ok && vol_ok && fil_ok)) {
        reboot_after_radio_init_failure("group/volume/filters");
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(DEFAULT_PIN_PTT, OUTPUT);
    digitalWrite(DEFAULT_PIN_PTT, HIGH);
    pinMode(DEFAULT_PIN_PD, OUTPUT);
    digitalWrite(DEFAULT_PIN_PD, HIGH);
    delay(RF_POWERUP_DELAY_MS);
    pinMode(DEFAULT_PIN_LED, OUTPUT);
    digitalWrite(DEFAULT_PIN_LED, LOW);
    init_radio_module();
    switch_to_rx_audio();
}

void loop() {
    handle_kiss_rx();
    if (adc_active) {
        size_t bytes =  adc.readBytes((uint8_t *)rx_pcm_i16, sizeof(rx_pcm_i16));
        if (bytes > 0) {
            size_t samples = bytes / sizeof(int16_t);
            for (size_t i = 0; i < samples; i++) {
                rx_pcm_i16[i] = remove_dc(rx_pcm_i16[i]);
            }
            demod.processSamples(rx_pcm_i16, samples);
        }
    }   
}
