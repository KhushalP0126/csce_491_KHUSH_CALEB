#include <Arduino.h>
#include <math.h>

#if __has_include("array.h")
#include "array.h"
#define HAS_AUDIO_ARRAY 1
#else
#define HAS_AUDIO_ARRAY 0
#endif

#define AUDIO_PIN 33
#define LED_PIN 14

#define PWM_CHANNEL 0
#define PWM_RES_BITS 8

#define NUM_LEDS 100
#define COLOR_STEPS 96
#define COLOR_STEP_DELAY_MS 20
#define SPEAKER_TEST_HZ 880.0f
#define SPEAKER_TEST_MS 250U

#if HAS_AUDIO_ARRAY
#define AUDIO_SAMPLE_RATE sampleRate
#else
#define AUDIO_SAMPLE_RATE 16000U
#endif

static uint32_t sample_period_us;
static rmt_data_t led_frame[NUM_LEDS * 24];

void setup_LEDC();
void setup_RMT();
void play_square_wave_once(float frequency_hz, uint32_t duration_ms);
void play_audio_array_once();
void stop_audio_output();

void build_single_led_frame(uint32_t active_index, uint32_t color);
void send_led_frame();
void clear_leds();
uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);
uint32_t wheel_color(uint8_t pos);
void encode_color_to_rmt(uint32_t color, rmt_data_t *out);
void run_setup_stage();

void setup() {
  Serial.begin(115200);
  delay(500);

  sample_period_us = 1000000UL / AUDIO_SAMPLE_RATE;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setup_LEDC();
  setup_RMT();

  Serial.println("SETUP STAGE: speaker + LED hardware test starting");
  run_setup_stage();

  #if HAS_AUDIO_ARRAY
  Serial.println("AUDIO ARRAY: playing array.h once");
  setup_LEDC();
  play_audio_array_once();
  stop_audio_output();
  #else
  Serial.println("AUDIO ARRAY: array.h not found, skipping playback");
  #endif

  Serial.println("SETUP STAGE COMPLETE");
}

void loop() {
  delay(1000);
}

void setup_LEDC() {
  if (!ledcAttachChannel(AUDIO_PIN, AUDIO_SAMPLE_RATE, PWM_RES_BITS, PWM_CHANNEL)) {
    Serial.println("LEDC attach failed!");
    while (true) {
      delay(1000);
    }
  }

  ledcWriteChannel(PWM_CHANNEL, 0);
}

void setup_RMT() {
  if (!rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) {
    Serial.println("RMT init failed!");
    while (true) {
      delay(1000);
    }
  }

  rmtSetEOT(LED_PIN, 0);
  digitalWrite(LED_PIN, LOW);
}

void play_square_wave_once(float frequency_hz, uint32_t duration_ms) {
  uint32_t total_samples = (duration_ms * AUDIO_SAMPLE_RATE) / 1000U;
  if (total_samples == 0U) {
    total_samples = 1U;
  }

  float phase = 0.0f;
  float phase_step = frequency_hz / (float)AUDIO_SAMPLE_RATE;

  for (uint32_t i = 0; i < total_samples; ++i) {
    uint8_t sample = (phase < 0.5f) ? 255U : 0U;
    ledcWriteChannel(PWM_CHANNEL, sample);
    delayMicroseconds(sample_period_us);

    phase += phase_step;
    if (phase >= 1.0f) {
      phase -= 1.0f;
    }
  }
}

void play_audio_array_once() {
  #if HAS_AUDIO_ARRAY
  uint32_t sample_count = sizeof(sampleArray) / sizeof(sampleArray[0]);
  for (uint32_t i = 0; i < sample_count; ++i) {
    ledcWriteChannel(PWM_CHANNEL, sampleArray[i] & 0xFFU);
    delayMicroseconds(sample_period_us);
  }
  #endif
}

void stop_audio_output() {
  ledcWriteChannel(PWM_CHANNEL, 0);
  ledcDetach(AUDIO_PIN);
  pinMode(AUDIO_PIN, OUTPUT);
  digitalWrite(AUDIO_PIN, LOW);
}

void build_single_led_frame(uint32_t active_index, uint32_t color) {
  for (uint32_t i = 0; i < NUM_LEDS; ++i) {
    uint32_t pixel_color = (i == active_index) ? color : 0U;
    encode_color_to_rmt(pixel_color, &led_frame[i * 24]);
  }
}

void send_led_frame() {
  while (!rmtWrite(LED_PIN, led_frame, NUM_LEDS * 24, 100)) {
    delay(1);
  }
}

void clear_leds() {
  build_single_led_frame(NUM_LEDS, 0U);
  send_led_frame();
}

uint32_t make_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

uint32_t wheel_color(uint8_t pos) {
  pos = 255U - pos;
  if (pos < 85U) {
    return make_color(255U - pos * 3U, 0U, pos * 3U);
  }
  if (pos < 170U) {
    pos -= 85U;
    return make_color(0U, pos * 3U, 255U - pos * 3U);
  }

  pos -= 170U;
  return make_color(pos * 3U, 255U - pos * 3U, 0U);
}

void encode_color_to_rmt(uint32_t color, rmt_data_t *out) {
  uint8_t g = (uint8_t)((color >> 8) & 0xFFU);
  uint8_t r = (uint8_t)((color >> 16) & 0xFFU);
  uint8_t b = (uint8_t)(color & 0xFFU);
  uint8_t bytes[3] = {g, r, b};

  int idx = 0;
  for (int byte_i = 0; byte_i < 3; ++byte_i) {
    for (int bit = 7; bit >= 0; --bit) {
      bool one = ((bytes[byte_i] >> bit) & 0x01U) != 0;

      if (one) {
        out[idx].level0 = 1;
        out[idx].duration0 = 8;
        out[idx].level1 = 0;
        out[idx].duration1 = 4;
      } else {
        out[idx].level0 = 1;
        out[idx].duration0 = 4;
        out[idx].level1 = 0;
        out[idx].duration1 = 8;
      }
      ++idx;
    }
  }
}

void run_setup_stage() {
  clear_leds();

  play_square_wave_once(SPEAKER_TEST_HZ, SPEAKER_TEST_MS);
  stop_audio_output();

  for (uint32_t led_index = 0; led_index < NUM_LEDS; ++led_index) {
    for (uint32_t step = 0; step < COLOR_STEPS; ++step) {
      uint8_t wheel_pos = (uint8_t)((step * 256U) / COLOR_STEPS);
      build_single_led_frame(led_index, wheel_color(wheel_pos));
      send_led_frame();
      delay(COLOR_STEP_DELAY_MS);
    }
  }

  clear_leds();
}
