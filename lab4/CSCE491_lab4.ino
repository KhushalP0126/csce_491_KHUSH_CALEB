#include <Arduino.h>
#include <math.h>

#include "esp_cpu.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/ledc_reg.h"
#include "soc/soc.h"
#include "soc/system_reg.h"

#if __has_include("array.h")
#include "array.h"
#define HAS_AUDIO_ARRAY 1
#else
#define HAS_AUDIO_ARRAY 0
#endif

// ===== Pin Configuration =====
#define AUDIO_PIN 33
#define LED_PIN 14

// ===== Audio Configuration =====
#define AUDIO_RES_BITS 8U
#define AUDIO_PWM_RATE 62500UL
#define AUDIO_MAX_SAMPLE ((1U << AUDIO_RES_BITS) - 1U)
#define LEDC_BASE_CLK_HZ 80000000UL

#if HAS_AUDIO_ARRAY
#define AUDIO_SAMPLE_RATE sampleRate
#else
#define AUDIO_SAMPLE_RATE 16000U
#endif

// ===== LED Configuration =====
#define NUM_LEDS 100U
#define AUDIO_LED_UPDATE_INTERVAL 256U
#define NUM_BANDS 5U
#define LEDS_PER_BAND (NUM_LEDS / NUM_BANDS)

#ifndef F_CPU
#define F_CPU 240000000UL
#endif

#define WS2812_T0H_NS 350U
#define WS2812_T1H_NS 700U
#define WS2812_TOTAL_NS 1250U
#define WS2812_RESET_US 80U

#define NS_TO_CPU_CYCLES(ns) ((uint32_t)(((uint64_t)F_CPU * (uint64_t)(ns) + 999999999ULL) / 1000000000ULL))
#define LED_PIN_MASK (1UL << LED_PIN)
#define AUDIO_PIN_HIGH_MASK (1UL << (AUDIO_PIN - 32U))

// ===== Globals =====
static uint32_t sample_period_us;
static uint32_t sample_period_rem;
static uint32_t g_led_colors[NUM_LEDS];
static float g_lowpass_120 = 0.0f;
static float g_lowpass_300 = 0.0f;
static float g_lowpass_600 = 0.0f;
static float g_lowpass_1200 = 0.0f;
static float g_lowpass_2400 = 0.0f;
static float g_band_envelopes[NUM_BANDS] = {0.0f};

// ===== Audio Functions =====
void setupAudio();
void setup_LEDC();
void updateAudioSample(uint8_t sample);
void play_audio_array_once();
void stop_audio_output();

// ===== LED Functions =====
void setupLedStrip();
void build_audio_visual_frame(uint8_t sample, uint32_t sample_index);
void send_led_frame();
void clear_leds();
void transmit_led_signal(uint32_t *colors);
uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);
uint32_t scale_color(uint32_t color, float scale);
static inline void wait_cycles(uint32_t cycles);
static inline void led_pin_high();
static inline void led_pin_low();
static inline void send_led_byte(uint8_t value);

void setup() {
  Serial.begin(115200);
  delay(500);

  sample_period_us = 1000000UL / AUDIO_SAMPLE_RATE;
  sample_period_rem = 1000000UL % AUDIO_SAMPLE_RATE;

  setupLedStrip();
  setupAudio();

  #if HAS_AUDIO_ARRAY
  Serial.println("AUDIO ARRAY: playing array.h once");
  play_audio_array_once();
  stop_audio_output();
  #else
  Serial.println("AUDIO ARRAY: array.h not found, skipping playback");
  #endif
}

void loop() {
  delay(1000);
}

// ===== Audio Section =====
void setupAudio() {
  setup_LEDC();
}

void setup_LEDC() {
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_LEDC_CLK_EN);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST);

  PIN_FUNC_SELECT(IO_MUX_GPIO33_REG, PIN_FUNC_GPIO);
  REG_WRITE(GPIO_FUNC33_OUT_SEL_CFG_REG, LEDC_LS_SIG_OUT0_IDX);

  REG_WRITE(LEDC_CONF_REG, LEDC_CLK_EN | (1U << LEDC_APB_CLK_SEL_S));

  // LEDC divider uses an 8-bit fractional part on ESP32-S3.
  uint32_t divider_fixed =
      (uint32_t)lroundf(((float)LEDC_BASE_CLK_HZ * 256.0f) /
                        ((float)AUDIO_PWM_RATE * (float)(1U << AUDIO_RES_BITS)));
  if (divider_fixed < 256U) divider_fixed = 256U;
  if (divider_fixed > LEDC_CLK_DIV_LSTIMER0_V) divider_fixed = LEDC_CLK_DIV_LSTIMER0_V;

  uint32_t timer_conf = 0U;
  timer_conf |= (divider_fixed << LEDC_CLK_DIV_LSTIMER0_S);
  timer_conf |= ((uint32_t)AUDIO_RES_BITS << LEDC_LSTIMER0_DUTY_RES_S);
  timer_conf &= ~LEDC_LSTIMER0_PAUSE;
  timer_conf &= ~LEDC_LSTIMER0_RST;
  REG_WRITE(LEDC_LSTIMER0_CONF_REG, timer_conf);
  REG_SET_BIT(LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_PARA_UP);

  REG_WRITE(LEDC_LSCH0_HPOINT_REG, 0U);
  REG_WRITE(LEDC_LSCH0_DUTY_REG, 0U);
  REG_WRITE(LEDC_LSCH0_CONF0_REG, LEDC_SIG_OUT_EN_LSCH0);
  REG_WRITE(LEDC_LSCH0_CONF1_REG, 0U);
  REG_WRITE(LEDC_INT_CLR_REG, LEDC_LSTIMER0_OVF_INT_CLR);
}

void updateAudioSample(uint8_t sample) {
  REG_WRITE(LEDC_LSCH0_DUTY_REG, ((uint32_t)sample & AUDIO_MAX_SAMPLE) << 4);
  REG_SET_BIT(LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);
  REG_SET_BIT(LEDC_LSCH0_CONF1_REG, LEDC_DUTY_START_LSCH0);
}

void play_audio_array_once() {
  #if HAS_AUDIO_ARRAY
  uint32_t sample_count = sizeof(sampleArray) / sizeof(sampleArray[0]);
  uint32_t next_sample_time = micros();
  uint32_t rem_accumulator = 0U;

  for (uint32_t i = 0; i < sample_count; ++i) {
    uint8_t sample = sampleArray[i] & 0xFFU;
    updateAudioSample(sample);

    if ((i % AUDIO_LED_UPDATE_INTERVAL) == 0U) {
      build_audio_visual_frame(sample, i);
      send_led_frame();
    }

    next_sample_time += sample_period_us;
    rem_accumulator += sample_period_rem;
    if (rem_accumulator >= AUDIO_SAMPLE_RATE) {
      next_sample_time += 1U;
      rem_accumulator -= AUDIO_SAMPLE_RATE;
    }

    while ((int32_t)(micros() - next_sample_time) < 0) {}
  }

  clear_leds();
  #endif
}

void stop_audio_output() {
  updateAudioSample(0U);
  REG_WRITE(LEDC_LSCH0_CONF0_REG, 0U);
  REG_WRITE(GPIO_FUNC33_OUT_SEL_CFG_REG, SIG_GPIO_OUT_IDX);
  REG_WRITE(GPIO_ENABLE1_W1TS_REG, AUDIO_PIN_HIGH_MASK);
  REG_WRITE(GPIO_OUT1_W1TC_REG, AUDIO_PIN_HIGH_MASK);
}

// ===== LED Section =====
void setupLedStrip() {
  PIN_FUNC_SELECT(IO_MUX_GPIO14_REG, PIN_FUNC_GPIO);
  REG_WRITE(GPIO_FUNC14_OUT_SEL_CFG_REG, SIG_GPIO_OUT_IDX);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, LED_PIN_MASK);
  REG_WRITE(GPIO_OUT_W1TC_REG, LED_PIN_MASK);
  clear_leds();
}

static inline void wait_cycles(uint32_t cycles) {
  uint32_t start = esp_cpu_get_cycle_count();
  while ((uint32_t)(esp_cpu_get_cycle_count() - start) < cycles) {
  }
}

static inline void led_pin_high() {
  REG_WRITE(GPIO_OUT_W1TS_REG, LED_PIN_MASK);
}

static inline void led_pin_low() {
  REG_WRITE(GPIO_OUT_W1TC_REG, LED_PIN_MASK);
}

static inline void send_led_byte(uint8_t value) {
  const uint32_t t0h_cycles = NS_TO_CPU_CYCLES(WS2812_T0H_NS);
  const uint32_t t1h_cycles = NS_TO_CPU_CYCLES(WS2812_T1H_NS);
  const uint32_t total_cycles = NS_TO_CPU_CYCLES(WS2812_TOTAL_NS);

  for (int bit = 7; bit >= 0; --bit) {
    bool one = ((value >> bit) & 0x01U) != 0;
    uint32_t bit_start = esp_cpu_get_cycle_count();

    led_pin_high();
    while ((uint32_t)(esp_cpu_get_cycle_count() - bit_start) < (one ? t1h_cycles : t0h_cycles)) {
    }

    led_pin_low();
    while ((uint32_t)(esp_cpu_get_cycle_count() - bit_start) < total_cycles) {
    }
  }
}

void transmit_led_signal(uint32_t *colors) {
  noInterrupts();

  for (uint32_t led = 0; led < NUM_LEDS; ++led) {
    uint8_t green = (uint8_t)((colors[led] >> 8) & 0xFFU);
    uint8_t red = (uint8_t)((colors[led] >> 16) & 0xFFU);
    uint8_t blue = (uint8_t)(colors[led] & 0xFFU);
    send_led_byte(green);
    send_led_byte(red);
    send_led_byte(blue);
  }

  interrupts();
  wait_cycles(NS_TO_CPU_CYCLES(WS2812_RESET_US * 1000U));
}

void build_audio_visual_frame(uint8_t sample, uint32_t sample_index) {
  const float x = ((float)sample - 128.0f) / 128.0f;
  const uint32_t band_colors[NUM_BANDS] = {
      make_color(255, 80, 0),
      make_color(255, 0, 70),
      make_color(60, 255, 40),
      make_color(0, 180, 255),
      make_color(180, 0, 255)
  };

  g_lowpass_120 += 0.045f * (x - g_lowpass_120);
  g_lowpass_300 += 0.10f * (x - g_lowpass_300);
  g_lowpass_600 += 0.18f * (x - g_lowpass_600);
  g_lowpass_1200 += 0.33f * (x - g_lowpass_1200);
  g_lowpass_2400 += 0.55f * (x - g_lowpass_2400);

  const float band_levels[NUM_BANDS] = {
      fabsf(g_lowpass_120),
      fabsf(g_lowpass_300 - g_lowpass_120),
      fabsf(g_lowpass_600 - g_lowpass_300),
      fabsf(g_lowpass_1200 - g_lowpass_600),
      fabsf(g_lowpass_2400 - g_lowpass_1200),
  };

  for (uint32_t i = 0; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }

  for (uint32_t band = 0; band < NUM_BANDS; ++band) {
    float target = band_levels[band] * 2.8f;
    if (target > 1.0f) target = 1.0f;

    float response = (target > g_band_envelopes[band]) ? 0.50f : 0.15f;
    g_band_envelopes[band] += response * (target - g_band_envelopes[band]);

    uint32_t lit_count = (uint32_t)(g_band_envelopes[band] * LEDS_PER_BAND + 0.5f);
    uint32_t start = band * LEDS_PER_BAND;
    uint32_t end = start + LEDS_PER_BAND;
    uint32_t peak = start + ((sample_index / AUDIO_LED_UPDATE_INTERVAL) % LEDS_PER_BAND);

    for (uint32_t led = start; led < end; ++led) {
      uint32_t color = 0U;
      uint32_t relative = led - start;

      if (relative < lit_count) {
        float fade = 1.0f - ((float)relative / (float)LEDS_PER_BAND);
        color = scale_color(band_colors[band], 0.25f + 0.75f * fade);
      }

      if ((led == peak) && (g_band_envelopes[band] > 0.08f)) {
        color = scale_color(band_colors[band], 1.0f);
      }

      g_led_colors[led] = color;
    }
  }
}

void send_led_frame() {
  transmit_led_signal(g_led_colors);
}

void clear_leds() {
  for (uint32_t i = 0; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }
  send_led_frame();
}

uint32_t make_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

uint32_t scale_color(uint32_t color, float scale) {
  if (scale < 0.0f) scale = 0.0f;
  if (scale > 1.0f) scale = 1.0f;

  uint8_t r = (uint8_t)(((color >> 16) & 0xFFU) * scale);
  uint8_t g = (uint8_t)(((color >> 8) & 0xFFU) * scale);
  uint8_t b = (uint8_t)((color & 0xFFU) * scale);
  return make_color(r, g, b);
}
