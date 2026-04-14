#include <Arduino.h>
#include <math.h>

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

#define AUDIO_PIN 33
#define LED_PIN 14

#define PWM_CHANNEL 0
#define PWM_RES_BITS 8
#define AUDIO_PWM_RATE 62500U
#define AUDIO_MAX_SAMPLE ((1U << PWM_RES_BITS) - 1U)
#define LEDC_BASE_CLK_HZ 80000000UL

#define NUM_LEDS 100
#define AUDIO_LED_UPDATE_INTERVAL 256
#define NUM_BANDS 5
#define LEDS_PER_BAND (NUM_LEDS / NUM_BANDS)

#if HAS_AUDIO_ARRAY
#define AUDIO_SAMPLE_RATE sampleRate
#else
#define AUDIO_SAMPLE_RATE 16000U
#endif

static uint32_t sample_period_us;
static uint32_t sample_period_rem;
static rmt_data_t led_frame[NUM_LEDS * 24];
static float g_lowpass_120 = 0.0f;
static float g_lowpass_300 = 0.0f;
static float g_lowpass_600 = 0.0f;
static float g_lowpass_1200 = 0.0f;
static float g_lowpass_2400 = 0.0f;
static float g_band_envelopes[NUM_BANDS] = {0.0f};

void setup_LEDC();
void update_audio_sample(uint8_t sample);
void setup_RMT();
void play_audio_array_once();
void stop_audio_output();

void build_audio_visual_frame(uint8_t sample, uint32_t sample_index);
void send_led_frame();
void clear_leds();
uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);
uint32_t scale_color(uint32_t color, float scale);
void encode_color_to_rmt(uint32_t color, rmt_data_t *out);

void setup() {
  Serial.begin(115200);
  delay(500);

  sample_period_us = 1000000UL / AUDIO_SAMPLE_RATE;
  sample_period_rem = 1000000UL % AUDIO_SAMPLE_RATE;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setup_LEDC();
  setup_RMT();

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

void setup_LEDC() {
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_LEDC_CLK_EN);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST);

  PIN_FUNC_SELECT(IO_MUX_GPIO33_REG, PIN_FUNC_GPIO);
  REG_WRITE(GPIO_FUNC33_OUT_SEL_CFG_REG, LEDC_LS_SIG_OUT0_IDX);

  REG_WRITE(LEDC_CONF_REG, LEDC_CLK_EN | (1U << LEDC_APB_CLK_SEL_S));

  uint32_t divider_fixed =
      (uint32_t)lroundf(((float)LEDC_BASE_CLK_HZ * 256.0f) /
                        ((float)AUDIO_PWM_RATE * (float)(1U << PWM_RES_BITS)));
  if (divider_fixed < 256U) divider_fixed = 256U;
  if (divider_fixed > LEDC_CLK_DIV_LSTIMER0_V) divider_fixed = LEDC_CLK_DIV_LSTIMER0_V;

  uint32_t timer_conf = 0U;
  timer_conf |= (divider_fixed << LEDC_CLK_DIV_LSTIMER0_S);
  timer_conf |= ((uint32_t)PWM_RES_BITS << LEDC_LSTIMER0_DUTY_RES_S);
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

void update_audio_sample(uint8_t sample) {
  REG_WRITE(LEDC_LSCH0_DUTY_REG, ((uint32_t)sample & AUDIO_MAX_SAMPLE) << 4);
  REG_SET_BIT(LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);
  REG_SET_BIT(LEDC_LSCH0_CONF1_REG, LEDC_DUTY_START_LSCH0);
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

void play_audio_array_once() {
  #if HAS_AUDIO_ARRAY
  uint32_t sample_count = sizeof(sampleArray) / sizeof(sampleArray[0]);
  uint32_t next_sample_time = micros();
  uint32_t rem_accumulator = 0U;

  for (uint32_t i = 0; i < sample_count; ++i) {
    uint8_t sample = sampleArray[i] & 0xFFU;
    update_audio_sample(sample);

    if (((i % AUDIO_LED_UPDATE_INTERVAL) == 0U) && rmtTransmitCompleted(LED_PIN)) {
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
  update_audio_sample(0U);
  REG_WRITE(LEDC_LSCH0_CONF0_REG, 0U);
  REG_WRITE(GPIO_FUNC33_OUT_SEL_CFG_REG, SIG_GPIO_OUT_IDX);
  REG_WRITE(GPIO_ENABLE1_W1TS_REG, (1UL << (AUDIO_PIN - 32U)));
  REG_WRITE(GPIO_OUT1_W1TC_REG, (1UL << (AUDIO_PIN - 32U)));
}

void build_audio_visual_frame(uint8_t sample, uint32_t sample_index) {
  const float x = ((float)sample - 128.0f) / 128.0f;
  const uint32_t band_colors[NUM_BANDS] = {
      make_color(255, 0, 0),
      make_color(255, 0, 0),
      make_color(0, 0, 255),
      make_color(0, 255, 0),
      make_color(0, 255, 0)
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
    encode_color_to_rmt(0U, &led_frame[i * 24]);
  }

  for (uint32_t band = 0; band < NUM_BANDS; ++band) {
    float target = band_levels[band] * 2.4f;
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
        color = scale_color(band_colors[band], 0.18f + 0.42f * fade);
      }

      if ((led == peak) && (g_band_envelopes[band] > 0.08f)) {
        color = scale_color(band_colors[band], 0.72f);
      }

      encode_color_to_rmt(color, &led_frame[led * 24]);
    }
  }
}

void send_led_frame() {
  if (!rmtWriteAsync(LED_PIN, led_frame, NUM_LEDS * 24)) {
  }
}

void clear_leds() {
  for (uint32_t i = 0; i < NUM_LEDS; ++i) {
    encode_color_to_rmt(0U, &led_frame[i * 24]);
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
