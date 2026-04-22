#include <Arduino.h>
#include <math.h>

#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/ledc_reg.h"
#include "soc/rmt_reg.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/system_reg.h"

#if __has_include("array.h")
#include "array.h"
#define HAS_AUDIO_ARRAY 1
#else
#define HAS_AUDIO_ARRAY 0
#endif

namespace {

// The lab handout has stale pin guidance. The board schematic validates:
// - SPKR is routed to GPIO33
// - WS2812 DIN is routed to GPIO14
#define AUDIO_PIN 33U
#define LED_PIN 14U

#define AUDIO_GPIO_OUT_SEL_REG GPIO_FUNC33_OUT_SEL_CFG_REG
#define LED_GPIO_OUT_SEL_REG GPIO_FUNC14_OUT_SEL_CFG_REG

#define AUDIO_GPIO_IO_MUX_REG IO_MUX_GPIO33_REG
#define LED_GPIO_IO_MUX_REG IO_MUX_GPIO14_REG

#define AUDIO_GPIO_ENABLE_MASK (1UL << (AUDIO_PIN - 32U))
#define LED_GPIO_ENABLE_MASK (1UL << LED_PIN)

#define GPIO_OUT_SEL_LEDC_CH0 73U
#define GPIO_OUT_SEL_RMT_CH0 81U

#define AUDIO_PWM_RES_BITS 8U
#define AUDIO_MAX_SAMPLE ((1U << AUDIO_PWM_RES_BITS) - 1U)
#define LEDC_BASE_CLK_HZ 80000000UL
#define LEDC_TIMER_TICKS_PER_SAMPLE (1UL << AUDIO_PWM_RES_BITS)

#define NUM_LEDS 100U
#define AUDIO_LED_UPDATE_INTERVAL 256U
#define NUM_BANDS 5U
#define LEDS_PER_BAND (NUM_LEDS / NUM_BANDS)

#define RMT_CLK_DIV 1U
#define RMT_MEM_BLOCKS 1U
#define RMT_MEM_ITEMS_PER_BLOCK SOC_RMT_MEM_WORDS_PER_CHANNEL
#define RMT_MEM_ITEMS_TOTAL (RMT_MEM_BLOCKS * RMT_MEM_ITEMS_PER_BLOCK)
#define RMT_MEM_ITEMS_HALF (RMT_MEM_ITEMS_TOTAL / 2U)
#define RMT_T0H_TICKS 32U
#define RMT_T0L_TICKS 68U
#define RMT_T1H_TICKS 64U
#define RMT_T1L_TICKS 36U
#define RMT_TX_TIMEOUT_LOOPS 400000U
#define RMT_MEM_BASE_ADDR (DR_REG_RMT_BASE + 0x800U)

#if HAS_AUDIO_ARRAY
#define AUDIO_SAMPLE_RATE sampleRate
#else
#define AUDIO_SAMPLE_RATE 16000U
#endif

volatile uint32_t gLedColors[NUM_LEDS] = {0U};
float gLowpass120 = 0.0f;
float gLowpass300 = 0.0f;
float gLowpass600 = 0.0f;
float gLowpass1200 = 0.0f;
float gLowpass2400 = 0.0f;
float gBandEnvelopes[NUM_BANDS] = {0.0f};

void appSetup();
void appLoop();
void fill_leds(uint32_t color);
void clear_leds();

void setup_LEDC();
void update_PWM(uint32_t initial, uint32_t sample);
void stop_audio_output();

void setup_RMT();
void transmit_led_signal(uint32_t *colors);

void play_audio_array_once();
void wait_us(uint32_t delay_us);

uint32_t encode_rmt_word(uint32_t duration0, uint32_t level0, uint32_t duration1, uint32_t level1);
uint32_t color_to_grb(uint32_t color);
void rmt_load_color(uint32_t color);
bool rmt_send_and_wait();

uint32_t map_colors(uint8_t value);
void build_visuals(uint8_t sample, uint32_t sample_index);

uint32_t encode_rmt_word(uint32_t duration0, uint32_t level0, uint32_t duration1, uint32_t level1) {
  return (duration0 & 0x7FFFU) |
         ((level0 & 0x1U) << 15) |
         ((duration1 & 0x7FFFU) << 16) |
         ((level1 & 0x1U) << 31);
}

void wait_us(uint32_t delay_us) {
  esp_rom_delay_us(delay_us);
}

void fill_leds(uint32_t color) {
  for (uint32_t i = 0; i < NUM_LEDS; ++i) {
    gLedColors[i] = color;
  }
}

void clear_leds() {
  fill_leds(0U);
  transmit_led_signal((uint32_t *)gLedColors);
}

void appSetup() {
  setup_LEDC();
  setup_RMT();
  stop_audio_output();

  // Startup check: flash the whole matrix a known color so it is obvious
  // whether newly flashed code is running.
  fill_leds(0x00FF0000U);
  transmit_led_signal((uint32_t *)gLedColors);
  wait_us(30000000U);

  clear_leds();
}

void appLoop() {
#if HAS_AUDIO_ARRAY
  play_audio_array_once();
  wait_us(250000U);
#else
  wait_us(1000000U);
#endif
}

void setup_LEDC() {
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_LEDC_CLK_EN);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST);

  PIN_FUNC_SELECT(AUDIO_GPIO_IO_MUX_REG, PIN_FUNC_GPIO);
  REG_WRITE(GPIO_ENABLE1_W1TS_REG, AUDIO_GPIO_ENABLE_MASK);
  REG_WRITE(AUDIO_GPIO_OUT_SEL_REG, GPIO_OUT_SEL_LEDC_CH0);

  REG_WRITE(LEDC_CONF_REG, LEDC_CLK_EN | (1U << LEDC_APB_CLK_SEL_S));

  // LEDC's timer divider must generate one 8-bit PWM period per audio sample.
  uint32_t divider_fixed = (uint32_t)lroundf(
      ((float)LEDC_BASE_CLK_HZ * 256.0f) /
      ((float)AUDIO_SAMPLE_RATE * (float)LEDC_TIMER_TICKS_PER_SAMPLE));
  if (divider_fixed < 256U) divider_fixed = 256U;
  if (divider_fixed > LEDC_CLK_DIV_LSTIMER0_V) divider_fixed = LEDC_CLK_DIV_LSTIMER0_V;

  uint32_t timer_conf = 0U;
  timer_conf |= (divider_fixed << LEDC_CLK_DIV_LSTIMER0_S);
  timer_conf |= ((uint32_t)AUDIO_PWM_RES_BITS << LEDC_LSTIMER0_DUTY_RES_S);
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

void update_PWM(uint32_t initial, uint32_t sample) {
  if (!initial && ((REG_READ(LEDC_INT_RAW_REG) & LEDC_LSTIMER0_OVF_INT_RAW) == 0U)) return;

  REG_WRITE(LEDC_INT_CLR_REG, LEDC_LSTIMER0_OVF_INT_CLR);
  REG_WRITE(LEDC_LSCH0_DUTY_REG, (sample & AUDIO_MAX_SAMPLE) << 4);
  REG_SET_BIT(LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);
  REG_SET_BIT(LEDC_LSCH0_CONF1_REG, LEDC_DUTY_START_LSCH0);
}

void stop_audio_output() {
  // Unsigned 8-bit audio is centered at 128, so 50% duty is the quiet resting
  // point and avoids the click produced by forcing the speaker path low.
  update_PWM(1U, AUDIO_MAX_SAMPLE >> 1);
}

void setup_RMT() {
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_RMT_CLK_EN);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST);

  PIN_FUNC_SELECT(LED_GPIO_IO_MUX_REG, PIN_FUNC_GPIO);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, LED_GPIO_ENABLE_MASK);
  REG_WRITE(LED_GPIO_OUT_SEL_REG, GPIO_OUT_SEL_RMT_CH0);

  REG_WRITE(
      RMT_SYS_CONF_REG,
      RMT_CLK_EN |
          RMT_SCLK_ACTIVE |
          (1U << RMT_SCLK_SEL_S) |
          (0U << RMT_SCLK_DIV_NUM_S) |
          RMT_MEM_FORCE_PU |
          RMT_MEM_CLK_FORCE_ON |
          RMT_APB_FIFO_MASK);

  REG_WRITE(
      RMT_CH0CONF0_REG,
      (RMT_MEM_BLOCKS << RMT_MEM_SIZE_CH0_S) |
          (RMT_CLK_DIV << RMT_DIV_CNT_CH0_S) |
          RMT_IDLE_OUT_EN_CH0);

  REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_ERR_INT_CLR | RMT_CH0_TX_THR_EVENT_INT_CLR | RMT_CH0_TX_END_INT_CLR);
}

uint32_t color_to_grb(uint32_t color) {
  uint8_t red = (uint8_t)((color >> 16) & 0xFFU);
  uint8_t green = (uint8_t)((color >> 8) & 0xFFU);
  uint8_t blue = (uint8_t)(color & 0xFFU);
  return ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
}

void rmt_load_color(uint32_t color) {
  volatile uint32_t *ram = (volatile uint32_t *)RMT_MEM_BASE_ADDR;
  uint32_t grb = color_to_grb(color);

  for (int bit = 23; bit >= 0; --bit) {
    bool one = ((grb >> bit) & 0x1U) != 0U;
    ram[23 - bit] =
        one ? encode_rmt_word(RMT_T1H_TICKS, 1U, RMT_T1L_TICKS, 0U)
            : encode_rmt_word(RMT_T0H_TICKS, 1U, RMT_T0L_TICKS, 0U);
  }

  ram[24] = 0U;
}

bool rmt_send_and_wait() {
  volatile uint32_t *conf0_reg = (volatile uint32_t *)RMT_CH0CONF0_REG;
  volatile uint32_t *int_raw_reg = (volatile uint32_t *)RMT_INT_RAW_REG;

  const uint32_t base =
      (RMT_MEM_BLOCKS << RMT_MEM_SIZE_CH0_S) |
      (RMT_CLK_DIV << RMT_DIV_CNT_CH0_S) |
      RMT_IDLE_OUT_EN_CH0;


  REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_ERR_INT_CLR | RMT_CH0_TX_THR_EVENT_INT_CLR | RMT_CH0_TX_END_INT_CLR);

  // Reset channel read pointer so each transfer starts from symbol 0.
  *conf0_reg = base | RMT_MEM_RD_RST_CH0;
  *conf0_reg = base;

  // Latch register changes into the channel state machine.
  *conf0_reg = base | RMT_CONF_UPDATE_CH0;
  *conf0_reg = base;

  // Pulse TX start (do not leave the bit high).
  *conf0_reg = base | RMT_TX_START_CH0;
  *conf0_reg = base;

  uint32_t timeout = RMT_TX_TIMEOUT_LOOPS;
  while (timeout-- > 0U) {
    uint32_t raw = *int_raw_reg;

    if ((raw & RMT_CH0_TX_END_INT_RAW) != 0U) {
      REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_CLR);
      return true;
    }

    if ((raw & RMT_CH0_ERR_INT_RAW) != 0U) {
      REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_ERR_INT_CLR | RMT_CH0_TX_THR_EVENT_INT_CLR | RMT_CH0_TX_END_INT_CLR);
      return false;
    }
  }

  REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_ERR_INT_CLR | RMT_CH0_TX_THR_EVENT_INT_CLR | RMT_CH0_TX_END_INT_CLR);
  return false;
}

void transmit_led_signal(uint32_t *colors) {
  // Keep the entire WS2812 frame contiguous; ISR jitter between per-LED
  // transfers can create a >50 us latch gap and truncate the chain.
  noInterrupts();
  for (uint32_t i = 0; i < NUM_LEDS; ++i) {
    rmt_load_color(colors[i]);
    if (!rmt_send_and_wait()) {
      REG_SET_BIT(RMT_CH0CONF0_REG, RMT_MEM_RD_RST_CH0);
      REG_CLR_BIT(RMT_CH0CONF0_REG, RMT_MEM_RD_RST_CH0);
      REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_ERR_INT_CLR | RMT_CH0_TX_THR_EVENT_INT_CLR | RMT_CH0_TX_END_INT_CLR);
      break;
    }
  }
  interrupts();

  wait_us(80U);
}

uint32_t map_colors(uint8_t value) {
  uint8_t red = 0U;
  uint8_t green = 0U;
  uint8_t blue = 0U;

  if (value <= 51U) {
    red = 0U;
    green = value * 5U;
    blue = 255U;
  } else if (value <= 102U) {
    red = 0U;
    green = 255U;
    blue = (uint8_t)(255U - (value - 51U) * 5U);
  } else if (value <= 153U) {
    red = (uint8_t)((value - 102U) * 5U);
    green = 255U;
    blue = 0U;
  } else if (value <= 204U) {
    red = 255U;
    green = (uint8_t)(255U - (value - 153U) * 5U);
    blue = 0U;
  } else {
    red = 255U;
    green = 0U;
    blue = (value < 230U) ? 0U : (uint8_t)((value - 204U) * 4U);
  }

  return ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
}

void build_visuals(uint8_t sample, uint32_t sample_index) {
  const float x = ((float)sample - 128.0f) / 128.0f;
  const uint32_t band_colors[NUM_BANDS] = {
      0x00FF0000U,  // red
      0x00FF0000U,  // red
      0x000000FFU,  // blue
      0x0000FF00U,  // green
      0x0000FF00U   // green
  };

  gLowpass120 += 0.045f * (x - gLowpass120);
  gLowpass300 += 0.10f * (x - gLowpass300);
  gLowpass600 += 0.18f * (x - gLowpass600);
  gLowpass1200 += 0.33f * (x - gLowpass1200);
  gLowpass2400 += 0.55f * (x - gLowpass2400);

  const float band_levels[NUM_BANDS] = {
      fabsf(gLowpass120),
      fabsf(gLowpass300 - gLowpass120),
      fabsf(gLowpass600 - gLowpass300),
      fabsf(gLowpass1200 - gLowpass600),
      fabsf(gLowpass2400 - gLowpass1200),
  };

  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    gLedColors[i] = 0U;
  }

  for (uint32_t band = 0U; band < NUM_BANDS; ++band) {
    float target = band_levels[band] * 2.4f;
    if (target > 1.0f) target = 1.0f;

    const float response = (target > gBandEnvelopes[band]) ? 0.50f : 0.15f;
    gBandEnvelopes[band] += response * (target - gBandEnvelopes[band]);

    const uint32_t lit_count = (uint32_t)(gBandEnvelopes[band] * (float)LEDS_PER_BAND + 0.5f);
    const uint32_t start = band * LEDS_PER_BAND;
    const uint32_t end = start + LEDS_PER_BAND;
    const uint32_t peak = start + ((sample_index / AUDIO_LED_UPDATE_INTERVAL) % LEDS_PER_BAND);

    for (uint32_t led = start; led < end; ++led) {
      uint32_t color = 0U;
      const uint32_t relative = led - start;
      const uint8_t br = (uint8_t)((band_colors[band] >> 16) & 0xFFU);
      const uint8_t bg = (uint8_t)((band_colors[band] >> 8) & 0xFFU);
      const uint8_t bb = (uint8_t)(band_colors[band] & 0xFFU);

      if (relative < lit_count) {
        float fade = 1.0f - ((float)relative / (float)LEDS_PER_BAND);
        float scale = 0.18f + 0.42f * fade;
        if (scale < 0.0f) scale = 0.0f;
        if (scale > 1.0f) scale = 1.0f;
        const uint8_t r = (uint8_t)((float)br * scale);
        const uint8_t g = (uint8_t)((float)bg * scale);
        const uint8_t b = (uint8_t)((float)bb * scale);
        color = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
      }

      if ((led == peak) && (gBandEnvelopes[band] > 0.08f)) {
        const float peak_scale = 0.72f;
        const uint8_t r = (uint8_t)((float)br * peak_scale);
        const uint8_t g = (uint8_t)((float)bg * peak_scale);
        const uint8_t b = (uint8_t)((float)bb * peak_scale);
        color = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
      }

      gLedColors[led] = color;
    }
  }
}

void play_audio_array_once() {
#if HAS_AUDIO_ARRAY
  const uint32_t sample_count = sizeof(sampleArray) / sizeof(sampleArray[0]);

  clear_leds();

  for (uint32_t i = 0; i < sample_count; ++i) {
    uint32_t sample = sampleArray[i] & AUDIO_MAX_SAMPLE;

    if (i == 0U) {
      update_PWM(1U, sample);
    } else {
      while ((REG_READ(LEDC_INT_RAW_REG) & LEDC_LSTIMER0_OVF_INT_RAW) == 0U) {}
      update_PWM(0U, sample);
    }

    if ((i % AUDIO_LED_UPDATE_INTERVAL) == 0U) {
      build_visuals((uint8_t)sample, i);
      transmit_led_signal((uint32_t *)gLedColors);
    }
  }

  clear_leds();
  stop_audio_output();
#endif
}

}  // namespace

void setup() {
  appSetup();
}

void loop() {
  appLoop();
}
