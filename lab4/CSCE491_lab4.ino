#include <Arduino.h>
#include <math.h>

#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/ledc_reg.h"
#include "soc/rmt_reg.h"
#include "soc/soc.h"
#include "soc/system_reg.h"

#if __has_include("array.h")
#include "array.h"
#define LAB_HAS_AUDIO_ARRAY 1
#else
#define LAB_HAS_AUDIO_ARRAY 0
#endif

#define LAB_AUDIO_PIN 14
#define LAB_LED_PIN 33
#define LAB_LED_COUNT 100
#define LAB_LED_BRIGHTNESS 32
#define LAB_AUDIO_RESOLUTION_BITS 8
#define LAB_AUDIO_MAX_SAMPLE ((1U << LAB_AUDIO_RESOLUTION_BITS) - 1U)

#define LAB_RMT_RAM_REG RMT_CH0DATA_REG
#define LAB_RMT_CH0_CONF0_REG RMT_CH0CONF0_REG
#define LAB_RMT_SYS_CONF_REG RMT_SYS_CONF_REG
#define LAB_RMT_INT_RAW_REG RMT_INT_RAW_REG
#define LAB_RMT_INT_CLR_REG RMT_INT_CLR_REG
#define LAB_RMT_CLOCK_EN_REG SYSTEM_PERIP_CLK_EN0_REG
#define LAB_RMT_RESET_REG SYSTEM_PERIP_RST_EN0_REG

#define LAB_LEDC_CONF_REG LEDC_CONF_REG
#define LAB_LEDC_TIMER0_CONF_REG LEDC_LSTIMER0_CONF_REG
#define LAB_LEDC_CH0_CONF0_REG LEDC_LSCH0_CONF0_REG
#define LAB_LEDC_CH0_CONF1_REG LEDC_LSCH0_CONF1_REG
#define LAB_LEDC_CH0_HPOINT_REG LEDC_LSCH0_HPOINT_REG
#define LAB_LEDC_CH0_DUTY_REG LEDC_LSCH0_DUTY_REG
#define LAB_LEDC_INT_RAW_REG LEDC_INT_RAW_REG
#define LAB_LEDC_INT_CLR_REG LEDC_INT_CLR_REG
#define LAB_LEDC_CLOCK_EN_REG SYSTEM_PERIP_CLK_EN0_REG
#define LAB_LEDC_RESET_REG SYSTEM_PERIP_RST_EN0_REG

#define LAB_GPIO_AUDIO_OUT_SEL_REG GPIO_FUNC14_OUT_SEL_CFG_REG
#define LAB_GPIO_LED_OUT_SEL_REG GPIO_FUNC33_OUT_SEL_CFG_REG
#define LAB_IO_MUX_AUDIO_REG IO_MUX_GPIO14_REG
#define LAB_IO_MUX_LED_REG IO_MUX_GPIO33_REG

#define LAB_GPIO_OUT_SEL_MASK 0x1FFU
#define LAB_GPIO_OUT_INV_SEL (1U << 9)
#define LAB_GPIO_OEN_SEL (1U << 10)
#define LAB_GPIO_OEN_INV_SEL (1U << 11)

#define LAB_LEDC_GPIO_FUNC_SEL LEDC_LS_SIG_OUT0_IDX
#define LAB_RMT_GPIO_FUNC_SEL RMT_SIG_OUT0_IDX

#define LAB_RMT_T0H_TICKS 32
#define LAB_RMT_T0L_TICKS 68
#define LAB_RMT_T1H_TICKS 64
#define LAB_RMT_T1L_TICKS 36
#define LAB_RMT_RESET_US 80

#define LAB_FALLBACK_SAMPLE_RATE 8000U
#define LAB_SERIAL_BAUD 115200
#define LAB_AUDIO_START_DELAY_MS 10000UL
#define LAB_LED_REFRESH_INTERVAL_US 33333UL
#define LAB_POST_WIPE_DELAY_MS 12U

typedef union {
  struct {
    uint32_t duration0 : 15;
    uint32_t level0 : 1;
    uint32_t duration1 : 15;
    uint32_t level1 : 1;
  };
  uint32_t val;
} RmtItem32;

static volatile RmtItem32 *const gRmtRam = reinterpret_cast<volatile RmtItem32 *>(LAB_RMT_RAM_REG);

static uint32_t gLedColors[LAB_LED_COUNT];
static uint32_t gCurrentSampleRate = LAB_FALLBACK_SAMPLE_RATE;
static float gFallbackPhase = 0.0f;
static uint32_t gPlaybackIndex = 0;
static uint32_t gBootMillis = 0;
static uint32_t gLastLedRefreshUs = 0;

void setup_RMT();
void setup_LEDC();
void update_PWM(uint32_t initial, uint32_t sample);
void transmit_led_signal(uint32_t *colors);
void fillAllLeds(uint32_t color);
void runPowerOnSelfTest();

static uint32_t makeColor(uint8_t red, uint8_t green, uint8_t blue) {
  return ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
}

static uint32_t scaleColor(uint32_t color, uint8_t scale) {
  uint8_t green = (uint8_t)(((color >> 16) & 0xFFU) * scale / 255U);
  uint8_t red = (uint8_t)(((color >> 8) & 0xFFU) * scale / 255U);
  uint8_t blue = (uint8_t)((color & 0xFFU) * scale / 255U);
  return makeColor(red, green, blue);
}

static uint32_t colorForLevel(float normalized) {
  if (normalized < 0.0f) normalized = 0.0f;
  if (normalized > 1.0f) normalized = 1.0f;

  const uint32_t anchors[5] = {
      makeColor(255, 0, 0),
      makeColor(255, 255, 0),
      makeColor(255, 255, 85),
      makeColor(255, 255, 171),
      makeColor(255, 255, 255),
  };

  float scaled = normalized * 4.0f;
  int index = (int)scaled;
  if (index >= 4) return anchors[4];

  float mix = scaled - (float)index;
  uint8_t r0 = (uint8_t)((anchors[index] >> 8) & 0xFFU);
  uint8_t g0 = (uint8_t)((anchors[index] >> 16) & 0xFFU);
  uint8_t b0 = (uint8_t)(anchors[index] & 0xFFU);

  uint8_t r1 = (uint8_t)((anchors[index + 1] >> 8) & 0xFFU);
  uint8_t g1 = (uint8_t)((anchors[index + 1] >> 16) & 0xFFU);
  uint8_t b1 = (uint8_t)(anchors[index + 1] & 0xFFU);

  uint8_t red = (uint8_t)(r0 + (r1 - r0) * mix);
  uint8_t green = (uint8_t)(g0 + (g1 - g0) * mix);
  uint8_t blue = (uint8_t)(b0 + (b1 - b0) * mix);
  return makeColor(red, green, blue);
}

static uint32_t computeLedCountFromSample(uint32_t sample) {
  float ratio = ((float)sample + 1.0f) / 256.0f;
  float logValue = log10f(ratio);
  float normalized = (logValue + 2.4f) / 2.4f;
  if (normalized < 0.0f) normalized = 0.0f;
  if (normalized > 1.0f) normalized = 1.0f;
  return (uint32_t)lroundf(normalized * (float)LAB_LED_COUNT);
}

static void updateVisualization(uint32_t sample) {
  uint32_t ledsOn = computeLedCountFromSample(sample);
  float normalized = (float)sample / (float)LAB_AUDIO_MAX_SAMPLE;
  uint32_t activeColor = scaleColor(colorForLevel(normalized), LAB_LED_BRIGHTNESS);

  for (uint32_t i = 0; i < LAB_LED_COUNT; ++i) {
    gLedColors[i] = (i < ledsOn) ? activeColor : 0U;
  }

  transmit_led_signal(gLedColors);
}

static uint32_t nextToneSample(float frequencyHz) {
  const float step = 2.0f * PI * 440.0f / (float)gCurrentSampleRate;
  gFallbackPhase += step * (frequencyHz / 440.0f);
  if (gFallbackPhase > 2.0f * PI) gFallbackPhase -= 2.0f * PI;

  float s = sinf(gFallbackPhase);
  return (uint32_t)lroundf((s * 0.5f + 0.5f) * (float)LAB_AUDIO_MAX_SAMPLE);
}

static uint32_t nextStartupSample() {
  uint32_t elapsedMs = millis() - gBootMillis;
  uint32_t patternMs = elapsedMs % 1000UL;

  if ((patternMs < 180UL) || (patternMs >= 300UL && patternMs < 480UL)) {
    return nextToneSample(880.0f);
  }

  return LAB_AUDIO_MAX_SAMPLE / 2U;
}

static uint32_t nextAudioSample() {
#if LAB_HAS_AUDIO_ARRAY
  uint32_t sample = sampleArray[gPlaybackIndex++];
  if (gPlaybackIndex >= sizeof(sampleArray) / sizeof(sampleArray[0])) {
    gPlaybackIndex = 0;
  }
  return sample;
#else
  const float step = 2.0f * PI * 440.0f / (float)gCurrentSampleRate;
  gFallbackPhase += step;
  if (gFallbackPhase > 2.0f * PI) gFallbackPhase -= 2.0f * PI;

  float s = sinf(gFallbackPhase);
  return (uint32_t)lroundf((s * 0.5f + 0.5f) * (float)LAB_AUDIO_MAX_SAMPLE);
#endif
}

static bool audioPlaybackEnabled() {
  return (millis() - gBootMillis) >= LAB_AUDIO_START_DELAY_MS;
}

static void maybeUpdateVisualization(uint32_t sample) {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - gLastLedRefreshUs) < LAB_LED_REFRESH_INTERVAL_US) return;
  gLastLedRefreshUs = nowUs;
  updateVisualization(sample);
}

void setup_RMT() {
  REG_SET_BIT(LAB_RMT_CLOCK_EN_REG, SYSTEM_RMT_CLK_EN);
  REG_SET_BIT(LAB_RMT_RESET_REG, SYSTEM_RMT_RST);
  REG_CLR_BIT(LAB_RMT_RESET_REG, SYSTEM_RMT_RST);

  uint32_t sysConf = 0;
  sysConf &= ~RMT_APB_FIFO_MASK;
  sysConf |= RMT_MEM_CLK_FORCE_ON;
  sysConf |= RMT_CLK_EN;
  sysConf |= RMT_SCLK_ACTIVE;
  sysConf |= (1U << RMT_SCLK_SEL_S);
  REG_WRITE(LAB_RMT_SYS_CONF_REG, sysConf);

  uint32_t chConf = 0;
  chConf |= (1U << RMT_DIV_CNT_CH0_S);
  chConf |= (1U << RMT_MEM_SIZE_CH0_S);
  chConf |= RMT_IDLE_OUT_EN_CH0;
  chConf &= ~RMT_IDLE_OUT_LV_CH0;
  chConf &= ~RMT_CARRIER_EN_CH0;
  REG_WRITE(LAB_RMT_CH0_CONF0_REG, chConf);

  REG_WRITE(LAB_RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_CLR);

  pinMode(LAB_LED_PIN, OUTPUT);
  PIN_FUNC_SELECT(LAB_IO_MUX_LED_REG, PIN_FUNC_GPIO);
  REG_WRITE(LAB_GPIO_LED_OUT_SEL_REG, LAB_RMT_GPIO_FUNC_SEL);
}

void setup_LEDC() {
  REG_SET_BIT(LAB_LEDC_CLOCK_EN_REG, SYSTEM_LEDC_CLK_EN);
  REG_SET_BIT(LAB_LEDC_RESET_REG, SYSTEM_LEDC_RST);
  REG_CLR_BIT(LAB_LEDC_RESET_REG, SYSTEM_LEDC_RST);

  REG_WRITE(LAB_LEDC_CONF_REG, LEDC_CLK_EN | (1U << LEDC_APB_CLK_SEL_S));

  uint32_t dividerFixed = (uint32_t)lroundf((80000000.0f / (float)gCurrentSampleRate) * 256.0f);
  if (dividerFixed < 256U) dividerFixed = 256U;
  if (dividerFixed > LEDC_CLK_DIV_LSTIMER0_V) dividerFixed = LEDC_CLK_DIV_LSTIMER0_V;

  uint32_t timerConf = 0;
  timerConf |= ((uint32_t)LAB_AUDIO_RESOLUTION_BITS << LEDC_LSTIMER0_DUTY_RES_S);
  timerConf |= (dividerFixed << LEDC_CLK_DIV_LSTIMER0_S);
  timerConf |= LEDC_TICK_SEL_LSTIMER0;
  timerConf &= ~LEDC_LSTIMER0_PAUSE;
  timerConf &= ~LEDC_LSTIMER0_RST;
  REG_WRITE(LAB_LEDC_TIMER0_CONF_REG, timerConf);
  REG_SET_BIT(LAB_LEDC_TIMER0_CONF_REG, LEDC_LSTIMER0_PARA_UP);

  REG_WRITE(LAB_LEDC_CH0_HPOINT_REG, 0);
  REG_WRITE(LAB_LEDC_CH0_DUTY_REG, 0);
  REG_WRITE(LAB_LEDC_CH0_CONF0_REG, LEDC_SIG_OUT_EN_LSCH0);
  REG_WRITE(LAB_LEDC_CH0_CONF1_REG, 0);
  REG_WRITE(LAB_LEDC_INT_CLR_REG, LEDC_LSTIMER0_OVF_INT_CLR);

  pinMode(LAB_AUDIO_PIN, OUTPUT);
  PIN_FUNC_SELECT(LAB_IO_MUX_AUDIO_REG, PIN_FUNC_GPIO);
  REG_WRITE(LAB_GPIO_AUDIO_OUT_SEL_REG, LAB_LEDC_GPIO_FUNC_SEL);
}

void update_PWM(uint32_t initial, uint32_t sample) {
  if (!initial && !(REG_READ(LAB_LEDC_INT_RAW_REG) & LEDC_LSTIMER0_OVF_INT_RAW)) {
    return;
  }

  REG_WRITE(LAB_LEDC_INT_CLR_REG, LEDC_LSTIMER0_OVF_INT_CLR);

  uint32_t clamped = sample & LAB_AUDIO_MAX_SAMPLE;
  REG_WRITE(LAB_LEDC_CH0_DUTY_REG, clamped << 4);
  REG_SET_BIT(LAB_LEDC_CH0_CONF0_REG, LEDC_PARA_UP_LSCH0);
  REG_SET_BIT(LAB_LEDC_CH0_CONF1_REG, LEDC_DUTY_START_LSCH0);
}

static void waitForRmtDone() {
  while (!(REG_READ(LAB_RMT_INT_RAW_REG) & RMT_CH0_TX_END_INT_RAW)) {
  }
  REG_WRITE(LAB_RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_CLR);
}

static void loadRmtWord(uint32_t index, uint8_t value) {
  for (int bit = 7; bit >= 0; --bit) {
    bool one = ((value >> bit) & 0x1U) != 0;
    uint32_t itemIndex = index++;
    gRmtRam[itemIndex].duration0 = one ? LAB_RMT_T1H_TICKS : LAB_RMT_T0H_TICKS;
    gRmtRam[itemIndex].level0 = 1;
    gRmtRam[itemIndex].duration1 = one ? LAB_RMT_T1L_TICKS : LAB_RMT_T0L_TICKS;
    gRmtRam[itemIndex].level1 = 0;
  }
}

void transmit_led_signal(uint32_t *colors) {
  for (uint32_t led = 0; led < LAB_LED_COUNT; ++led) {
    uint32_t color = colors[led];
    uint8_t green = (uint8_t)((color >> 16) & 0xFFU);
    uint8_t red = (uint8_t)((color >> 8) & 0xFFU);
    uint8_t blue = (uint8_t)(color & 0xFFU);

    loadRmtWord(0, green);
    loadRmtWord(8, red);
    loadRmtWord(16, blue);
    gRmtRam[24].val = 0;

    REG_WRITE(LAB_RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_CLR);
    REG_SET_BIT(LAB_RMT_CH0_CONF0_REG, RMT_APB_MEM_RST_CH0);
    REG_CLR_BIT(LAB_RMT_CH0_CONF0_REG, RMT_APB_MEM_RST_CH0);
    REG_SET_BIT(LAB_RMT_CH0_CONF0_REG, RMT_MEM_RD_RST_CH0);
    REG_CLR_BIT(LAB_RMT_CH0_CONF0_REG, RMT_MEM_RD_RST_CH0);
    REG_SET_BIT(LAB_RMT_CH0_CONF0_REG, RMT_TX_START_CH0);
    waitForRmtDone();
  }

  delayMicroseconds(LAB_RMT_RESET_US);
}

void fillAllLeds(uint32_t color) {
  for (uint32_t i = 0; i < LAB_LED_COUNT; ++i) {
    gLedColors[i] = color;
  }
  transmit_led_signal(gLedColors);
}

void runPowerOnSelfTest() {
  const uint32_t postColor = scaleColor(makeColor(255, 255, 255), LAB_LED_BRIGHTNESS);

  Serial.println("POST: initializing RMT and LEDC peripherals");
  fillAllLeds(0U);

  Serial.println("POST: wiping all 100 LEDs with dim white");
  for (uint32_t i = 0; i < LAB_LED_COUNT; ++i) {
    gLedColors[i] = postColor;
    transmit_led_signal(gLedColors);
    delay(LAB_POST_WIPE_DELAY_MS);
  }

  Serial.println("POST: hardware ready");
}

void setup() {
  Serial.begin(LAB_SERIAL_BAUD);
  delay(200);
  gBootMillis = millis();
  gLastLedRefreshUs = micros();

  setup_RMT();
  setup_LEDC();
  runPowerOnSelfTest();

#if LAB_HAS_AUDIO_ARRAY
  gCurrentSampleRate = sampleRate;
  Serial.printf("array.h detected, sampleRate = %lu Hz\n", (unsigned long)gCurrentSampleRate);
#else
  Serial.printf("array.h not found, fallback synth sampleRate = %lu Hz\n",
                (unsigned long)gCurrentSampleRate);
#endif
  Serial.printf("Startup beep pattern active for %lu ms before audio playback\n",
                (unsigned long)LAB_AUDIO_START_DELAY_MS);

  uint32_t firstSample = nextStartupSample();
  update_PWM(1, firstSample);
  maybeUpdateVisualization(firstSample);
}

void loop() {
  if (!(REG_READ(LAB_LEDC_INT_RAW_REG) & LEDC_LSTIMER0_OVF_INT_RAW)) return;

  uint32_t sample = audioPlaybackEnabled() ? nextAudioSample() : nextStartupSample();
  update_PWM(0, sample);
  maybeUpdateVisualization(sample);
}
