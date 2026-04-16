#include <Arduino.h>
#include <math.h>

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

// ---------------- Configuration ----------------

// Default to motor connector 1 from the project-board schematic:
// DIR1 -> GPIO4, PWM1 -> GPIO5, ENC_A1 -> GPIO6, BRAKE -> GPIO21.
constexpr uint8_t kMotorDirPin = 4;
constexpr uint8_t kMotorEncoderPin = 6;
constexpr uint8_t kMotorPwmPin = 5;
constexpr uint8_t kMotorBrakePin = 21;
constexpr uint8_t kSensePin = 15;
constexpr uint8_t kMotorDirState = LOW;

constexpr uint8_t kPwmChannel = 0;
constexpr uint32_t kPwmFreqHz = 20000;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint32_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;
constexpr bool kPwmActiveLow = true;
constexpr float kTargetRpm = 300.0f;
constexpr uint32_t kBaseDuty = 30U;
constexpr float kKp = 0.06f;
constexpr float kKi = 0.015f;
constexpr float kIntegralClamp = 400.0f;
constexpr float kErrorDeadbandRpm = 8.0f;
constexpr float kIntegralBleed = 0.90f;

constexpr float kEncoderPulsesPerRev = 100.0f;

constexpr uint32_t kMonitorPeriodMs = 50;
constexpr uint32_t kControlPeriodMs = 50;
constexpr uint32_t kTelemetryPeriodMs = 100;
constexpr TickType_t kMonitorPeriodTicks = pdMS_TO_TICKS(kMonitorPeriodMs);
constexpr TickType_t kControlPeriodTicks = pdMS_TO_TICKS(kControlPeriodMs);

constexpr float kBenchSupplyVoltage = 12.0f;

pcnt_unit_handle_t gPcntUnit = nullptr;
pcnt_channel_handle_t gPcntChannel = nullptr;
TaskHandle_t gMonitorTaskHandle = nullptr;
TaskHandle_t gControlTaskHandle = nullptr;

portMUX_TYPE gDataMux = portMUX_INITIALIZER_UNLOCKED;

float gMeasuredPulseHz = 0.0f;
float gMeasuredRpm = 0.0f;
float gTargetRpm = kTargetRpm;
float gLastError = 0.0f;
float gIntegralTerm = 0.0f;
int gLastPulseCount = 0;
uint32_t gCurrentDuty = 0U;
uint64_t gTotalPulses = 0U;
uint64_t gStartTimeUs = 0U;
uint64_t gMonitorCpuUs = 0U;
uint64_t gControlCpuUs = 0U;

// ---------------- Shared Telemetry State ----------------

struct TelemetrySnapshot {
  float targetRpm;
  float measuredPulseHz;
  float measuredRpm;
  float lastError;
  float integralTerm;
  int pulseCount;
  uint32_t duty;
  uint64_t totalPulses;
  uint64_t monitorCpuUs;
  uint64_t controlCpuUs;
  uint64_t elapsedUs;
};

// ---------------- Utility Helpers ----------------

void failFast(const char *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

void checkEsp(esp_err_t err, const char *what) {
  if (err == ESP_OK) return;

  Serial.printf("%s failed: %s\n", what, esp_err_to_name(err));
  failFast("Halting");
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float rampFloatToward(float currentValue, float targetValue, float rampUpStep, float rampDownStep) {
  if (targetValue > currentValue) {
    float step = min(rampUpStep, targetValue - currentValue);
    return currentValue + step;
  }
  if (targetValue < currentValue) {
    float step = min(rampDownStep, currentValue - targetValue);
    return currentValue - step;
  }
  return currentValue;
}

uint32_t clampDuty(float duty) {
  if (duty <= 0.0f) return 0U;
  if (duty >= (float)kPwmMaxDuty) return kPwmMaxDuty;
  return (uint32_t)lroundf(duty);
}

uint32_t toHardwareDuty(uint32_t logicalDuty) {
  if (!kPwmActiveLow) return logicalDuty;
  return kPwmMaxDuty - logicalDuty;
}

float readSupplyVoltage() {
  return kBenchSupplyVoltage;
}

// ---------------- Serial Monitor Helpers ----------------

TelemetrySnapshot readTelemetrySnapshot() {
  TelemetrySnapshot snapshot{};

  portENTER_CRITICAL(&gDataMux);
  snapshot.targetRpm = gTargetRpm;
  snapshot.measuredPulseHz = gMeasuredPulseHz;
  snapshot.measuredRpm = gMeasuredRpm;
  snapshot.lastError = gLastError;
  snapshot.integralTerm = gIntegralTerm;
  snapshot.pulseCount = gLastPulseCount;
  snapshot.duty = gCurrentDuty;
  snapshot.totalPulses = gTotalPulses;
  snapshot.monitorCpuUs = gMonitorCpuUs;
  snapshot.controlCpuUs = gControlCpuUs;
  snapshot.elapsedUs = esp_timer_get_time() - gStartTimeUs;
  portEXIT_CRITICAL(&gDataMux);

  return snapshot;
}

void printSerialTelemetry() {
  TelemetrySnapshot snapshot = readTelemetrySnapshot();
  if (snapshot.elapsedUs == 0U) return;
  float supplyVoltage = readSupplyVoltage();

  Serial.printf(
      "target_rpm:%.2f\tmeasured_rpm:%.2f\tduty:%lu\terror:%.2f\ti_term:%.2f\tpulse_count:%d\tencoder_hz:%.2f\tsupply_v:%.2f\n",
      snapshot.targetRpm,
      snapshot.measuredRpm,
      (unsigned long)snapshot.duty,
      snapshot.lastError,
      snapshot.integralTerm,
      snapshot.pulseCount,
      snapshot.measuredPulseHz,
      supplyVoltage);
}

// ---------------- Motor Hardware Setup ----------------

void setupMotorPinsAndPwm() {
  pinMode(kMotorDirPin, OUTPUT);
  pinMode(kMotorBrakePin, OUTPUT);
  pinMode(kSensePin, INPUT);
  pinMode(kMotorEncoderPin, INPUT_PULLUP);

  // Lab instructions: use one direction only and keep the brake disabled.
  digitalWrite(kMotorDirPin, kMotorDirState);
  digitalWrite(kMotorBrakePin, HIGH);
  Serial.printf(
      "Direction locked on GPIO%u (state=%s)\n",
      kMotorDirPin,
      (kMotorDirState == LOW) ? "LOW" : "HIGH");
  Serial.printf("Brake disengaged on GPIO%u (state=HIGH)\n", kMotorBrakePin);

  if (!ledcAttachChannel(kMotorPwmPin, kPwmFreqHz, kPwmResolutionBits, kPwmChannel)) {
    failFast("LEDC attach failed");
  }

  if (!ledcWrite(kMotorPwmPin, 0U)) {
    failFast("Initial PWM write failed");
  }
}

void setupEncoderCounter() {
  pcnt_unit_config_t unitConfig = {};
  unitConfig.low_limit = -1;
  unitConfig.high_limit = (1 << 14);
  unitConfig.intr_priority = 0;
  unitConfig.flags.accum_count = 0;

  pcnt_chan_config_t channelConfig = {};
  channelConfig.edge_gpio_num = kMotorEncoderPin;
  channelConfig.level_gpio_num = -1;
  channelConfig.flags.invert_edge_input = 0;
  channelConfig.flags.invert_level_input = 0;
  channelConfig.flags.virt_edge_io_level = 0;
  channelConfig.flags.virt_level_io_level = 0;
  channelConfig.flags.io_loop_back = 0;

  checkEsp(pcnt_new_unit(&unitConfig, &gPcntUnit), "pcnt_new_unit");
  checkEsp(pcnt_new_channel(gPcntUnit, &channelConfig, &gPcntChannel), "pcnt_new_channel");
  checkEsp(
      pcnt_channel_set_edge_action(
          gPcntChannel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD),
      "pcnt_channel_set_edge_action");
  checkEsp(pcnt_unit_clear_count(gPcntUnit), "pcnt_unit_clear_count");
  checkEsp(pcnt_unit_enable(gPcntUnit), "pcnt_unit_enable");
  checkEsp(pcnt_unit_start(gPcntUnit), "pcnt_unit_start");
}

// ---------------- Motor Speed Measurement ----------------

void motorMonitorTask(void *args) {
  (void)args;

  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    uint64_t startTaskUs = esp_timer_get_time();

    int pulseCount = 0;
    checkEsp(pcnt_unit_get_count(gPcntUnit, &pulseCount), "pcnt_unit_get_count");
    checkEsp(pcnt_unit_clear_count(gPcntUnit), "pcnt_unit_clear_count");

    float pulseHz = (float)pulseCount * (1000.0f / (float)kMonitorPeriodMs);
    float rpm = pulseHz * (60.0f / kEncoderPulsesPerRev);

    portENTER_CRITICAL(&gDataMux);
    gMeasuredPulseHz = pulseHz;
    gMeasuredRpm = rpm;
    gLastPulseCount = pulseCount;
    gTotalPulses += (uint64_t)max(pulseCount, 0);
    gMonitorCpuUs += esp_timer_get_time() - startTaskUs;
    portEXIT_CRITICAL(&gDataMux);

    vTaskDelayUntil(&lastWakeTime, kMonitorPeriodTicks);
  }
}

// ---------------- RPM Control ----------------

void motorControlTask(void *args) {
  (void)args;

  TickType_t lastWakeTime = xTaskGetTickCount();
  constexpr float kDtSeconds = (float)kControlPeriodMs / 1000.0f;
  float integralError = 0.0f;
  for (;;) {
    uint64_t startTaskUs = esp_timer_get_time();
    float measuredRpm = 0.0f;
    portENTER_CRITICAL(&gDataMux);
    measuredRpm = gMeasuredRpm;
    portEXIT_CRITICAL(&gDataMux);

    float rawError = kTargetRpm - measuredRpm;
    float error = rawError;
    if (fabsf(rawError) <= kErrorDeadbandRpm) {
      error = 0.0f;
      integralError *= kIntegralBleed;
    } else {
      integralError = clampFloat(
          integralError + (error * kDtSeconds), -kIntegralClamp, kIntegralClamp);
    }
    float controlDuty = (float)kBaseDuty + (kKp * error) + (kKi * integralError);
    uint32_t targetDuty = clampDuty(controlDuty);
    uint32_t hardwareDuty = toHardwareDuty(targetDuty);
    if (!ledcWrite(kMotorPwmPin, hardwareDuty)) {
      failFast("PWM update failed");
    }

    portENTER_CRITICAL(&gDataMux);
    gTargetRpm = kTargetRpm;
    gLastError = error;
    gIntegralTerm = kKi * integralError;
    gCurrentDuty = targetDuty;
    gControlCpuUs += esp_timer_get_time() - startTaskUs;
    portEXIT_CRITICAL(&gDataMux);
    vTaskDelayUntil(&lastWakeTime, kControlPeriodTicks);
  }
}

}  // namespace

// ---------------- Arduino Entry Points ----------------

void setup() {
  Serial.begin(115200);

  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 4000U)) {
    delay(10);
  }
  delay(5000);

  Serial.println("CSCE 491 Lab 5 motor controller");
  Serial.printf(
      "Motor 1 mapping: DIR=%u, PWM=%u, BRAKE=%u, ENC_A=%u\n",
      kMotorDirPin,
      kMotorPwmPin,
      kMotorBrakePin,
      kMotorEncoderPin);

  setupMotorPinsAndPwm();
  setupEncoderCounter();

  gStartTimeUs = esp_timer_get_time();

  BaseType_t monitorOk = xTaskCreate(
      motorMonitorTask, "motor_monitor", 4096, nullptr, 2, &gMonitorTaskHandle);
  BaseType_t controlOk = xTaskCreate(
      motorControlTask, "motor_control", 4096, nullptr, 2, &gControlTaskHandle);

  if (monitorOk != pdPASS || controlOk != pdPASS) {
    failFast("Task creation failed");
  }

  Serial.printf(
      "Closed-loop PI mode, target %.2f RPM, base duty %lu, monitor period %lu ms, control period %lu ms\n",
      kTargetRpm,
      (unsigned long)kBaseDuty,
      (unsigned long)kMonitorPeriodMs,
      (unsigned long)kControlPeriodMs);
}

void loop() {
  static uint32_t lastReportMs = 0U;
  uint32_t now = millis();

  if (now - lastReportMs >= kTelemetryPeriodMs) {
    lastReportMs = now;
    printSerialTelemetry();
  }

  delay(20);
}
