#include <Arduino.h>
#include <math.h>

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

// ---------------- Configuration ----------------

// LEDC = LED Control peripheral on the ESP32. It generates the PWM signal that
// drives the motor speed input.
// PCNT = Pulse Counter peripheral on the ESP32. It counts encoder pulses so the
// code can estimate motor speed in RPM.
// This lab does not use bitbanging. It uses ESP32 hardware peripherals plus
// FreeRTOS tasks:
// - the monitor task reads encoder pulses with PCNT and estimates RPM
// - the control task uses the RPM error in the PID/PI math to choose a PWM duty
// - LEDC outputs that PWM duty to the motor driver

// Default to motor connector 1 from the project-board schematic:
// DIR1 -> GPIO4, PWM1 -> GPIO5, ENC_A1 -> GPIO6, BRAKE -> GPIO21.

// reduced error down to 2.1% and tuned to 300 rpm
constexpr uint8_t kMotorDirPin = 4;
constexpr uint8_t kMotorEncoderPin = 6;
constexpr uint8_t kMotorPwmPin = 5;
constexpr uint8_t kMotorBrakePin = 21;
constexpr uint8_t kSensePin = 15;
constexpr uint8_t kMotorDirState = LOW;
constexpr uint8_t kMotorBrakeState = HIGH;

constexpr uint8_t kPwmChannel = 0;
constexpr uint32_t kPwmFreqHz = 8000;
constexpr uint8_t kPwmResolutionBits = 12;
constexpr uint32_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;
constexpr uint32_t kControlDutyMax = 255U;
constexpr bool kPwmActiveLow = true;
constexpr uint32_t kTargetStepPeriodMs = 20000;
constexpr float kTargetRpmSequence[] = {300.0f, 600.0f, 3000.0f, 1200.0f, 1500.0f};
constexpr size_t kTargetRpmSequenceLength = sizeof(kTargetRpmSequence) / sizeof(kTargetRpmSequence[0]);
constexpr float kKp = 0.10f;
constexpr float kKi = 0.10f;
constexpr float kKd = 0.0f;
constexpr float kIntegralClamp = 2000.0f;
constexpr float kErrorDeadbandRpm = 3.0f;
constexpr float kIntegralBleed = 0.98f;
constexpr float kRpmFilterAlpha = 0.25f;
constexpr float kDutyRampUpPerCycle = 6.0f;
constexpr float kDutyRampDownPerCycle = 10.0f;

// Lab handout uses one encoder signal and one counted edge, so speed is based
// on 100 pulses per shaft revolution.
constexpr float kEncoderPulsesPerRev = 100.0f;

constexpr uint32_t kMonitorPeriodMs = 50;
constexpr uint32_t kControlPeriodMs = 50;
constexpr uint32_t kTelemetryPeriodMs = 100;
constexpr TickType_t kMonitorPeriodTicks = pdMS_TO_TICKS(kMonitorPeriodMs);
constexpr TickType_t kControlPeriodTicks = pdMS_TO_TICKS(kControlPeriodMs);

pcnt_unit_handle_t gPcntUnit = nullptr;
pcnt_channel_handle_t gPcntChannel = nullptr;

portMUX_TYPE gDataMux = portMUX_INITIALIZER_UNLOCKED;

float gMeasuredRpm = 0.0f;
float gTargetRpm = kTargetRpmSequence[0];
float gPTerm = 0.0f;
float gITerm = 0.0f;
float gDTerm = 0.0f;
float gTotalPid = 0.0f;

// ---------------- Shared Telemetry State ----------------

struct TelemetrySnapshot {
  float targetRpm;
  float measuredRpm;
  float pTerm;
  float iTerm;
  float dTerm;
  float totalPid;
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
  if (duty >= (float)kControlDutyMax) return kControlDutyMax;
  return (uint32_t)lroundf(duty);
}

uint32_t toHardwareDuty(uint32_t logicalDuty) {
  uint32_t pwmCounts =
      (uint32_t)lroundf(((float)logicalDuty * (float)kPwmMaxDuty) / (float)kControlDutyMax);
  if (!kPwmActiveLow) return pwmCounts;
  return kPwmMaxDuty - pwmCounts;
}

// ---------------- Serial Monitor Helpers ----------------

TelemetrySnapshot readTelemetrySnapshot() {
  TelemetrySnapshot snapshot{};

  portENTER_CRITICAL(&gDataMux);
  snapshot.targetRpm = gTargetRpm;
  snapshot.measuredRpm = gMeasuredRpm;
  snapshot.pTerm = gPTerm;
  snapshot.iTerm = gITerm;
  snapshot.dTerm = gDTerm;
  snapshot.totalPid = gTotalPid;
  portEXIT_CRITICAL(&gDataMux);

  return snapshot;
}

void printSerialTelemetry() {
  TelemetrySnapshot snapshot = readTelemetrySnapshot();
  Serial.printf(
      "target_rpm:%.2f\tmeasured_rpm:%.2f\tp_term:%.2f\ti_term:%.2f\td_term:%.2f\ttotal_pid:%.2f\n",
      snapshot.targetRpm,
      snapshot.measuredRpm,
      snapshot.pTerm,
      snapshot.iTerm,
      snapshot.dTerm,
      snapshot.totalPid);
}

// ---------------- Motor Hardware Setup ----------------

void setupMotorPinsAndPwm() {
  pinMode(kMotorDirPin, OUTPUT);
  pinMode(kMotorBrakePin, OUTPUT);
  pinMode(kSensePin, INPUT);
  pinMode(kMotorEncoderPin, INPUT_PULLUP);

  // Lab instructions: use one direction only, with direction = 0 and brake = 0.
  digitalWrite(kMotorDirPin, kMotorDirState);
  digitalWrite(kMotorBrakePin, kMotorBrakeState);
  Serial.printf(
      "Direction locked on GPIO%u (state=%s)\n",
      kMotorDirPin,
      (kMotorDirState == LOW) ? "LOW" : "HIGH");
  Serial.printf(
      "Brake fixed on GPIO%u (state=%s)\n",
      kMotorBrakePin,
      (kMotorBrakeState == LOW) ? "LOW" : "HIGH");

  if (!ledcAttachChannel(kMotorPwmPin, kPwmFreqHz, kPwmResolutionBits, kPwmChannel)) {
    failFast("LEDC attach failed");
  }

  if (!ledcWrite(kMotorPwmPin, toHardwareDuty(0U))) {
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
    // This loop runs at a fixed rate to read encoder pulses and update the
    // filtered RPM estimate that the controller uses.
    int pulseCount = 0;
    checkEsp(pcnt_unit_get_count(gPcntUnit, &pulseCount), "pcnt_unit_get_count");
    checkEsp(pcnt_unit_clear_count(gPcntUnit), "pcnt_unit_clear_count");

    float pulseHz = (float)pulseCount * (1000.0f / (float)kMonitorPeriodMs);
    float rawRpm = pulseHz * (60.0f / kEncoderPulsesPerRev);
    float filteredRpm = (kRpmFilterAlpha * rawRpm) + ((1.0f - kRpmFilterAlpha) * gMeasuredRpm);

    portENTER_CRITICAL(&gDataMux);
    gMeasuredRpm = filteredRpm;
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
  float previousError = 0.0f;
  float appliedDuty = 0.0f;
  for (;;) {
    // This loop runs at a fixed rate to compute PID terms from the latest RPM
    // error and update the PWM command sent to the motor driver.
    float measuredRpm = 0.0f;
    float targetRpm = 0.0f;
    portENTER_CRITICAL(&gDataMux);
    measuredRpm = gMeasuredRpm;
    targetRpm = gTargetRpm;
    portEXIT_CRITICAL(&gDataMux);

    float rawError = targetRpm - measuredRpm;
    float error = rawError;
    if (fabsf(rawError) <= kErrorDeadbandRpm) {
      error = 0.0f;
      integralError *= kIntegralBleed;
    } else {
      integralError = clampFloat(
          integralError + (error * kDtSeconds), -kIntegralClamp, kIntegralClamp);
    }
    float pTerm = kKp * error;
    float iTerm = kKi * integralError;
    float dTerm = kKd * ((error - previousError) / kDtSeconds);
    float totalPid = pTerm + iTerm + dTerm;
    float commandedDuty = clampFloat(totalPid, 0.0f, (float)kControlDutyMax);
    appliedDuty =
        rampFloatToward(appliedDuty, commandedDuty, kDutyRampUpPerCycle, kDutyRampDownPerCycle);
    uint32_t targetDuty = clampDuty(appliedDuty);
    uint32_t hardwareDuty = toHardwareDuty(targetDuty);
    if (!ledcWrite(kMotorPwmPin, hardwareDuty)) {
      failFast("PWM update failed");
    }
    previousError = error;

    portENTER_CRITICAL(&gDataMux);
    gPTerm = pTerm;
    gITerm = iTerm;
    gDTerm = dTerm;
    gTotalPid = totalPid;
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

  BaseType_t monitorOk = xTaskCreate(motorMonitorTask, "motor_monitor", 4096, nullptr, 2, nullptr);
  BaseType_t controlOk = xTaskCreate(motorControlTask, "motor_control", 4096, nullptr, 2, nullptr);

  if (monitorOk != pdPASS || controlOk != pdPASS) {
    failFast("Task creation failed");
  }

  Serial.printf(
      "Closed-loop PID mode, initial target %.2f RPM, step period %lu ms, monitor period %lu ms, control period %lu ms\n",
      gTargetRpm,
      (unsigned long)kTargetStepPeriodMs,
      (unsigned long)kMonitorPeriodMs,
      (unsigned long)kControlPeriodMs);
}

void loop() {
  static uint32_t lastReportMs = 0U;
  static uint32_t lastTargetStepMs = 0U;
  static size_t targetIndex = 0U;
  uint32_t now = millis();

  // This loop advances the target-RPM schedule every 20 seconds and prints the
  // latest telemetry often enough for the Serial Plotter.
  if (now - lastTargetStepMs >= kTargetStepPeriodMs) {
    lastTargetStepMs = now;
    targetIndex = (targetIndex + 1U) % kTargetRpmSequenceLength;
    portENTER_CRITICAL(&gDataMux);
    gTargetRpm = kTargetRpmSequence[targetIndex];
    portEXIT_CRITICAL(&gDataMux);
  }

  if (now - lastReportMs >= kTelemetryPeriodMs) {
    lastReportMs = now;
    printSerialTelemetry();
  }

  delay(20);
}
