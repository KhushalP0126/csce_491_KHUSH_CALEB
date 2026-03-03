#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

static const int ADC_PIN = 15;  // SENSE -> XTAL_32K_P -> GPIO15
static const float ADC_VREF = 3.3f;
static const int ADC_MAX = 4095;

static const int PSU_RX_PIN = 12;
static const int PSU_TX_PIN = 13;
static const unsigned long PSU_TIMEOUT_MS = 1200;

static const int SEGMENTS = 16;
static const int SEGMENT_WIDTH = 256;  // 4096/16
static const float DIVIDER_SCALE = 43.0f / 10.0f;  // Vbatt = Vadc * 4.3

static const float SWEEP_START_V = 0.0f;
static const float SWEEP_END_V = 14.2f;
static const float SWEEP_STEP_V = 0.05f;

struct LinStats {
  double n;
  double sumX;
  double sumY;
  double sumXX;
  double sumXY;
};

struct LinearModel {
  float m;
  float b;
};

static LinStats segStats[SEGMENTS];
static LinearModel segModel[SEGMENTS];
static bool gMonitoringEnabled = false;

static void clearStats(LinStats &s) {
  s.n = 0.0;
  s.sumX = 0.0;
  s.sumY = 0.0;
  s.sumXX = 0.0;
  s.sumXY = 0.0;
}

static void addSample(LinStats &s, float x, float y) {
  s.n += 1.0;
  s.sumX += x;
  s.sumY += y;
  s.sumXX += (double)x * (double)x;
  s.sumXY += (double)x * (double)y;
}

static bool fitLinear(const LinStats &s, LinearModel &model) {
  if (s.n < 2.0) return false;

  double denom = s.n * s.sumXX - s.sumX * s.sumX;
  if (fabs(denom) < 1e-9) return false;

  model.m = (float)((s.n * s.sumXY - s.sumX * s.sumY) / denom);
  model.b = (float)((s.sumY - (double)model.m * s.sumX) / s.n);
  return true;
}

static void scpiSend(const String &cmd) {
  Serial2.print(cmd);
  Serial2.print("\r\n");
}

static String scpiReadLine(unsigned long timeoutMs) {
  unsigned long start = millis();
  String out;
  while (millis() - start < timeoutMs) {
    while (Serial2.available() > 0) {
      char c = (char)Serial2.read();
      if (c == '\n') {
        out.trim();
        return out;
      }
      out += c;
    }
    delay(1);
  }
  out.trim();
  return out;
}

static void clearSerial2Input() {
  while (Serial2.available() > 0) {
    (void)Serial2.read();
  }
}

static bool psuTakeControl() {
  clearSerial2Input();
  scpiSend("*IDN?");
  String id = scpiReadLine(PSU_TIMEOUT_MS);
  if (id.length() == 0) return false;

  scpiSend("SYST:REM");
  delay(60);
  scpiSend("INST:NSEL 1");
  scpiSend("CURR 1.0");
  scpiSend("OUTP:ENAB 1");
  scpiSend("OUTP 1");
  delay(120);
  return true;
}

static void psuReleaseControl() {
  scpiSend("SYST:LOC");
  delay(50);
}

static void psuSetVoltage(float v) {
  Serial2.printf("VOLT %.3f\r\n", v);
}

static int readAdcAveraged(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(ADC_PIN);
    delay(2);
  }
  return (int)(sum / samples);
}

static float adcRawToVoltage(int rawAdc) {
  return ((float)rawAdc * ADC_VREF) / (float)ADC_MAX;
}

static int segmentForRaw(int rawAdc) {
  if (rawAdc < 0) rawAdc = 0;
  if (rawAdc > ADC_MAX) rawAdc = ADC_MAX;
  int seg = rawAdc / SEGMENT_WIDTH;
  if (seg >= SEGMENTS) seg = SEGMENTS - 1;
  return seg;
}

static float applyModel(int rawAdc) {
  int seg = segmentForRaw(rawAdc);
  float y = segModel[seg].m * (float)rawAdc + segModel[seg].b;
  if (y < 0.0f) y = 0.0f;
  return y;
}

static bool runCalibration() {
  for (int i = 0; i < SEGMENTS; ++i) {
    clearStats(segStats[i]);
    segModel[i].m = ADC_VREF / (float)ADC_MAX;
    segModel[i].b = 0.0f;
  }

  LinStats globalStats;
  clearStats(globalStats);

  const int totalSteps = (int)((SWEEP_END_V - SWEEP_START_V) / SWEEP_STEP_V + 0.5f) + 1;
  int nextPercent = 10;

  for (int step = 0; step < totalSteps; ++step) {
    float vBatt = SWEEP_START_V + (float)step * SWEEP_STEP_V;
    if (vBatt > SWEEP_END_V) vBatt = SWEEP_END_V;

    psuSetVoltage(vBatt);
    delay(120);

    int raw = readAdcAveraged(12);
    float idealAdcVoltage = vBatt / DIVIDER_SCALE;
    int seg = segmentForRaw(raw);

    addSample(segStats[seg], (float)raw, idealAdcVoltage);
    addSample(globalStats, (float)raw, idealAdcVoltage);

    int pct = (step + 1) * 100 / totalSteps;
    if (pct >= nextPercent) {
      Serial.printf("%d%% complete\n", nextPercent);
      nextPercent += 10;
    }
  }

  LinearModel globalModel;
  if (!fitLinear(globalStats, globalModel)) return false;

  for (int i = 0; i < SEGMENTS; ++i) {
    if (!fitLinear(segStats[i], segModel[i])) {
      segModel[i] = globalModel;
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(150);

  pinMode(ADC_PIN, INPUT);

  Serial.println("Entering auto-calibration mode, please wait");
  Serial2.begin(9600, SERIAL_8N1, PSU_RX_PIN, PSU_TX_PIN);

  Serial.print("Taking control of power supply...");
  if (!psuTakeControl()) {
    Serial.println("failed");
    Serial.println("Cannot continue without PSU control.");
    return;
  }
  Serial.println("success");

  Serial.println("Autocalibrating");
  bool ok = runCalibration();
  psuReleaseControl();

  if (!ok) {
    Serial.println("Calibration failed");
    return;
  }

  Serial.println("Entering monitoring mode");
  gMonitoringEnabled = true;
}

void loop() {
  if (!gMonitoringEnabled) {
    delay(250);
    return;
  }

  static unsigned long lastMs = 0;
  if (millis() - lastMs < 1000) return;
  lastMs = millis();

  int raw = analogRead(ADC_PIN);
  float rawVoltage = adcRawToVoltage(raw);
  float correctedVoltage = applyModel(raw);
  float vBattEstimate = correctedVoltage * DIVIDER_SCALE;

  Serial.printf(
      "Raw voltage: %.2f V [ADC read as %d], corrected voltage: %.2f V, actual voltage: %.2f V\n",
      rawVoltage, raw, correctedVoltage, vBattEstimate);
}
