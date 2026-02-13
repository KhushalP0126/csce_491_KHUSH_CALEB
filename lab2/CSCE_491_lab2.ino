#include <Arduino.h>
#include <math.h>

#define SCL 1
#define SDA 2

static const uint8_t MPU_ADDR = 0x68;

static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_ACCEL_XOUT_L = 0x3C;
static const uint8_t REG_ACCEL_YOUT_H = 0x3D;
static const uint8_t REG_ACCEL_YOUT_L = 0x3E;
static const uint8_t REG_ACCEL_ZOUT_H = 0x3F;
static const uint8_t REG_ACCEL_ZOUT_L = 0x40;

static const int I2C_DELAY_US = 10;

static void i2c_delay() {
  delayMicroseconds(I2C_DELAY_US);
}

static void sda_output() {
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
}

static void sda_input() {
  digitalWrite(SDA, HIGH); // release line before switching to input
  pinMode(SDA, INPUT_PULLUP);
}

static void scl_high() {
  digitalWrite(SCL, HIGH);
}

static void scl_low() {
  digitalWrite(SCL, LOW);
}

static void sda_high() {
  digitalWrite(SDA, HIGH);
}

static void sda_low() {
  digitalWrite(SDA, LOW);
}

static void i2c_start() {
  sda_output();
  sda_high();
  scl_high();
  i2c_delay();
  sda_low();
  i2c_delay();
  scl_low();
  i2c_delay();
}

static void i2c_stop() {
  sda_output();
  sda_low();
  i2c_delay();
  scl_high();
  i2c_delay();
  sda_high();
  i2c_delay();
}

static bool i2c_write_byte(uint8_t data) {
  sda_output();
  for (int i = 7; i >= 0; --i) {
    if (data & (1 << i)) {
      sda_high();
    } else {
      sda_low();
    }
    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
    i2c_delay();
  }

  // ACK bit
  sda_input();
  i2c_delay();
  scl_high();
  i2c_delay();
  bool ack = (digitalRead(SDA) == LOW);
  scl_low();
  i2c_delay();
  sda_output();
  return ack;
}

static uint8_t i2c_read_byte(bool send_ack) {
  uint8_t data = 0;
  sda_input();
  for (int i = 7; i >= 0; --i) {
    i2c_delay();
    scl_high();
    i2c_delay();
    if (digitalRead(SDA)) {
      data |= (1 << i);
    }
    scl_low();
    i2c_delay();
  }

  // ACK/NACK bit
  sda_output();
  if (send_ack) {
    sda_low();
  } else {
    sda_high();
  }
  i2c_delay();
  scl_high();
  i2c_delay();
  scl_low();
  i2c_delay();
  return data;
}

static bool i2c_write_register(uint8_t dev_addr, uint8_t reg, uint8_t value) {
  Serial.printf("I2C WRITE dev=0x%02X reg=0x%02X val=0x%02X\n", dev_addr, reg, value);
  i2c_start();
  bool ack1 = i2c_write_byte((dev_addr << 1) | 0);
  if (!ack1) {
    Serial.println("NACK after device address (write)");
  }
  bool ack2 = i2c_write_byte(reg);
  if (!ack2) {
    Serial.println("NACK after register address");
  }
  bool ack3 = i2c_write_byte(value);
  if (!ack3) {
    Serial.println("NACK after data byte");
  }
  i2c_stop();
  return ack1 && ack2 && ack3;
}

static bool i2c_read_registers(uint8_t dev_addr, uint8_t start_reg, uint8_t *buf, size_t len) {
  if (len == 0) return true;
  Serial.printf("I2C READ dev=0x%02X start_reg=0x%02X len=%u\n", dev_addr, start_reg, (unsigned)len);

  // Transaction 1: write start register
  i2c_start();
  bool ack1 = i2c_write_byte((dev_addr << 1) | 0);
  if (!ack1) {
    Serial.println("NACK after device address (write)");
  }
  bool ack2 = i2c_write_byte(start_reg);
  if (!ack2) {
    Serial.println("NACK after register address");
  }
  i2c_stop();

  // Transaction 2: read data
  i2c_start();
  bool ack3 = i2c_write_byte((dev_addr << 1) | 1);
  if (!ack3) {
    Serial.println("NACK after device address (read)");
  }

  for (size_t i = 0; i < len; ++i) {
    bool send_ack = (i + 1 < len);
    buf[i] = i2c_read_byte(send_ack);
  }

  i2c_stop();
  return ack1 && ack2 && ack3;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting MPU6050 initialization...");

  pinMode(SCL, OUTPUT_OPEN_DRAIN);
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  scl_high();
  sda_high();

  // Required by lab: write 0 to power management register 0x6B on startup.
  delay(50);
  bool init_ok = i2c_write_register(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
  if (init_ok) {
    Serial.println("MPU6050 woke up successfully");
  } else {
    Serial.println("MPU6050 initialization FAILED - check wiring!");
  }

  delay(100);
  uint8_t who_am_i = 0;
  if (i2c_read_registers(MPU_ADDR, 0x75, &who_am_i, 1)) {
    Serial.printf("WHO_AM_I register: 0x%02X (expected 0x68)\n", who_am_i);
  } else {
    Serial.println("Failed to read WHO_AM_I register!");
  }
  Serial.println("Starting main loop...");
}

void loop() {
  uint8_t data[6] = {0};
  if (i2c_read_registers(MPU_ADDR, REG_ACCEL_XOUT_H, data, sizeof(data))) {
    int16_t ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t az = (int16_t)((data[4] << 8) | data[5]);

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float angle_xy = atan2(ax_g, ay_g) * RAD_TO_DEG;
    float angle_xz = atan2(ax_g, az_g) * RAD_TO_DEG;
    float angle_yz = atan2(ay_g, az_g) * RAD_TO_DEG;

    Serial.printf(
        "RAW ax=%d ay=%d az=%d | g ax=%.4f ay=%.4f az=%.4f | ANG XY=%.2f XZ=%.2f YZ=%.2f\n",
        ax, ay, az, ax_g, ay_g, az_g, angle_xy, angle_xz, angle_yz);
  } else {
    Serial.println("I2C read failed");
  }

  delay(1000);
}
