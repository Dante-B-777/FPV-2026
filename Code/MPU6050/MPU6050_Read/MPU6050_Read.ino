#include <Wire.h>
#include <math.h>

// Your wiring:
// MPU6050 SDA -> ESP32 D22 / GPIO22
// MPU6050 SCL -> ESP32 D21 / GPIO21
#define SDA_PIN 22
#define SCL_PIN 21

#define MPU_ADDR 0x68  // AD0 grounded = 0x68

// MPU6050 registers
#define PWR_MGMT_1    0x6B
#define CONFIG        0x1A
#define SMPLRT_DIV    0x19
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B

// Scale factors for selected ranges:
// Accel +/-2g  -> 16384 LSB/g
// Gyro +/-250 deg/s -> 131 LSB/(deg/s)
const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;

// ================================
// YOUR MEASURED CALIBRATION VALUES
// ================================

// Accel raw bias values
const float accelX_bias_raw = 1428.036;
const float accelY_bias_raw = -69.152;

// For Z accel, subtract only the error from expected +1g
const float accelZ_bias_raw = 53.696;

// Gyro raw bias values
const float gyroX_bias_raw = -206.781;
const float gyroY_bias_raw = 53.341;
const float gyroZ_bias_raw = -110.713;

// ================================
// FILTER SETTINGS
// ================================

// Lower cutoff = smoother but more delay.
// Higher cutoff = faster response but noisier.
const float ACCEL_CUTOFF_HZ = 8.0;
const float GYRO_CUTOFF_HZ  = 15.0;

// Raw readings
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

// Bias-corrected, unfiltered readings
float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;

// Bias-corrected, filtered readings
float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;
float gx_f = 0.0, gy_f = 0.0, gz_f = 0.0;

bool filterInitialized = false;
unsigned long lastMicros = 0;

void writeMPURegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

bool readMPU6050Raw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  int bytesReceived = Wire.requestFrom(MPU_ADDR, 14, true);

  if (bytesReceived != 14) {
    return false;
  }

  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();

  int16_t temp_raw = (Wire.read() << 8) | Wire.read();

  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();

  return true;
}

float lowPassFilter(float previousValue, float newValue, float cutoffHz, float dt) {
  float rc = 1.0 / (2.0 * PI * cutoffHz);
  float alpha = dt / (rc + dt);

  return previousValue + alpha * (newValue - previousValue);
}

void setupMPU6050() {
  // Wake up MPU6050
  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Digital low-pass filter inside MPU6050
  // 0x03 gives around 44 Hz accel bandwidth and 42 Hz gyro bandwidth.
  // Good balance for smoother readings without too much lag.
  writeMPURegister(CONFIG, 0x03);

  // Sample rate divider.
  // Internal gyro output rate is 1 kHz when DLPF is enabled.
  // Sample rate = 1000 / (1 + SMPLRT_DIV)
  // 0x04 gives 200 Hz sample rate.
  writeMPURegister(SMPLRT_DIV, 0x04);

  // Gyro range: +/-250 deg/s
  writeMPURegister(GYRO_CONFIG, 0x00);

  // Accel range: +/-2g
  writeMPURegister(ACCEL_CONFIG, 0x00);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  setupMPU6050();

  Serial.println("MPU6050 calibrated and filtered IMU readings");
  Serial.println("Ax_g,Ay_g,Az_g,Gx_dps,Gy_dps,Gz_dps");

  lastMicros = micros();
}

void loop() {
  if (!readMPU6050Raw()) {
    Serial.println("I2C_read_error");
    delay(10);
    return;
  }

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastMicros) / 1000000.0;
  lastMicros = nowMicros;

  // Protect against weird dt values from reset or serial delay
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.005;  // fallback around 200 Hz
  }

  // ================================
  // BIAS CORRECTION + UNIT CONVERSION
  // ================================

  // Accel in g
  ax_g = ((float)ax_raw - accelX_bias_raw) / ACCEL_SCALE;
  ay_g = ((float)ay_raw - accelY_bias_raw) / ACCEL_SCALE;
  az_g = ((float)az_raw - accelZ_bias_raw) / ACCEL_SCALE;

  // Gyro in deg/s
  gx_dps = ((float)gx_raw - gyroX_bias_raw) / GYRO_SCALE;
  gy_dps = ((float)gy_raw - gyroY_bias_raw) / GYRO_SCALE;
  gz_dps = ((float)gz_raw - gyroZ_bias_raw) / GYRO_SCALE;

  // ================================
  // SOFTWARE LOW-PASS FILTER
  // ================================

  if (!filterInitialized) {
    ax_f = ax_g;
    ay_f = ay_g;
    az_f = az_g;

    gx_f = gx_dps;
    gy_f = gy_dps;
    gz_f = gz_dps;

    filterInitialized = true;
  } else {
    ax_f = lowPassFilter(ax_f, ax_g, ACCEL_CUTOFF_HZ, dt);
    ay_f = lowPassFilter(ay_f, ay_g, ACCEL_CUTOFF_HZ, dt);
    az_f = lowPassFilter(az_f, az_g, ACCEL_CUTOFF_HZ, dt);

    gx_f = lowPassFilter(gx_f, gx_dps, GYRO_CUTOFF_HZ, dt);
    gy_f = lowPassFilter(gy_f, gy_dps, GYRO_CUTOFF_HZ, dt);
    gz_f = lowPassFilter(gz_f, gz_dps, GYRO_CUTOFF_HZ, dt);
  }

  // ================================
  // SERIAL PLOTTER OUTPUT
  // ================================

  Serial.print("Ax_g:");
  Serial.print(ax_f, 5);
  Serial.print("\t");

  Serial.print("Ay_g:");
  Serial.print(ay_f, 5);
  Serial.print("\t");

  Serial.print("Az_g:");
  Serial.print(az_f, 5);
  Serial.print("\t");

  Serial.print("Gx_dps:");
  Serial.print(gx_f, 5);
  Serial.print("\t");

  Serial.print("Gy_dps:");
  Serial.print(gy_f, 5);
  Serial.print("\t");

  Serial.print("Gz_dps:");
  Serial.println(gz_f, 5);

  // Around 200 Hz loop target
  delay(5);
}