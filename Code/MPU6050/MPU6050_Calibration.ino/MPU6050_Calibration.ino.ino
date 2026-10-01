#include <Wire.h>
#include <math.h>

// Your requested wiring:
// MPU6050 SDA -> ESP32 D22 / GPIO22
// MPU6050 SCL -> ESP32 D21 / GPIO21
#define SDA_PIN 22
#define SCL_PIN 21

#define MPU_ADDR 0x68  // AD0 grounded = 0x68

// MPU6050 registers
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define CONFIG_REG    0x1A
#define SMPLRT_DIV    0x19

// Calibration settings
#define NUM_SAMPLES 8000
#define DISCARD_SAMPLES 500

// Target sample rate
// 2000 us = 500 Hz
#define SAMPLE_PERIOD_US 2000

// Scale factors for selected ranges
// Accel +/-2g: 16384 LSB/g
// Gyro +/-250 deg/s: 131 LSB/(deg/s)
const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;

// Runtime gyro preview low-pass filter
const float GYRO_ALPHA = 0.85;

// Raw MPU6050 data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Body-frame corrected data
int16_t bax, bay, baz;
int16_t bgx, bgy, bgz;

// Sums for mean
double sumAX = 0, sumAY = 0, sumAZ = 0;
double sumGX = 0, sumGY = 0, sumGZ = 0;

// Sums of squares for noise/std dev
double sumSqAX = 0, sumSqAY = 0, sumSqAZ = 0;
double sumSqGX = 0, sumSqGY = 0, sumSqGZ = 0;

// Accelerometer angle sums
double sumRollAngle = 0.0;
double sumPitchAngle = 0.0;
double sumSqRollAngle = 0.0;
double sumSqPitchAngle = 0.0;

// Min/max, using body-frame corrected data
int16_t minAX = 32767, minAY = 32767, minAZ = 32767;
int16_t minGX = 32767, minGY = 32767, minGZ = 32767;

int16_t maxAX = -32768, maxAY = -32768, maxAZ = -32768;
int16_t maxGX = -32768, maxGY = -32768, maxGZ = -32768;

// Angle min/max
float minRollAngle = 100000.0f;
float maxRollAngle = -100000.0f;
float minPitchAngle = 100000.0f;
float maxPitchAngle = -100000.0f;

// Final offsets, body-frame corrected
float accelX_bias_raw = 0;
float accelY_bias_raw = 0;
float accelZ_bias_raw = 0;
float gyroX_bias_raw  = 0;
float gyroY_bias_raw  = 0;
float gyroZ_bias_raw  = 0;

// NEW: level-angle offsets for self-level flight code
float rollAngleOffset_deg = 0.0f;
float pitchAngleOffset_deg = 0.0f;

// Filtered gyro preview values
float gyroX_filtered = 0;
float gyroY_filtered = 0;
float gyroZ_filtered = 0;

// Live angle preview
float rollAngle_raw_deg = 0.0f;
float pitchAngle_raw_deg = 0.0f;
float rollAngle_corrected_deg = 0.0f;
float pitchAngle_corrected_deg = 0.0f;

void writeMPURegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

void applyAxisCorrection() {
  // Axis correction for your mounting:
  //
  // IMU is mounted upside down.
  // IMU Y axis points toward the drone/front direction.
  //
  // Body-frame mapping:
  // body X = -raw IMU X
  // body Y =  raw IMU Y
  // body Z = -raw IMU Z
  //
  // Apply the same sign changes to gyro axes.
  bax = -ax;
  bay =  ay;
  baz = -az;

  bgx = -gx;
  bgy =  gy;
  bgz = -gz;
}

void computeAccelAngles() {
  float accelX_g = (float)bax / ACCEL_SCALE;
  float accelY_g = (float)bay / ACCEL_SCALE;
  float accelZ_g = (float)baz / ACCEL_SCALE;

  // These match the self-level flight code:
  //
  // physical roll  = rotation about IMU Y
  // physical pitch = rotation about IMU X
  //
  // Rotation about Y mainly changes accel X/Z.
  // Rotation about X mainly changes accel Y/Z.
  rollAngle_raw_deg =
    atan2f(-accelX_g, sqrtf(accelY_g * accelY_g + accelZ_g * accelZ_g)) *
    180.0f / PI;

  pitchAngle_raw_deg =
    atan2f(accelY_g, accelZ_g) *
    180.0f / PI;

  rollAngle_corrected_deg = rollAngle_raw_deg - rollAngleOffset_deg;
  pitchAngle_corrected_deg = pitchAngle_raw_deg - pitchAngleOffset_deg;
}

bool readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  int bytesReceived = Wire.requestFrom(MPU_ADDR, 14, true);

  if (bytesReceived != 14) {
    return false;
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  // Temperature, not used here
  int16_t tempRaw = (Wire.read() << 8) | Wire.read();

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  applyAxisCorrection();
  computeAccelAngles();

  return true;
}

void updateMinMax() {
  if (bax < minAX) minAX = bax;
  if (bay < minAY) minAY = bay;
  if (baz < minAZ) minAZ = baz;

  if (bgx < minGX) minGX = bgx;
  if (bgy < minGY) minGY = bgy;
  if (bgz < minGZ) minGZ = bgz;

  if (bax > maxAX) maxAX = bax;
  if (bay > maxAY) maxAY = bay;
  if (baz > maxAZ) maxAZ = baz;

  if (bgx > maxGX) maxGX = bgx;
  if (bgy > maxGY) maxGY = bgy;
  if (bgz > maxGZ) maxGZ = bgz;

  if (rollAngle_raw_deg < minRollAngle) minRollAngle = rollAngle_raw_deg;
  if (rollAngle_raw_deg > maxRollAngle) maxRollAngle = rollAngle_raw_deg;

  if (pitchAngle_raw_deg < minPitchAngle) minPitchAngle = pitchAngle_raw_deg;
  if (pitchAngle_raw_deg > maxPitchAngle) maxPitchAngle = pitchAngle_raw_deg;
}

void printChannelRaw(const char* name, double meanRaw, double stdRaw, int16_t minVal, int16_t maxVal) {
  Serial.print(name);
  Serial.print(" | mean raw: ");
  Serial.print(meanRaw, 3);
  Serial.print(" | noise std raw: ");
  Serial.print(stdRaw, 3);
  Serial.print(" | min: ");
  Serial.print(minVal);
  Serial.print(" | max: ");
  Serial.println(maxVal);
}

void printChannelAccel(const char* name, double meanRaw, double stdRaw, double expectedG) {
  double meanG = meanRaw / ACCEL_SCALE;
  double stdG = stdRaw / ACCEL_SCALE;
  double biasG = meanG - expectedG;
  double biasRaw = meanRaw - expectedG * ACCEL_SCALE;

  Serial.print(name);
  Serial.print(" | mean: ");
  Serial.print(meanG, 6);
  Serial.print(" g | bias: ");
  Serial.print(biasG, 6);
  Serial.print(" g | bias raw: ");
  Serial.print(biasRaw, 2);
  Serial.print(" | noise std: ");
  Serial.print(stdG, 6);
  Serial.println(" g");
}

void printChannelGyro(const char* name, double meanRaw, double stdRaw) {
  double meanDPS = meanRaw / GYRO_SCALE;
  double stdDPS = stdRaw / GYRO_SCALE;

  Serial.print(name);
  Serial.print(" | bias: ");
  Serial.print(meanDPS, 6);
  Serial.print(" deg/s | bias raw: ");
  Serial.print(meanRaw, 2);
  Serial.print(" | noise std: ");
  Serial.print(stdDPS, 6);
  Serial.println(" deg/s");
}

void printAngleChannel(const char* name,
                       double meanAngle,
                       double stdAngle,
                       float minAngle,
                       float maxAngle) {
  Serial.print(name);
  Serial.print(" | mean offset: ");
  Serial.print(meanAngle, 4);
  Serial.print(" deg | noise std: ");
  Serial.print(stdAngle, 4);
  Serial.print(" deg | min: ");
  Serial.print(minAngle, 4);
  Serial.print(" deg | max: ");
  Serial.print(maxAngle, 4);
  Serial.println(" deg");
}

void waitForNextSample(unsigned long& nextSampleTime) {
  while ((long)(micros() - nextSampleTime) < 0) {
    // wait
  }

  nextSampleTime += SAMPLE_PERIOD_US;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("Improved MPU6050 noise, bias, and level-angle calibration");
  Serial.println("IMU mounting correction enabled:");
  Serial.println("IMU is upside down and IMU Y points forward.");
  Serial.println("Using body-frame mapping: X=-rawX, Y=rawY, Z=-rawZ");
  Serial.println();
  Serial.println("IMPORTANT:");
  Serial.println("Keep the drone completely still AND physically level.");
  Serial.println("The measured roll/pitch angle during this calibration becomes your self-level zero.");
  Serial.println();

  // Wake up MPU6050
  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Internal Digital Low-Pass Filter
  writeMPURegister(CONFIG_REG, 0x03);

  // Sample rate divider.
  // With DLPF enabled, gyro output rate is usually 1 kHz.
  // Sample rate = 1000 / (1 + SMPLRT_DIV)
  // 0x01 gives 500 Hz internal sample rate.
  writeMPURegister(SMPLRT_DIV, 0x01);

  // Gyro range +/-250 deg/s
  writeMPURegister(GYRO_CONFIG, 0x00);

  // Accel range +/-2g
  writeMPURegister(ACCEL_CONFIG, 0x00);

  delay(500);

  Serial.println("MPU6050 configured:");
  Serial.println("Accel range: +/-2g");
  Serial.println("Gyro range: +/-250 deg/s");
  Serial.println("DLPF setting: 0x03");
  Serial.println("Sample rate target: 500 Hz");
  Serial.println();

  Serial.print("Discarding ");
  Serial.print(DISCARD_SAMPLES);
  Serial.println(" startup samples...");

  unsigned long nextSampleTime = micros();

  for (int i = 0; i < DISCARD_SAMPLES; i++) {
    waitForNextSample(nextSampleTime);
    readMPU();
  }

  Serial.println("Startup discard complete.");
  Serial.println();

  Serial.print("Collecting ");
  Serial.print(NUM_SAMPLES);
  Serial.println(" calibration samples...");
  Serial.println();

  int goodSamples = 0;
  int badSamples = 0;

  nextSampleTime = micros();

  while (goodSamples < NUM_SAMPLES) {
    waitForNextSample(nextSampleTime);

    if (!readMPU()) {
      badSamples++;
      continue;
    }

    // Use body-frame corrected data
    sumAX += bax;
    sumAY += bay;
    sumAZ += baz;

    sumGX += bgx;
    sumGY += bgy;
    sumGZ += bgz;

    sumSqAX += (double)bax * bax;
    sumSqAY += (double)bay * bay;
    sumSqAZ += (double)baz * baz;

    sumSqGX += (double)bgx * bgx;
    sumSqGY += (double)bgy * bgy;
    sumSqGZ += (double)bgz * bgz;

    // NEW: collect accelerometer angle offsets
    sumRollAngle += rollAngle_raw_deg;
    sumPitchAngle += pitchAngle_raw_deg;

    sumSqRollAngle += (double)rollAngle_raw_deg * rollAngle_raw_deg;
    sumSqPitchAngle += (double)pitchAngle_raw_deg * pitchAngle_raw_deg;

    updateMinMax();

    if (goodSamples % 500 == 0) {
      Serial.print(".");
    }

    goodSamples++;
  }

  Serial.println();
  Serial.println();
  Serial.println("Calibration complete.");
  Serial.print("Good samples: ");
  Serial.println(goodSamples);
  Serial.print("Bad I2C reads: ");
  Serial.println(badSamples);
  Serial.println();

  double meanAX = sumAX / NUM_SAMPLES;
  double meanAY = sumAY / NUM_SAMPLES;
  double meanAZ = sumAZ / NUM_SAMPLES;

  double meanGX = sumGX / NUM_SAMPLES;
  double meanGY = sumGY / NUM_SAMPLES;
  double meanGZ = sumGZ / NUM_SAMPLES;

  double stdAX = sqrt((sumSqAX / NUM_SAMPLES) - (meanAX * meanAX));
  double stdAY = sqrt((sumSqAY / NUM_SAMPLES) - (meanAY * meanAY));
  double stdAZ = sqrt((sumSqAZ / NUM_SAMPLES) - (meanAZ * meanAZ));

  double stdGX = sqrt((sumSqGX / NUM_SAMPLES) - (meanGX * meanGX));
  double stdGY = sqrt((sumSqGY / NUM_SAMPLES) - (meanGY * meanGY));
  double stdGZ = sqrt((sumSqGZ / NUM_SAMPLES) - (meanGZ * meanGZ));

  double meanRollAngle = sumRollAngle / NUM_SAMPLES;
  double meanPitchAngle = sumPitchAngle / NUM_SAMPLES;

  double stdRollAngle =
    sqrt((sumSqRollAngle / NUM_SAMPLES) - (meanRollAngle * meanRollAngle));

  double stdPitchAngle =
    sqrt((sumSqPitchAngle / NUM_SAMPLES) - (meanPitchAngle * meanPitchAngle));

  // Store final offsets in body-frame corrected raw units
  accelX_bias_raw = meanAX;
  accelY_bias_raw = meanAY;

  // For accel Z, subtract only the error from expected +1g,
  // not the whole Z reading.
  accelZ_bias_raw = meanAZ - ACCEL_SCALE;

  gyroX_bias_raw = meanGX;
  gyroY_bias_raw = meanGY;
  gyroZ_bias_raw = meanGZ;

  // NEW: level offsets in degrees
  rollAngleOffset_deg = meanRollAngle;
  pitchAngleOffset_deg = meanPitchAngle;

  Serial.println("========== RAW CHANNEL METRICS, BODY FRAME ==========");
  printChannelRaw("Accel X", meanAX, stdAX, minAX, maxAX);
  printChannelRaw("Accel Y", meanAY, stdAY, minAY, maxAY);
  printChannelRaw("Accel Z", meanAZ, stdAZ, minAZ, maxAZ);

  printChannelRaw("Gyro X ", meanGX, stdGX, minGX, maxGX);
  printChannelRaw("Gyro Y ", meanGY, stdGY, minGY, maxGY);
  printChannelRaw("Gyro Z ", meanGZ, stdGZ, minGZ, maxGZ);

  Serial.println();
  Serial.println("========== ACCEL METRICS, BODY FRAME ==========");
  Serial.println("Assuming drone is flat and still with body Z seeing +1g.");
  printChannelAccel("Accel X", meanAX, stdAX, 0.0);
  printChannelAccel("Accel Y", meanAY, stdAY, 0.0);
  printChannelAccel("Accel Z", meanAZ, stdAZ, 1.0);

  Serial.println();
  Serial.println("========== ACCELEROMETER ANGLE METRICS ==========");
  Serial.println("These are the self-level angle offsets.");
  Serial.println("If the drone was level during calibration, these should become zero in flight after subtraction.");
  printAngleChannel("Roll angle ", meanRollAngle, stdRollAngle, minRollAngle, maxRollAngle);
  printAngleChannel("Pitch angle", meanPitchAngle, stdPitchAngle, minPitchAngle, maxPitchAngle);

  Serial.println();
  Serial.println("========== GYRO METRICS, BODY FRAME ==========");
  Serial.println("Expected gyro rate is 0 deg/s on all axes.");
  printChannelGyro("Gyro X ", meanGX, stdGX);
  printChannelGyro("Gyro Y ", meanGY, stdGY);
  printChannelGyro("Gyro Z ", meanGZ, stdGZ);

  Serial.println();
  Serial.println("========== COPY THESE OFFSETS ==========");
  Serial.println("These offsets are already corrected for your upside-down IMU mounting.");
  Serial.println("Use these to subtract bias from future BODY-FRAME raw readings:");
  Serial.println();

  Serial.print("float accelX_bias_raw = "); Serial.print(accelX_bias_raw, 3); Serial.println(";");
  Serial.print("float accelY_bias_raw = "); Serial.print(accelY_bias_raw, 3); Serial.println(";");

  Serial.println("// For accel Z, subtract only the error from expected +1g, not the whole mean:");
  Serial.print("float accelZ_bias_raw = "); Serial.print(accelZ_bias_raw, 3); Serial.println(";");

  Serial.print("float gyroX_bias_raw  = "); Serial.print(gyroX_bias_raw, 3); Serial.println(";");
  Serial.print("float gyroY_bias_raw  = "); Serial.print(gyroY_bias_raw, 3); Serial.println(";");
  Serial.print("float gyroZ_bias_raw  = "); Serial.print(gyroZ_bias_raw, 3); Serial.println(";");

  Serial.println();
  Serial.println("// NEW: use these for accelerometer self-level angle correction:");
  Serial.print("float rollAngleOffset_deg  = "); Serial.print(rollAngleOffset_deg, 4); Serial.println(";");
  Serial.print("float pitchAngleOffset_deg = "); Serial.print(pitchAngleOffset_deg, 4); Serial.println(";");

  Serial.println();
  Serial.println("========== HOW TO USE IN FLIGHT CODE ==========");
  Serial.println("1. Read raw ax, ay, az, gx, gy, gz.");
  Serial.println("2. Apply mounting correction:");
  Serial.println("   bax = -ax; bay = ay; baz = -az;");
  Serial.println("   bgx = -gx; bgy = gy; bgz = -gz;");
  Serial.println("3. Subtract gyro offsets from gyro readings.");
  Serial.println("4. Compute rollAccAngle_deg and pitchAccAngle_deg.");
  Serial.println("5. Subtract angle offsets:");
  Serial.println("   rollAccAngle_deg  -= rollAngleOffset_deg;");
  Serial.println("   pitchAccAngle_deg -= pitchAngleOffset_deg;");
  Serial.println();

  Serial.println("========== LIVE GYRO + ACCEL ANGLE PREVIEW ==========");
  Serial.print("GYRO_ALPHA = ");
  Serial.println(GYRO_ALPHA, 3);

  Serial.println();
  Serial.println("Open Serial Plotter to see:");
  Serial.println("gyroX_raw,gyroX_filtered,gyroY_raw,gyroY_filtered,gyroZ_raw,gyroZ_filtered,rollAngle_raw,rollAngle_corrected,pitchAngle_raw,pitchAngle_corrected");
  Serial.println();
}

void loop() {
  static unsigned long nextSampleTime = micros();

  waitForNextSample(nextSampleTime);

  if (!readMPU()) {
    return;
  }

  // Bias-corrected body-frame gyro readings in deg/s
  float gyroX_raw_dps = (bgx - gyroX_bias_raw) / GYRO_SCALE;
  float gyroY_raw_dps = (bgy - gyroY_bias_raw) / GYRO_SCALE;
  float gyroZ_raw_dps = (bgz - gyroZ_bias_raw) / GYRO_SCALE;

  // Simple low-pass filtered gyro readings
  gyroX_filtered = GYRO_ALPHA * gyroX_filtered + (1.0 - GYRO_ALPHA) * gyroX_raw_dps;
  gyroY_filtered = GYRO_ALPHA * gyroY_filtered + (1.0 - GYRO_ALPHA) * gyroY_raw_dps;
  gyroZ_filtered = GYRO_ALPHA * gyroZ_filtered + (1.0 - GYRO_ALPHA) * gyroZ_raw_dps;

  rollAngle_corrected_deg = rollAngle_raw_deg - rollAngleOffset_deg;
  pitchAngle_corrected_deg = pitchAngle_raw_deg - pitchAngleOffset_deg;

  // Serial Plotter format
  Serial.print("gyroX_raw:");
  Serial.print(gyroX_raw_dps, 4);
  Serial.print(",");

  Serial.print("gyroX_filtered:");
  Serial.print(gyroX_filtered, 4);
  Serial.print(",");

  Serial.print("gyroY_raw:");
  Serial.print(gyroY_raw_dps, 4);
  Serial.print(",");

  Serial.print("gyroY_filtered:");
  Serial.print(gyroY_filtered, 4);
  Serial.print(",");

  Serial.print("gyroZ_raw:");
  Serial.print(gyroZ_raw_dps, 4);
  Serial.print(",");

  Serial.print("gyroZ_filtered:");
  Serial.print(gyroZ_filtered, 4);
  Serial.print(",");

  Serial.print("rollAngle_raw:");
  Serial.print(rollAngle_raw_deg, 4);
  Serial.print(",");

  Serial.print("rollAngle_corrected:");
  Serial.print(rollAngle_corrected_deg, 4);
  Serial.print(",");

  Serial.print("pitchAngle_raw:");
  Serial.print(pitchAngle_raw_deg, 4);
  Serial.print(",");

  Serial.print("pitchAngle_corrected:");
  Serial.println(pitchAngle_corrected_deg, 4);
}