/*
  PMW3901 FULL-BURST + IMU ROTATION-COMPENSATION DIAGNOSTIC

  Hardware from the current aircraft:
    PMW3901 SCK=18, MISO=19, MOSI=23, CS=13, RESET=33
    MPU6050 SDA=22, SCL=21, address 0x68

  PROPELLERS OFF. Preferably unplug the ESC power while doing the axis tests.

  This sketch does not fly the aircraft. It answers four questions that the
  current flight code cannot answer:
    1. Which PMW raw axis/sign corresponds to body right and body forward?
    2. Is the PMW reporting valid motion (motion byte), useful SQUAL, and a
       reasonable shutter value on the actual floor and lighting?
    3. Do roll/pitch gyro compensation signs remove apparent motion when the
       drone is rotated in place?
    4. Is motor vibration creating false optical flow?

  Serial monitor: 460800 baud, Newline ending.

  Commands:
    help
    map 0..7       Select raw-count to body-axis map (listed by help).
    comp 0|1       Disable/enable current roll/pitch/yaw compensation.
    scale <value>  Radians per PMW count. Start with 0.0025974 (1/385).
    height <m>     Fixed sensor height used only for m/s conversion.
    offsety <m>    PMW forward offset used for yaw lever-arm correction.
    zero            Reset integrated body displacement.
    mark <text>     Print a marker into the CSV log.

  Recommended axis test:
    - Hold the aircraft LEVEL, 0.4 to 0.7 m above a matte textured floor.
    - Enter "zero".
    - Translate it slowly to the aircraft RIGHT without rotating it.
    - The correct map has VX positive, VY near zero.
    - Translate it FORWARD. The correct map has VY positive, VX near zero.

  Recommended compensation test:
    - Keep the sensor over one point on the floor.
    - With comp=1, roll and pitch the aircraft by hand and return to level.
    - Correct mapping/signs produce much smaller integrated displacement than
      comp=0. If comp=1 makes it larger, a compensation sign/axis is wrong.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Bitcraze_PMW3901.h>
#include <math.h>
#include <string.h>

static constexpr uint8_t PMW_SCK_PIN = 18;
static constexpr uint8_t PMW_MISO_PIN = 19;
static constexpr uint8_t PMW_MOSI_PIN = 23;
static constexpr uint8_t PMW_CS_PIN = 13;
static constexpr uint8_t PMW_RST_PIN = 33;

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t SDA_PIN = 22;
static constexpr uint8_t SCL_PIN = 21;
static constexpr float GYRO_LSB_PER_DPS = 131.0f; // +/-250 dps
static constexpr float DEG_TO_RAD_F = 0.01745329251994329577f;

static constexpr uint32_t SAMPLE_PERIOD_US = 10000; // 100 Hz
static constexpr uint32_t PMW_SPI_HZ = 2000000;     // conservative, proven by ESP-Drone

Bitcraze_PMW3901 flow(PMW_CS_PIN);

struct PmwMotionBurst
{
  uint8_t motion = 0;
  uint8_t observation = 0;
  int16_t deltaX = 0;
  int16_t deltaY = 0;
  uint8_t squal = 0;
  uint8_t rawDataSum = 0;
  uint8_t maxRawData = 0;
  uint8_t minRawData = 0;
  uint16_t shutter = 0;
  bool transferPlausible = false;
};

float gyroXBiasRaw = 0.0f;
float gyroYBiasRaw = 0.0f;
float gyroZBiasRaw = 0.0f;

int mappingMode = 1; // Current flight-code map: bodyX=+rawX, bodyY=-rawY
bool compensationEnabled = true;
float radiansPerCount = 1.0f / 385.0f;
float assumedHeightM = 0.50f;
float pmwOffsetYM = 0.04f;
float pmwOffsetXM = 0.00f;

float integratedBodyX = 0.0f;
float integratedBodyY = 0.0f;
uint32_t previousSampleUs = 0;
uint32_t sequenceNumber = 0;

char serialLine[96] = {};
size_t serialLength = 0;

uint32_t statsStartMs = 0;
uint32_t statsSamples = 0;
uint32_t statsGoodMotion = 0;
uint32_t statsPlausible = 0;
uint64_t statsSqualSum = 0;
uint32_t statsShutterMax = 0;

static int16_t makeSigned16(uint8_t lowByte, uint8_t highByte)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(lowByte) |
    (static_cast<uint16_t>(highByte) << 8U)
  );
}

bool writeMpuRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readMpuGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t requested = 6;
  if (Wire.requestFrom(MPU_ADDR, requested, true) != requested) return false;

  gx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  gy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  gz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  return true;
}

bool initializeMpu()
{
  if (!writeMpuRegister(0x6B, 0x00)) return false; // wake
  delay(100);
  if (!writeMpuRegister(0x1B, 0x00)) return false; // gyro +/-250 dps
  if (!writeMpuRegister(0x1A, 0x03)) return false; // DLPF ~44 Hz
  return true;
}

bool calibrateGyro()
{
  Serial.println("GYRO_CAL_START,keep_aircraft_still_and_level");

  static constexpr int SAMPLE_COUNT = 1500;
  int64_t sumX = 0;
  int64_t sumY = 0;
  int64_t sumZ = 0;
  int good = 0;

  for (int i = 0; i < SAMPLE_COUNT; ++i)
  {
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    if (readMpuGyroRaw(gx, gy, gz))
    {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      good++;
    }
    delay(2);
  }

  if (good < SAMPLE_COUNT / 2) return false;

  gyroXBiasRaw = static_cast<float>(sumX) / static_cast<float>(good);
  gyroYBiasRaw = static_cast<float>(sumY) / static_cast<float>(good);
  gyroZBiasRaw = static_cast<float>(sumZ) / static_cast<float>(good);

  Serial.print("GYRO_CAL_DONE,GXBIAS:"); Serial.print(gyroXBiasRaw, 3);
  Serial.print(",GYBIAS:"); Serial.print(gyroYBiasRaw, 3);
  Serial.print(",GZBIAS:"); Serial.println(gyroZBiasRaw, 3);
  return true;
}

void resetPmw()
{
  pinMode(PMW_RST_PIN, OUTPUT);
  digitalWrite(PMW_RST_PIN, HIGH);
  delay(10);
  digitalWrite(PMW_RST_PIN, LOW);
  delay(1);
  digitalWrite(PMW_RST_PIN, HIGH);
  delay(50);
}

bool readPmwMotionBurst(PmwMotionBurst &out)
{
  uint8_t raw[12] = {};

  SPI.beginTransaction(SPISettings(PMW_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(PMW_CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.transfer(0x16U); // Motion_Burst register, read direction (MSB clear)
  delayMicroseconds(50);

  for (size_t i = 0; i < sizeof(raw); ++i)
  {
    raw[i] = SPI.transfer(0x00);
  }

  delayMicroseconds(50);
  digitalWrite(PMW_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);

  out.motion = raw[0];
  out.observation = raw[1];
  out.deltaX = makeSigned16(raw[2], raw[3]);
  out.deltaY = makeSigned16(raw[4], raw[5]);
  out.squal = raw[6];
  out.rawDataSum = raw[7];
  out.maxRawData = raw[8];
  out.minRawData = raw[9];
  out.shutter = static_cast<uint16_t>(
    (static_cast<uint16_t>(raw[10]) << 8U) |
    static_cast<uint16_t>(raw[11])
  );

  bool allZero = true;
  bool allOnes = true;
  for (uint8_t value : raw)
  {
    allZero = allZero && value == 0x00U;
    allOnes = allOnes && value == 0xFFU;
  }

  out.transferPlausible = !allZero && !allOnes &&
    abs(static_cast<int>(out.deltaX)) <= 1000 &&
    abs(static_cast<int>(out.deltaY)) <= 1000;

  return out.transferPlausible;
}

void mapRawCountsToBody(
  int16_t rawX,
  int16_t rawY,
  float &bodyXCounts,
  float &bodyYCounts
)
{
  switch (mappingMode)
  {
    case 0: bodyXCounts = +rawX; bodyYCounts = +rawY; break;
    case 1: bodyXCounts = +rawX; bodyYCounts = -rawY; break;
    case 2: bodyXCounts = -rawX; bodyYCounts = +rawY; break;
    case 3: bodyXCounts = -rawX; bodyYCounts = -rawY; break;
    case 4: bodyXCounts = +rawY; bodyYCounts = +rawX; break;
    case 5: bodyXCounts = +rawY; bodyYCounts = -rawX; break;
    case 6: bodyXCounts = -rawY; bodyYCounts = +rawX; break;
    case 7: bodyXCounts = -rawY; bodyYCounts = -rawX; break;
    default:
      bodyXCounts = +rawX;
      bodyYCounts = -rawY;
      mappingMode = 1;
      break;
  }
}

void printHelp()
{
  Serial.println("HELP,commands: map 0..7 | comp 0|1 | scale <rad/count> | height <m> | offsety <m> | zero | mark <text>");
  Serial.println("MAP0,BX=+rawX,BY=+rawY");
  Serial.println("MAP1,BX=+rawX,BY=-rawY,current_flight_code");
  Serial.println("MAP2,BX=-rawX,BY=+rawY");
  Serial.println("MAP3,BX=-rawX,BY=-rawY");
  Serial.println("MAP4,BX=+rawY,BY=+rawX");
  Serial.println("MAP5,BX=+rawY,BY=-rawX");
  Serial.println("MAP6,BX=-rawY,BY=+rawX");
  Serial.println("MAP7,BX=-rawY,BY=-rawX,Bitcraze_flowdeck_mapping");
}

void processCommand(char *command)
{
  while (*command == ' ' || *command == '\t') command++;

  int integerValue = 0;
  float floatValue = 0.0f;

  if (strcasecmp(command, "help") == 0)
  {
    printHelp();
  }
  else if (sscanf(command, "map %d", &integerValue) == 1)
  {
    if (integerValue >= 0 && integerValue <= 7)
    {
      mappingMode = integerValue;
      integratedBodyX = 0.0f;
      integratedBodyY = 0.0f;
      Serial.print("SET,MAP:"); Serial.println(mappingMode);
    }
    else
    {
      Serial.println("ERROR,map_must_be_0_to_7");
    }
  }
  else if (sscanf(command, "comp %d", &integerValue) == 1)
  {
    compensationEnabled = integerValue != 0;
    integratedBodyX = 0.0f;
    integratedBodyY = 0.0f;
    Serial.print("SET,COMP:"); Serial.println(compensationEnabled ? 1 : 0);
  }
  else if (sscanf(command, "scale %f", &floatValue) == 1)
  {
    if (isfinite(floatValue) && floatValue > 0.0005f && floatValue < 0.010f)
    {
      radiansPerCount = floatValue;
      Serial.print("SET,SCALE:"); Serial.println(radiansPerCount, 8);
    }
    else
    {
      Serial.println("ERROR,scale_out_of_range");
    }
  }
  else if (sscanf(command, "height %f", &floatValue) == 1)
  {
    if (isfinite(floatValue) && floatValue >= 0.08f && floatValue <= 2.0f)
    {
      assumedHeightM = floatValue;
      Serial.print("SET,HEIGHT:"); Serial.println(assumedHeightM, 3);
    }
    else
    {
      Serial.println("ERROR,height_must_be_0.08_to_2.0_m");
    }
  }
  else if (sscanf(command, "offsety %f", &floatValue) == 1)
  {
    if (isfinite(floatValue) && fabsf(floatValue) <= 0.25f)
    {
      pmwOffsetYM = floatValue;
      Serial.print("SET,OFFSETY:"); Serial.println(pmwOffsetYM, 4);
    }
    else
    {
      Serial.println("ERROR,offsety_out_of_range");
    }
  }
  else if (strcasecmp(command, "zero") == 0)
  {
    integratedBodyX = 0.0f;
    integratedBodyY = 0.0f;
    Serial.println("MARK,ZERO");
  }
  else if (strncasecmp(command, "mark ", 5) == 0)
  {
    Serial.print("MARK,");
    Serial.println(command + 5);
  }
  else if (*command != '\0')
  {
    Serial.print("ERROR,unknown_command:");
    Serial.println(command);
  }
}

void pollSerial()
{
  while (Serial.available() > 0)
  {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n')
    {
      serialLine[serialLength] = '\0';
      processCommand(serialLine);
      serialLength = 0;
    }
    else if (c != '\r')
    {
      if (serialLength < sizeof(serialLine) - 1)
      {
        serialLine[serialLength++] = c;
      }
      else
      {
        serialLength = 0;
        Serial.println("ERROR,command_too_long");
      }
    }
  }
}

void setup()
{
  Serial.setTxBufferSize(4096);
  Serial.begin(460800);
  delay(1000);

  Serial.println();
  Serial.println("PMW3901_FULL_BURST_DIAGNOSTIC");
  Serial.println("PROPELLERS_OFF");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(5);

  if (!initializeMpu())
  {
    Serial.println("FATAL,MPU6050_not_detected");
    while (true) delay(1000);
  }

  if (!calibrateGyro())
  {
    Serial.println("FATAL,gyro_calibration_failed");
    while (true) delay(1000);
  }

  pinMode(PMW_CS_PIN, OUTPUT);
  digitalWrite(PMW_CS_PIN, HIGH);
  resetPmw();
  SPI.begin(PMW_SCK_PIN, PMW_MISO_PIN, PMW_MOSI_PIN, PMW_CS_PIN);

  if (!flow.begin())
  {
    Serial.println("FATAL,PMW3901_initialization_failed");
    while (true) delay(1000);
  }

  // Drain the first accumulated sample.
  PmwMotionBurst ignored;
  readPmwMotionBurst(ignored);

  previousSampleUs = micros();
  statsStartMs = millis();
  printHelp();
  Serial.println("CSV,seq,ms,dt_us,map,comp,spi_ok,motion_hex,motion_b0,obs,raw_x,raw_y,squal,raw_sum,raw_max,raw_min,shutter,roll_d_rad,pitch_d_rad,yaw_d_cw_rad,bx_counts,by_counts,vx_mps,vy_mps,int_x_m,int_y_m");
}

void loop()
{
  pollSerial();

  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - previousSampleUs) < SAMPLE_PERIOD_US)
  {
    delayMicroseconds(100);
    return;
  }

  const uint32_t intervalUs = static_cast<uint32_t>(nowUs - previousSampleUs);
  previousSampleUs = nowUs;
  const float dt = static_cast<float>(intervalUs) * 1.0e-6f;

  int16_t gxRaw = 0;
  int16_t gyRaw = 0;
  int16_t gzRaw = 0;
  const bool imuOk = readMpuGyroRaw(gxRaw, gyRaw, gzRaw);

  // Preserve the controller's established conventions:
  // roll rate = -gyroY, pitch rate = +gyroX, yaw rate CW = -gyroZ.
  float rollDeltaRad = 0.0f;
  float pitchDeltaRad = 0.0f;
  float yawDeltaCwRad = 0.0f;

  if (imuOk)
  {
    const float gyroX =
      (static_cast<float>(gxRaw) - gyroXBiasRaw) / GYRO_LSB_PER_DPS;
    const float gyroY =
      (static_cast<float>(gyRaw) - gyroYBiasRaw) / GYRO_LSB_PER_DPS;
    const float gyroZ =
      (static_cast<float>(gzRaw) - gyroZBiasRaw) / GYRO_LSB_PER_DPS;

    rollDeltaRad = (-gyroY) * DEG_TO_RAD_F * dt;
    pitchDeltaRad = (+gyroX) * DEG_TO_RAD_F * dt;
    yawDeltaCwRad = (-gyroZ) * DEG_TO_RAD_F * dt;
  }

  PmwMotionBurst motion;
  const bool spiOk = readPmwMotionBurst(motion);

  float bodyXCounts = 0.0f;
  float bodyYCounts = 0.0f;
  mapRawCountsToBody(
    motion.deltaX,
    motion.deltaY,
    bodyXCounts,
    bodyYCounts
  );

  float deltaBodyXM = bodyXCounts * radiansPerCount * assumedHeightM;
  float deltaBodyYM = bodyYCounts * radiansPerCount * assumedHeightM;

  if (compensationEnabled)
  {
    // Same compensation convention as the current flight code.
    deltaBodyXM += assumedHeightM * rollDeltaRad;
    deltaBodyYM -= assumedHeightM * pitchDeltaRad;
    deltaBodyXM -= pmwOffsetYM * yawDeltaCwRad;
    deltaBodyYM += pmwOffsetXM * yawDeltaCwRad;
  }

  const float velocityX = dt > 0.0f ? deltaBodyXM / dt : 0.0f;
  const float velocityY = dt > 0.0f ? deltaBodyYM / dt : 0.0f;

  const bool motionB0 = motion.motion == 0xB0U;
  const bool sampleUsable = spiOk && motionB0 && motion.squal > 0U;
  if (sampleUsable)
  {
    integratedBodyX += deltaBodyXM;
    integratedBodyY += deltaBodyYM;
  }

  sequenceNumber++;
  statsSamples++;
  statsPlausible += spiOk ? 1U : 0U;
  statsGoodMotion += motionB0 ? 1U : 0U;
  statsSqualSum += motion.squal;
  if (motion.shutter > statsShutterMax) statsShutterMax = motion.shutter;

  Serial.print("D,"); Serial.print(sequenceNumber);
  Serial.print(','); Serial.print(millis());
  Serial.print(','); Serial.print(intervalUs);
  Serial.print(','); Serial.print(mappingMode);
  Serial.print(','); Serial.print(compensationEnabled ? 1 : 0);
  Serial.print(','); Serial.print(spiOk ? 1 : 0);
  Serial.print(",0x"); Serial.print(motion.motion, HEX);
  Serial.print(','); Serial.print(motionB0 ? 1 : 0);
  Serial.print(','); Serial.print(motion.observation);
  Serial.print(','); Serial.print(motion.deltaX);
  Serial.print(','); Serial.print(motion.deltaY);
  Serial.print(','); Serial.print(motion.squal);
  Serial.print(','); Serial.print(motion.rawDataSum);
  Serial.print(','); Serial.print(motion.maxRawData);
  Serial.print(','); Serial.print(motion.minRawData);
  Serial.print(','); Serial.print(motion.shutter);
  Serial.print(','); Serial.print(rollDeltaRad, 7);
  Serial.print(','); Serial.print(pitchDeltaRad, 7);
  Serial.print(','); Serial.print(yawDeltaCwRad, 7);
  Serial.print(','); Serial.print(bodyXCounts, 1);
  Serial.print(','); Serial.print(bodyYCounts, 1);
  Serial.print(','); Serial.print(velocityX, 4);
  Serial.print(','); Serial.print(velocityY, 4);
  Serial.print(','); Serial.print(integratedBodyX, 4);
  Serial.print(','); Serial.println(integratedBodyY, 4);

  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - statsStartMs) >= 1000U)
  {
    const float denominator = statsSamples > 0U
      ? static_cast<float>(statsSamples)
      : 1.0f;

    Serial.print("SUMMARY,samples:"); Serial.print(statsSamples);
    Serial.print(",spi_ok_pct:");
    Serial.print(100.0f * static_cast<float>(statsPlausible) / denominator, 1);
    Serial.print(",motion_b0_pct:");
    Serial.print(100.0f * static_cast<float>(statsGoodMotion) / denominator, 1);
    Serial.print(",avg_squal:");
    Serial.print(static_cast<float>(statsSqualSum) / denominator, 1);
    Serial.print(",max_shutter:"); Serial.println(statsShutterMax);

    statsStartMs = nowMs;
    statsSamples = 0;
    statsGoodMotion = 0;
    statsPlausible = 0;
    statsSqualSum = 0;
    statsShutterMax = 0;
  }
}
