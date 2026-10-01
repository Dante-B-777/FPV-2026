/*
  ESP32 ESP-NOW + MPU6050 Angle/Rate PID
  + Serial Adjustable Hover DShot

  NORMAL DSHOT300
  NO BIDIRECTIONAL TELEMETRY

  Motor pins preserved exactly as provided:
    M1 = GPIO26
    M2 = GPIO14
    M3 = GPIO27
    M4 = GPIO25

  IMU:
    MPU6050
    SDA = GPIO22
    SCL = GPIO21
    Mounted upside down

  Serial commands:
    arm
    disarm
    kill
    stop
    150
    +
    -
    level
    status

  Receiver-side roll/pitch deadband ensures neutral sticks produce:
    DES_R_ANG = 0.00
    DES_P_ANG = 0.00

  REMOVE PROPELLERS BEFORE DIRECTION TESTING.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <DShotRMT.h>

// ================================================================
// MOTOR CONFIGURATION
// ================================================================

static constexpr gpio_num_t MOTOR1_PIN = GPIO_NUM_26;
static constexpr gpio_num_t MOTOR2_PIN = GPIO_NUM_14;
static constexpr gpio_num_t MOTOR3_PIN = GPIO_NUM_27;
static constexpr gpio_num_t MOTOR4_PIN = GPIO_NUM_25;

static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;
static constexpr bool IS_BIDIRECTIONAL = false;

DShotRMT motor1(MOTOR1_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor2(MOTOR2_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor3(MOTOR3_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor4(MOTOR4_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

static constexpr uint16_t DSHOT_STOP = DSHOT_CMD_MOTOR_STOP;
static constexpr uint16_t DSHOT_MIN_THROTTLE = 48;
static constexpr uint16_t DSHOT_MAX_TEST = 700;

static constexpr int HOVER_STEP = 10;

/*
  Centered throttle stick uses hoverDshot.

  Full throttle-stick movement changes the base by +/-80 DShot units.
*/
static constexpr float THROTTLE_TRIM_DSHOT = 80.0f;

// ================================================================
// ESP-NOW PACKET
// ================================================================

static constexpr uint32_t PACKET_MAGIC = 0x44524F4EUL;

typedef struct ControlData
{
  uint32_t magic;
  uint32_t seq;

  float throttle;
  float pitch;
  float roll;
  float yaw;

  bool motorsEnabled;
  bool kill;
} ControlData;

ControlData receivedData = {};
ControlData controlData = {};

volatile bool packetReceived = false;
volatile uint32_t lastPacketTimeMs = 0;

portMUX_TYPE espNowMux = portMUX_INITIALIZER_UNLOCKED;

static constexpr uint32_t FAILSAFE_TIMEOUT_MS = 300;

// ================================================================
// SERIAL STATE
// ================================================================

bool serialArmed = false;
int hoverDshot = 0;

char serialBuffer[48];
size_t serialLength = 0;

// ================================================================
// MPU6050 CONFIGURATION
// ================================================================

static constexpr uint8_t MPU_ADDR = 0x68;

static constexpr int SDA_PIN = 22;
static constexpr int SCL_PIN = 21;

static constexpr float GYRO_LSB_PER_DPS = 131.0f;

static constexpr bool IMU_UPSIDE_DOWN = true;

/*
  Previous axis mapping:
    aircraft roll  = MPU gyro Y
    aircraft pitch = MPU gyro X
*/
static constexpr float ROLL_GYRO_SIGN = -1.0f;
static constexpr float PITCH_GYRO_SIGN = 1.0f;

static constexpr float ROLL_ACCEL_SIGN = 1.0f;
static constexpr float PITCH_ACCEL_SIGN = 1.0f;

int16_t axRaw = 0;
int16_t ayRaw = 0;
int16_t azRaw = 0;

int16_t gxRaw = 0;
int16_t gyRaw = 0;
int16_t gzRaw = 0;

float gyroXBiasRaw = 0.0f;
float gyroYBiasRaw = 0.0f;
float gyroZBiasRaw = 0.0f;

float rollLevelTrimDeg = 0.0f;
float pitchLevelTrimDeg = 0.0f;

float measuredRollAngleDeg = 0.0f;
float measuredPitchAngleDeg = 0.0f;

float measuredRollRateDps = 0.0f;
float measuredPitchRateDps = 0.0f;

static constexpr float COMPLEMENTARY_GYRO_WEIGHT = 0.985f;

// ================================================================
// ANGLE OUTER LOOP
// ================================================================

static constexpr float MAX_ROLL_ANGLE_DEG = 10.0f;
static constexpr float MAX_PITCH_ANGLE_DEG = 10.0f;

/*
  Hover log showed neutral values around +0.035 to +0.041.

  Values inside +/-0.055 become exactly zero.
*/
static constexpr float ROLL_STICK_DEADBAND = 0.055f;
static constexpr float PITCH_STICK_DEADBAND = 0.055f;

float rollAngleKp = 4.0f;
float pitchAngleKp = 4.0f;

static constexpr float MAX_DESIRED_RATE_DPS = 100.0f;

// ================================================================
// RATE INNER LOOP
// ================================================================

float rollRateKp = 1.20f;
float rollRateKi = 0.00f;
float rollRateKd = 0.008f;

float pitchRateKp = 1.20f;
float pitchRateKi = 0.00f;
float pitchRateKd = 0.008f;

float rollRateIntegral = 0.0f;
float pitchRateIntegral = 0.0f;

float previousRollRateError = 0.0f;
float previousPitchRateError = 0.0f;

float filteredRollDerivative = 0.0f;
float filteredPitchDerivative = 0.0f;

static constexpr float DERIVATIVE_FILTER_ALPHA = 0.20f;
static constexpr float RATE_INTEGRAL_LIMIT = 40.0f;

static constexpr float MAX_AXIS_CORRECTION = 80.0f;

// ================================================================
// SAFETY
// ================================================================

static constexpr float MAX_SAFE_TILT_DEG = 35.0f;
static constexpr uint32_t TILT_KILL_DELAY_MS = 200;

uint32_t excessiveTiltStartMs = 0;

// ================================================================
// CONTROL VARIABLES
// ================================================================

float desiredRollAngleDeg = 0.0f;
float desiredPitchAngleDeg = 0.0f;

float desiredRollRateDps = 0.0f;
float desiredPitchRateDps = 0.0f;

float rollCorrection = 0.0f;
float pitchCorrection = 0.0f;

float correctedRollStick = 0.0f;
float correctedPitchStick = 0.0f;

int baseThrottleDshot = 0;

uint16_t motor1Command = DSHOT_STOP;
uint16_t motor2Command = DSHOT_STOP;
uint16_t motor3Command = DSHOT_STOP;
uint16_t motor4Command = DSHOT_STOP;

// ================================================================
// TIMING
// ================================================================

static constexpr uint32_t CONTROL_FREQUENCY_HZ = 250;

static constexpr uint32_t CONTROL_PERIOD_US =
  1000000UL / CONTROL_FREQUENCY_HZ;

static constexpr uint32_t DEBUG_PERIOD_MS = 100;

uint32_t previousControlUs = 0;
uint32_t previousDebugMs = 0;

// ================================================================
// HELPERS
// ================================================================

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }

  if (value > maximum)
  {
    return maximum;
  }

  return value;
}

int clampInt(int value, int minimum, int maximum)
{
  if (value < minimum)
  {
    return minimum;
  }

  if (value > maximum)
  {
    return maximum;
  }

  return value;
}

/*
  Converts a slightly off-center stick into an exact zero.

  Examples with deadband = 0.055:

    input  0.040 -> output  0.000
    input -0.030 -> output  0.000
    input  0.100 -> output  0.0476
    input  1.000 -> output  1.000
*/
float applyCenteredStickDeadband(
  float input,
  float deadband
)
{
  input = clampFloat(input, -1.0f, 1.0f);

  if (fabsf(input) <= deadband)
  {
    return 0.0f;
  }

  if (input > 0.0f)
  {
    return
      (input - deadband) /
      (1.0f - deadband);
  }

  return
    (input + deadband) /
    (1.0f - deadband);
}

uint16_t clampMotorCommand(int command)
{
  if (command <= 0)
  {
    return DSHOT_STOP;
  }

  command = clampInt(
    command,
    DSHOT_MIN_THROTTLE,
    DSHOT_MAX_TEST
  );

  return static_cast<uint16_t>(command);
}

// ================================================================
// MOTOR FUNCTIONS
// ================================================================

void sendAllMotors(uint16_t command)
{
  motor1.sendThrottle(command);
  motor2.sendThrottle(command);
  motor3.sendThrottle(command);
  motor4.sendThrottle(command);
}

void sendMotorCommands()
{
  motor1.sendThrottle(motor1Command);
  motor2.sendThrottle(motor2Command);
  motor3.sendThrottle(motor3Command);
  motor4.sendThrottle(motor4Command);
}

void resetControllers()
{
  rollRateIntegral = 0.0f;
  pitchRateIntegral = 0.0f;

  previousRollRateError = 0.0f;
  previousPitchRateError = 0.0f;

  filteredRollDerivative = 0.0f;
  filteredPitchDerivative = 0.0f;

  desiredRollRateDps = 0.0f;
  desiredPitchRateDps = 0.0f;

  rollCorrection = 0.0f;
  pitchCorrection = 0.0f;
}

void stopMotors()
{
  resetControllers();

  baseThrottleDshot = 0;

  motor1Command = DSHOT_STOP;
  motor2Command = DSHOT_STOP;
  motor3Command = DSHOT_STOP;
  motor4Command = DSHOT_STOP;
}

void immediateStopFrames()
{
  stopMotors();

  for (int index = 0; index < 30; index++)
  {
    sendAllMotors(DSHOT_STOP);
    delayMicroseconds(500);
  }
}

// ================================================================
// MPU6050 FUNCTIONS
// ================================================================

void writeMPU(uint8_t registerAddress, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
}

bool readMPURaw()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(14)) != 14)
  {
    return false;
  }

  axRaw =
    static_cast<int16_t>(
      (Wire.read() << 8) | Wire.read()
    );

  ayRaw =
    static_cast<int16_t>(
      (Wire.read() << 8) | Wire.read()
    );

  azRaw =
    static_cast<int16_t>(
      (Wire.read() << 8) | Wire.read()
    );

  // Skip temperature
  Wire.read();
  Wire.read();

  gxRaw =
    static_cast<int16_t>(
      (Wire.read() << 8) | Wire.read()
    );

  gyRaw =
    static_cast<int16_t>(
      (Wire.read() << 8) | Wire.read()
    );

  gzRaw =
    static_cast<int16_t>(
      (Wire.read() << 8) | Wire.read()
    );

  return true;
}

bool setupMPU()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  delay(100);

  writeMPU(0x6B, 0x00);
  delay(100);

  writeMPU(0x1B, 0x00);
  writeMPU(0x1C, 0x00);
  writeMPU(0x1A, 0x03);

  delay(100);

  return readMPURaw();
}

// ================================================================
// UPSIDE-DOWN ACCELEROMETER ANGLES
// ================================================================

float calculateAccelRollDeg(float ax, float ay, float az)
{
  const float verticalZ =
    IMU_UPSIDE_DOWN ? -az : az;

  return
    ROLL_ACCEL_SIGN *
    atan2f(-ax, verticalZ) *
    RAD_TO_DEG;
}

float calculateAccelPitchDeg(float ax, float ay, float az)
{
  const float verticalZ =
    IMU_UPSIDE_DOWN ? -az : az;

  return
    PITCH_ACCEL_SIGN *
    atan2f(ay, verticalZ) *
    RAD_TO_DEG;
}

// ================================================================
// IMU CALIBRATION
// ================================================================

bool calibrateIMU()
{
  Serial.println();
  Serial.println("IMU calibration starting.");
  Serial.println("Keep the drone level and completely still.");
  delay(1500);

  constexpr int SAMPLE_COUNT = 1500;

  int64_t gxTotal = 0;
  int64_t gyTotal = 0;
  int64_t gzTotal = 0;

  double rollAccelTotal = 0.0;
  double pitchAccelTotal = 0.0;

  int validSamples = 0;

  for (int sample = 0; sample < SAMPLE_COUNT; sample++)
  {
    if (readMPURaw())
    {
      gxTotal += gxRaw;
      gyTotal += gyRaw;
      gzTotal += gzRaw;

      const float ax = static_cast<float>(axRaw);
      const float ay = static_cast<float>(ayRaw);
      const float az = static_cast<float>(azRaw);

      rollAccelTotal +=
        calculateAccelRollDeg(ax, ay, az);

      pitchAccelTotal +=
        calculateAccelPitchDeg(ax, ay, az);

      validSamples++;
    }

    delay(2);
  }

  if (validSamples < SAMPLE_COUNT / 2)
  {
    Serial.println("IMU calibration failed.");
    return false;
  }

  gyroXBiasRaw =
    static_cast<float>(gxTotal) / validSamples;

  gyroYBiasRaw =
    static_cast<float>(gyTotal) / validSamples;

  gyroZBiasRaw =
    static_cast<float>(gzTotal) / validSamples;

  rollLevelTrimDeg =
    static_cast<float>(
      rollAccelTotal / validSamples
    );

  pitchLevelTrimDeg =
    static_cast<float>(
      pitchAccelTotal / validSamples
    );

  measuredRollAngleDeg = 0.0f;
  measuredPitchAngleDeg = 0.0f;

  Serial.println("IMU calibration complete.");

  Serial.print("gyroXBiasRaw:");
  Serial.println(gyroXBiasRaw, 3);

  Serial.print("gyroYBiasRaw:");
  Serial.println(gyroYBiasRaw, 3);

  Serial.print("gyroZBiasRaw:");
  Serial.println(gyroZBiasRaw, 3);

  Serial.print("rollLevelTrimDeg:");
  Serial.println(rollLevelTrimDeg, 3);

  Serial.print("pitchLevelTrimDeg:");
  Serial.println(pitchLevelTrimDeg, 3);

  Serial.println();

  return true;
}

// ================================================================
// ATTITUDE UPDATE
// ================================================================

bool updateAttitude(float dt)
{
  if (!readMPURaw())
  {
    return false;
  }

  const float gyroX =
    (
      static_cast<float>(gxRaw) -
      gyroXBiasRaw
    ) /
    GYRO_LSB_PER_DPS;

  const float gyroY =
    (
      static_cast<float>(gyRaw) -
      gyroYBiasRaw
    ) /
    GYRO_LSB_PER_DPS;

  measuredRollRateDps =
    ROLL_GYRO_SIGN * gyroY;

  measuredPitchRateDps =
    PITCH_GYRO_SIGN * gyroX;

  const float ax = static_cast<float>(axRaw);
  const float ay = static_cast<float>(ayRaw);
  const float az = static_cast<float>(azRaw);

  float accelerometerRollDeg =
    calculateAccelRollDeg(ax, ay, az);

  float accelerometerPitchDeg =
    calculateAccelPitchDeg(ax, ay, az);

  accelerometerRollDeg -= rollLevelTrimDeg;
  accelerometerPitchDeg -= pitchLevelTrimDeg;

  const float predictedRoll =
    measuredRollAngleDeg +
    measuredRollRateDps * dt;

  const float predictedPitch =
    measuredPitchAngleDeg +
    measuredPitchRateDps * dt;

  measuredRollAngleDeg =
    COMPLEMENTARY_GYRO_WEIGHT * predictedRoll +
    (1.0f - COMPLEMENTARY_GYRO_WEIGHT) *
    accelerometerRollDeg;

  measuredPitchAngleDeg =
    COMPLEMENTARY_GYRO_WEIGHT * predictedPitch +
    (1.0f - COMPLEMENTARY_GYRO_WEIGHT) *
    accelerometerPitchDeg;

  return true;
}

// ================================================================
// RATE PID
// ================================================================

float updateRatePID(
  float desiredRate,
  float measuredRate,
  float &integral,
  float &previousError,
  float &filteredDerivative,
  float kp,
  float ki,
  float kd,
  float dt
)
{
  const float error =
    desiredRate - measuredRate;

  integral += error * dt;

  integral = clampFloat(
    integral,
    -RATE_INTEGRAL_LIMIT,
    RATE_INTEGRAL_LIMIT
  );

  const float rawDerivative =
    (error - previousError) / dt;

  previousError = error;

  filteredDerivative +=
    DERIVATIVE_FILTER_ALPHA *
    (rawDerivative - filteredDerivative);

  return
    kp * error +
    ki * integral +
    kd * filteredDerivative;
}

// ================================================================
// DESIRED COMMANDS
// ================================================================

void calculateDesiredCommands()
{
  /*
    Apply receiver-side neutral correction.

    Neutral input noise or offset inside +/-0.055 becomes exactly zero.
  */
  correctedRollStick =
    applyCenteredStickDeadband(
      controlData.roll,
      ROLL_STICK_DEADBAND
    );

  correctedPitchStick =
    applyCenteredStickDeadband(
      controlData.pitch,
      PITCH_STICK_DEADBAND
    );

  desiredRollAngleDeg =
    correctedRollStick *
    MAX_ROLL_ANGLE_DEG;

  desiredPitchAngleDeg =
    correctedPitchStick *
    MAX_PITCH_ANGLE_DEG;

  /*
    Throttle is left unchanged.
  */
  const float throttleTrim =
    clampFloat(
      controlData.throttle,
      -1.0f,
      1.0f
    ) *
    THROTTLE_TRIM_DSHOT;

  if (hoverDshot <= 0)
  {
    baseThrottleDshot = 0;
    return;
  }

  baseThrottleDshot =
    static_cast<int>(
      roundf(
        static_cast<float>(hoverDshot) +
        throttleTrim
      )
    );

  baseThrottleDshot = clampInt(
    baseThrottleDshot,
    DSHOT_MIN_THROTTLE,
    DSHOT_MAX_TEST
  );
}

// ================================================================
// FLIGHT CONTROLLER
// ================================================================

void updateFlightController(float dt)
{
  calculateDesiredCommands();

  if (baseThrottleDshot <= 0)
  {
    stopMotors();
    return;
  }

  // --------------------------------------------------------------
  // OUTER ANGLE LOOP
  // --------------------------------------------------------------

  const float rollAngleError =
    desiredRollAngleDeg -
    measuredRollAngleDeg;

  const float pitchAngleError =
    desiredPitchAngleDeg -
    measuredPitchAngleDeg;

  desiredRollRateDps =
    clampFloat(
      rollAngleKp * rollAngleError,
      -MAX_DESIRED_RATE_DPS,
      MAX_DESIRED_RATE_DPS
    );

  desiredPitchRateDps =
    clampFloat(
      pitchAngleKp * pitchAngleError,
      -MAX_DESIRED_RATE_DPS,
      MAX_DESIRED_RATE_DPS
    );

  // --------------------------------------------------------------
  // INNER RATE LOOP
  // --------------------------------------------------------------

  rollCorrection =
    updateRatePID(
      desiredRollRateDps,
      measuredRollRateDps,
      rollRateIntegral,
      previousRollRateError,
      filteredRollDerivative,
      rollRateKp,
      rollRateKi,
      rollRateKd,
      dt
    );

  pitchCorrection =
    updateRatePID(
      desiredPitchRateDps,
      measuredPitchRateDps,
      pitchRateIntegral,
      previousPitchRateError,
      filteredPitchDerivative,
      pitchRateKp,
      pitchRateKi,
      pitchRateKd,
      dt
    );

  rollCorrection =
    clampFloat(
      rollCorrection,
      -MAX_AXIS_CORRECTION,
      MAX_AXIS_CORRECTION
    );

  pitchCorrection =
    clampFloat(
      pitchCorrection,
      -MAX_AXIS_CORRECTION,
      MAX_AXIS_CORRECTION
    );

  const int rollMix =
    static_cast<int>(
      roundf(rollCorrection)
    );

  const int pitchMix =
    static_cast<int>(
      roundf(pitchCorrection)
    );

  const int base = baseThrottleDshot;

  /*
    Existing mixer preserved:

           FRONT

       M1          M2

       M4          M3
           REAR
  */

  const int m1 =
    base - pitchMix - rollMix;

  const int m2 =
    base - pitchMix + rollMix;

  const int m3 =
    base + pitchMix + rollMix;

  const int m4 =
    base + pitchMix - rollMix;

  motor1Command = clampMotorCommand(m1);
  motor2Command = clampMotorCommand(m2);
  motor3Command = clampMotorCommand(m3);
  motor4Command = clampMotorCommand(m4);
}

// ================================================================
// TILT SAFETY
// ================================================================

bool excessiveTiltDetected(uint32_t nowMs)
{
  const bool tilted =
    fabsf(measuredRollAngleDeg) >
      MAX_SAFE_TILT_DEG ||
    fabsf(measuredPitchAngleDeg) >
      MAX_SAFE_TILT_DEG;

  if (!tilted)
  {
    excessiveTiltStartMs = 0;
    return false;
  }

  if (excessiveTiltStartMs == 0)
  {
    excessiveTiltStartMs = nowMs;
    return false;
  }

  return
    static_cast<uint32_t>(
      nowMs - excessiveTiltStartMs
    ) >= TILT_KILL_DELAY_MS;
}

// ================================================================
// SERIAL STATUS
// ================================================================

void printStatus()
{
  Serial.print("SERIAL_ARM:");
  Serial.print(serialArmed ? 1 : 0);

  Serial.print(",TX_ENABLE:");
  Serial.print(
    controlData.motorsEnabled ? 1 : 0
  );

  Serial.print(",TX_KILL:");
  Serial.print(controlData.kill ? 1 : 0);

  Serial.print(",HOVER:");
  Serial.print(hoverDshot);

  Serial.print(",BASE:");
  Serial.print(baseThrottleDshot);

  Serial.print(",RAW_ROLL:");
  Serial.print(controlData.roll, 3);

  Serial.print(",CORR_ROLL:");
  Serial.print(correctedRollStick, 3);

  Serial.print(",RAW_PITCH:");
  Serial.print(controlData.pitch, 3);

  Serial.print(",CORR_PITCH:");
  Serial.print(correctedPitchStick, 3);

  Serial.print(",ROLL_ANGLE:");
  Serial.print(measuredRollAngleDeg, 2);

  Serial.print(",PITCH_ANGLE:");
  Serial.print(measuredPitchAngleDeg, 2);

  Serial.print(",ROLL_RATE:");
  Serial.print(measuredRollRateDps, 2);

  Serial.print(",PITCH_RATE:");
  Serial.println(measuredPitchRateDps, 2);
}

// ================================================================
// SERIAL COMMAND PROCESSING
// ================================================================

void processSerialCommand(char *command)
{
  while (
    *command == ' ' ||
    *command == '\t'
  )
  {
    command++;
  }

  size_t length = strlen(command);

  while (
    length > 0 &&
    (
      command[length - 1] == ' ' ||
      command[length - 1] == '\t' ||
      command[length - 1] == '\r'
    )
  )
  {
    command[length - 1] = '\0';
    length--;
  }

  if (length == 0)
  {
    return;
  }

  if (strcasecmp(command, "arm") == 0)
  {
    serialArmed = true;
    resetControllers();

    Serial.println(
      "Serial armed. Transmitter enable is also required."
    );

    return;
  }

  if (
    strcasecmp(command, "kill") == 0 ||
    strcasecmp(command, "disarm") == 0
  )
  {
    serialArmed = false;
    hoverDshot = 0;

    immediateStopFrames();

    Serial.println(
      "KILL: motors stopped and serial disarmed."
    );

    return;
  }

  if (strcasecmp(command, "stop") == 0)
  {
    hoverDshot = 0;
    immediateStopFrames();

    Serial.println(
      "Hover command set to zero."
    );

    return;
  }

  if (strcasecmp(command, "status") == 0)
  {
    printStatus();
    return;
  }

  if (strcasecmp(command, "level") == 0)
  {
    rollLevelTrimDeg +=
      measuredRollAngleDeg;

    pitchLevelTrimDeg +=
      measuredPitchAngleDeg;

    measuredRollAngleDeg = 0.0f;
    measuredPitchAngleDeg = 0.0f;

    resetControllers();

    Serial.println(
      "Current orientation stored as level."
    );

    return;
  }

  if (strcmp(command, "+") == 0)
  {
    if (hoverDshot == 0)
    {
      hoverDshot = DSHOT_MIN_THROTTLE;
    }
    else
    {
      hoverDshot += HOVER_STEP;
    }

    hoverDshot = clampInt(
      hoverDshot,
      DSHOT_MIN_THROTTLE,
      DSHOT_MAX_TEST
    );

    Serial.print("Hover DShot:");
    Serial.println(hoverDshot);

    return;
  }

  if (strcmp(command, "-") == 0)
  {
    hoverDshot -= HOVER_STEP;

    if (hoverDshot < DSHOT_MIN_THROTTLE)
    {
      hoverDshot = 0;
    }

    Serial.print("Hover DShot:");
    Serial.println(hoverDshot);

    return;
  }

  char *endPointer = nullptr;

  const long requestedValue =
    strtol(
      command,
      &endPointer,
      10
    );

  if (
    endPointer != command &&
    *endPointer == '\0'
  )
  {
    if (requestedValue <= 0)
    {
      hoverDshot = 0;
    }
    else
    {
      hoverDshot = clampInt(
        static_cast<int>(requestedValue),
        DSHOT_MIN_THROTTLE,
        DSHOT_MAX_TEST
      );
    }

    resetControllers();

    Serial.print("Hover DShot:");
    Serial.println(hoverDshot);

    return;
  }

  Serial.println(
    "Commands: arm, disarm, number, +, -, stop, kill, level, status"
  );
}

void readSerialCommands()
{
  while (Serial.available() > 0)
  {
    const char character =
      static_cast<char>(Serial.read());

    if (character == '\n')
    {
      serialBuffer[serialLength] = '\0';

      processSerialCommand(
        serialBuffer
      );

      serialLength = 0;
    }
    else if (character != '\r')
    {
      if (
        serialLength <
        sizeof(serialBuffer) - 1
      )
      {
        serialBuffer[serialLength++] =
          character;
      }
      else
      {
        serialLength = 0;

        Serial.println(
          "Serial command too long."
        );
      }
    }
  }
}

// ================================================================
// ESP-NOW CALLBACK
// ================================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int length
)
{
  if (length != sizeof(ControlData))
  {
    return;
  }

  ControlData temporary;

  memcpy(
    &temporary,
    incomingData,
    sizeof(ControlData)
  );

  if (temporary.magic != PACKET_MAGIC)
  {
    return;
  }

  portENTER_CRITICAL(&espNowMux);

  memcpy(
    &receivedData,
    &temporary,
    sizeof(ControlData)
  );

  packetReceived = true;
  lastPacketTimeMs = millis();

  portEXIT_CRITICAL(&espNowMux);
}

// ================================================================
// DEBUG
// ================================================================

void printDebug(
  bool signalLost,
  bool safetyStop,
  bool tiltStop,
  bool imuValid
)
{
  Serial.print("LOST:");
  Serial.print(signalLost ? 1 : 0);

  Serial.print(",IMU_OK:");
  Serial.print(imuValid ? 1 : 0);

  Serial.print(",SAFE_STOP:");
  Serial.print(safetyStop ? 1 : 0);

  Serial.print(",TILT_STOP:");
  Serial.print(tiltStop ? 1 : 0);

  Serial.print(",SER_ARM:");
  Serial.print(serialArmed ? 1 : 0);

  Serial.print(",TX_ARM:");
  Serial.print(
    controlData.motorsEnabled ? 1 : 0
  );

  Serial.print(",TX_KILL:");
  Serial.print(controlData.kill ? 1 : 0);

  Serial.print(",HOVER:");
  Serial.print(hoverDshot);

  Serial.print(",BASE:");
  Serial.print(baseThrottleDshot);

  Serial.print(",RAW_ROLL:");
  Serial.print(controlData.roll, 3);

  Serial.print(",CORR_ROLL:");
  Serial.print(correctedRollStick, 3);

  Serial.print(",RAW_PITCH:");
  Serial.print(controlData.pitch, 3);

  Serial.print(",CORR_PITCH:");
  Serial.print(correctedPitchStick, 3);

  Serial.print(",DES_R_ANG:");
  Serial.print(desiredRollAngleDeg, 2);

  Serial.print(",R_ANG:");
  Serial.print(measuredRollAngleDeg, 2);

  Serial.print(",DES_P_ANG:");
  Serial.print(desiredPitchAngleDeg, 2);

  Serial.print(",P_ANG:");
  Serial.print(measuredPitchAngleDeg, 2);

  Serial.print(",DES_R_RATE:");
  Serial.print(desiredRollRateDps, 2);

  Serial.print(",R_RATE:");
  Serial.print(measuredRollRateDps, 2);

  Serial.print(",DES_P_RATE:");
  Serial.print(desiredPitchRateDps, 2);

  Serial.print(",P_RATE:");
  Serial.print(measuredPitchRateDps, 2);

  Serial.print(",R_CORR:");
  Serial.print(rollCorrection, 2);

  Serial.print(",P_CORR:");
  Serial.print(pitchCorrection, 2);

  Serial.print(",M1:");
  Serial.print(motor1Command);

  Serial.print(",M2:");
  Serial.print(motor2Command);

  Serial.print(",M3:");
  Serial.print(motor3Command);

  Serial.print(",M4:");
  Serial.println(motor4Command);
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(
    "ANGLE + RATE PID + SERIAL HOVER DSHOT"
  );

  Serial.println(
    "NORMAL DSHOT300"
  );

  Serial.println(
    "IMU UPSIDE DOWN MODE ENABLED"
  );

  Serial.println(
    "RECEIVER STICK DEADBAND ENABLED"
  );

  Serial.println(
    "REMOVE PROPELLERS FOR INITIAL TESTING"
  );

  Serial.println();

  Serial.println("Motor pins:");
  Serial.println("M1 = GPIO26");
  Serial.println("M2 = GPIO14");
  Serial.println("M3 = GPIO27");
  Serial.println("M4 = GPIO25");
  Serial.println();

  controlData.magic = PACKET_MAGIC;
  controlData.seq = 0;
  controlData.throttle = 0.0f;
  controlData.pitch = 0.0f;
  controlData.roll = 0.0f;
  controlData.yaw = 0.0f;
  controlData.motorsEnabled = false;
  controlData.kill = true;

  receivedData = controlData;

  if (!setupMPU())
  {
    Serial.println(
      "MPU6050 not detected."
    );

    while (true)
    {
      delay(1000);
    }
  }

  if (!calibrateIMU())
  {
    while (true)
    {
      delay(1000);
    }
  }

  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();

  Serial.println(
    "Sending DShot stop for five seconds."
  );

  const uint32_t stopStartMs = millis();

  while (
    static_cast<uint32_t>(
      millis() - stopStartMs
    ) < 5000
  )
  {
    sendAllMotors(DSHOT_STOP);
    delay(2);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Receiver MAC:");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println(
      "ESP-NOW initialization failed."
    );

    while (true)
    {
      sendAllMotors(DSHOT_STOP);
      delay(2);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  previousControlUs = micros();
  previousDebugMs = millis();

  portENTER_CRITICAL(&espNowMux);
  lastPacketTimeMs = millis();
  packetReceived = false;
  portEXIT_CRITICAL(&espNowMux);

  Serial.println();
  Serial.println("Serial commands:");
  Serial.println("  arm");
  Serial.println("  100");
  Serial.println("  +");
  Serial.println("  -");
  Serial.println("  stop");
  Serial.println("  kill");
  Serial.println("  level");
  Serial.println("  status");
  Serial.println();
}

// ================================================================
// LOOP
// ================================================================

void loop()
{
  readSerialCommands();

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  bool hasPacket = false;
  uint32_t packetTimeCopy = 0;

  portENTER_CRITICAL(&espNowMux);

  memcpy(
    &controlData,
    &receivedData,
    sizeof(ControlData)
  );

  hasPacket = packetReceived;
  packetTimeCopy = lastPacketTimeMs;

  portEXIT_CRITICAL(&espNowMux);

  const bool signalLost =
    !hasPacket ||
    static_cast<uint32_t>(
      nowMs - packetTimeCopy
    ) > FAILSAFE_TIMEOUT_MS;

  if (
    static_cast<uint32_t>(
      nowUs - previousControlUs
    ) >= CONTROL_PERIOD_US
  )
  {
    float dt =
      static_cast<float>(
        nowUs - previousControlUs
      ) /
      1000000.0f;

    previousControlUs = nowUs;

    dt = clampFloat(
      dt,
      0.002f,
      0.010f
    );

    const bool imuValid =
      updateAttitude(dt);

    const bool tiltStop =
      excessiveTiltDetected(nowMs);

    const bool safetyStop =
      signalLost ||
      !imuValid ||
      tiltStop ||
      controlData.kill ||
      !controlData.motorsEnabled ||
      !serialArmed;

    if (safetyStop)
    {
      correctedRollStick = 0.0f;
      correctedPitchStick = 0.0f;

      desiredRollAngleDeg = 0.0f;
      desiredPitchAngleDeg = 0.0f;

      stopMotors();
    }
    else
    {
      updateFlightController(dt);
    }

    sendMotorCommands();

    if (
      static_cast<uint32_t>(
        nowMs - previousDebugMs
      ) >= DEBUG_PERIOD_MS
    )
    {
      previousDebugMs = nowMs;

      printDebug(
        signalLost,
        safetyStop,
        tiltStop,
        imuValid
      );
    }
  }
}