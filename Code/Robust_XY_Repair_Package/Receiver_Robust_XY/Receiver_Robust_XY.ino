/*
  ============================================================================
  FOOTNOTES / READ BEFORE TESTING
  ============================================================================

  1. REMOVE ALL PROPELLERS for the first power-up, sensor-validity test,
     motor-mixer test, and yaw-direction test. Then use a restrained/tethered
     test stand before attempting free flight.

  2. Architecture in this revision:
       IMU at 500 Hz -> 9-error-state ESKF prediction
       PMW3901 SPI acquisition at 100 Hz -> dedicated opposite-core task
       latest PMW sample -> Core-1 body-velocity / ESKF measurement update
       VL53L0X at 100 Hz polled -> asynchronous altitude/vertical-velocity update
       ESKF state -> X/Y/Z outer loops
       preserved angle/rate PID -> final feedback stabilization and mixer
       bidirectional DShot300 task at 500 Hz -> Bluejay mechanical RPM
       RPM bridge -> measured rotor state or smooth command-model fallback
       supplied inverse-dynamics model -> bounded rate and motor feed-forward

     The original MPU6050 signs, angle/rate gains, motor numbering, mixer
     polarities, ESP-NOW packet, ramps, and safety behavior are preserved.

  3. Required Arduino libraries:
       - Bitcraze PMW3901
       - VL53L0X by Pololu
     Bidirectional DShot is implemented by the local BidirectionalDShot module
     and uses the Arduino-ESP32 3.3.10 / ESP-IDF 5.5 RMT API directly.

  4. Coordinate convention used by the navigation controller:
       +X = drone right
       +Y = drone forward
       +Z = upward / increasing range
     The PMW3901 installation was measured as:
       sensor +X = right, sensor +Y = forward.

  5. Stick behavior:
       roll stick     -> desired right/left velocity, +/-0.50 m/s
       pitch stick    -> desired forward/back velocity, +/-0.50 m/s
       throttle stick -> manual throttle below altitude-hold engagement;
                         after engagement, desired vertical velocity,
                         +/-0.30 m/s
       yaw            -> desired yaw rate while commanded; on release, capture
                         the current heading and hold it magnetically.

  6. X/Y hold remains commissioning-gated. Once commissioned, it activates
     above 0.30 m only after the PMW3901 measurement path is healthy for a
     sustained dwell and at least one recent accepted B0 velocity update exists.
     Healthy 0x30 "no new motion" bursts do NOT reset readiness. The current
     location is captured as the target. The target advances while a roll/pitch
     stick is held and stops when the stick is centered.

  7. Altitude hold activates at or above 0.50 m for 300 ms, after the startup
     throttle ramp has completed. It captures the current height and a hover
     throttle estimate limited to DShot 470..490. Altitude correction is
     limited to +/-40 DShot.

  8. Runtime range failure:
       - up to 500 ms: use the last valid measurement/control output;
       - longer than 500 ms: command base DShot 450 for a slow descent;
       - after 5 valid readings return: capture the recovered height as the
         new target instead of climbing back to the old target.

  9. Runtime optical-flow handling:
       - coherent 0x30 bursts are treated as healthy sensor/no-new-motion data,
         not as failed flow samples and not as zero-velocity measurements;
       - only 0xB0 bursts create optical-flow velocity measurements;
       - after 250 ms without a healthy burst, disable X/Y hold;
       - after 25 healthy bursts return, allow the normal stable-dwell arm logic;
       - substantial yaw rate down-weights flow, and high yaw rate rejects the
         velocity update while keeping sensor health separate.
     While X/Y hold is unavailable, roll/pitch sticks fall back to the original
     direct angle command. Centered sticks therefore command level attitude.

 10. The VL53L0X, MPU6050, and QMC5883P share I2C on SDA=22/SCL=21. The
     QMC5883P is sampled at 100 Hz on the Arduino/control core. The PMW3901
     SPI read is isolated in a dedicated task pinned to the opposite core.
     All ESKF mutation remains on the flight-control core.

 11. The mounted QMC5883P supplies an absolute magnetic yaw reference. Its
     mounted calibration is applied in the sensor frame, then the measured
     mounting transform sensor [X,Y,Z] -> body [-X,-Y,+Z] is applied. The
     ESKF quaternion supplies tilt compensation. Magnetic yaw is referenced
     to the heading captured at each navigation reset so local +X/+Y remain
     aligned to the drone at arm. In FULL_HOVER_INTEGRATION mode, magnetic
     yaw fusion starts automatically and yaw-angle hold arms at the end of
     the startup throttle ramp. A stale compass suspends absolute heading
     correction without unlatching the flight-mode request.

 12. The ESKF follows the predict -> asynchronous measurement update ->
     attitude-error injection/reset concept from koledickarlo/ESKF-ESP32. It
     is implemented with fixed-size arrays so this sketch remains a direct
     Arduino IDE build without an Eigen dependency. Optical-flow and ToF
     scale/noise still require validation on the actual airframe.

 13. The Python inverse-dynamics model is used as conservative two-degree-of-
     freedom feed-forward. It does not replace the tuned feedback PID. Bluejay
     bidirectional DShot supplies filtered mechanical RPM to the rotor model.
     Fresh RPM is used as a known model input; stale data falls back smoothly
     to a first-order command-to-speed estimate. The learned local omega/DShot
     scale replaces the prior fixed hover-only conversion.

 14. Sensor offsets supplied/measured for this airframe:
       PMW3901: X=0.00 m, Y=+0.04 m
       VL53L0X: X=0.00 m, Y=+0.07 m
       VL53L0X lens is about 0.03 m above the landing-gear bottom.
     Hand-yaw tests were not used to invent a different PMW lever arm because
     yaw rotates the floor image and the result was not a clean rigid-body
     one-parameter fit. The physical PMW offset is retained. The altitude
     controller intentionally uses raw sensor-to-floor range, as requested;
     it does not subtract the 0.03 m landing-gear offset.

 15. Motor layout and directions, viewed from above:

                         FRONT

                    M1             M2
                   CCW             CW

                    M4             M3
                    CW            CCW

     Existing roll/pitch mixer terms are unchanged. Added yaw terms are:
       M1/M3 += yawMix, M2/M4 -= yawMix.

  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Bitcraze_PMW3901.h>
#include <VL53L0X.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "Eskf9_Robust_XY.h"
#include "MotionModelFF.h"
#include "RotorFeedbackModel.h"
#include "BidirectionalDShot.h"
#include "OpticalFlowAsync_Robust.h"
#include "AsyncDebugTelemetry.h"

// ================================================================
// MOTOR CONFIGURATION - PRESERVED
// ================================================================

static constexpr gpio_num_t MOTOR1_PIN = GPIO_NUM_26;
static constexpr gpio_num_t MOTOR2_PIN = GPIO_NUM_14;
static constexpr gpio_num_t MOTOR3_PIN = GPIO_NUM_27;
static constexpr gpio_num_t MOTOR4_PIN = GPIO_NUM_25;

static constexpr gpio_num_t MOTOR_PINS[4] = {
  MOTOR1_PIN,
  MOTOR2_PIN,
  MOTOR3_PIN,
  MOTOR4_PIN
};

// Fourteen magnetic poles = seven pole pairs for mechanical-RPM conversion.
static constexpr uint8_t MOTOR_POLES[4] = {14, 14, 14, 14};

BidirectionalDShot bidirectionalDshot(MOTOR_PINS, MOTOR_POLES);

static constexpr uint16_t DSHOT_STOP = 0;
static constexpr uint16_t DSHOT_MIN_THROTTLE = 48;
static constexpr uint16_t DSHOT_MAX_TEST = 700;

static constexpr int ENABLE_RAMP_START_DSHOT = 200;
static constexpr int THROTTLE_MIN_DSHOT = 350;
static constexpr int THROTTLE_CENTER_DSHOT = 505;
static constexpr int THROTTLE_MAX_DSHOT = 600;

static constexpr uint32_t ENABLE_RAMP_DURATION_MS = 2000;
static constexpr int DISABLE_RAMP_END_DSHOT = ENABLE_RAMP_START_DSHOT;
static constexpr uint32_t DISABLE_RAMP_DURATION_MS = 2000;
static constexpr float ENABLE_THROTTLE_TOLERANCE = 0.05f;

// ================================================================
// ESP-NOW PACKET - MUST MATCH THE EXISTING TRANSMITTER
// ================================================================

static constexpr uint32_t PACKET_MAGIC = 0x44524F4EUL;

struct ControlData
{
  uint32_t magic;
  uint32_t seq;

  float throttle;
  float pitch;
  float roll;
  float yaw;

  bool motorsEnabled;
  bool kill;
};

ControlData receivedData = {};
ControlData controlData = {};

volatile bool packetReceived = false;
volatile uint32_t lastPacketTimeMs = 0;
portMUX_TYPE espNowMux = portMUX_INITIALIZER_UNLOCKED;

static constexpr uint32_t FAILSAFE_TIMEOUT_MS = 300;

// ================================================================
// SERIAL INPUT / LOCAL EMERGENCY STOP
// ================================================================

bool serialStopLatch = false;
char serialBuffer[AsyncDebugTelemetry::COMMAND_LINE_CAPACITY];

// Runtime UART output is owned exclusively by a low-priority task on the
// processor core opposite Arduino loop(). Setup/calibration prints remain
// synchronous because the flight controller is not running yet.
AsyncDebugTelemetry asyncDebug;
bool asyncDebugStarted = false;

// ================================================================
// I2C / MPU6050 CONFIGURATION - PRESERVED SIGNS
// ================================================================

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t VL53L0X_ADDR = 0x29;
static constexpr uint8_t VL53_RESULT_INTERRUPT_STATUS = 0x13;

static constexpr int SDA_PIN = 22;
static constexpr int SCL_PIN = 21;

// ================================================================
// QMC5883P MAGNETOMETER - FINAL MOUNTED CONFIGURATION
// ================================================================
// Shared I2C bus: SDA=22, SCL=21.
// Mounted compass orientation measured on the airframe:
//   sensor +X = body left     -> body X = -sensor X
//   sensor +Y = body backward -> body Y = -sensor Y
//   sensor +Z = body up       -> body Z = +sensor Z
//
// Mounted-drone calibration from the 2026-08-10 raw data set, validated
// against a separate flat 360-degree yaw sweep. Calibration is applied in
// SENSOR coordinates before the fixed installed-frame transform
// sensor [X,Y,Z] -> body [-X,-Y,+Z].
//
// correctedSensor = MAG_SCALE .* (rawSensor - MAG_BIAS)
//
// Recalibrate if the compass, battery, power wiring, motors, steel hardware,
// or other magnetic parts are moved relative to the sensor.
static constexpr uint8_t MAG_ADDR = 0x2C;
static constexpr uint8_t MAG_CHIP_ID_REG = 0x00;
static constexpr uint8_t MAG_X_LSB_REG = 0x01;
static constexpr uint8_t MAG_STATUS_REG = 0x09;
static constexpr uint8_t MAG_CTRL1_REG = 0x0A;
static constexpr uint8_t MAG_CTRL2_REG = 0x0B;
static constexpr uint8_t MAG_AXIS_SIGN_REG = 0x29;
// QMC5883P: OSR2=8, OSR1=8, ODR=100 Hz, continuous mode.
static constexpr uint8_t MAG_CTRL1_CONFIG = 0xCB;

// ================================================================
// QMC5883P YAW-VALIDATED CALIBRATION
//
// Apply in SENSOR coordinates before the fixed mounting transform:
//
//   sensor +X = body left
//   sensor +Y = body backward
//   sensor +Z = body up
//
// Therefore after calibration:
//   body X = -sensor X
//   body Y = -sensor Y
//   body Z = +sensor Z
//
// Derived from the new full 3D calibration plus the separate
// flat 360-degree yaw verification run.
// ================================================================

static constexpr float MAG_BIAS_X =  48.000f;
static constexpr float MAG_BIAS_Y = -12.500f;
static constexpr float MAG_BIAS_Z = -281.000f;

static constexpr float MAG_SCALE_X = 0.980290f;
static constexpr float MAG_SCALE_Y = 1.020518f;
static constexpr float MAG_SCALE_Z = 1.009972f;

static constexpr uint32_t MAG_POLL_PERIOD_US = 10000; // 100 Hz
static constexpr uint32_t MAG_STALE_MS = 250;
static constexpr float MAG_FILTER_ALPHA = 0.20f;
static constexpr float MAG_YAW_STD_DEG = 10.0f;
static constexpr float MAG_MAX_INNOVATION_DEG = 35.0f;
static constexpr float MAG_FIELD_RATIO_MIN = 0.60f;
static constexpr float MAG_FIELD_RATIO_MAX = 1.40f;
static constexpr float MAG_MIN_HORIZONTAL_COUNTS = 40.0f;

// Robust QMC startup/recovery policy. The compass is treated as a required
// flight sensor: startup does not complete until the device both accepts its
// configuration and produces a real XYZ sample. Runtime recovery is allowed
// only while the motors and ramps are completely stopped.
static constexpr uint32_t MAG_FIRST_SAMPLE_TIMEOUT_MS = 500;
static constexpr uint32_t MAG_STARTUP_RETRY_DELAY_MS = 250;
static constexpr uint32_t MAG_AUTO_RECOVERY_STALE_MS = 500;
static constexpr uint32_t MAG_AUTO_RECOVERY_RETRY_MS = 1000;

bool magSensorInitialized = false;
bool magMeasurementValid = false;
bool magReferenceValid = false;
bool magReferencePending = true;

// Commissioning-safe default. Magnetic yaw fusion and heading hold remain
// OFF at boot so yaw problems cannot contaminate optical-flow mapping or X/Y
// controller tests. After the compass and yaw mixer are independently proven,
// enable them explicitly with the stopped-airframe serial commands.
static constexpr bool FULL_HOVER_INTEGRATION = false;
bool magYawFusionEnabled = FULL_HOVER_INTEGRATION;

// Separate the operator's request from the actively-running heading hold.
// A manual "yawhold on" request now survives the enable edge and startup ramp.
// The active hold is deliberately suspended during the ramp, then armed as
// soon as fresh accepted magnetic yaw is available after ramp completion.
bool yawHeadingHoldRequested = FULL_HOVER_INTEGRATION;
bool yawHeadingHoldEnabled = false;

float magBodyX = 0.0f;
float magBodyY = 0.0f;
float magBodyZ = 0.0f;
float magFilteredBodyX = 0.0f;
float magFilteredBodyY = 0.0f;
float magFilteredBodyZ = 0.0f;
bool magFilterInitialized = false;

float magAbsoluteYawRhDeg = 0.0f;   // +CCW, magnetic-north referenced
float magRelativeYawRhDeg = 0.0f;   // +CCW, local heading since nav reset
float magRelativeYawCwDeg = 0.0f;   // +CW, controller convention
float magReferenceYawRhDeg = 0.0f;
float magFieldMagnitude = 0.0f;
float magReferenceFieldMagnitude = 0.0f;
float magLastInnovationDeg = 0.0f;
uint32_t lastMagSampleMs = 0;
uint32_t lastMagAcceptedMs = 0;
uint32_t magAcceptedUpdates = 0;
uint32_t magRejectedUpdates = 0;

// QMC5883P runtime diagnostics. These distinguish a bus/register failure,
// DRDY staying low, and a failed six-byte XYZ burst read.
enum class MagReadResult : uint8_t
{
  Good = 0,
  StatusReadFailed,
  NotReady,
  DataReadFailed
};

uint32_t magPollCount = 0;
uint32_t magStatusReadFailCount = 0;
uint32_t magNotReadyCount = 0;
uint32_t magDataReadFailCount = 0;
uint32_t magGoodRawCount = 0;
uint8_t magLastStatus = 0;
bool magLastStatusValid = false;
uint8_t magLastCtrl1 = 0;
bool magLastCtrl1Valid = false;
uint32_t lastMagAutoRecoveryAttemptMs = 0;
bool magStaleInFlightLatched = false;

static constexpr float GYRO_LSB_PER_DPS = 131.0f;
static constexpr bool IMU_UPSIDE_DOWN = true;

/*
  Existing tuned axis mapping; do not change without revalidating the
  original attitude controller.
*/
static constexpr float ROLL_GYRO_SIGN = -1.0f;
static constexpr float PITCH_GYRO_SIGN = +1.0f;
static constexpr float YAW_GYRO_SIGN = -1.0f;  // supplied: gzRaw is + clockwise

static constexpr float ROLL_ACCEL_SIGN = +1.0f;
static constexpr float PITCH_ACCEL_SIGN = -1.0f;

/*
  Explicit bridge from the already-tested controller convention to the
  right-handed body frame used inside the ESKF:

    body +X = right, +Y = forward, +Z = up
    controller pitch = -RH rotation about body +X
    controller roll  = -RH rotation about body +Y
    controller yaw   = clockwise = -RH rotation about body +Z

  The accelerometer signs follow directly from the existing angle formulas:
    roll  = +atan2(-ax, -az)
    pitch = -atan2(+ay, -az)
*/
static constexpr float ESKF_ACCEL_X_FROM_RAW_SIGN = -1.0f;
static constexpr float ESKF_ACCEL_Y_FROM_RAW_SIGN = +1.0f;
static constexpr float ESKF_ACCEL_Z_FROM_RAW_SIGN = -1.0f;

static constexpr float CONTROLLER_PITCH_TO_RH_SIGN = -1.0f;
static constexpr float CONTROLLER_ROLL_TO_RH_SIGN = -1.0f;
static constexpr float CONTROLLER_YAW_CW_TO_RH_SIGN = 1.0f;

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
float measuredYawRateDps = 0.0f;      // + clockwise, per supplied test
float yawAngleCwDeg = 0.0f;           // dead-reckoned local-frame heading

static constexpr float COMPLEMENTARY_GYRO_WEIGHT = 0.9925f;  // ~250 ms TC at 500 Hz

// MPU6050 is configured for +/-2 g. These signs map the installed sensor to
// the navigation convention: +X right and +Y forward.
static constexpr float ACCEL_LSB_PER_G = 16384.0f;
static constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
static constexpr float IMU_ACCEL_FILTER_ALPHA = 0.22f;
static constexpr float IMU_ACCEL_DEADBAND_MPS2 = 0.18f;
static constexpr float IMU_ACCEL_LIMIT_MPS2 = 8.0f;

float accelXBiasRaw = 0.0f;
float accelYBiasRaw = 0.0f;
// Bias of the already-mapped body-Z signal (-azRaw), excluding the expected
// +1 g at rest.  This lets the ESKF receive a calibrated full 3-axis specific
// force vector while preserving the existing X/Y bias convention.
float accelZBodyBiasRaw = 0.0f;

float bodySpecificForceX_mps2 = 0.0f;
float bodySpecificForceY_mps2 = 0.0f;
float bodySpecificForceZ_mps2 = STANDARD_GRAVITY_MPS2;

float complementaryRollAngleDeg = 0.0f;
float complementaryPitchAngleDeg = 0.0f;

float linearAccelBodyX_mps2 = 0.0f;
float linearAccelBodyY_mps2 = 0.0f;
float filteredLinearAccelBodyX_mps2 = 0.0f;
float filteredLinearAccelBodyY_mps2 = 0.0f;
float fusedBodyVelocityX_mps = 0.0f;
float fusedBodyVelocityY_mps = 0.0f;
bool fusedVelocityInitialized = false;

// ================================================================
// ESKF STATE ESTIMATION
// ================================================================

// Fixed-array 9-error-state estimator.  The nominal state is local position,
// local velocity, and a body-to-navigation quaternion.  Optical flow and ToF
// arrive asynchronously and perform scalar/low-dimensional Kalman updates.
Eskf9 stateEstimator;

static constexpr float ESKF_TOF_STD_M = 0.025f;              // tighter range trust
static constexpr float ESKF_RANGE_VELOCITY_STD_MPS = 0.15f;
static constexpr float ESKF_FLOW_VELOCITY_STD_MPS = 0.08f;    // tighter flow trust
static constexpr float ESKF_GRAVITY_STD_ON_GROUND = 0.035f;
static constexpr float ESKF_GRAVITY_STD_IN_FLIGHT = 0.10f;
static constexpr float ESKF_MAX_ATTITUDE_DISAGREEMENT_DEG = 15.0f;
static constexpr uint32_t ESKF_ZERO_VELOCITY_PERIOD_MS = 40;

bool eskfAttitudeUsed = false;
// Commissioning interlock: the ESKF always runs and supplies navigation
// states, but it does not replace the already-flown complementary attitude
// estimate until its signs are verified propeller-off and explicitly enabled.
bool eskfAttitudeControlEnabled = false;
bool eskfStateHealthy = true;
bool eskfFlowVelocityReady = false;
bool eskfRangeReady = false;
uint32_t lastEskfRangeAcceptedMs = 0;
uint32_t previousEskfZeroVelocityMs = 0;
float eskfRollAngleDeg = 0.0f;
float eskfPitchAngleDeg = 0.0f;
float eskfYawCwDeg = 0.0f;
float rangeDerivedVerticalVelocityMps = 0.0f;

// ================================================================
// PYTHON MOTION-MODEL FEED-FORWARD
// ================================================================

MotionModelFeedforward motionModelFeedforward;
MotionModelFeedforward::Output motionModelOutput;

// Commissioning interlock: keep the new inverse-dynamics feed-forward at zero
// on first boot.  Enable it with "model on" after the motor-pattern test.
bool motionModelEnabled = false;
float motionModelMotorBlend = 0.12f;
float motionModelRateBlend = 0.20f;
float appliedModelCollectiveDshot = 0.0f;
float appliedModelMotorDeltaDshot[4] = {};

// ================================================================
// BLUEJAY RPM FEEDBACK / ROTOR MODEL INPUT
// ================================================================

BidirectionalDShot::RpmSnapshot rpmSnapshot;
RotorFeedbackModel rotorFeedbackModel;
RotorFeedbackModel::State rotorModelState;
MotionModelFeedforward::RotorFeedbackInput motionModelRotorInput;

bool rpmTelemetryHealthy = false;
uint8_t rpmFreshMask = 0;
uint32_t lastRpmSnapshotSequence = 0;
float eskfActuatorProcessNoiseScale = 1.0f;

// ================================================================
// PMW3901 CONFIGURATION
// ================================================================

static constexpr uint8_t FLOW_SCK_PIN = 18;
static constexpr uint8_t FLOW_MISO_PIN = 19;
static constexpr uint8_t FLOW_MOSI_PIN = 23;
static constexpr uint8_t FLOW_CS_PIN = 13;
static constexpr uint8_t FLOW_RST_PIN = 33;

static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
static constexpr float RAD_TO_DEG_F = 180.0f / PI_F;

// PX4's PMW3901 driver uses 1/385 rad/count. The bench diagnostic can be
// used to calibrate this value for the actual lens, height, and floor.
static constexpr float FLOW_RAD_PER_COUNT = 1.0f / 385.0f;
static constexpr float FLOW_SCALE_X = 1.0f;
static constexpr float FLOW_SCALE_Y = 1.0f;
static constexpr float FLOW_VELOCITY_FILTER_ALPHA = 0.25f;
static constexpr float FLOW_INVALID_DECAY = 0.85f;
static constexpr uint32_t FLOW_SPI_HZ = 2000000UL;

static constexpr float MIN_FLOW_RANGE_M = 0.100f;
static constexpr float MAX_FLOW_RANGE_M = 2.000f;
static constexpr int32_t FLOW_OUTLIER_LIMIT_COUNTS = 100;
static constexpr float MAX_PLAUSIBLE_FLOW_SPEED_MPS = 3.0f;
static constexpr uint8_t MIN_FLOW_SQUAL = 1;

static constexpr float PMW_OFFSET_X_M = 0.00f;
static constexpr float PMW_OFFSET_Y_M = 0.04f;

/*
  FLOW AXIS MAPPING - COMMISSIONED ON THIS AIRFRAME.

  Full-controller translation testing established:
    physical right    -> +rawX -> +body X
    physical left     -> -rawX -> -body X
    physical forward  -> -rawY -> +body Y
    physical backward -> +rawY -> -body Y

  Therefore MAP1 / PlusXMinusY is the flight default:
    bodyXCounts = +rawX
    bodyYCounts = -rawY

  The runtime flowmap command remains available only for stopped-airframe
  diagnostics and always forces XY commissioning back OFF after a change.
*/
enum class FlowAxisMap : uint8_t
{
  PlusXPlusY = 0,
  PlusXMinusY = 1,
  MinusXPlusY = 2,
  MinusXMinusY = 3,
  PlusYPlusX = 4,
  PlusYMinusX = 5,
  MinusYPlusX = 6,
  MinusYMinusX = 7
};

FlowAxisMap flowAxisMap = FlowAxisMap::PlusXMinusY;

/*
  Rotational-flow compensation.

  The original MAP0 bench calibration established magnitudes:
    roll  = 1.34
    pitch = 1.65

  Final flight mapping is MAP1, which reverses the mapped Y flow axis.
  Therefore the flight equations are:

    deltaX += -1.34 * height * rollDelta
    deltaY += +1.65 * height * pitchDelta
*/
static constexpr bool ENABLE_FLOW_GYRO_COMPENSATION = true;
static constexpr float FLOW_ROLL_COMP_GAIN = -1.34f;
static constexpr float FLOW_PITCH_COMP_GAIN = 1.65f;

// Yaw is not treated as a clean lever-arm-only effect. Preserve the physically
// measured PMW offset, but reduce estimator trust as yaw rate rises and reject
// optical-flow velocity updates entirely at substantial yaw rates.
static constexpr float FLOW_YAW_RATE_SOFT_DPS = 5.0f;
static constexpr float FLOW_YAW_RATE_HARD_DPS = 15.0f;
static constexpr float FLOW_YAW_NOISE_SCALE_MAX = 4.0f;

Bitcraze_PMW3901 flow(FLOW_CS_PIN);

uint8_t lastFlowMotion = 0;
uint8_t lastFlowSqual = 0;
uint16_t lastFlowShutter = 0;
uint8_t lastFlowAxisAcceptedMask = 0;
float lastInstantFlowVelocityX_mps = 0.0f;
float lastInstantFlowVelocityY_mps = 0.0f;
uint32_t flowInvalidTimingCount = 0;
uint32_t flowInvalidCountCount = 0;
uint32_t flowInvalidHeightCount = 0;
uint32_t flowInvalidQualityCount = 0;
uint32_t flowInvalidSpeedCount = 0;
uint32_t flowEstimatorRejectCount = 0;

// ================================================================
// VL53L0X CONFIGURATION
// ================================================================

static constexpr float RANGE_OFFSET_X_M = 0.00f;
static constexpr float RANGE_OFFSET_Y_M = 0.07f;
static constexpr float RANGE_SENSOR_ABOVE_GEAR_M = 0.03f;

static constexpr float MIN_VALID_RANGE_M = 0.030f;
static constexpr float MAX_VALID_RANGE_M = 2.000f;
static constexpr uint32_t RANGE_TIMING_BUDGET_US = 20000;
static constexpr uint32_t RANGE_CONTROL_FREEZE_MS = 100;
static constexpr uint32_t RANGE_LONG_FAILURE_MS = 500;
static constexpr uint8_t RANGE_RECOVERY_SAMPLES = 5;
static constexpr int RANGE_FAILURE_DESCENT_DSHOT = 450;

VL53L0X rangeSensor;

// ================================================================
// PILOT COMMAND LIMITS
// ================================================================

static constexpr float MAX_RIGHT_VELOCITY_MPS = 0.50f;
static constexpr float MAX_FORWARD_VELOCITY_MPS = 0.50f;
static constexpr float MAX_VERTICAL_VELOCITY_MPS = 0.30f;
static constexpr float MAX_YAW_RATE_DPS = 60.0f;

// Existing direct-angle fallback limits are preserved.
static constexpr float MAX_ROLL_ANGLE_DEG = 10.0f;
static constexpr float MAX_PITCH_ANGLE_DEG = 10.0f;

static constexpr float ROLL_STICK_DEADBAND = 0.055f;
static constexpr float PITCH_STICK_DEADBAND = 0.055f;
static constexpr float THROTTLE_STICK_DEADBAND = 0.050f;
static constexpr float YAW_STICK_DEADBAND = 0.050f;

// ================================================================
// X/Y POSITION AND VELOCITY CONTROLLER
// ================================================================

static constexpr float XY_HOLD_ACTIVATION_RANGE_M = 0.30f;
static constexpr uint8_t FLOW_HEALTH_RECOVERY_SAMPLES = 25; // 0.25 s at 100 Hz
static constexpr uint32_t FLOW_HEALTH_LOSS_MS = 250;
static constexpr uint32_t FLOW_ARM_MEASUREMENT_FRESH_MS = 500;
static constexpr uint32_t XY_HOLD_BLEND_MS = 400;
static constexpr uint32_t XY_ARM_STABLE_DWELL_MS = 1000;
static constexpr float MAX_POSITION_TARGET_ERROR_M = 0.50f;
static constexpr float XY_POSITION_DEADBAND_M = 0.02f;
static constexpr float MAX_XY_CONTROLLER_ANGLE_DEG = 5.0f;
static constexpr float MAX_CONTROLLER_VELOCITY_MPS = 0.35f;
static constexpr float MAX_XY_ACCELERATION_MPS2 = 0.80f;
static constexpr float MAX_XY_ACCEL_ASSIST_ANGLE_DEG = 2.0f;
static constexpr float XY_COMMAND_FILTER_TAU_S = 0.12f;
static constexpr float XY_ANGLE_SLEW_RATE_DPS = 30.0f;

static constexpr float XY_ARM_MAX_VERTICAL_SPEED_MPS = 0.12f;
static constexpr float XY_ARM_MAX_HORIZONTAL_SPEED_MPS = 0.20f;
static constexpr float XY_ARM_MAX_YAW_RATE_DPS = 8.0f;
static constexpr float XY_ARM_MAX_TILT_DEG = 6.0f;
static constexpr float XY_ABORT_MAX_VERTICAL_SPEED_MPS = 0.40f;
static constexpr float XY_ABORT_MAX_YAW_RATE_DPS = 25.0f;
static constexpr float XY_ABORT_MAX_TILT_DEG = 15.0f;

// Safety interlock. Leave false until the bench diagnostic proves the PMW
// axis map, scale, quality, and gyro-compensation signs. It can then be
// enabled while stopped with the serial command: xycommission on
bool xyHoldCommissioned = false;
uint32_t xyArmStableStartMs = 0;

// Cascaded controller gains in physical units.
//   position error [m] * xyPositionKp [1/s] -> velocity target [m/s]
//   velocity error [m/s] * xyVelocityKp [1/s] -> acceleration target [m/s^2]
float xyPositionKp = 0.60f;
float xyVelocityKp = 1.50f;
float xyAccelerationKp = 0.00f; // optional IMU assist; keep zero initially

float filteredHoldRollAngleDeg = 0.0f;
float filteredHoldPitchAngleDeg = 0.0f;

float positionNavX_m = 0.0f;
float positionNavY_m = 0.0f;
float targetNavX_m = 0.0f;
float targetNavY_m = 0.0f;

float measuredBodyVelocityX_mps = 0.0f;
float measuredBodyVelocityY_mps = 0.0f;
float measuredNavVelocityX_mps = 0.0f;
float measuredNavVelocityY_mps = 0.0f;

float commandedBodyVelocityX_mps = 0.0f;
float commandedBodyVelocityY_mps = 0.0f;
float controllerBodyVelocityTargetX_mps = 0.0f;
float controllerBodyVelocityTargetY_mps = 0.0f;

bool xyHoldActive = false;
uint32_t xyHoldBlendStartMs = 0;

// Sensor health is intentionally separate from motion-measurement acceptance.
// A coherent 0x30 burst keeps the PMW path healthy but does not inject a
// zero-velocity measurement into the ESKF.
uint8_t consecutiveHealthyFlowSamples = 0;
bool flowSensorHealthy = false;
uint32_t lastHealthyFlowMs = 0;

// Accepted B0 measurement bookkeeping.
uint8_t consecutiveValidFlowSamples = 0;
uint32_t lastValidFlowMs = 0;      // compatibility/debug: last accepted B0
uint32_t lastFlowMeasurementMs = 0;
bool lastFlowSampleValid = false;  // true only for the most recent accepted B0

uint32_t flowNoMotionCount = 0;
uint32_t flowYawRateRejectCount = 0;
uint32_t flowYawRateDownweightCount = 0;
float lastFlowYawRateDps = 0.0f;
float lastFlowYawNoiseScale = 1.0f;

// Gyro angle accumulated over the same interval as PMW motion counts.
float accumulatedFlowRollRad = 0.0f;
float accumulatedFlowPitchRad = 0.0f;
float accumulatedFlowYawCwRad = 0.0f;

// ================================================================
// Z / ALTITUDE CONTROLLER
// ================================================================

static constexpr float ALTITUDE_HOLD_ACTIVATION_M = 0.50f;
static constexpr uint32_t ALTITUDE_ACTIVATION_DWELL_MS = 300;
static constexpr uint32_t ALTITUDE_HOLD_BLEND_MS = 500;

// Nominal requested operating ceiling is 1.0 m. A lower landing target is
// permitted after hold has engaged so the pilot can command a controlled descent.
static constexpr float ALTITUDE_TARGET_MIN_M = 0.12f;
static constexpr float ALTITUDE_TARGET_MAX_M = 1.00f;
static constexpr float MAX_ALTITUDE_VELOCITY_TARGET_MPS = 0.40f;

static constexpr int HOVER_DSHOT_MIN = 470;
static constexpr int HOVER_DSHOT_MAX = 490;
static constexpr float MAX_ALTITUDE_CORRECTION_DSHOT = 40.0f;
static constexpr float ALTITUDE_INTEGRAL_LIMIT = 1.00f;

// Conservative starting gains; tune after validating range and velocity signs.
float altitudePositionKp = 2.00f;  // (m/s) per meter
float altitudeVelocityKp = 75.0f;  // DShot per (m/s)
float altitudeVelocityKi = 25.0f;  // DShot per integrated velocity error

float rangeMeters = -1.0f;
float filteredRangeMeters = -1.0f;
float verticalVelocityMps = 0.0f;
float altitudeTargetM = 0.0f;
float desiredVerticalVelocityMps = 0.0f;
float altitudeVelocityIntegral = 0.0f;
float altitudeCorrectionDshot = 0.0f;

bool altitudeHoldActive = false;
uint32_t altitudeActivationStartMs = 0;
uint32_t altitudeBlendStartMs = 0;
int altitudeBlendStartDshot = THROTTLE_CENTER_DSHOT;
int learnedHoverDshot = THROTTLE_CENTER_DSHOT;
int lastAltitudeBaseDshot = THROTTLE_CENTER_DSHOT;

bool rangeHasEverBeenValid = false;
bool rangeRecoveryStable = false;
bool rangeJustRecovered = false;
bool rangeLongFailureLatched = false;
uint8_t consecutiveValidRangeSamples = 0;
uint32_t lastValidRangeMs = 0;
uint32_t previousValidRangeUs = 0;
float previousFilteredRangeM = -1.0f;

// VL53L0X runtime diagnostics. These let us distinguish a shared-I2C problem
// from the sensor simply not asserting data-ready or returning bad ranges.
uint32_t rangePollCount = 0;
uint32_t rangeStatusReadFailCount = 0;
uint32_t rangeNotReadyCount = 0;
uint32_t rangeReadFailCount = 0;
uint32_t rangeInvalidValueCount = 0;
uint32_t rangeGoodCount = 0;
uint8_t rangeLastInterruptStatus = 0;
bool rangeLastInterruptStatusValid = false;

// ================================================================
// EXISTING ANGLE OUTER LOOP - PRESERVED
// ================================================================

float rollAngleKp = 4.0f;
float pitchAngleKp = 4.0f;
static constexpr float MAX_DESIRED_RATE_DPS = 100.0f;

// ================================================================
// EXISTING ROLL/PITCH RATE INNER LOOPS - PRESERVED
// ================================================================

float rollRateKp = 1.20f;
float rollRateKi = 0.10f;
float rollRateKd = 0.004f;

float pitchRateKp = 1.20f;
float pitchRateKi = 0.10f;
float pitchRateKd = 0.004f;

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
// YAW RATE DAMPING
// ================================================================

float yawRateKp = 0.15f;
float yawRateKi = 0.00f;
float yawRateKd = 0.000f;

float yawRateIntegral = 0.0f;
float previousYawRateError = 0.0f;
float filteredYawDerivative = 0.0f;

static constexpr float YAW_INTEGRAL_LIMIT = 10.0f;
static constexpr float MAX_YAW_CORRECTION = 12.0f;

// Absolute/local heading outer loop. This creates a desired yaw RATE for the
// already-tested inner yaw-rate controller; it does not bypass that loop.
float yawHeadingKp = 1.0f;  // (deg/s) per degree heading error
static constexpr float MAX_YAW_HOLD_RATE_DPS = 35.0f;
float yawHeadingTargetCwDeg = 0.0f;
bool yawStickWasActive = false;

// ================================================================
// SAFETY
// ================================================================

static constexpr float MAX_SAFE_TILT_DEG = 35.0f;
static constexpr uint32_t TILT_KILL_DELAY_MS = 200;
uint32_t excessiveTiltStartMs = 0;

// ================================================================
// CONTROL VARIABLES
// ================================================================

float correctedRollStick = 0.0f;
float correctedPitchStick = 0.0f;
float correctedThrottleStick = 0.0f;
float correctedYawStick = 0.0f;

float desiredRollAngleDeg = 0.0f;
float desiredPitchAngleDeg = 0.0f;
float desiredRollRateDps = 0.0f;
float desiredPitchRateDps = 0.0f;
float desiredYawRateDps = 0.0f;

float rollCorrection = 0.0f;
float pitchCorrection = 0.0f;
float yawCorrection = 0.0f;

float rollPTerm = 0.0f;
float rollITerm = 0.0f;
float rollDTerm = 0.0f;
float pitchPTerm = 0.0f;
float pitchITerm = 0.0f;
float pitchDTerm = 0.0f;
float yawPTerm = 0.0f;
float yawITerm = 0.0f;
float yawDTerm = 0.0f;

int throttleTargetDshot = THROTTLE_CENTER_DSHOT;
int baseThrottleDshot = 0;

bool flightEnabled = false;
bool previousRemoteEnable = false;
uint32_t flightEnableMs = 0;

bool enableRampActive = false;
uint32_t enableRampStartMs = 0;
float enableRampProgress = 0.0f;

bool disableRampActive = false;
uint32_t disableRampStartMs = 0;
int disableRampStartDshot = THROTTLE_CENTER_DSHOT;
float disableRampProgress = 0.0f;

uint16_t motor1Command = DSHOT_STOP;
uint16_t motor2Command = DSHOT_STOP;
uint16_t motor3Command = DSHOT_STOP;
uint16_t motor4Command = DSHOT_STOP;

// ================================================================
// TIMING
// ================================================================

static constexpr uint32_t CONTROL_FREQUENCY_HZ = 200;
static constexpr uint32_t CONTROL_PERIOD_US =
  1000000UL / CONTROL_FREQUENCY_HZ;

static constexpr uint32_t FLOW_PERIOD_US = 10000;
static constexpr uint32_t RANGE_POLL_PERIOD_US = 20000;
static constexpr uint32_t DEBUG_PERIOD_MS = 1000;

uint32_t previousControlUs = 0;
uint32_t previousSetupFlowDrainUs = 0;
uint32_t previousRangePollUs = 0;
uint32_t previousMagPollUs = 0;
uint32_t previousDebugMs = 0;

uint32_t controlOverrunCount = 0;
uint32_t maximumObservedControlDtUs = 0;
uint32_t minimumObservedControlDtUs = UINT32_MAX;

// ================================================================
// NONBLOCKING EXECUTION-TIME PROFILER
// ================================================================
// Each probe records only integer timestamps in RAM. No Serial output occurs
// on the flight-control core. The 1 Hz async debug snapshot copies the
// accumulated averages/maxima to the UART task on the other core.
struct TimingAccumulator
{
  uint32_t maxUs = 0;
  uint64_t totalUs = 0;
  uint32_t samples = 0;
};

inline void recordTiming(TimingAccumulator &timing, uint32_t durationUs)
{
  if (durationUs > timing.maxUs) timing.maxUs = durationUs;
  timing.totalUs += durationUs;
  timing.samples++;
}

inline uint32_t averageTimingUs(const TimingAccumulator &timing)
{
  if (timing.samples == 0) return 0;
  return static_cast<uint32_t>(timing.totalUs / timing.samples);
}

inline void clearTiming(TimingAccumulator &timing)
{
  timing.maxUs = 0;
  timing.totalUs = 0;
  timing.samples = 0;
}

TimingAccumulator timingCommandProcessing;
TimingAccumulator timingEspNowCopy;
TimingAccumulator timingControlBlock;
TimingAccumulator timingRotorFeedback;
TimingAccumulator timingAttitude;
TimingAccumulator timingImuRead;
TimingAccumulator timingEskf;
TimingAccumulator timingEnableState;
TimingAccumulator timingFlightController;
TimingAccumulator timingMotorPublish;
TimingAccumulator timingRangePoll;
TimingAccumulator timingFlowUpdate;
TimingAccumulator timingLoopActive;
TimingAccumulator timingAsyncPublish;

// ESP-NOW callback runs asynchronously, so these are read/reset while holding
// espNowMux. Packet interval is the actual valid control-packet arrival period.
TimingAccumulator timingEspNowCallback;
uint32_t previousEspNowRxUs = 0;
uint32_t lastEspNowIntervalUs = 0;
uint32_t minEspNowIntervalUs = UINT32_MAX;
uint32_t maxEspNowIntervalUs = 0;

void resetMainTimingWindow()
{
  clearTiming(timingCommandProcessing);
  clearTiming(timingEspNowCopy);
  clearTiming(timingControlBlock);
  clearTiming(timingRotorFeedback);
  clearTiming(timingAttitude);
  clearTiming(timingImuRead);
  clearTiming(timingEskf);
  clearTiming(timingEnableState);
  clearTiming(timingFlightController);
  clearTiming(timingMotorPublish);
  clearTiming(timingRangePoll);
  clearTiming(timingFlowUpdate);
  clearTiming(timingLoopActive);
  clearTiming(timingAsyncPublish);
}

bool lastImuValidState = false;
bool lastTiltStopState = false;
bool lastSafetyStopState = true;

// ================================================================
// OPPOSITE-CORE PMW3901 ACQUISITION
// ================================================================
// Only the SPI transaction runs on the opposite core. The flight-control core
// remains the sole owner of the ESKF and navigation/controller state. This
// avoids cross-core estimator races while removing the long PMW3901 SPI read
// from the 500 Hz Arduino loop.
QueueHandle_t opticalFlowSampleQueue = nullptr;
TaskHandle_t opticalFlowTaskHandle = nullptr;
BaseType_t opticalFlowTaskCore = -1;
bool opticalFlowTaskStarted = false;
uint32_t lastProcessedFlowSequence = 0;
uint32_t skippedFlowSamplesWindow = 0;

// Gyro angle integrated on the 500 Hz control core and atomically handed to
// the PMW task at each sensor read. The old implementation reset these angles
// immediately after flow.readMotionCount(); the same ordering is preserved.
portMUX_TYPE flowGyroMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE flowTaskStatsMux = portMUX_INITIALIZER_UNLOCKED;
OpticalFlowTaskStats opticalFlowTaskStats;

// ================================================================
// ASYNCHRONOUS DEBUG HELPERS
// ================================================================

void runtimeLogLine(const char *line)
{
  if (line == nullptr) return;

  if (asyncDebugStarted)
  {
    // Never wait for the UART. A full queue drops this noncritical line.
    asyncDebug.postText(line);
  }
  else
  {
    Serial.println(line);
  }
}

void runtimeLogf(const char *format, ...)
{
  if (format == nullptr) return;

  char line[AsyncDebugTelemetry::TEXT_LINE_CAPACITY];
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(line, sizeof(line), format, arguments);
  va_end(arguments);
  line[sizeof(line) - 1] = '\0';
  runtimeLogLine(line);
}

void postFlightEvent(
  AsyncFlightEventType type,
  uint32_t nowMs,
  float value0 = 0.0f,
  float value1 = 0.0f,
  int32_t integer0 = 0,
  int32_t integer1 = 0,
  uint32_t unsigned0 = 0
)
{
  if (!asyncDebugStarted) return;

  AsyncFlightEvent event;
  event.type = type;
  event.timestampMs = nowMs;
  event.value0 = value0;
  event.value1 = value1;
  event.integer0 = integer0;
  event.integer1 = integer1;
  event.unsigned0 = unsigned0;
  asyncDebug.postEvent(event);
}

// ================================================================
// GENERAL HELPERS
// ================================================================

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

int clampInt(int value, int minimum, int maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float applySymmetricDeadband(float value, float deadband)
{
  if (fabsf(value) <= deadband)
  {
    return 0.0f;
  }

  return value > 0.0f ? value - deadband : value + deadband;
}

float wrapAngle180(float angleDeg)
{
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

float applyCenteredStickDeadband(float input, float deadband)
{
  input = clampFloat(input, -1.0f, 1.0f);

  if (fabsf(input) <= deadband)
  {
    return 0.0f;
  }

  if (input > 0.0f)
  {
    return (input - deadband) / (1.0f - deadband);
  }

  return (input + deadband) / (1.0f - deadband);
}

void limitVector2(float &x, float &y, float maximumMagnitude)
{
  const float magnitudeSquared = x * x + y * y;
  const float maximumSquared = maximumMagnitude * maximumMagnitude;

  if (magnitudeSquared > maximumSquared && magnitudeSquared > 0.0f)
  {
    const float scale = maximumMagnitude / sqrtf(magnitudeSquared);
    x *= scale;
    y *= scale;
  }
}

bool throttleCenteredForEnable(float throttle)
{
  return fabsf(clampFloat(throttle, -1.0f, 1.0f)) <=
    ENABLE_THROTTLE_TOLERANCE;
}

int mapThrottleToDshot(float throttle)
{
  const float limitedThrottle = clampFloat(throttle, -1.0f, 1.0f);

  const int mapped = static_cast<int>(
    roundf(
      static_cast<float>(THROTTLE_CENTER_DSHOT) +
      limitedThrottle * 100.0f
    )
  );

  return clampInt(mapped, THROTTLE_MIN_DSHOT, THROTTLE_MAX_DSHOT);
}

uint16_t clampMotorCommand(int command)
{
  if (command <= 0)
  {
    return DSHOT_STOP;
  }

  command = clampInt(command, DSHOT_MIN_THROTTLE, DSHOT_MAX_TEST);
  return static_cast<uint16_t>(command);
}

void bodyToNavigation(
  float bodyX,
  float bodyY,
  float &navX,
  float &navY
)
{
  const float yawRad = yawAngleCwDeg * DEG_TO_RAD_F;
  const float cosineYaw = cosf(yawRad);
  const float sineYaw = sinf(yawRad);

  // Positive yawAngleCwDeg is clockwise viewed from above.
  navX = cosineYaw * bodyX + sineYaw * bodyY;
  navY = -sineYaw * bodyX + cosineYaw * bodyY;
}

void navigationToBody(
  float navX,
  float navY,
  float &bodyX,
  float &bodyY
)
{
  const float yawRad = yawAngleCwDeg * DEG_TO_RAD_F;
  const float cosineYaw = cosf(yawRad);
  const float sineYaw = sinf(yawRad);

  bodyX = cosineYaw * navX - sineYaw * navY;
  bodyY = sineYaw * navX + cosineYaw * navY;
}

uint32_t rangeAgeMs(uint32_t nowMs)
{
  if (!rangeHasEverBeenValid)
  {
    return static_cast<uint32_t>(nowMs - flightEnableMs);
  }

  return static_cast<uint32_t>(nowMs - lastValidRangeMs);
}

bool rangeControlUsable(uint32_t nowMs)
{
  const bool estimatorRangeFresh =
    eskfRangeReady &&
    static_cast<uint32_t>(nowMs - lastEskfRangeAcceptedMs) <=
      RANGE_LONG_FAILURE_MS;

  return
    rangeHasEverBeenValid &&
    estimatorRangeFresh &&
    rangeAgeMs(nowMs) <= RANGE_LONG_FAILURE_MS &&
    filteredRangeMeters >= MIN_VALID_RANGE_M &&
    filteredRangeMeters <= MAX_VALID_RANGE_M;
}

bool flowHealthFresh(uint32_t nowMs)
{
  return
    flowSensorHealthy &&
    lastHealthyFlowMs != 0U &&
    static_cast<uint32_t>(nowMs - lastHealthyFlowMs) <= FLOW_HEALTH_LOSS_MS;
}

bool flowMeasurementFreshForArm(uint32_t nowMs)
{
  return
    eskfFlowVelocityReady &&
    lastFlowMeasurementMs != 0U &&
    static_cast<uint32_t>(nowMs - lastFlowMeasurementMs) <=
      FLOW_ARM_MEASUREMENT_FRESH_MS;
}

bool rangeFailureIsLong(uint32_t nowMs)
{
  if (!flightEnabled)
  {
    return false;
  }

  const bool sensorTooOld =
    rangeAgeMs(nowMs) > RANGE_LONG_FAILURE_MS;

  const bool estimatorTooOld =
    !eskfRangeReady ||
    static_cast<uint32_t>(nowMs - lastEskfRangeAcceptedMs) >
      RANGE_LONG_FAILURE_MS;

  return sensorTooOld || estimatorTooOld;
}

bool magnetometerSampleFresh(uint32_t nowMs)
{
  return
    magSensorInitialized &&
    magMeasurementValid &&
    lastMagSampleMs != 0U &&
    static_cast<uint32_t>(nowMs - lastMagSampleMs) <= MAG_STALE_MS;
}

bool magnetometerAcceptedFresh(uint32_t nowMs)
{
  return
    magYawFusionEnabled &&
    magReferenceValid &&
    lastMagAcceptedMs != 0U &&
    static_cast<uint32_t>(nowMs - lastMagAcceptedMs) <= MAG_STALE_MS;
}

bool magnetometerReadyForFlight(uint32_t nowMs)
{
  return
    magSensorInitialized &&
    magMeasurementValid &&
    magReferenceValid &&
    magnetometerSampleFresh(nowMs) &&
    magLastCtrl1Valid &&
    magLastCtrl1 == MAG_CTRL1_CONFIG;
}

bool motorsCompletelyStoppedForSensorService()
{
  return
    !flightEnabled &&
    !enableRampActive &&
    !disableRampActive &&
    baseThrottleDshot == 0;
}

// ================================================================
// ESTIMATOR HELPERS
// ================================================================

void synchronizeEstimatorState()
{
  eskfStateHealthy = stateEstimator.healthy();

  positionNavX_m = stateEstimator.positionX();
  positionNavY_m = stateEstimator.positionY();

  float velocityNavigation[3] = {};
  stateEstimator.getVelocityNavigation(velocityNavigation);
  measuredNavVelocityX_mps = velocityNavigation[0];
  measuredNavVelocityY_mps = velocityNavigation[1];
  verticalVelocityMps = velocityNavigation[2];

  float velocityBody[3] = {};
  stateEstimator.getBodyVelocity(velocityBody);
  fusedBodyVelocityX_mps = velocityBody[0];
  fusedBodyVelocityY_mps = velocityBody[1];
}

float altitudeForControl()
{
  if (stateEstimator.healthy() && rangeHasEverBeenValid && eskfRangeReady)
  {
    const float estimate = stateEstimator.positionZ();
    if (isfinite(estimate) &&
        estimate >= MIN_VALID_RANGE_M &&
        estimate <= MAX_VALID_RANGE_M)
    {
      return estimate;
    }
  }

  return filteredRangeMeters;
}

void resetEstimatorLocalFrame()
{
  const float initialAltitude =
    filteredRangeMeters >= MIN_VALID_RANGE_M
      ? filteredRangeMeters
      : 0.0f;

  // ESKF uses right-handed rotations about body X/Y/Z.  The already-tested
  // controller/mixer convention is the negative of those RH angles for both
  // pitch and roll: positive controller pitch commands the rear pair up
  // (nose-forward/nose-down), positive controller roll commands the right
  // pair up, and positive yaw is clockwise viewed from above.
  stateEstimator.reset(
    0.0f,
    0.0f,
    initialAltitude,
    CONTROLLER_PITCH_TO_RH_SIGN * measuredPitchAngleDeg * DEG_TO_RAD_F,
    CONTROLLER_ROLL_TO_RH_SIGN * measuredRollAngleDeg * DEG_TO_RAD_F,
    CONTROLLER_YAW_CW_TO_RH_SIGN * yawAngleCwDeg * DEG_TO_RAD_F
  );

  eskfStateHealthy = true;
  eskfAttitudeUsed = false;
  eskfFlowVelocityReady = false;
  eskfRangeReady = initialAltitude >= MIN_VALID_RANGE_M;
  lastEskfRangeAcceptedMs = millis();
  previousEskfZeroVelocityMs = millis();
  synchronizeEstimatorState();
}

void resetMotionModelFeedforward()
{
  motionModelFeedforward.reset();
  motionModelOutput = MotionModelFeedforward::Output{};
  appliedModelCollectiveDshot = 0.0f;
  for (int motor = 0; motor < 4; ++motor)
  {
    appliedModelMotorDeltaDshot[motor] = 0.0f;
  }
}

void updateRotorFeedback(float dt)
{
  bool newSnapshotAvailable = false;
  BidirectionalDShot::RpmSnapshot latestSnapshot;
  if (bidirectionalDshot.getSnapshot(latestSnapshot))
  {
    newSnapshotAvailable =
      latestSnapshot.sequence != lastRpmSnapshotSequence;
    rpmSnapshot = latestSnapshot;
    lastRpmSnapshotSequence = latestSnapshot.sequence;
    rpmFreshMask = latestSnapshot.freshMask;
    rpmTelemetryHealthy = latestSnapshot.allFresh;
  }

  RotorFeedbackModel::Observation observation;
  for (int motor = 0; motor < 4; ++motor)
  {
    observation.omegaRadPerSec[motor] =
      rpmSnapshot.motor[motor].omegaRadPerSec;
    observation.valid[motor] =
      rpmSnapshot.motor[motor].valid &&
      !rpmSnapshot.motor[motor].lost;
    observation.newMeasurement[motor] =
      newSnapshotAvailable &&
      rpmSnapshot.motor[motor].lastPacketValid;
    observation.ageUs[motor] = rpmSnapshot.motor[motor].ageUs;
  }

  const uint16_t commands[4] = {
    rpmSnapshot.sequence != 0
      ? rpmSnapshot.motor[0].dshotCommand : motor1Command,
    rpmSnapshot.sequence != 0
      ? rpmSnapshot.motor[1].dshotCommand : motor2Command,
    rpmSnapshot.sequence != 0
      ? rpmSnapshot.motor[2].dshotCommand : motor3Command,
    rpmSnapshot.sequence != 0
      ? rpmSnapshot.motor[3].dshotCommand : motor4Command
  };

  rotorModelState = rotorFeedbackModel.update(
    commands,
    observation,
    dt
  );

  for (int motor = 0; motor < 4; ++motor)
  {
    motionModelRotorInput.omegaRadPerSecond[motor] =
      rotorModelState.omegaRadPerSec[motor];
    motionModelRotorInput.omegaPerDshot[motor] =
      rotorModelState.omegaPerDshot[motor];
    motionModelRotorInput.measured[motor] =
      rotorModelState.measured[motor];
    motionModelRotorInput.stale[motor] =
      rotorModelState.stale[motor];
    motionModelRotorInput.ageUs[motor] =
      rotorModelState.ageUs[motor];
  }

  // The nominal ESKF state remains IMU-propagated. RPM health only scales its
  // process uncertainty; rotor thrust is not added a second time on top of the
  // measured accelerometer specific force.
  eskfActuatorProcessNoiseScale =
    (flightEnabled || disableRampActive)
      ? rotorModelState.processNoiseScale
      : 1.0f;
}

// ================================================================
// MOTOR FUNCTIONS
// ================================================================

void sendAllMotors(uint16_t command)
{
  const uint16_t commands[4] = {
    command,
    command,
    command,
    command
  };
  bidirectionalDshot.setAllMotorCommands(commands);
}

void sendMotorCommands()
{
  const uint16_t commands[4] = {
    motor1Command,
    motor2Command,
    motor3Command,
    motor4Command
  };

  // The DShot task copies this set atomically and repeats the newest complete
  // mixer output at 500 Hz. The 500 Hz controller never waits for telemetry.
  bidirectionalDshot.setAllMotorCommands(commands);
}

// ================================================================
// CONTROLLER RESET FUNCTIONS
// ================================================================

void resetAttitudeControllers()
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

  rollPTerm = 0.0f;
  rollITerm = 0.0f;
  rollDTerm = 0.0f;
  pitchPTerm = 0.0f;
  pitchITerm = 0.0f;
  pitchDTerm = 0.0f;
}

void resetYawController()
{
  yawRateIntegral = 0.0f;
  previousYawRateError = 0.0f;
  filteredYawDerivative = 0.0f;

  desiredYawRateDps = 0.0f;
  yawCorrection = 0.0f;

  yawPTerm = 0.0f;
  yawITerm = 0.0f;
  yawDTerm = 0.0f;
}

void deactivateXYHold()
{
  xyHoldActive = false;
  xyArmStableStartMs = 0;
  targetNavX_m = positionNavX_m;
  targetNavY_m = positionNavY_m;
  controllerBodyVelocityTargetX_mps = 0.0f;
  controllerBodyVelocityTargetY_mps = 0.0f;
  filteredHoldRollAngleDeg = 0.0f;
  filteredHoldPitchAngleDeg = 0.0f;
}

void resetAltitudeController()
{
  altitudeHoldActive = false;
  altitudeActivationStartMs = 0;
  altitudeVelocityIntegral = 0.0f;
  altitudeCorrectionDshot = 0.0f;
  desiredVerticalVelocityMps = 0.0f;
  altitudeTargetM =
    filteredRangeMeters > 0.0f ? altitudeForControl() : 0.0f;
  learnedHoverDshot = THROTTLE_CENTER_DSHOT;
  lastAltitudeBaseDshot = THROTTLE_CENTER_DSHOT;
}

void resetFlightNavigationState(uint32_t nowMs)
{
  positionNavX_m = 0.0f;
  positionNavY_m = 0.0f;
  targetNavX_m = 0.0f;
  targetNavY_m = 0.0f;

  measuredBodyVelocityX_mps = 0.0f;
  measuredBodyVelocityY_mps = 0.0f;
  measuredNavVelocityX_mps = 0.0f;
  measuredNavVelocityY_mps = 0.0f;
  fusedBodyVelocityX_mps = 0.0f;
  fusedBodyVelocityY_mps = 0.0f;
  fusedVelocityInitialized = false;
  eskfFlowVelocityReady = false;

  linearAccelBodyX_mps2 = 0.0f;
  linearAccelBodyY_mps2 = 0.0f;
  filteredLinearAccelBodyX_mps2 = 0.0f;
  filteredLinearAccelBodyY_mps2 = 0.0f;

  commandedBodyVelocityX_mps = 0.0f;
  commandedBodyVelocityY_mps = 0.0f;
  controllerBodyVelocityTargetX_mps = 0.0f;
  controllerBodyVelocityTargetY_mps = 0.0f;

  yawAngleCwDeg = 0.0f;
  yawHeadingTargetCwDeg = 0.0f;
  yawStickWasActive = false;

  // Local yaw is intentionally zeroed at every navigation reset. Prefer the
  // most recent DISARMED/previous sample as the new magnetic zero so arm-time
  // motor current does not define the reference. If no fresh sample exists,
  // the next valid QMC5883P sample captures the local reference.
  const bool freshMagForReference =
    magSensorInitialized &&
    magMeasurementValid &&
    lastMagSampleMs != 0U &&
    static_cast<uint32_t>(nowMs - lastMagSampleMs) <= MAG_STALE_MS &&
    isfinite(magAbsoluteYawRhDeg) &&
    isfinite(magFieldMagnitude) &&
    magFieldMagnitude > 1.0f;

  if (freshMagForReference)
  {
    magReferenceYawRhDeg = magAbsoluteYawRhDeg;
    magReferenceFieldMagnitude = magFieldMagnitude;
    magReferenceValid = true;
    magReferencePending = false;
  }
  else
  {
    magReferenceValid = false;
    magReferencePending = true;
  }

  magRelativeYawRhDeg = 0.0f;
  magRelativeYawCwDeg = 0.0f;

  portENTER_CRITICAL(&flowGyroMux);
  accumulatedFlowRollRad = 0.0f;
  accumulatedFlowPitchRad = 0.0f;
  accumulatedFlowYawCwRad = 0.0f;
  portEXIT_CRITICAL(&flowGyroMux);
  lastProcessedFlowSequence = 0;

  consecutiveHealthyFlowSamples = 0;
  flowSensorHealthy = false;
  lastHealthyFlowMs = 0;

  consecutiveValidFlowSamples = 0;
  lastValidFlowMs = 0;
  lastFlowMeasurementMs = 0;
  lastFlowSampleValid = false;

  flowNoMotionCount = 0;
  flowYawRateRejectCount = 0;
  flowYawRateDownweightCount = 0;
  lastFlowYawRateDps = 0.0f;
  lastFlowYawNoiseScale = 1.0f;

  consecutiveValidRangeSamples = 0;
  rangeRecoveryStable = false;
  rangeJustRecovered = false;
  rangeLongFailureLatched = false;

  resetAltitudeController();
  resetAttitudeControllers();
  resetYawController();
  resetMotionModelFeedforward();
  resetEstimatorLocalFrame();
  deactivateXYHold();
}

void stopMotors()
{
  resetAttitudeControllers();
  resetYawController();
  resetMotionModelFeedforward();

  baseThrottleDshot = 0;

  motor1Command = DSHOT_STOP;
  motor2Command = DSHOT_STOP;
  motor3Command = DSHOT_STOP;
  motor4Command = DSHOT_STOP;
}

void immediateStopFrames()
{
  stopMotors();
  bidirectionalDshot.stopAll();

  // The dedicated service sends zero immediately and then every 2 ms. This
  // short wait guarantees several stop frames without bypassing the RMT task.
  delay(20);
}

// ================================================================
// I2C AND MPU6050 FUNCTIONS
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
  for (int attempt = 0; attempt < 2; ++attempt)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(14)) == 14)
    {
      axRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      ayRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      azRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

      // Temperature is not used by this controller, but its two bytes must be
      // consumed so the following reads remain aligned with the gyro fields.
      const int16_t unusedTemperatureRaw =
        static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      (void)unusedTemperatureRaw;

      gxRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      gyRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      gzRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      return true;
    }

    // Drain any partial response before retrying.
    while (Wire.available() > 0)
    {
      Wire.read();
    }

    if (attempt == 0)
    {
      delayMicroseconds(20);  // brief pause before retry
    }
  }

  return false;
}

bool setupMPU()
{
  delay(100);

  writeMPU(0x6B, 0x00);
  delay(100);

  writeMPU(0x1B, 0x00);
  writeMPU(0x1C, 0x00);
  writeMPU(0x1A, 0x03);

  delay(100);
  return readMPURaw();
}

float calculateAccelRollDeg(float ax, float ay, float az)
{
  (void)ay;
  const float verticalZ = IMU_UPSIDE_DOWN ? -az : az;

  return
    ROLL_ACCEL_SIGN *
    atan2f(-ax, verticalZ) *
    RAD_TO_DEG_F;
}

float calculateAccelPitchDeg(float ax, float ay, float az)
{
  (void)ax;
  const float verticalZ = IMU_UPSIDE_DOWN ? -az : az;

  return
    PITCH_ACCEL_SIGN *
    atan2f(ay, verticalZ) *
    RAD_TO_DEG_F;
}

bool calibrateIMU()
{
  Serial.println();
  Serial.println("IMU calibration starting.");
  Serial.println("Keep the drone level and completely still.");
  delay(1500);

  constexpr int SAMPLE_COUNT = 1500;

  int64_t axTotal = 0;
  int64_t ayTotal = 0;
  int64_t azTotal = 0;
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
      axTotal += axRaw;
      ayTotal += ayRaw;
      azTotal += azRaw;
      gxTotal += gxRaw;
      gyTotal += gyRaw;
      gzTotal += gzRaw;

      const float ax = static_cast<float>(axRaw);
      const float ay = static_cast<float>(ayRaw);
      const float az = static_cast<float>(azRaw);

      rollAccelTotal += calculateAccelRollDeg(ax, ay, az);
      pitchAccelTotal += calculateAccelPitchDeg(ax, ay, az);
      validSamples++;
    }

    delay(2);
  }

  if (validSamples < SAMPLE_COUNT / 2)
  {
    Serial.println("IMU calibration failed.");
    return false;
  }

  accelXBiasRaw = static_cast<float>(axTotal) / validSamples;
  accelYBiasRaw = static_cast<float>(ayTotal) / validSamples;

  const float meanAzRaw =
    static_cast<float>(azTotal) / validSamples;
  accelZBodyBiasRaw = -meanAzRaw - ACCEL_LSB_PER_G;

  gyroXBiasRaw = static_cast<float>(gxTotal) / validSamples;
  gyroYBiasRaw = static_cast<float>(gyTotal) / validSamples;
  gyroZBiasRaw = static_cast<float>(gzTotal) / validSamples;

  rollLevelTrimDeg = static_cast<float>(rollAccelTotal / validSamples);
  pitchLevelTrimDeg = static_cast<float>(pitchAccelTotal / validSamples);

  measuredRollAngleDeg = 0.0f;
  measuredPitchAngleDeg = 0.0f;
  complementaryRollAngleDeg = 0.0f;
  complementaryPitchAngleDeg = 0.0f;
  yawAngleCwDeg = 0.0f;

  Serial.println("IMU calibration complete.");
  Serial.print("accelXBiasRaw:"); Serial.println(accelXBiasRaw, 3);
  Serial.print("accelYBiasRaw:"); Serial.println(accelYBiasRaw, 3);
  Serial.print("accelZBodyBiasRaw:"); Serial.println(accelZBodyBiasRaw, 3);
  Serial.print("gyroXBiasRaw:"); Serial.println(gyroXBiasRaw, 3);
  Serial.print("gyroYBiasRaw:"); Serial.println(gyroYBiasRaw, 3);
  Serial.print("gyroZBiasRaw:"); Serial.println(gyroZBiasRaw, 3);
  Serial.print("rollLevelTrimDeg:"); Serial.println(rollLevelTrimDeg, 3);
  Serial.print("pitchLevelTrimDeg:"); Serial.println(pitchLevelTrimDeg, 3);
  Serial.println();

  return true;
}

bool updateAttitude(float dt, float processNoiseScale)
{
  const uint32_t imuReadStartUs = micros();
  const bool imuReadOk = readMPURaw();
  recordTiming(timingImuRead, micros() - imuReadStartUs);

  if (!imuReadOk)
  {
    return false;
  }

  const float gyroX =
    (static_cast<float>(gxRaw) - gyroXBiasRaw) / GYRO_LSB_PER_DPS;

  const float gyroY =
    (static_cast<float>(gyRaw) - gyroYBiasRaw) / GYRO_LSB_PER_DPS;

  const float gyroZ =
    (static_cast<float>(gzRaw) - gyroZBiasRaw) / GYRO_LSB_PER_DPS;

  measuredRollRateDps = ROLL_GYRO_SIGN * gyroY;
  measuredPitchRateDps = PITCH_GYRO_SIGN * gyroX;
  measuredYawRateDps = YAW_GYRO_SIGN * gyroZ;

  if (!isfinite(measuredRollRateDps) ||
      !isfinite(measuredPitchRateDps) ||
      !isfinite(measuredYawRateDps))
  {
    return false;
  }

  const float ax = static_cast<float>(axRaw);
  const float ay = static_cast<float>(ayRaw);
  const float az = static_cast<float>(azRaw);

  float accelerometerRollDeg = calculateAccelRollDeg(ax, ay, az);
  float accelerometerPitchDeg = calculateAccelPitchDeg(ax, ay, az);

  accelerometerRollDeg -= rollLevelTrimDeg;
  accelerometerPitchDeg -= pitchLevelTrimDeg;

  const float predictedComplementaryRoll =
    complementaryRollAngleDeg + measuredRollRateDps * dt;

  const float predictedComplementaryPitch =
    complementaryPitchAngleDeg + measuredPitchRateDps * dt;

  complementaryRollAngleDeg =
    COMPLEMENTARY_GYRO_WEIGHT * predictedComplementaryRoll +
    (1.0f - COMPLEMENTARY_GYRO_WEIGHT) * accelerometerRollDeg;

  complementaryPitchAngleDeg =
    COMPLEMENTARY_GYRO_WEIGHT * predictedComplementaryPitch +
    (1.0f - COMPLEMENTARY_GYRO_WEIGHT) * accelerometerPitchDeg;

  // Installed upside-down IMU -> body convention used by the ESKF:
  // +X right, +Y forward, +Z up.  At rest, specific force is [0,0,+g].
  bodySpecificForceX_mps2 =
    ESKF_ACCEL_X_FROM_RAW_SIGN *
    (static_cast<float>(axRaw) - accelXBiasRaw) /
    ACCEL_LSB_PER_G * STANDARD_GRAVITY_MPS2;

  // The existing pitch-angle expression is -atan2(ay, -az).  Therefore the
  // installed IMU's calibrated +body-Y (forward) specific force is +ay,
  // unlike body X, which is -ax.  Keeping this sign consistent is essential:
  // controller-positive pitch is -RH rotation about body X.
  bodySpecificForceY_mps2 =
    ESKF_ACCEL_Y_FROM_RAW_SIGN *
    (static_cast<float>(ayRaw) - accelYBiasRaw) /
    ACCEL_LSB_PER_G * STANDARD_GRAVITY_MPS2;

  bodySpecificForceZ_mps2 =
    (ESKF_ACCEL_Z_FROM_RAW_SIGN * static_cast<float>(azRaw) -
      accelZBodyBiasRaw) /
    ACCEL_LSB_PER_G * STANDARD_GRAVITY_MPS2;

  const float measuredSpecificForceBody[3] = {
    bodySpecificForceX_mps2,
    bodySpecificForceY_mps2,
    bodySpecificForceZ_mps2
  };

  // Convert the already-tested controller signs to right-handed body rates.
  // Both controller pitch and roll are the negative of their RH rotations;
  // clockwise yaw is the negative of RH yaw about +Z.
  const float gyroBodyRadPerSecond[3] = {
    CONTROLLER_PITCH_TO_RH_SIGN * measuredPitchRateDps * DEG_TO_RAD_F,
    CONTROLLER_ROLL_TO_RH_SIGN * measuredRollRateDps * DEG_TO_RAD_F,
    CONTROLLER_YAW_CW_TO_RH_SIGN * measuredYawRateDps * DEG_TO_RAD_F
  };

  const bool airborneControlActive = flightEnabled || disableRampActive;

  // Lateral accel now feeds the ESKF in flight so it can predict short-term
  // X/Y motion between 100 Hz optical-flow corrections.
  const float eskfSpecificForceBody[3] = {
    bodySpecificForceX_mps2,
    bodySpecificForceY_mps2,
    bodySpecificForceZ_mps2
  };

  const uint32_t eskfStartUs = micros();

  stateEstimator.predict(
    eskfSpecificForceBody,
    gyroBodyRadPerSecond,
    dt,
    processNoiseScale
  );

  // Tighter attitude correction at 500 Hz.
  stateEstimator.updateGravityDirection(
    measuredSpecificForceBody,
    airborneControlActive
      ? ESKF_GRAVITY_STD_IN_FLIGHT
      : ESKF_GRAVITY_STD_ON_GROUND
  );

  const uint32_t nowMs = millis();
  if (!flightEnabled && !disableRampActive &&
      static_cast<uint32_t>(nowMs - previousEskfZeroVelocityMs) >=
        ESKF_ZERO_VELOCITY_PERIOD_MS)
  {
    previousEskfZeroVelocityMs = nowMs;
    stateEstimator.updateZeroVelocity(0.03f);
  }

  if (!stateEstimator.healthy())
  {
    recordTiming(timingEskf, micros() - eskfStartUs);
    eskfStateHealthy = false;
    return false;
  }

  synchronizeEstimatorState();

  float pitchAboutXRad = 0.0f;
  float rollAboutYRad = 0.0f;
  float yawRightHandRad = 0.0f;
  stateEstimator.getEulerRadians(
    pitchAboutXRad,
    rollAboutYRad,
    yawRightHandRad
  );

  recordTiming(timingEskf, micros() - eskfStartUs);

  eskfPitchAngleDeg =
    CONTROLLER_PITCH_TO_RH_SIGN * pitchAboutXRad * RAD_TO_DEG_F;
  eskfRollAngleDeg =
    CONTROLLER_ROLL_TO_RH_SIGN * rollAboutYRad * RAD_TO_DEG_F;
  eskfYawCwDeg = wrapAngle180(
    CONTROLLER_YAW_CW_TO_RH_SIGN * yawRightHandRad * RAD_TO_DEG_F
  );

  const float rollDisagreement = fabsf(
    wrapAngle180(eskfRollAngleDeg - complementaryRollAngleDeg)
  );

  const float pitchDisagreement = fabsf(
    wrapAngle180(eskfPitchAngleDeg - complementaryPitchAngleDeg)
  );

  const bool eskfAttitudeUsable =
    isfinite(eskfRollAngleDeg) &&
    isfinite(eskfPitchAngleDeg) &&
    isfinite(eskfYawCwDeg) &&
    rollDisagreement <= ESKF_MAX_ATTITUDE_DISAGREEMENT_DEG &&
    pitchDisagreement <= ESKF_MAX_ATTITUDE_DISAGREEMENT_DEG &&
    fabsf(eskfRollAngleDeg) < 80.0f &&
    fabsf(eskfPitchAngleDeg) < 80.0f;

  eskfAttitudeUsed =
    eskfAttitudeControlEnabled && eskfAttitudeUsable;

  if (eskfAttitudeUsed)
  {
    measuredRollAngleDeg = eskfRollAngleDeg;
    measuredPitchAngleDeg = eskfPitchAngleDeg;
  }
  else
  {
    // Independent complementary fallback preserves the already-proven
    // roll/pitch attitude loop if an ESKF innovation or sign check is poor.
    measuredRollAngleDeg = complementaryRollAngleDeg;
    measuredPitchAngleDeg = complementaryPitchAngleDeg;
  }

  // Yaw can use the magnetically corrected ESKF independently of whether
  // ESKF roll/pitch attitude replacement (EATT) is enabled. If magnetic yaw
  // fusion is disabled, preserve the original gyro-dead-reckoned behavior.
  if (magYawFusionEnabled && magReferenceValid && stateEstimator.healthy())
  {
    yawAngleCwDeg = eskfYawCwDeg;
  }
  else
  {
    yawAngleCwDeg = wrapAngle180(
      yawAngleCwDeg + measuredYawRateDps * dt
    );
  }

  // Synchronize gyro integration with the PMW3901 count interval. The
  // opposite-core PMW task snapshots/resets these three values after each
  // SPI read, so keep this critical section very short.
  const float flowRollIncrementRad =
    measuredRollRateDps * DEG_TO_RAD_F * dt;
  const float flowPitchIncrementRad =
    measuredPitchRateDps * DEG_TO_RAD_F * dt;
  const float flowYawIncrementCwRad =
    measuredYawRateDps * DEG_TO_RAD_F * dt;
  portENTER_CRITICAL(&flowGyroMux);
  accumulatedFlowRollRad += flowRollIncrementRad;
  accumulatedFlowPitchRad += flowPitchIncrementRad;
  accumulatedFlowYawCwRad += flowYawIncrementCwRad;
  portEXIT_CRITICAL(&flowGyroMux);

  // Remove the body-frame gravity-specific-force component using the ESKF
  // quaternion.  This is used only by the optional acceleration assist and
  // debug output; the ESKF itself remains the position/velocity source.
  float rotationNavigationFromBody[3][3] = {};
  stateEstimator.getRotationMatrix(rotationNavigationFromBody);

  const float gravitySpecificBodyX =
    STANDARD_GRAVITY_MPS2 * rotationNavigationFromBody[2][0];
  const float gravitySpecificBodyY =
    STANDARD_GRAVITY_MPS2 * rotationNavigationFromBody[2][1];

  linearAccelBodyX_mps2 = applySymmetricDeadband(
    bodySpecificForceX_mps2 - gravitySpecificBodyX,
    IMU_ACCEL_DEADBAND_MPS2
  );

  linearAccelBodyY_mps2 = applySymmetricDeadband(
    bodySpecificForceY_mps2 - gravitySpecificBodyY,
    IMU_ACCEL_DEADBAND_MPS2
  );

  linearAccelBodyX_mps2 = clampFloat(
    linearAccelBodyX_mps2,
    -IMU_ACCEL_LIMIT_MPS2,
    IMU_ACCEL_LIMIT_MPS2
  );

  linearAccelBodyY_mps2 = clampFloat(
    linearAccelBodyY_mps2,
    -IMU_ACCEL_LIMIT_MPS2,
    IMU_ACCEL_LIMIT_MPS2
  );

  filteredLinearAccelBodyX_mps2 += IMU_ACCEL_FILTER_ALPHA *
    (linearAccelBodyX_mps2 - filteredLinearAccelBodyX_mps2);

  filteredLinearAccelBodyY_mps2 += IMU_ACCEL_FILTER_ALPHA *
    (linearAccelBodyY_mps2 - filteredLinearAccelBodyY_mps2);

  return
    isfinite(measuredRollAngleDeg) &&
    isfinite(measuredPitchAngleDeg) &&
    isfinite(yawAngleCwDeg);
}

// ================================================================
// QMC5883P MAGNETOMETER
// ================================================================

bool writeMagRegister(uint8_t registerAddress, uint8_t value)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMagRegister(uint8_t registerAddress, uint8_t &value)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(MAG_ADDR, static_cast<uint8_t>(1)) != 1)
  {
    return false;
  }

  value = Wire.read();
  return true;
}

bool readMagBytes(uint8_t startRegister, uint8_t *buffer, uint8_t length)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(startRegister);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(MAG_ADDR, static_cast<uint8_t>(length)) != length)
  {
    while (Wire.available() > 0) Wire.read();
    return false;
  }

  for (uint8_t index = 0; index < length; ++index)
  {
    buffer[index] = Wire.read();
  }
  return true;
}

bool initializeMagnetometer()
{
  Wire.beginTransmission(MAG_ADDR);
  if (Wire.endTransmission() != 0)
  {
    return false;
  }

  uint8_t chipId = 0;
  if (!readMagRegister(MAG_CHIP_ID_REG, chipId) || chipId != 0x80)
  {
    return false;
  }

  // QMC5883P soft reset.
  if (!writeMagRegister(MAG_CTRL2_REG, 0x80))
    return false;

  delay(20);

  if (!writeMagRegister(MAG_AXIS_SIGN_REG, 0x06))
    return false;

  if (!writeMagRegister(MAG_CTRL2_REG, 0x08))
    return false;

  // 100 Hz continuous mode.
  if (!writeMagRegister(MAG_CTRL1_REG, MAG_CTRL1_CONFIG))
    return false;

  delay(50);

  uint8_t control1 = 0;
  if (!readMagRegister(MAG_CTRL1_REG, control1))
  {
    magLastCtrl1Valid = false;
    return false;
  }

  magLastCtrl1 = control1;
  magLastCtrl1Valid = true;

  return control1 == MAG_CTRL1_CONFIG;
}

MagReadResult readMagRaw(int16_t &rawX, int16_t &rawY, int16_t &rawZ)
{
  ++magPollCount;

  uint8_t status = 0;
  if (!readMagRegister(MAG_STATUS_REG, status))
  {
    ++magStatusReadFailCount;
    magLastStatusValid = false;
    return MagReadResult::StatusReadFailed;
  }

  magLastStatus = status;
  magLastStatusValid = true;

  // DRDY is bit 0. A poll with no fresh sample is not a bus failure.
  if ((status & 0x01U) == 0U)
  {
    ++magNotReadyCount;
    return MagReadResult::NotReady;
  }

  uint8_t data[6] = {};
  if (!readMagBytes(MAG_X_LSB_REG, data, 6))
  {
    ++magDataReadFailCount;
    return MagReadResult::DataReadFailed;
  }

  rawX = static_cast<int16_t>(
    (static_cast<uint16_t>(data[1]) << 8) | data[0]
  );
  rawY = static_cast<int16_t>(
    (static_cast<uint16_t>(data[3]) << 8) | data[2]
  );
  rawZ = static_cast<int16_t>(
    (static_cast<uint16_t>(data[5]) << 8) | data[4]
  );

  ++magGoodRawCount;
  return MagReadResult::Good;
}

bool verifyMagnetometerProducesData()
{
  const uint32_t startMs = millis();

  while (
    static_cast<uint32_t>(millis() - startMs) <
      MAG_FIRST_SAMPLE_TIMEOUT_MS
  )
  {
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    const MagReadResult result = readMagRaw(rawX, rawY, rawZ);
    if (result == MagReadResult::Good)
    {
      return true;
    }

    // Status/read failures and DRDY-low are all retryable during startup.
    delay(5);
  }

  return false;
}

// Forward declaration because the reliable startup wrapper resets diagnostics
// before each attempt.
void clearMagDiagnostics();

bool initializeMagnetometerReliableStartup()
{
  uint32_t attempt = 0;

  while (true)
  {
    ++attempt;
    clearMagDiagnostics();

    Serial.print("QMC5883P initialization attempt ");
    Serial.println(attempt);

    const bool configured = initializeMagnetometer();
    const bool liveData = configured && verifyMagnetometerProducesData();

    if (configured && liveData)
    {
      magSensorInitialized = true;
      previousMagPollUs = micros() - MAG_POLL_PERIOD_US;

      Serial.println(
        "QMC5883P initialization VERIFIED: CTRL1=0xCB and live XYZ data."
      );
      return true;
    }

    magSensorInitialized = false;
    magMeasurementValid = false;
    magReferenceValid = false;
    magReferencePending = true;

    if (!configured)
    {
      Serial.println(
        "QMC5883P configuration/init failed; retrying automatically."
      );
    }
    else
    {
      Serial.println(
        "QMC5883P configured but produced no live XYZ sample; retrying automatically."
      );
    }

    // The dedicated DShot task continues sending stop commands independently.
    delay(MAG_STARTUP_RETRY_DELAY_MS);
  }
}

bool calculateMagneticYawRightHandDeg(float &yawRightHandDeg)
{
  // Use the ESKF quaternion ONLY for tilt. For R_NB (body -> navigation),
  // row 2 is navigation +Z expressed in body coordinates and is independent
  // of yaw. This avoids the bad duplicate accelerometer tilt calculation from
  // the preliminary standalone fusion test.
  float rotationNavigationFromBody[3][3] = {};
  stateEstimator.getRotationMatrix(rotationNavigationFromBody);

  float upX = rotationNavigationFromBody[2][0];
  float upY = rotationNavigationFromBody[2][1];
  float upZ = rotationNavigationFromBody[2][2];

  const float upNorm = sqrtf(upX * upX + upY * upY + upZ * upZ);
  if (!isfinite(upNorm) || upNorm < 0.5f)
  {
    return false;
  }
  upX /= upNorm;
  upY /= upNorm;
  upZ /= upNorm;

  // Body forward is +Y. Project it into the horizontal plane.
  float forwardX = -upY * upX;
  float forwardY = 1.0f - upY * upY;
  float forwardZ = -upY * upZ;
  const float forwardNorm = sqrtf(
    forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ
  );
  if (!isfinite(forwardNorm) || forwardNorm < 0.10f)
  {
    return false;
  }
  forwardX /= forwardNorm;
  forwardY /= forwardNorm;
  forwardZ /= forwardNorm;

  // right = forward x up. At level this is exactly body +X.
  const float rightX = forwardY * upZ - forwardZ * upY;
  const float rightY = forwardZ * upX - forwardX * upZ;
  const float rightZ = forwardX * upY - forwardY * upX;

  // Remove the vertical magnetic component.
  const float verticalMag =
    magFilteredBodyX * upX +
    magFilteredBodyY * upY +
    magFilteredBodyZ * upZ;

  const float horizontalX = magFilteredBodyX - verticalMag * upX;
  const float horizontalY = magFilteredBodyY - verticalMag * upY;
  const float horizontalZ = magFilteredBodyZ - verticalMag * upZ;

  const float northForward =
    horizontalX * forwardX +
    horizontalY * forwardY +
    horizontalZ * forwardZ;

  const float northRight =
    horizontalX * rightX +
    horizontalY * rightY +
    horizontalZ * rightZ;

  const float horizontalMagnitude = sqrtf(
    northForward * northForward + northRight * northRight
  );

  if (!isfinite(horizontalMagnitude) ||
      horizontalMagnitude < MAG_MIN_HORIZONTAL_COUNTS)
  {
    return false;
  }

  // Right-handed yaw is CCW positive. A clockwise physical turn therefore
  // makes this angle more negative, exactly matching the ESKF yaw convention.
  yawRightHandDeg = wrapAngle180(
    -atan2f(northRight, northForward) * RAD_TO_DEG_F
  );
  return isfinite(yawRightHandDeg);
}

void updateMagnetometer(uint32_t nowUs, uint32_t nowMs)
{
  if (!magSensorInitialized)
  {
    magMeasurementValid = false;
    return;
  }

  // A stale sample must not continue to look healthy in status output.
  if (lastMagSampleMs != 0U &&
      static_cast<uint32_t>(nowMs - lastMagSampleMs) > MAG_STALE_MS)
  {
    magMeasurementValid = false;

    // Never block the flight loop trying to reset the shared I2C device in
    // flight. Suspend active heading hold, preserve the operator's request, and
    // fall back to the already-tested zero-yaw-rate damping. If fresh accepted
    // magnetic yaw returns, updatePendingYawHeadingHold() can arm it again.
    if (flightEnabled && yawHeadingHoldEnabled && !magStaleInFlightLatched)
    {
      yawHeadingHoldEnabled = false;
      yawHeadingTargetCwDeg = yawAngleCwDeg;
      resetYawController();
      magStaleInFlightLatched = true;
      runtimeLogLine(
        "MAG STALE IN FLIGHT: heading hold suspended; yaw-rate damping remains active."
      );
    }
  }

  if (static_cast<uint32_t>(nowUs - previousMagPollUs) < MAG_POLL_PERIOD_US)
  {
    return;
  }

  previousMagPollUs = nowUs;

  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;
  const MagReadResult readResult = readMagRaw(rawX, rawY, rawZ);
  if (readResult != MagReadResult::Good)
  {
    return;
  }

// ================================================================
// QMC5883P HARD-IRON + AXIS-SCALE CALIBRATION
//
// IMPORTANT:
// Calibration is performed in the RAW SENSOR FRAME first.
// Do not apply mounting signs until after calibration.
// ================================================================

const float sensorX =
  (
    static_cast<float>(rawX) -
    MAG_BIAS_X
  ) * MAG_SCALE_X;


const float sensorY =
  (
    static_cast<float>(rawY) -
    MAG_BIAS_Y
  ) * MAG_SCALE_Y;


const float sensorZ =
  (
    static_cast<float>(rawZ) -
    MAG_BIAS_Z
  ) * MAG_SCALE_Z;


// ================================================================
// SENSOR FRAME -> DRONE BODY FRAME
//
// Compass mounting:
//
//   sensor +X = body left
//   sensor +Y = body backward
//   sensor +Z = body up
//
// Therefore:
//
//   body X = -sensor X
//   body Y = -sensor Y
//   body Z = +sensor Z
// ================================================================

magBodyX = -sensorX;
magBodyY = -sensorY;
magBodyZ =  sensorZ;

  if (!magFilterInitialized)
  {
    magFilteredBodyX = magBodyX;
    magFilteredBodyY = magBodyY;
    magFilteredBodyZ = magBodyZ;
    magFilterInitialized = true;
  }
  else
  {
    magFilteredBodyX += MAG_FILTER_ALPHA * (magBodyX - magFilteredBodyX);
    magFilteredBodyY += MAG_FILTER_ALPHA * (magBodyY - magFilteredBodyY);
    magFilteredBodyZ += MAG_FILTER_ALPHA * (magBodyZ - magFilteredBodyZ);
  }

  magFieldMagnitude = sqrtf(
    magFilteredBodyX * magFilteredBodyX +
    magFilteredBodyY * magFilteredBodyY +
    magFilteredBodyZ * magFilteredBodyZ
  );

  float absoluteYawRhDeg = 0.0f;
  if (!isfinite(magFieldMagnitude) ||
      !calculateMagneticYawRightHandDeg(absoluteYawRhDeg))
  {
    magMeasurementValid = false;
    ++magRejectedUpdates;
    return;
  }

  magAbsoluteYawRhDeg = absoluteYawRhDeg;
  magMeasurementValid = true;
  lastMagSampleMs = nowMs;
  magStaleInFlightLatched = false;

  // Each navigation reset requests a new local magnetic origin. This preserves
  // the existing meaning of local X/Y rather than suddenly rotating the whole
  // local frame to magnetic north.
  if (magReferencePending || !magReferenceValid)
  {
    magReferenceYawRhDeg = magAbsoluteYawRhDeg;
    magReferenceFieldMagnitude = magFieldMagnitude;
    magRelativeYawRhDeg = 0.0f;
    magRelativeYawCwDeg = 0.0f;
    magReferenceValid = true;
    magReferencePending = false;
    magLastInnovationDeg = 0.0f;
    return;
  }

  magRelativeYawRhDeg = wrapAngle180(
    magAbsoluteYawRhDeg - magReferenceYawRhDeg
  );
  magRelativeYawCwDeg = wrapAngle180(-magRelativeYawRhDeg);

  if (!magYawFusionEnabled)
  {
    return;
  }

  // Basic field-magnitude interference gate relative to the field captured at
  // the local reference. Large throttle-dependent magnetic distortion should
  // be rejected instead of steering the ESKF yaw.
  if (!isfinite(magReferenceFieldMagnitude) ||
      magReferenceFieldMagnitude < 1.0f)
  {
    ++magRejectedUpdates;
    return;
  }

  const float fieldRatio = magFieldMagnitude / magReferenceFieldMagnitude;
  if (fieldRatio < MAG_FIELD_RATIO_MIN || fieldRatio > MAG_FIELD_RATIO_MAX)
  {
    ++magRejectedUpdates;
    return;
  }

  float pitchXRad = 0.0f;
  float rollYRad = 0.0f;
  float currentYawRhRad = 0.0f;
  stateEstimator.getEulerRadians(pitchXRad, rollYRad, currentYawRhRad);

  const float currentYawRhDeg = currentYawRhRad * RAD_TO_DEG_F;
  magLastInnovationDeg = wrapAngle180(
    magRelativeYawRhDeg - currentYawRhDeg
  );

  // At 100 Hz a real aircraft cannot create a 35-degree one-sample heading
  // jump. Treat such a discrepancy as magnetic interference / bad data.
  if (fabsf(magLastInnovationDeg) > MAG_MAX_INNOVATION_DEG)
  {
    ++magRejectedUpdates;
    return;
  }

  const bool accepted = stateEstimator.updateYawRightHand(
    magRelativeYawRhDeg * DEG_TO_RAD_F,
    MAG_YAW_STD_DEG * DEG_TO_RAD_F,
    6.0f
  );

  if (accepted)
  {
    ++magAcceptedUpdates;
    lastMagAcceptedMs = nowMs;
    synchronizeEstimatorState();
  }
  else
  {
    ++magRejectedUpdates;
  }
}

void printMagStatus()
{
  const uint32_t nowMs = millis();
  const uint32_t sampleAge =
    lastMagSampleMs == 0U ? UINT32_MAX : nowMs - lastMagSampleMs;
  const uint32_t acceptedAge =
    lastMagAcceptedMs == 0U ? UINT32_MAX : nowMs - lastMagAcceptedMs;

  const bool sampleFresh = magnetometerSampleFresh(nowMs);
  const bool acceptedFresh = magnetometerAcceptedFresh(nowMs);

  runtimeLogf(
    "MAG,INIT:%d,FUSE_EN:%d,FRESH:%d,ACC_FRESH:%d,HOLD_REQ:%d,HOLD:%d,OK:%d,REF:%d,SAMP_AGE:%lu,ACC_AGE:%lu,ABS_RH:%.2f,REL_RH:%.2f,REL_CW:%.2f,INNOV:%.2f,FIELD:%.1f,REF_FIELD:%.1f,MX:%.1f,MY:%.1f,MZ:%.1f,ACC:%lu,REJ:%lu",
    magSensorInitialized ? 1 : 0,
    magYawFusionEnabled ? 1 : 0,
    sampleFresh ? 1 : 0,
    acceptedFresh ? 1 : 0,
    yawHeadingHoldRequested ? 1 : 0,
    yawHeadingHoldEnabled ? 1 : 0,
    magMeasurementValid ? 1 : 0,
    magReferenceValid ? 1 : 0,
    static_cast<unsigned long>(sampleAge),
    static_cast<unsigned long>(acceptedAge),
    magAbsoluteYawRhDeg,
    magRelativeYawRhDeg,
    magRelativeYawCwDeg,
    magLastInnovationDeg,
    magFieldMagnitude,
    magReferenceFieldMagnitude,
    magFilteredBodyX,
    magFilteredBodyY,
    magFilteredBodyZ,
    static_cast<unsigned long>(magAcceptedUpdates),
    static_cast<unsigned long>(magRejectedUpdates)
  );

  runtimeLogf(
    "MAGDBG,POLL:%lu,RAWOK:%lu,STATFAIL:%lu,NORDY:%lu,DATAFAIL:%lu,STAT_OK:%d,STAT:0x%02X,CTRL1_OK:%d,CTRL1:0x%02X",
    static_cast<unsigned long>(magPollCount),
    static_cast<unsigned long>(magGoodRawCount),
    static_cast<unsigned long>(magStatusReadFailCount),
    static_cast<unsigned long>(magNotReadyCount),
    static_cast<unsigned long>(magDataReadFailCount),
    magLastStatusValid ? 1 : 0,
    static_cast<unsigned>(magLastStatus),
    magLastCtrl1Valid ? 1 : 0,
    static_cast<unsigned>(magLastCtrl1)
  );
}

void clearMagDiagnostics()
{
  magPollCount = 0;
  magStatusReadFailCount = 0;
  magNotReadyCount = 0;
  magDataReadFailCount = 0;
  magGoodRawCount = 0;
  magLastStatus = 0;
  magLastStatusValid = false;
  magLastCtrl1 = 0;
  magLastCtrl1Valid = false;
}

bool reinitializeMagnetometerCore(bool preserveFlightModeRequest)
{
  const bool previousFusionRequest = magYawFusionEnabled;
  const bool previousHoldRequest = yawHeadingHoldRequested;

  // Active heading torque is always suspended while the sensor is reset.
  magYawFusionEnabled = preserveFlightModeRequest
    ? previousFusionRequest
    : false;
  yawHeadingHoldRequested = preserveFlightModeRequest
    ? previousHoldRequest
    : false;
  yawHeadingHoldEnabled = false;
  yawHeadingTargetCwDeg = yawAngleCwDeg;
  resetYawController();

  magSensorInitialized = false;
  magMeasurementValid = false;
  magReferenceValid = false;
  magReferencePending = true;
  magFilterInitialized = false;
  magStaleInFlightLatched = false;

  magBodyX = 0.0f;
  magBodyY = 0.0f;
  magBodyZ = 0.0f;
  magFilteredBodyX = 0.0f;
  magFilteredBodyY = 0.0f;
  magFilteredBodyZ = 0.0f;
  magFieldMagnitude = 0.0f;
  magReferenceFieldMagnitude = 0.0f;
  magLastInnovationDeg = 0.0f;
  lastMagSampleMs = 0;
  lastMagAcceptedMs = 0;
  magAcceptedUpdates = 0;
  magRejectedUpdates = 0;
  clearMagDiagnostics();

  const bool configured = initializeMagnetometer();
  const bool liveData = configured && verifyMagnetometerProducesData();

  magSensorInitialized = configured && liveData;
  previousMagPollUs = micros() - MAG_POLL_PERIOD_US;

  if (!magSensorInitialized)
  {
    magMeasurementValid = false;
    magReferenceValid = false;
    magReferencePending = true;
    return false;
  }

  return true;
}

bool resetMagnetometerWhileStopped()
{
  if (!motorsCompletelyStoppedForSensorService())
  {
    runtimeLogLine("MAGRESET REJECTED: motors/ramp must be completely stopped.");
    return false;
  }

  // Manual reset intentionally disables fusion/hold. Automatic recovery below
  // preserves the operator's requested mode instead. Never restart Wire here:
  // the MPU6050 and VL53L0X share this bus.
  if (!reinitializeMagnetometerCore(false))
  {
    runtimeLogLine(
      "MAGRESET FAILED: QMC5883P did not reinitialize/produce live data. Fusion/hold remain OFF."
    );
    return false;
  }

  runtimeLogLine(
    "MAGRESET OK: QMC5883P reinitialized and live XYZ verified; fusion/hold OFF."
  );
  return true;
}

void serviceMagnetometerRecovery(uint32_t nowMs)
{
  // The reset sequence is blocking by design. It is allowed only when all
  // motors and both throttle ramps are completely stopped.
  if (!motorsCompletelyStoppedForSensorService())
  {
    return;
  }

  bool recoveryNeeded = !magSensorInitialized;

  if (!recoveryNeeded && lastMagSampleMs != 0U)
  {
    recoveryNeeded =
      static_cast<uint32_t>(nowMs - lastMagSampleMs) >
        MAG_AUTO_RECOVERY_STALE_MS;
  }

  if (!recoveryNeeded)
  {
    return;
  }

  if (lastMagAutoRecoveryAttemptMs != 0U &&
      static_cast<uint32_t>(nowMs - lastMagAutoRecoveryAttemptMs) <
        MAG_AUTO_RECOVERY_RETRY_MS)
  {
    return;
  }

  lastMagAutoRecoveryAttemptMs = nowMs;
  runtimeLogLine(
    "MAG AUTO RECOVERY: QMC uninitialized/stale while stopped; reinitializing."
  );

  if (reinitializeMagnetometerCore(true))
  {
    runtimeLogLine(
      "MAG AUTO RECOVERY OK: configuration and live XYZ verified; magnetic reference recapturing."
    );
  }
  else
  {
    runtimeLogLine(
      "MAG AUTO RECOVERY FAILED: will retry while motors remain stopped."
    );
  }
}

// ================================================================
// SENSOR INITIALIZATION
// ================================================================

void resetFlowSensor()
{
  pinMode(FLOW_RST_PIN, OUTPUT);

  digitalWrite(FLOW_RST_PIN, HIGH);
  delay(10);
  digitalWrite(FLOW_RST_PIN, LOW);
  delay(1);
  digitalWrite(FLOW_RST_PIN, HIGH);
  delay(50);
}

bool initializeOpticalFlow()
{
  pinMode(FLOW_CS_PIN, OUTPUT);
  digitalWrite(FLOW_CS_PIN, HIGH);

  resetFlowSensor();

  SPI.begin(
    FLOW_SCK_PIN,
    FLOW_MISO_PIN,
    FLOW_MOSI_PIN,
    FLOW_CS_PIN
  );

  return flow.begin();
}

bool initializeRangeFinder()
{
  // Same initialization sequence as the working standalone sketch
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  rangeSensor.setTimeout(100);

  if (!rangeSensor.init())
  {
    return false;
  }

  // Approximately 20 ms per measurement
  if (!rangeSensor.setMeasurementTimingBudget(RANGE_TIMING_BUDGET_US))
  {
    Serial.println(
      "Warning: VL53L0X 20 ms timing budget was not accepted."
    );
  }

  // Continuous back-to-back ranging
  rangeSensor.startContinuous();

  return true;
}

// ================================================================
// NON-BLOCKING VL53L0X UPDATE
// ================================================================

bool readVL53DataReady(bool &ready)
{
  ready = false;

  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(VL53_RESULT_INTERRUPT_STATUS);

  if (Wire.endTransmission(false) != 0)
  {
    rangeLastInterruptStatusValid = false;
    return false;
  }

  if (Wire.requestFrom(VL53L0X_ADDR, static_cast<uint8_t>(1)) != 1)
  {
    while (Wire.available() > 0) Wire.read();
    rangeLastInterruptStatusValid = false;
    return false;
  }

  rangeLastInterruptStatus = Wire.read();
  rangeLastInterruptStatusValid = true;
  ready = (rangeLastInterruptStatus & 0x07U) != 0U;
  return true;
}

void registerInvalidRangeSample()
{
  consecutiveValidRangeSamples = 0;
  rangeRecoveryStable = false;
}

void updateRangeSensor(uint32_t nowUs, uint32_t nowMs)
{
  if (static_cast<uint32_t>(nowUs - previousRangePollUs) <
      RANGE_POLL_PERIOD_US)
  {
    return;
  }

  previousRangePollUs = nowUs;
  ++rangePollCount;

  bool dataReady = false;
  if (!readVL53DataReady(dataReady))
  {
    ++rangeStatusReadFailCount;
    registerInvalidRangeSample();
    return;
  }

  if (!dataReady)
  {
    // Not an invalid sample; the current ranging cycle is simply unfinished.
    ++rangeNotReadyCount;
    return;
  }

  const uint16_t rangeMillimeters =
    rangeSensor.readRangeContinuousMillimeters();

  if (rangeSensor.timeoutOccurred())
  {
    ++rangeReadFailCount;
    registerInvalidRangeSample();
    return;
  }

  const bool rangeValueValid =
    rangeMillimeters >= static_cast<uint16_t>(MIN_VALID_RANGE_M * 1000.0f) &&
    rangeMillimeters <= static_cast<uint16_t>(MAX_VALID_RANGE_M * 1000.0f);

  if (!rangeValueValid)
  {
    ++rangeInvalidValueCount;
    registerInvalidRangeSample();
    return;
  }

  ++rangeGoodCount;

  const float newRangeM =
    static_cast<float>(rangeMillimeters) * 0.001f;

  if (!rangeHasEverBeenValid || filteredRangeMeters < 0.0f)
  {
    rangeMeters = newRangeM;
    filteredRangeMeters = newRangeM;
    previousFilteredRangeM = newRangeM;
    rangeDerivedVerticalVelocityMps = 0.0f;
    previousValidRangeUs = nowUs;
  }
  else
  {
    rangeMeters = newRangeM;

    static constexpr float RANGE_FILTER_ALPHA = 0.30f;
    filteredRangeMeters +=
      RANGE_FILTER_ALPHA * (newRangeM - filteredRangeMeters);

    const float rangeDt =
      static_cast<float>(nowUs - previousValidRangeUs) * 1.0e-6f;

    if (rangeDt >= 0.005f && rangeDt <= 0.200f)
    {
      const float rawVerticalVelocity =
        (filteredRangeMeters - previousFilteredRangeM) / rangeDt;

      static constexpr float VERTICAL_VELOCITY_FILTER_ALPHA = 0.25f;
      rangeDerivedVerticalVelocityMps +=
        VERTICAL_VELOCITY_FILTER_ALPHA *
        (rawVerticalVelocity - rangeDerivedVerticalVelocityMps);
    }
    else
    {
      rangeDerivedVerticalVelocityMps = 0.0f;
    }

    previousFilteredRangeM = filteredRangeMeters;
    previousValidRangeUs = nowUs;
  }

  // Asynchronous ESKF measurement updates.  The raw sensor-to-floor range is
  // intentionally retained to match the existing altitude-control convention.
  const bool estimatorAcceptedAltitude = stateEstimator.updateAltitude(
    filteredRangeMeters,
    ESKF_TOF_STD_M
  );

  if (estimatorAcceptedAltitude)
  {
    eskfRangeReady = true;
    lastEskfRangeAcceptedMs = nowMs;

    if (previousValidRangeUs != 0U &&
        isfinite(rangeDerivedVerticalVelocityMps))
    {
      stateEstimator.updateVerticalVelocity(
        rangeDerivedVerticalVelocityMps,
        ESKF_RANGE_VELOCITY_STD_MPS
      );
    }

    synchronizeEstimatorState();
  }

  const bool wasStable = rangeRecoveryStable;

  rangeHasEverBeenValid = true;
  lastValidRangeMs = nowMs;

  if (consecutiveValidRangeSamples < RANGE_RECOVERY_SAMPLES)
  {
    consecutiveValidRangeSamples++;
  }

  rangeRecoveryStable =
    consecutiveValidRangeSamples >= RANGE_RECOVERY_SAMPLES;

  if (!wasStable && rangeRecoveryStable && rangeLongFailureLatched)
  {
    rangeJustRecovered = true;
    rangeLongFailureLatched = false;
  }
}


void printRangeStatus()
{
  const uint32_t nowMs = millis();

  runtimeLogf(
    "RNGDBG,POLL:%lu,GOOD:%lu,STATFAIL:%lu,NORDY:%lu,READFAIL:%lu,INVALID:%lu,STAT_OK:%d,STAT:0x%02X,RANGE_OK:%d,AGE:%lu,RANGE_M:%.3f",
    static_cast<unsigned long>(rangePollCount),
    static_cast<unsigned long>(rangeGoodCount),
    static_cast<unsigned long>(rangeStatusReadFailCount),
    static_cast<unsigned long>(rangeNotReadyCount),
    static_cast<unsigned long>(rangeReadFailCount),
    static_cast<unsigned long>(rangeInvalidValueCount),
    rangeLastInterruptStatusValid ? 1 : 0,
    static_cast<unsigned>(rangeLastInterruptStatus),
    rangeControlUsable(nowMs) ? 1 : 0,
    static_cast<unsigned long>(rangeAgeMs(nowMs)),
    filteredRangeMeters
  );
}

bool probeI2cAddress(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool readI2cRegister8(
  uint8_t address,
  uint8_t registerAddress,
  uint8_t &value
)
{
  Wire.beginTransmission(address);
  Wire.write(registerAddress);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1)
  {
    while (Wire.available() > 0) Wire.read();
    return false;
  }

  value = Wire.read();
  return true;
}

void printI2cDiagnostics()
{
  if (!motorsCompletelyStoppedForSensorService())
  {
    runtimeLogLine("I2CDIAG REJECTED: motors/ramp must be completely stopped.");
    return;
  }

  const bool mpuAck = probeI2cAddress(MPU_ADDR);
  const bool rangeAck = probeI2cAddress(VL53L0X_ADDR);
  const bool magAck = probeI2cAddress(MAG_ADDR);

  runtimeLogf(
    "I2C,MPU:%s,VL53:%s,QMC:%s",
    mpuAck ? "ACK" : "NACK",
    rangeAck ? "ACK" : "NACK",
    magAck ? "ACK" : "NACK"
  );

  uint8_t mpuWhoAmI = 0;
  uint8_t rangeInterrupt = 0;
  uint8_t magChipId = 0;
  uint8_t magStatus = 0;
  uint8_t magCtrl1 = 0;

  const bool mpuWhoOk =
    readI2cRegister8(MPU_ADDR, 0x75, mpuWhoAmI);
  const bool rangeStatusOk =
    readI2cRegister8(
      VL53L0X_ADDR,
      VL53_RESULT_INTERRUPT_STATUS,
      rangeInterrupt
    );
  const bool magIdOk =
    readI2cRegister8(MAG_ADDR, MAG_CHIP_ID_REG, magChipId);
  const bool magStatusOk =
    readI2cRegister8(MAG_ADDR, MAG_STATUS_REG, magStatus);
  const bool magCtrl1Ok =
    readI2cRegister8(MAG_ADDR, MAG_CTRL1_REG, magCtrl1);

  if (magStatusOk)
  {
    magLastStatus = magStatus;
    magLastStatusValid = true;
  }

  if (magCtrl1Ok)
  {
    magLastCtrl1 = magCtrl1;
    magLastCtrl1Valid = true;
  }

  if (rangeStatusOk)
  {
    rangeLastInterruptStatus = rangeInterrupt;
    rangeLastInterruptStatusValid = true;
  }

  runtimeLogf(
    "I2CREG,MPU_WHO_OK:%d,MPU_WHO:0x%02X,VL53_STAT_OK:%d,VL53_STAT:0x%02X,QMC_ID_OK:%d,QMC_ID:0x%02X,QMC_STAT_OK:%d,QMC_STAT:0x%02X,QMC_CTRL1_OK:%d,QMC_CTRL1:0x%02X",
    mpuWhoOk ? 1 : 0,
    static_cast<unsigned>(mpuWhoAmI),
    rangeStatusOk ? 1 : 0,
    static_cast<unsigned>(rangeInterrupt),
    magIdOk ? 1 : 0,
    static_cast<unsigned>(magChipId),
    magStatusOk ? 1 : 0,
    static_cast<unsigned>(magStatus),
    magCtrl1Ok ? 1 : 0,
    static_cast<unsigned>(magCtrl1)
  );
}

// ================================================================
// PMW3901 OPPOSITE-CORE ACQUISITION + CORE-1 ESTIMATOR UPDATE
// ================================================================

void snapshotAndResetFlowGyroDeltas(
  float &rollDeltaRad,
  float &pitchDeltaRad,
  float &yawDeltaCwRad
)
{
  portENTER_CRITICAL(&flowGyroMux);
  rollDeltaRad = accumulatedFlowRollRad;
  pitchDeltaRad = accumulatedFlowPitchRad;
  yawDeltaCwRad = accumulatedFlowYawCwRad;
  accumulatedFlowRollRad = 0.0f;
  accumulatedFlowPitchRad = 0.0f;
  accumulatedFlowYawCwRad = 0.0f;
  portEXIT_CRITICAL(&flowGyroMux);
}

// setup() uses this only before the opposite-core task is started. Draining
// the PMW counters during the five-second DShot-stop period prevents a large
// first delta when the runtime task begins.
void drainOpticalFlowDuringSetup(uint32_t nowUs)
{
  if (
    static_cast<uint32_t>(nowUs - previousSetupFlowDrainUs) <
    FLOW_PERIOD_US
  )
  {
    return;
  }

  previousSetupFlowDrainUs = nowUs;
  int16_t ignoredX = 0;
  int16_t ignoredY = 0;
  flow.readMotionCount(&ignoredX, &ignoredY);
}

static int16_t makeFlowSigned16(uint8_t lowByte, uint8_t highByte)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(lowByte) |
    (static_cast<uint16_t>(highByte) << 8U)
  );
}

bool readPmwMotionBurst(OpticalFlowRawSample &sample)
{
  uint8_t raw[12] = {};

  SPI.beginTransaction(SPISettings(FLOW_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(FLOW_CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.transfer(0x16U); // PMW3901 Motion_Burst register
  delayMicroseconds(50);

  for (size_t i = 0; i < sizeof(raw); ++i)
  {
    raw[i] = SPI.transfer(0x00U);
  }

  delayMicroseconds(50);
  digitalWrite(FLOW_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);

  sample.motion = raw[0];
  sample.observation = raw[1];
  sample.rawDeltaX = makeFlowSigned16(raw[2], raw[3]);
  sample.rawDeltaY = makeFlowSigned16(raw[4], raw[5]);
  sample.squal = raw[6];
  sample.rawDataSum = raw[7];
  sample.maxRawData = raw[8];
  sample.minRawData = raw[9];
  sample.shutter = static_cast<uint16_t>(
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

  sample.transferPlausible = !allZero && !allOnes &&
    abs(static_cast<int>(sample.rawDeltaX)) <= 1000 &&
    abs(static_cast<int>(sample.rawDeltaY)) <= 1000;

  return sample.transferPlausible;
}

void mapFlowCountsToBody(
  int16_t rawX,
  int16_t rawY,
  float &bodyXCounts,
  float &bodyYCounts
)
{
  switch (flowAxisMap)
  {
    case FlowAxisMap::PlusXPlusY:
      bodyXCounts = +rawX; bodyYCounts = +rawY; break;
    case FlowAxisMap::PlusXMinusY:
      bodyXCounts = +rawX; bodyYCounts = -rawY; break;
    case FlowAxisMap::MinusXPlusY:
      bodyXCounts = -rawX; bodyYCounts = +rawY; break;
    case FlowAxisMap::MinusXMinusY:
      bodyXCounts = -rawX; bodyYCounts = -rawY; break;
    case FlowAxisMap::PlusYPlusX:
      bodyXCounts = +rawY; bodyYCounts = +rawX; break;
    case FlowAxisMap::PlusYMinusX:
      bodyXCounts = +rawY; bodyYCounts = -rawX; break;
    case FlowAxisMap::MinusYPlusX:
      bodyXCounts = -rawY; bodyYCounts = +rawX; break;
    case FlowAxisMap::MinusYMinusX:
      bodyXCounts = -rawY; bodyYCounts = -rawX; break;
  }
}

void opticalFlowTaskEntry(void *parameter)
{
  (void)parameter;

  TickType_t lastWakeTick = xTaskGetTickCount();
  uint32_t previousSampleUs = micros();
  uint32_t sequence = 0;

  for (;;)
  {
    // 100 Hz PMW3901 service. This task is intentionally low priority and is
    // pinned to the core opposite the Arduino flight loop.
    vTaskDelayUntil(
      &lastWakeTick,
      pdMS_TO_TICKS(FLOW_PERIOD_US / 1000U)
    );

    const uint32_t ioStartUs = micros();

    OpticalFlowRawSample sample;
    readPmwMotionBurst(sample);

    // Preserve the original compensation timing: snapshot/reset integrated
    // gyro angle immediately AFTER readMotionCount() completes.
    snapshotAndResetFlowGyroDeltas(
      sample.rollDeltaRad,
      sample.pitchDeltaRad,
      sample.yawDeltaCwRad
    );

    sample.sampleUs = micros();
    sample.sampleMs = millis();
    sample.intervalUs =
      static_cast<uint32_t>(sample.sampleUs - previousSampleUs);
    previousSampleUs = sample.sampleUs;
    sample.sequence = ++sequence;

    const uint32_t ioDurationUs =
      static_cast<uint32_t>(micros() - ioStartUs);

    portENTER_CRITICAL(&flowTaskStatsMux);
    opticalFlowTaskStats.ioTotalUs += ioDurationUs;
    opticalFlowTaskStats.ioSamples++;
    if (ioDurationUs > opticalFlowTaskStats.ioMaxUs)
    {
      opticalFlowTaskStats.ioMaxUs = ioDurationUs;
    }
    if (sample.intervalUs > 0U)
    {
      opticalFlowTaskStats.intervalTotalUs += sample.intervalUs;
      opticalFlowTaskStats.intervalSamples++;
    }
    if (sample.transferPlausible) opticalFlowTaskStats.plausibleTransfers++;
    if (sample.motion == 0xB0U) opticalFlowTaskStats.motionB0Samples++;
    opticalFlowTaskStats.squalTotal += sample.squal;
    if (sample.shutter > opticalFlowTaskStats.maximumShutter)
    {
      opticalFlowTaskStats.maximumShutter = sample.shutter;
    }
    portEXIT_CRITICAL(&flowTaskStatsMux);

    // One-slot latest-value queue: the PMW task never waits for Core 1. If
    // Core 1 falls behind, the newer sample replaces the older one.
    if (opticalFlowSampleQueue != nullptr)
    {
      xQueueOverwrite(opticalFlowSampleQueue, &sample);
    }
  }
}

bool startOpticalFlowTask(BaseType_t coreId)
{
  if (opticalFlowTaskStarted)
  {
    return true;
  }

  opticalFlowSampleQueue = xQueueCreate(1, sizeof(OpticalFlowRawSample));
  if (opticalFlowSampleQueue == nullptr)
  {
    return false;
  }

  opticalFlowTaskCore = coreId;

  const BaseType_t taskResult = xTaskCreatePinnedToCore(
    opticalFlowTaskEntry,
    "PMW3901_100Hz",
    4096,
    nullptr,
    1,
    &opticalFlowTaskHandle,
    coreId
  );

  if (taskResult != pdPASS)
  {
    vQueueDelete(opticalFlowSampleQueue);
    opticalFlowSampleQueue = nullptr;
    opticalFlowTaskHandle = nullptr;
    opticalFlowTaskCore = -1;
    return false;
  }

  opticalFlowTaskStarted = true;
  return true;
}

void processOpticalFlowSample(
  const OpticalFlowRawSample &sample,
  uint32_t nowMs
)
{
  const float flowDt =
    static_cast<float>(sample.intervalUs) * 1.0e-6f;

  lastFlowMotion = sample.motion;
  lastFlowSqual = sample.squal;
  lastFlowShutter = sample.shutter;

  float bodyXCounts = 0.0f;
  float bodyYCounts = 0.0f;
  mapFlowCountsToBody(
    sample.rawDeltaX,
    sample.rawDeltaY,
    bodyXCounts,
    bodyYCounts
  );

  const bool timingValid = flowDt >= 0.006f && flowDt <= 0.030f;
  const bool countsValid =
    fabsf(bodyXCounts) < static_cast<float>(FLOW_OUTLIER_LIMIT_COUNTS) &&
    fabsf(bodyYCounts) < static_cast<float>(FLOW_OUTLIER_LIMIT_COUNTS);

  const bool heightValid =
    rangeControlUsable(nowMs) &&
    filteredRangeMeters >= MIN_FLOW_RANGE_M &&
    filteredRangeMeters <= MAX_FLOW_RANGE_M;

  const bool motionOccurred = (sample.motion & 0x80U) != 0U;
  const bool motionFormatPlausible =
    sample.motion == 0x30U || sample.motion == 0xB0U;

  // Sensor/path health is deliberately broader than "new motion available".
  // 0x30 is a coherent PMW burst with no new motion; it must not reset the
  // health streak or disable XY hold by itself.
  const bool qualityHealthy =
    sample.transferPlausible &&
    motionFormatPlausible &&
    sample.squal >= MIN_FLOW_SQUAL &&
    sample.shutter > 0U;

  const bool healthSampleValid =
    timingValid && countsValid && heightValid && qualityHealthy;

  if (!timingValid) flowInvalidTimingCount++;
  if (!countsValid) flowInvalidCountCount++;
  if (!heightValid) flowInvalidHeightCount++;
  if (!qualityHealthy) flowInvalidQualityCount++;

  if (!healthSampleValid)
  {
    lastFlowSampleValid = false;
    consecutiveHealthyFlowSamples = 0;

    // Do not instantly call one bad burst a dead sensor. Health expires only
    // after the configured age, so an isolated bad transfer cannot flap XY.
    if (lastHealthyFlowMs == 0U ||
        static_cast<uint32_t>(nowMs - lastHealthyFlowMs) >
          FLOW_HEALTH_LOSS_MS)
    {
      flowSensorHealthy = false;
    }

    measuredBodyVelocityX_mps *= FLOW_INVALID_DECAY;
    measuredBodyVelocityY_mps *= FLOW_INVALID_DECAY;

    if (xyHoldActive && !flowHealthFresh(nowMs))
    {
      deactivateXYHold();
      postFlightEvent(
        AsyncFlightEventType::XyHoldDisabledFlowTimeout,
        nowMs
      );
    }
    return;
  }

  lastHealthyFlowMs = nowMs;
  if (consecutiveHealthyFlowSamples < FLOW_HEALTH_RECOVERY_SAMPLES)
  {
    consecutiveHealthyFlowSamples++;
  }

  if (consecutiveHealthyFlowSamples >= FLOW_HEALTH_RECOVERY_SAMPLES)
  {
    flowSensorHealthy = true;
  }

  // A healthy 0x30 burst is NOT a zero-velocity observation. Keep sensor health
  // alive, leave the most recent accepted velocity untouched, and wait for the
  // next actual motion measurement.
  if (!motionOccurred || sample.motion != 0xB0U)
  {
    ++flowNoMotionCount;
    lastFlowSampleValid = false;
    return;
  }

  float deltaBodyX_m =
    bodyXCounts * FLOW_RAD_PER_COUNT * filteredRangeMeters * FLOW_SCALE_X;

  float deltaBodyY_m =
    bodyYCounts * FLOW_RAD_PER_COUNT * filteredRangeMeters * FLOW_SCALE_Y;

  if (ENABLE_FLOW_GYRO_COMPENSATION)
  {
    deltaBodyX_m += FLOW_ROLL_COMP_GAIN *
      filteredRangeMeters * sample.rollDeltaRad;

    deltaBodyY_m += FLOW_PITCH_COMP_GAIN *
      filteredRangeMeters * sample.pitchDeltaRad;
  }

  // Rigid-body omega x r correction using the physically measured PMW
  // placement. The hand-yaw tests were intentionally not used to invent a
  // different lever arm because yaw also rotates the floor image.
  deltaBodyX_m -= PMW_OFFSET_Y_M * sample.yawDeltaCwRad;
  deltaBodyY_m += PMW_OFFSET_X_M * sample.yawDeltaCwRad;

  const float instantBodyVelocityX = deltaBodyX_m / flowDt;
  const float instantBodyVelocityY = deltaBodyY_m / flowDt;
  lastInstantFlowVelocityX_mps = instantBodyVelocityX;
  lastInstantFlowVelocityY_mps = instantBodyVelocityY;

  const float speedSquared =
    instantBodyVelocityX * instantBodyVelocityX +
    instantBodyVelocityY * instantBodyVelocityY;

  if (
    !isfinite(instantBodyVelocityX) ||
    !isfinite(instantBodyVelocityY) ||
    speedSquared >
      MAX_PLAUSIBLE_FLOW_SPEED_MPS * MAX_PLAUSIBLE_FLOW_SPEED_MPS
  )
  {
    lastFlowSampleValid = false;
    flowInvalidSpeedCount++;
    return;
  }

  // Use the gyro angle integrated over this exact PMW read interval to decide
  // how much to trust the velocity measurement. Normal heading-hold hover
  // should stay in the low-yaw region.
  lastFlowYawRateDps =
    fabsf(sample.yawDeltaCwRad) * RAD_TO_DEG_F / flowDt;

  if (!isfinite(lastFlowYawRateDps))
  {
    lastFlowSampleValid = false;
    flowYawRateRejectCount++;
    return;
  }

  if (lastFlowYawRateDps >= FLOW_YAW_RATE_HARD_DPS)
  {
    // Measurement rejected, sensor health preserved.
    lastFlowSampleValid = false;
    flowYawRateRejectCount++;
    lastFlowYawNoiseScale = FLOW_YAW_NOISE_SCALE_MAX;
    return;
  }

  float yawNoiseScale = 1.0f;
  if (lastFlowYawRateDps > FLOW_YAW_RATE_SOFT_DPS)
  {
    const float blend = clampFloat(
      (lastFlowYawRateDps - FLOW_YAW_RATE_SOFT_DPS) /
        (FLOW_YAW_RATE_HARD_DPS - FLOW_YAW_RATE_SOFT_DPS),
      0.0f,
      1.0f
    );

    yawNoiseScale =
      1.0f + blend * (FLOW_YAW_NOISE_SCALE_MAX - 1.0f);
    flowYawRateDownweightCount++;
  }
  lastFlowYawNoiseScale = yawNoiseScale;

  measuredBodyVelocityX_mps +=
    FLOW_VELOCITY_FILTER_ALPHA *
    (instantBodyVelocityX - measuredBodyVelocityX_mps);

  measuredBodyVelocityY_mps +=
    FLOW_VELOCITY_FILTER_ALPHA *
    (instantBodyVelocityY - measuredBodyVelocityY_mps);

  // Crazyflie/Espressif adapt pixel standard deviation from shutter. Convert
  // that pixel/count uncertainty into m/s for this estimator interface.
  const float pixelStd = fmaxf(
    0.10f,
    0.0007984f * static_cast<float>(sample.shutter) + 0.4335f
  );

  const float baseFlowVelocityStd = clampFloat(
    pixelStd * FLOW_RAD_PER_COUNT * filteredRangeMeters / flowDt,
    0.04f,
    0.50f
  );

  const float flowVelocityStd = clampFloat(
    baseFlowVelocityStd * yawNoiseScale,
    0.04f,
    1.50f
  );

  // Commissioning-safe update: flow corrects translational velocity only.
  // It does not alter the quaternion/attitude state. Both X and Y must pass
  // their innovation gates before this B0 measurement is considered accepted.
  lastFlowAxisAcceptedMask =
    stateEstimator.updateBodyVelocityVelocityOnly(
      measuredBodyVelocityX_mps,
      measuredBodyVelocityY_mps,
      flowVelocityStd,
      flowVelocityStd
    );

  if (lastFlowAxisAcceptedMask == 0x03U)
  {
    lastFlowSampleValid = true;
    eskfFlowVelocityReady = true;
    fusedVelocityInitialized = true;
    synchronizeEstimatorState();

      lastFlowMeasurementMs = nowMs;

    if (consecutiveValidFlowSamples < 255U)
    {
      consecutiveValidFlowSamples++;
    }
  }
  else
  {
    lastFlowSampleValid = false;
    consecutiveValidFlowSamples = 0;
    flowEstimatorRejectCount++;
  }
}

bool consumeOpticalFlowSample(uint32_t nowMs)
{
  if (opticalFlowSampleQueue == nullptr)
  {
    return false;
  }

  OpticalFlowRawSample sample;
  if (xQueueReceive(opticalFlowSampleQueue, &sample, 0) != pdTRUE)
  {
    return false;
  }

  if (lastProcessedFlowSequence != 0U)
  {
    const uint32_t sequenceDelta =
      static_cast<uint32_t>(sample.sequence - lastProcessedFlowSequence);
    if (sequenceDelta > 1U && sequenceDelta < 0x80000000UL)
    {
      skippedFlowSamplesWindow += sequenceDelta - 1U;
    }
  }
  lastProcessedFlowSequence = sample.sequence;

  processOpticalFlowSample(sample, nowMs);
  return true;
}

// ================================================================
// GENERIC RATE PID
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
  float integralLimit,
  float dt,
  float &pTerm,
  float &iTerm,
  float &dTerm
)
{
  const float error = desiredRate - measuredRate;

  integral += error * dt;
  integral = clampFloat(integral, -integralLimit, integralLimit);

  const float rawDerivative = (error - previousError) / dt;
  previousError = error;

  filteredDerivative +=
    DERIVATIVE_FILTER_ALPHA *
    (rawDerivative - filteredDerivative);

  pTerm = kp * error;
  iTerm = ki * integral;
  dTerm = kd * filteredDerivative;

  return pTerm + iTerm + dTerm;
}

// ================================================================
// OUTER X/Y CONTROLLER
// ================================================================

void activateXYHold(uint32_t nowMs)
{
  xyHoldActive = true;
  targetNavX_m = positionNavX_m;
  targetNavY_m = positionNavY_m;
  xyHoldBlendStartMs = nowMs;

  filteredHoldRollAngleDeg = measuredRollAngleDeg;
  filteredHoldPitchAngleDeg = measuredPitchAngleDeg;

  postFlightEvent(
    AsyncFlightEventType::XyHoldActivated,
    nowMs,
    positionNavX_m,
    positionNavY_m
  );
}

void updateHorizontalController(float dt, uint32_t nowMs)
{
  correctedRollStick = applyCenteredStickDeadband(
    controlData.roll,
    ROLL_STICK_DEADBAND
  );

  correctedPitchStick = applyCenteredStickDeadband(
    controlData.pitch,
    PITCH_STICK_DEADBAND
  );

  commandedBodyVelocityX_mps =
    correctedRollStick * MAX_RIGHT_VELOCITY_MPS;

  commandedBodyVelocityY_mps =
    correctedPitchStick * MAX_FORWARD_VELOCITY_MPS;

  const float manualRollAngleDeg =
    -correctedRollStick * MAX_ROLL_ANGLE_DEG;

  const float manualPitchAngleDeg =
    correctedPitchStick * MAX_PITCH_ANGLE_DEG;

  const bool flowRecovered =
    flowHealthFresh(nowMs) &&
    consecutiveHealthyFlowSamples >= FLOW_HEALTH_RECOVERY_SAMPLES;

  const bool flowMeasurementFresh =
    flowMeasurementFreshForArm(nowMs);

  const float horizontalSpeedMps = sqrtf(
    measuredBodyVelocityX_mps * measuredBodyVelocityX_mps +
    measuredBodyVelocityY_mps * measuredBodyVelocityY_mps
  );

  const bool armConditionsStable =
    xyHoldCommissioned &&
    flightEnabled &&
    !enableRampActive &&
    !disableRampActive &&
    altitudeHoldActive &&
    flowRecovered &&
    flowMeasurementFresh &&
    lastFlowAxisAcceptedMask == 0x03U &&
    eskfStateHealthy &&
    eskfFlowVelocityReady &&
    rangeControlUsable(nowMs) &&
    filteredRangeMeters >= XY_HOLD_ACTIVATION_RANGE_M &&
    fabsf(verticalVelocityMps) <= XY_ARM_MAX_VERTICAL_SPEED_MPS &&
    horizontalSpeedMps <= XY_ARM_MAX_HORIZONTAL_SPEED_MPS &&
    fabsf(measuredYawRateDps) <= XY_ARM_MAX_YAW_RATE_DPS &&
    fabsf(measuredRollAngleDeg) <= XY_ARM_MAX_TILT_DEG &&
    fabsf(measuredPitchAngleDeg) <= XY_ARM_MAX_TILT_DEG;

  if (!xyHoldActive)
  {
    if (armConditionsStable)
    {
      if (xyArmStableStartMs == 0U)
      {
        xyArmStableStartMs = nowMs;
      }
      else if (
        static_cast<uint32_t>(nowMs - xyArmStableStartMs) >=
          XY_ARM_STABLE_DWELL_MS
      )
      {
        activateXYHold(nowMs);
      }
    }
    else
    {
      xyArmStableStartMs = 0U;
    }
  }

  const bool abortConditions =
    !xyHoldCommissioned ||
    !rangeControlUsable(nowMs) ||
    !eskfStateHealthy ||
    !eskfFlowVelocityReady ||
    !flowHealthFresh(nowMs) ||
    fabsf(verticalVelocityMps) > XY_ABORT_MAX_VERTICAL_SPEED_MPS ||
    fabsf(measuredYawRateDps) > XY_ABORT_MAX_YAW_RATE_DPS ||
    fabsf(measuredRollAngleDeg) > XY_ABORT_MAX_TILT_DEG ||
    fabsf(measuredPitchAngleDeg) > XY_ABORT_MAX_TILT_DEG;

  if (xyHoldActive && abortConditions)
  {
    deactivateXYHold();
  }

  if (!xyHoldActive)
  {
    desiredRollAngleDeg = manualRollAngleDeg;
    desiredPitchAngleDeg = manualPitchAngleDeg;
    controllerBodyVelocityTargetX_mps = commandedBodyVelocityX_mps;
    controllerBodyVelocityTargetY_mps = commandedBodyVelocityY_mps;
    return;
  }

  float commandedNavVelocityX = 0.0f;
  float commandedNavVelocityY = 0.0f;

  bodyToNavigation(
    commandedBodyVelocityX_mps,
    commandedBodyVelocityY_mps,
    commandedNavVelocityX,
    commandedNavVelocityY
  );

  targetNavX_m += commandedNavVelocityX * dt;
  targetNavY_m += commandedNavVelocityY * dt;

  const float limitedPositionErrorX = clampFloat(
    targetNavX_m - positionNavX_m,
    -MAX_POSITION_TARGET_ERROR_M,
    MAX_POSITION_TARGET_ERROR_M
  );

  const float limitedPositionErrorY = clampFloat(
    targetNavY_m - positionNavY_m,
    -MAX_POSITION_TARGET_ERROR_M,
    MAX_POSITION_TARGET_ERROR_M
  );

  const float positionErrorX = applySymmetricDeadband(
    limitedPositionErrorX,
    XY_POSITION_DEADBAND_M
  );

  const float positionErrorY = applySymmetricDeadband(
    limitedPositionErrorY,
    XY_POSITION_DEADBAND_M
  );

  float desiredNavVelocityX =
    commandedNavVelocityX + xyPositionKp * positionErrorX;

  float desiredNavVelocityY =
    commandedNavVelocityY + xyPositionKp * positionErrorY;

  limitVector2(
    desiredNavVelocityX,
    desiredNavVelocityY,
    MAX_CONTROLLER_VELOCITY_MPS
  );

  navigationToBody(
    desiredNavVelocityX,
    desiredNavVelocityY,
    controllerBodyVelocityTargetX_mps,
    controllerBodyVelocityTargetY_mps
  );

  // Use the direct, quality-gated flow velocity in the fast inner navigation
  // loop. The ESKF position remains the slow outer-loop reference.
  const float bodyVelocityErrorX =
    controllerBodyVelocityTargetX_mps - measuredBodyVelocityX_mps;

  const float bodyVelocityErrorY =
    controllerBodyVelocityTargetY_mps - measuredBodyVelocityY_mps;

  float requestedAccelerationX = xyVelocityKp * bodyVelocityErrorX;
  float requestedAccelerationY = xyVelocityKp * bodyVelocityErrorY;
  limitVector2(
    requestedAccelerationX,
    requestedAccelerationY,
    MAX_XY_ACCELERATION_MPS2
  );

  const float stickMagnitude = fmaxf(
    fabsf(correctedRollStick),
    fabsf(correctedPitchStick)
  );

  const float accelerationAssistScale = clampFloat(
    1.0f - stickMagnitude,
    0.0f,
    1.0f
  );

  const float rollAccelerationAssistDeg = clampFloat(
    accelerationAssistScale * xyAccelerationKp *
      filteredLinearAccelBodyX_mps2,
    -MAX_XY_ACCEL_ASSIST_ANGLE_DEG,
    MAX_XY_ACCEL_ASSIST_ANGLE_DEG
  );

  const float pitchAccelerationAssistDeg = clampFloat(
    -accelerationAssistScale * xyAccelerationKp *
      filteredLinearAccelBodyY_mps2,
    -MAX_XY_ACCEL_ASSIST_ANGLE_DEG,
    MAX_XY_ACCEL_ASSIST_ANGLE_DEG
  );

  const float rawHoldRollAngleDeg = clampFloat(
    -atan2f(requestedAccelerationX, STANDARD_GRAVITY_MPS2) *
      RAD_TO_DEG_F + rollAccelerationAssistDeg,
    -MAX_XY_CONTROLLER_ANGLE_DEG,
    MAX_XY_CONTROLLER_ANGLE_DEG
  );

  const float rawHoldPitchAngleDeg = clampFloat(
    atan2f(requestedAccelerationY, STANDARD_GRAVITY_MPS2) *
      RAD_TO_DEG_F + pitchAccelerationAssistDeg,
    -MAX_XY_CONTROLLER_ANGLE_DEG,
    MAX_XY_CONTROLLER_ANGLE_DEG
  );

  const float commandFilterAlpha = clampFloat(
    dt / (XY_COMMAND_FILTER_TAU_S + dt),
    0.0f,
    1.0f
  );

  const float maximumAngleStep = XY_ANGLE_SLEW_RATE_DPS * dt;

  const float requestedRollStep = clampFloat(
    commandFilterAlpha *
      (rawHoldRollAngleDeg - filteredHoldRollAngleDeg),
    -maximumAngleStep,
    maximumAngleStep
  );

  const float requestedPitchStep = clampFloat(
    commandFilterAlpha *
      (rawHoldPitchAngleDeg - filteredHoldPitchAngleDeg),
    -maximumAngleStep,
    maximumAngleStep
  );

  filteredHoldRollAngleDeg = clampFloat(
    filteredHoldRollAngleDeg + requestedRollStep,
    -MAX_XY_CONTROLLER_ANGLE_DEG,
    MAX_XY_CONTROLLER_ANGLE_DEG
  );

  filteredHoldPitchAngleDeg = clampFloat(
    filteredHoldPitchAngleDeg + requestedPitchStep,
    -MAX_XY_CONTROLLER_ANGLE_DEG,
    MAX_XY_CONTROLLER_ANGLE_DEG
  );

  const float blend = clampFloat(
    static_cast<float>(nowMs - xyHoldBlendStartMs) /
      static_cast<float>(XY_HOLD_BLEND_MS),
    0.0f,
    1.0f
  );

  desiredRollAngleDeg = manualRollAngleDeg +
    blend * (filteredHoldRollAngleDeg - manualRollAngleDeg);

  desiredPitchAngleDeg = manualPitchAngleDeg +
    blend * (filteredHoldPitchAngleDeg - manualPitchAngleDeg);
}

// ================================================================
// ALTITUDE CONTROLLER
// ================================================================

void activateAltitudeHold(uint32_t nowMs, int manualBaseDshot)
{
  altitudeHoldActive = true;
  altitudeTargetM = altitudeForControl();
  altitudeVelocityIntegral = 0.0f;
  altitudeCorrectionDshot = 0.0f;
  desiredVerticalVelocityMps = 0.0f;

  learnedHoverDshot = clampInt(
    manualBaseDshot,
    HOVER_DSHOT_MIN,
    HOVER_DSHOT_MAX
  );

  altitudeBlendStartDshot = manualBaseDshot;
  altitudeBlendStartMs = nowMs;
  lastAltitudeBaseDshot = manualBaseDshot;
  rangeJustRecovered = false;

  postFlightEvent(
    AsyncFlightEventType::AltitudeHoldActivated,
    nowMs,
    altitudeTargetM,
    0.0f,
    learnedHoverDshot
  );
}

void updateAltitudeActivation(uint32_t nowMs, int manualBaseDshot)
{
  if (
    altitudeHoldActive ||
    enableRampActive ||
    disableRampActive ||
    !flightEnabled
  )
  {
    altitudeActivationStartMs = 0;
    return;
  }

  const bool activationConditions =
    rangeRecoveryStable &&
    eskfStateHealthy &&
    rangeControlUsable(nowMs) &&
    filteredRangeMeters >= ALTITUDE_HOLD_ACTIVATION_M;

  if (!activationConditions)
  {
    altitudeActivationStartMs = 0;
    return;
  }

  if (altitudeActivationStartMs == 0)
  {
    altitudeActivationStartMs = nowMs;
    return;
  }

  if (
    static_cast<uint32_t>(nowMs - altitudeActivationStartMs) >=
      ALTITUDE_ACTIVATION_DWELL_MS
  )
  {
    activateAltitudeHold(nowMs, manualBaseDshot);
    altitudeActivationStartMs = 0;
  }
}

int calculateAltitudeBaseDshot(float dt, uint32_t nowMs, int manualBaseDshot)
{
  correctedThrottleStick = applyCenteredStickDeadband(
    controlData.throttle,
    THROTTLE_STICK_DEADBAND
  );

  updateAltitudeActivation(nowMs, manualBaseDshot);

  // A long range outage always overrides manual or altitude-hold throttle
  // while the flight is enabled. This is the requested slow-descent fallback.
  if (rangeFailureIsLong(nowMs))
  {
    rangeLongFailureLatched = true;
    consecutiveValidRangeSamples = 0;
    rangeRecoveryStable = false;

    altitudeVelocityIntegral = 0.0f;
    altitudeCorrectionDshot = 0.0f;
    desiredVerticalVelocityMps = -MAX_VERTICAL_VELOCITY_MPS;
    lastAltitudeBaseDshot = RANGE_FAILURE_DESCENT_DSHOT;
    return RANGE_FAILURE_DESCENT_DSHOT;
  }

  if (rangeLongFailureLatched && !rangeRecoveryStable)
  {
    // A new reading has appeared, but wait for five consecutive good samples
    // before leaving the controlled-descent fallback.
    lastAltitudeBaseDshot = RANGE_FAILURE_DESCENT_DSHOT;
    return RANGE_FAILURE_DESCENT_DSHOT;
  }

  if (!altitudeHoldActive)
  {
    lastAltitudeBaseDshot = manualBaseDshot;
    return manualBaseDshot;
  }

  if (rangeAgeMs(nowMs) > RANGE_CONTROL_FREEZE_MS)
  {
    // Short transient: freeze target and controller state, and preserve the
    // previous altitude command until data returns or 500 ms is exceeded.
    return lastAltitudeBaseDshot;
  }

  if (!rangeControlUsable(nowMs))
  {
    return lastAltitudeBaseDshot;
  }

  if (rangeJustRecovered)
  {
    altitudeTargetM = altitudeForControl();
    altitudeVelocityIntegral = 0.0f;
    altitudeCorrectionDshot = 0.0f;
    altitudeBlendStartDshot = lastAltitudeBaseDshot;
    altitudeBlendStartMs = nowMs;
    rangeJustRecovered = false;

    postFlightEvent(
      AsyncFlightEventType::RangeRecovered,
      nowMs,
      altitudeTargetM
    );
  }

  const float pilotVerticalVelocity =
    correctedThrottleStick * MAX_VERTICAL_VELOCITY_MPS;

  altitudeTargetM += pilotVerticalVelocity * dt;
  altitudeTargetM = clampFloat(
    altitudeTargetM,
    ALTITUDE_TARGET_MIN_M,
    ALTITUDE_TARGET_MAX_M
  );

  const float estimatedAltitudeM = altitudeForControl();
  const float altitudeError = altitudeTargetM - estimatedAltitudeM;

  desiredVerticalVelocityMps = clampFloat(
    pilotVerticalVelocity + altitudePositionKp * altitudeError,
    -MAX_ALTITUDE_VELOCITY_TARGET_MPS,
    MAX_ALTITUDE_VELOCITY_TARGET_MPS
  );

  const float verticalVelocityError =
    desiredVerticalVelocityMps - verticalVelocityMps;

  altitudeVelocityIntegral += verticalVelocityError * dt;
  altitudeVelocityIntegral = clampFloat(
    altitudeVelocityIntegral,
    -ALTITUDE_INTEGRAL_LIMIT,
    ALTITUDE_INTEGRAL_LIMIT
  );

  altitudeCorrectionDshot =
    altitudeVelocityKp * verticalVelocityError +
    altitudeVelocityKi * altitudeVelocityIntegral;

  altitudeCorrectionDshot = clampFloat(
    altitudeCorrectionDshot,
    -MAX_ALTITUDE_CORRECTION_DSHOT,
    MAX_ALTITUDE_CORRECTION_DSHOT
  );

  int controllerBaseDshot = clampInt(
    learnedHoverDshot + static_cast<int>(roundf(altitudeCorrectionDshot)),
    learnedHoverDshot - static_cast<int>(MAX_ALTITUDE_CORRECTION_DSHOT),
    learnedHoverDshot + static_cast<int>(MAX_ALTITUDE_CORRECTION_DSHOT)
  );

  const float blend = clampFloat(
    static_cast<float>(nowMs - altitudeBlendStartMs) /
      static_cast<float>(ALTITUDE_HOLD_BLEND_MS),
    0.0f,
    1.0f
  );

  const int blendedBaseDshot = static_cast<int>(roundf(
    static_cast<float>(altitudeBlendStartDshot) +
    blend * static_cast<float>(controllerBaseDshot - altitudeBlendStartDshot)
  ));

  lastAltitudeBaseDshot = blendedBaseDshot;
  return blendedBaseDshot;
}

// ================================================================
// DESIRED COMMANDS AND ENABLE/DISABLE RAMPS
// ================================================================

void updatePendingYawHeadingHold(uint32_t nowMs)
{
  if (!yawHeadingHoldRequested || yawHeadingHoldEnabled)
  {
    return;
  }

  // Never apply heading-hold torque during the startup/shutdown ramps.
  if (!flightEnabled || enableRampActive || disableRampActive)
  {
    return;
  }

  if (!magYawFusionEnabled ||
      !magnetometerAcceptedFresh(nowMs) ||
      !stateEstimator.healthy())
  {
    return;
  }

  yawHeadingTargetCwDeg = yawAngleCwDeg;
  yawStickWasActive = false;
  yawHeadingHoldEnabled = true;
  resetYawController();

  runtimeLogLine(
    "Yaw heading hold armed after startup ramp; current heading captured."
  );
}

void calculateDesiredCommands(float dt, uint32_t nowMs)
{
  throttleTargetDshot = mapThrottleToDshot(controlData.throttle);

  // A manual yawhold request is a flight-mode request, not merely a stopped
  // state bit. It stays pending through arming and is activated after the
  // startup ramp once magnetic yaw is fresh and accepted.
  updatePendingYawHeadingHold(nowMs);

  correctedYawStick = applyCenteredStickDeadband(
    controlData.yaw,
    YAW_STICK_DEADBAND
  );

  const bool yawStickActive = fabsf(correctedYawStick) > 0.0f;

  if (yawStickActive)
  {
    // Preserve the original manual yaw-rate behavior while the stick is held.
    desiredYawRateDps = correctedYawStick * MAX_YAW_RATE_DPS;
    yawHeadingTargetCwDeg = yawAngleCwDeg;
    yawStickWasActive = true;
  }
  else
  {
    // On stick release, capture the new heading and hold it.
    if (yawStickWasActive)
    {
      yawHeadingTargetCwDeg = yawAngleCwDeg;
      yawStickWasActive = false;
    }

    const bool headingHoldAvailable =
      yawHeadingHoldEnabled &&
      magnetometerAcceptedFresh(nowMs) &&
      stateEstimator.healthy();

    if (headingHoldAvailable)
    {
      const float yawHeadingErrorDeg = wrapAngle180(
        yawHeadingTargetCwDeg - yawAngleCwDeg
      );

      desiredYawRateDps = clampFloat(
        yawHeadingKp * yawHeadingErrorDeg,
        -MAX_YAW_HOLD_RATE_DPS,
        MAX_YAW_HOLD_RATE_DPS
      );
    }
    else
    {
      // Original centered-stick behavior: zero desired yaw rate.
      desiredYawRateDps = 0.0f;
      yawHeadingTargetCwDeg = yawAngleCwDeg;
    }
  }

  if (disableRampActive)
  {
    correctedRollStick = 0.0f;
    correctedPitchStick = 0.0f;
    correctedThrottleStick = 0.0f;
    desiredRollAngleDeg = 0.0f;
    desiredPitchAngleDeg = 0.0f;
    desiredYawRateDps = 0.0f;

    const uint32_t elapsedMs =
      static_cast<uint32_t>(nowMs - disableRampStartMs);

    disableRampProgress = clampFloat(
      static_cast<float>(elapsedMs) /
        static_cast<float>(DISABLE_RAMP_DURATION_MS),
      0.0f,
      1.0f
    );

    baseThrottleDshot = static_cast<int>(roundf(
      static_cast<float>(disableRampStartDshot) +
      disableRampProgress *
      static_cast<float>(DISABLE_RAMP_END_DSHOT - disableRampStartDshot)
    ));

    if (disableRampProgress >= 1.0f)
    {
      disableRampActive = false;
      disableRampProgress = 1.0f;
      flightEnabled = false;
      baseThrottleDshot = 0;

      resetFlightNavigationState(nowMs);

      postFlightEvent(
        AsyncFlightEventType::DisableRampComplete,
        nowMs
      );
    }

    return;
  }

  updateHorizontalController(dt, nowMs);

  if (enableRampActive)
  {
    const uint32_t elapsedMs =
      static_cast<uint32_t>(nowMs - enableRampStartMs);

    enableRampProgress = clampFloat(
      static_cast<float>(elapsedMs) /
        static_cast<float>(ENABLE_RAMP_DURATION_MS),
      0.0f,
      1.0f
    );

    baseThrottleDshot = static_cast<int>(roundf(
      static_cast<float>(ENABLE_RAMP_START_DSHOT) +
      enableRampProgress *
      static_cast<float>(THROTTLE_CENTER_DSHOT - ENABLE_RAMP_START_DSHOT)
    ));

    if (enableRampProgress >= 1.0f)
    {
      enableRampActive = false;
      enableRampProgress = 1.0f;
      baseThrottleDshot = THROTTLE_CENTER_DSHOT;

      // Full-hover mode requests yaw hold automatically. Manual commissioning
      // can request it with "yawhold on" while stopped. In both cases the
      // request survives arming, but active heading-hold torque is not applied
      // during the startup ramp.
      if (FULL_HOVER_INTEGRATION)
      {
        magYawFusionEnabled = true;
        yawHeadingHoldRequested = true;
      }

      if (yawHeadingHoldRequested)
      {
        if (magYawFusionEnabled &&
            magnetometerAcceptedFresh(nowMs) &&
            stateEstimator.healthy())
        {
          yawHeadingTargetCwDeg = yawAngleCwDeg;
          yawStickWasActive = false;
          yawHeadingHoldEnabled = true;
          resetYawController();

          runtimeLogLine(
            "Enable ramp complete; yaw heading hold armed at current heading."
          );
        }
        else
        {
          yawHeadingHoldEnabled = false;
          yawHeadingTargetCwDeg = yawAngleCwDeg;

          runtimeLogLine(
            "Enable ramp complete; yaw heading hold request pending fresh accepted magnetic yaw."
          );
        }
      }

      postFlightEvent(
        AsyncFlightEventType::EnableRampComplete,
        nowMs
      );
    }

    return;
  }

  enableRampProgress = 1.0f;
  disableRampProgress = 0.0f;

  baseThrottleDshot = calculateAltitudeBaseDshot(
    dt,
    nowMs,
    throttleTargetDshot
  );
}

// ================================================================
// MODEL-BASED FEED-FORWARD
// ================================================================

void updateModelFeedforward(float dt, uint32_t nowMs)
{
  const bool modelAllowed =
    motionModelEnabled &&
    flightEnabled &&
    !enableRampActive &&
    !disableRampActive &&
    baseThrottleDshot > 0 &&
    !rangeFailureIsLong(nowMs);

  if (!modelAllowed)
  {
    resetMotionModelFeedforward();
    return;
  }

  const float requestedVerticalVelocity = altitudeHoldActive
    ? desiredVerticalVelocityMps
    : correctedThrottleStick * MAX_VERTICAL_VELOCITY_MPS;

  const float hoverReferenceDshot = altitudeHoldActive
    ? static_cast<float>(learnedHoverDshot)
    : static_cast<float>(THROTTLE_CENTER_DSHOT);

  motionModelOutput = motionModelFeedforward.update(
    desiredPitchAngleDeg,
    desiredRollAngleDeg,
    desiredYawRateDps,
    requestedVerticalVelocity,
    dt,
    hoverReferenceDshot,
    &motionModelRotorInput
  );

  if (!motionModelOutput.valid)
  {
    appliedModelCollectiveDshot = 0.0f;
    for (int motor = 0; motor < 4; ++motor)
    {
      appliedModelMotorDeltaDshot[motor] = 0.0f;
    }
    return;
  }

  appliedModelCollectiveDshot = motionModelMotorBlend *
    motionModelOutput.collectiveDeltaDshot;

  for (int motor = 0; motor < 4; ++motor)
  {
    appliedModelMotorDeltaDshot[motor] = motionModelMotorBlend *
      motionModelOutput.motorDeltaDshot[motor];
  }
}

// ================================================================
// COMPLETE FLIGHT CONTROLLER
// ================================================================

void updateFlightController(float dt, uint32_t nowMs)
{
  calculateDesiredCommands(dt, nowMs);

  if (baseThrottleDshot <= 0)
  {
    stopMotors();
    return;
  }

  updateModelFeedforward(dt, nowMs);

  // --------------------------------------------------------------
  // EXISTING OUTER ROLL/PITCH ANGLE LOOP - PRESERVED
  // --------------------------------------------------------------

  const float rollAngleError =
    desiredRollAngleDeg - measuredRollAngleDeg;

  const float pitchAngleError =
    desiredPitchAngleDeg - measuredPitchAngleDeg;

  const float modelRollRateFeedforward =
    motionModelOutput.valid
      ? motionModelRateBlend * motionModelOutput.desiredRollRateDps
      : 0.0f;

  const float modelPitchRateFeedforward =
    motionModelOutput.valid
      ? motionModelRateBlend * motionModelOutput.desiredPitchRateDps
      : 0.0f;

  desiredRollRateDps = clampFloat(
    rollAngleKp * rollAngleError + modelRollRateFeedforward,
    -MAX_DESIRED_RATE_DPS,
    MAX_DESIRED_RATE_DPS
  );

  desiredPitchRateDps = clampFloat(
    pitchAngleKp * pitchAngleError + modelPitchRateFeedforward,
    -MAX_DESIRED_RATE_DPS,
    MAX_DESIRED_RATE_DPS
  );

  // --------------------------------------------------------------
  // EXISTING INNER ROLL/PITCH RATE LOOPS - PRESERVED GAINS/SIGNS
  // --------------------------------------------------------------

  rollCorrection = updateRatePID(
    desiredRollRateDps,
    measuredRollRateDps,
    rollRateIntegral,
    previousRollRateError,
    filteredRollDerivative,
    rollRateKp,
    rollRateKi,
    rollRateKd,
    RATE_INTEGRAL_LIMIT,
    dt,
    rollPTerm,
    rollITerm,
    rollDTerm
  );

  pitchCorrection = updateRatePID(
    desiredPitchRateDps,
    measuredPitchRateDps,
    pitchRateIntegral,
    previousPitchRateError,
    filteredPitchDerivative,
    pitchRateKp,
    pitchRateKi,
    pitchRateKd,
    RATE_INTEGRAL_LIMIT,
    dt,
    pitchPTerm,
    pitchITerm,
    pitchDTerm
  );

  rollCorrection = clampFloat(
    rollCorrection,
    -MAX_AXIS_CORRECTION,
    MAX_AXIS_CORRECTION
  );

  pitchCorrection = clampFloat(
    pitchCorrection,
    -MAX_AXIS_CORRECTION,
    MAX_AXIS_CORRECTION
  );

  // --------------------------------------------------------------
  // ADDED YAW-RATE DAMPING
  // --------------------------------------------------------------

  yawCorrection = updateRatePID(
    desiredYawRateDps,
    measuredYawRateDps,
    yawRateIntegral,
    previousYawRateError,
    filteredYawDerivative,
    yawRateKp,
    yawRateKi,
    yawRateKd,
    YAW_INTEGRAL_LIMIT,
    dt,
    yawPTerm,
    yawITerm,
    yawDTerm
  );

  yawCorrection = clampFloat(
    yawCorrection,
    -MAX_YAW_CORRECTION,
    MAX_YAW_CORRECTION
  );

  const int rollMix = static_cast<int>(roundf(rollCorrection));
  const int pitchMix = static_cast<int>(roundf(pitchCorrection));
  const int yawMix = static_cast<int>(roundf(yawCorrection));
  const int base = baseThrottleDshot +
    static_cast<int>(roundf(appliedModelCollectiveDshot));

  /*
    EXISTING ROLL/PITCH MIXER TERMS ARE UNCHANGED:

                         FRONT

                    M1             M2
                    M4             M3
                         REAR

    Added yaw terms for:
      M1/M3 = CCW motors
      M2/M4 = CW motors
  */

  const int m1 = base - pitchMix - rollMix + yawMix +
    static_cast<int>(roundf(appliedModelMotorDeltaDshot[0]));
  const int m2 = base - pitchMix + rollMix - yawMix +
    static_cast<int>(roundf(appliedModelMotorDeltaDshot[1]));
  const int m3 = base + pitchMix + rollMix + yawMix +
    static_cast<int>(roundf(appliedModelMotorDeltaDshot[2]));
  const int m4 = base + pitchMix - rollMix - yawMix +
    static_cast<int>(roundf(appliedModelMotorDeltaDshot[3]));

  motor1Command = clampMotorCommand(m1);
  motor2Command = clampMotorCommand(m2);
  motor3Command = clampMotorCommand(m3);
  motor4Command = clampMotorCommand(m4);
}

// ================================================================
// TILT SAFETY - PRESERVED
// ================================================================

bool excessiveTiltDetected(uint32_t nowMs)
{
  const bool tilted =
    fabsf(measuredRollAngleDeg) > MAX_SAFE_TILT_DEG ||
    fabsf(measuredPitchAngleDeg) > MAX_SAFE_TILT_DEG;

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
    static_cast<uint32_t>(nowMs - excessiveTiltStartMs) >=
    TILT_KILL_DELAY_MS;
}

// ================================================================
// REMOTE ENABLE STATE MACHINE - PRESERVED WITH NAVIGATION RESETS
// ================================================================

void clearFlightEnableState(uint32_t nowMs)
{
  flightEnabled = false;

  // A completed/aborted flight clears the yaw-hold request. Re-enable it
  // explicitly while stopped before the next commissioning flight.
  yawHeadingHoldRequested = false;
  yawHeadingHoldEnabled = false;
  yawHeadingTargetCwDeg = yawAngleCwDeg;

  enableRampActive = false;
  enableRampProgress = 0.0f;
  enableRampStartMs = 0;

  disableRampActive = false;
  disableRampProgress = 0.0f;
  disableRampStartMs = 0;
  disableRampStartDshot = THROTTLE_CENTER_DSHOT;

  baseThrottleDshot = 0;
  resetFlightNavigationState(nowMs);
}

void startDisableRamp(uint32_t nowMs)
{
  enableRampActive = false;
  enableRampProgress = 0.0f;

  // Remote enable OFF marks the end of the active flight. The shutdown ramp
  // keeps the existing zero-rate behavior instead of trying to hold heading.
  yawHeadingHoldRequested = false;
  yawHeadingHoldEnabled = false;
  yawHeadingTargetCwDeg = yawAngleCwDeg;
  resetYawController();

  disableRampActive = true;
  disableRampStartMs = nowMs;
  disableRampProgress = 0.0f;

  disableRampStartDshot = clampInt(
    baseThrottleDshot,
    DISABLE_RAMP_END_DSHOT,
    DSHOT_MAX_TEST
  );

  deactivateXYHold();
  altitudeHoldActive = false;

  postFlightEvent(
    AsyncFlightEventType::RemoteEnableOff,
    nowMs,
    0.0f,
    0.0f,
    disableRampStartDshot,
    DISABLE_RAMP_END_DSHOT,
    DISABLE_RAMP_DURATION_MS
  );
}

void updateRemoteEnableState(
  bool signalLost,
  bool imuValid,
  bool tiltStop,
  uint32_t nowMs
)
{
  const bool remoteEnable = controlData.motorsEnabled;
  const bool enableRisingEdge = remoteEnable && !previousRemoteEnable;
  const bool enableFallingEdge = !remoteEnable && previousRemoteEnable;

  if (!remoteEnable)
  {
    serialStopLatch = false;
  }

  const bool immediateStop =
    signalLost ||
    !imuValid ||
    tiltStop ||
    controlData.kill ||
    serialStopLatch;

  if (immediateStop)
  {
    // Reset estimator/navigation state only on the transition from an active
    // drive state to stopped. Repeating clearFlightEnableState() every 5 ms
    // while already stopped would continually recapture the magnetic zero.
    const bool wasActivelyDriving =
      flightEnabled ||
      enableRampActive ||
      disableRampActive;

    if (wasActivelyDriving)
    {
      AsyncStopReason reason = AsyncStopReason::SerialStop;

      if (controlData.kill)
      {
        reason = AsyncStopReason::RemoteKill;
      }
      else if (signalLost)
      {
        reason = AsyncStopReason::SignalLost;
      }
      else if (!imuValid)
      {
        reason = AsyncStopReason::ImuFailure;
      }
      else if (tiltStop)
      {
        reason = AsyncStopReason::ExcessiveTilt;
      }

      postFlightEvent(
        AsyncFlightEventType::ImmediateFlightStop,
        nowMs,
        0.0f,
        0.0f,
        static_cast<int32_t>(reason)
      );

      clearFlightEnableState(nowMs);

      // Publish an immediate four-motor zero command.
      bidirectionalDshot.stopAll();
    }
    else
    {
      // Already safely stopped. Keep the drive state at zero without
      // repeatedly resetting the ESKF or magnetometer reference.
      flightEnabled = false;
      enableRampActive = false;
      disableRampActive = false;
      baseThrottleDshot = 0;
    }
  }
  else if (
    enableFallingEdge &&
    flightEnabled &&
    !disableRampActive
  )
  {
    startDisableRamp(nowMs);
  }
  else if (
    enableRisingEdge &&
    disableRampActive
  )
  {
    postFlightEvent(
      AsyncFlightEventType::EnableIgnoredDuringRampDown,
      nowMs
    );
  }
  else if (
    enableRisingEdge &&
    !flightEnabled &&
    !disableRampActive
  )
  {
    if (!magnetometerReadyForFlight(nowMs))
    {
      runtimeLogLine(
        "REMOTE ENABLE REJECTED: QMC5883P is not initialized, verified, and fresh. Toggle enable OFF, wait for recovery, then retry."
      );

      postFlightEvent(
        AsyncFlightEventType::RemoteEnableRejected,
        nowMs,
        controlData.throttle
      );
    }
    else if (throttleCenteredForEnable(controlData.throttle))
    {
      flightEnabled = true;
      flightEnableMs = nowMs;

      // Preserve a manually requested yaw-hold mode across the enable edge.
      // Active heading-hold torque remains suspended during the startup ramp;
      // the request is armed at ramp completion (or shortly afterward once
      // fresh accepted magnetic yaw becomes available).
      if (FULL_HOVER_INTEGRATION)
      {
        magYawFusionEnabled = true;
        yawHeadingHoldRequested = true;
      }

      const bool yawHoldRequestedForThisFlight =
        yawHeadingHoldRequested;

      yawHeadingHoldEnabled = false;
      yawHeadingTargetCwDeg = yawAngleCwDeg;
      yawStickWasActive = false;

      disableRampActive = false;
      disableRampProgress = 0.0f;

      enableRampActive = true;
      enableRampStartMs = nowMs;
      enableRampProgress = 0.0f;

      throttleTargetDshot = THROTTLE_CENTER_DSHOT;
      baseThrottleDshot = ENABLE_RAMP_START_DSHOT;

      // Capture a fresh navigation/magnetic zero once when enabling.
      resetFlightNavigationState(nowMs);

      // resetFlightNavigationState() resets controller state/targets but does
      // not own the operator's requested flight mode. Restore the request that
      // existed before the reset so "yawhold on" survives arming.
      yawHeadingHoldRequested = yawHoldRequestedForThisFlight;
      yawHeadingHoldEnabled = false;
      yawHeadingTargetCwDeg = yawAngleCwDeg;

      if (yawHeadingHoldRequested)
      {
        runtimeLogLine(
          "Yaw heading hold request retained across arm; waiting for startup ramp completion."
        );
      }

      postFlightEvent(
        AsyncFlightEventType::RemoteEnableAccepted,
        nowMs,
        0.0f,
        0.0f,
        ENABLE_RAMP_START_DSHOT,
        THROTTLE_CENTER_DSHOT,
        ENABLE_RAMP_DURATION_MS
      );
    }
    else
    {
      postFlightEvent(
        AsyncFlightEventType::RemoteEnableRejected,
        nowMs,
        controlData.throttle
      );
    }
  }

  previousRemoteEnable = remoteEnable;
}

// ================================================================
// LIVE TUNING / STATUS
// ================================================================

void printPIDGains()
{
  runtimeLogf(
    "PID,RKP:%.5f,RKI:%.5f,RKD:%.5f,PKP:%.5f,PKI:%.5f,PKD:%.5f,RAKP:%.3f,PAKP:%.3f,YKP:%.4f,YKI:%.4f,YKD:%.5f,YHKP:%.3f",
    rollRateKp,
    rollRateKi,
    rollRateKd,
    pitchRateKp,
    pitchRateKi,
    pitchRateKd,
    rollAngleKp,
    pitchAngleKp,
    yawRateKp,
    yawRateKi,
    yawRateKd,
    yawHeadingKp
  );
}

void printOuterGains()
{
  runtimeLogf(
    "OUTER,XYPOSKP:%.4f,XYVELKP:%.4f,XYACCKP:%.4f,ZPOSKP:%.4f,ZVELKP:%.4f,ZVELKI:%.4f",
    xyPositionKp,
    xyVelocityKp,
    xyAccelerationKp,
    altitudePositionKp,
    altitudeVelocityKp,
    altitudeVelocityKi
  );
}

void printModelSettings()
{
  runtimeLogf(
    "MODEL,EN:%d,VALID:%d,SAT:%d,RPM_INPUT:%d,RPM_MASK:%X,MOTOR_BLEND:%.3f,RATE_BLEND:%.3f,COLL:%.2f,D1:%.2f,D2:%.2f,D3:%.2f,D4:%.2f,HROTOR:%.7f",
    motionModelEnabled ? 1 : 0,
    motionModelOutput.valid ? 1 : 0,
    motionModelOutput.saturated ? 1 : 0,
    motionModelOutput.rotorFeedbackUsed ? 1 : 0,
    static_cast<unsigned>(motionModelOutput.measuredRotorMask),
    motionModelMotorBlend,
    motionModelRateBlend,
    appliedModelCollectiveDshot,
    appliedModelMotorDeltaDshot[0],
    appliedModelMotorDeltaDshot[1],
    appliedModelMotorDeltaDshot[2],
    appliedModelMotorDeltaDshot[3],
    motionModelOutput.rotorAngularMomentumInput
  );
}

void printRpmStatus()
{
  BidirectionalDShot::RpmSnapshot latestSnapshot;
  if (bidirectionalDshot.getSnapshot(latestSnapshot))
  {
    rpmSnapshot = latestSnapshot;
    rpmFreshMask = latestSnapshot.freshMask;
    rpmTelemetryHealthy = latestSnapshot.allFresh;
  }

  char line[AsyncDebugTelemetry::TEXT_LINE_CAPACITY];
  size_t used = static_cast<size_t>(snprintf(
    line,
    sizeof(line),
    "RPM,SEQ:%lu,MASK:%X,ALL:%d,QPROC:%.2f,FZ_N:%.3f,TX_NM:%.5f,TY_NM:%.5f",
    static_cast<unsigned long>(rpmSnapshot.sequence),
    static_cast<unsigned>(rpmFreshMask),
    rpmTelemetryHealthy ? 1 : 0,
    eskfActuatorProcessNoiseScale,
    rotorModelState.forceBodyN[2],
    rotorModelState.torqueBodyNm[0],
    rotorModelState.torqueBodyNm[1]
  ));

  if (used >= sizeof(line)) used = sizeof(line) - 1;

  for (int motor = 0; motor < 4 && used < sizeof(line) - 1; ++motor)
  {
    const BidirectionalDShot::MotorRpmSample &sample = rpmSnapshot.motor[motor];
    const long ageToPrint =
      sample.ageUs == UINT32_MAX ? -1L : static_cast<long>(sample.ageUs);

    const int written = snprintf(
      line + used,
      sizeof(line) - used,
      " | M%d:c%u r%.0f f%.0f w%.1f v%d lp%d loss%d q%.1f a%ld s%s src%c k%.3f",
      motor + 1,
      static_cast<unsigned>(sample.dshotCommand),
      sample.rpmRaw,
      sample.rpmFiltered,
      sample.omegaRadPerSec,
      sample.valid ? 1 : 0,
      sample.lastPacketValid ? 1 : 0,
      sample.lost ? 1 : 0,
      sample.packetQuality,
      ageToPrint,
      BidirectionalDShot::statusName(sample.status),
      rotorModelState.measured[motor] ? 'M' : 'F',
      rotorModelState.omegaPerDshot[motor]
    );

    if (written < 0) break;
    const size_t added = static_cast<size_t>(written);
    if (added >= sizeof(line) - used)
    {
      used = sizeof(line) - 1;
      break;
    }
    used += added;
  }

  line[sizeof(line) - 1] = '\0';
  runtimeLogLine(line);
}

void printEskfStatus()
{
  runtimeLogf(
    "ESKF,HEALTHY:%d,ATT_EN:%d,ATT_USED:%d,FLOW_READY:%d,RANGE_READY:%d,P:%.2f,R:%.2f,Y:%.2f,X:%.3f,Y_POS:%.3f,Z:%.3f,VX:%.3f,VY:%.3f,VZ:%.3f,NIS_SIGMA:%.2f,ACC:%lu,REJ:%lu",
    stateEstimator.healthy() ? 1 : 0,
    eskfAttitudeControlEnabled ? 1 : 0,
    eskfAttitudeUsed ? 1 : 0,
    eskfFlowVelocityReady ? 1 : 0,
    eskfRangeReady ? 1 : 0,
    eskfPitchAngleDeg,
    eskfRollAngleDeg,
    eskfYawCwDeg,
    stateEstimator.positionX(),
    stateEstimator.positionY(),
    stateEstimator.positionZ(),
    stateEstimator.velocityX(),
    stateEstimator.velocityY(),
    stateEstimator.velocityZ(),
    stateEstimator.lastInnovationRatio(),
    static_cast<unsigned long>(stateEstimator.acceptedUpdates()),
    static_cast<unsigned long>(stateEstimator.rejectedUpdates())
  );
}

bool parseGainCommand(
  const char *command,
  const char *name,
  float &gain,
  float minimum,
  float maximum
)
{
  const size_t nameLength = strlen(name);

  if (strncasecmp(command, name, nameLength) != 0)
  {
    return false;
  }

  const char next = command[nameLength];
  if (next != ' ' && next != '\t' && next != '=')
  {
    return false;
  }

  const char *valueText = command + nameLength;
  while (*valueText == ' ' || *valueText == '\t' || *valueText == '=')
  {
    valueText++;
  }

  char *endPointer = nullptr;
  const float requested = strtof(valueText, &endPointer);

  while (endPointer != nullptr &&
         (*endPointer == ' ' || *endPointer == '\t'))
  {
    endPointer++;
  }

  if (
    valueText == endPointer ||
    endPointer == nullptr ||
    *endPointer != '\0' ||
    !isfinite(requested)
  )
  {
    runtimeLogf("Invalid value for %s", name);
    return true;
  }

  if (requested < minimum || requested > maximum)
  {
    runtimeLogf(
      "%s must be between %.5f and %.5f",
      name,
      minimum,
      maximum
    );
    return true;
  }

  gain = requested;
  resetAttitudeControllers();
  resetYawController();

  runtimeLogf("%s set to %.5f", name, gain);
  return true;
}

bool processPIDGainCommand(char *command)
{
  if (parseGainCommand(command, "rkp", rollRateKp, 0.0f, 10.0f)) return true;
  if (parseGainCommand(command, "rki", rollRateKi, 0.0f, 5.0f)) return true;
  if (parseGainCommand(command, "rkd", rollRateKd, 0.0f, 1.0f)) return true;
  if (parseGainCommand(command, "pkp", pitchRateKp, 0.0f, 10.0f)) return true;
  if (parseGainCommand(command, "pki", pitchRateKi, 0.0f, 5.0f)) return true;
  if (parseGainCommand(command, "pkd", pitchRateKd, 0.0f, 1.0f)) return true;
  if (parseGainCommand(command, "ykp", yawRateKp, 0.0f, 5.0f)) return true;
  if (parseGainCommand(command, "yki", yawRateKi, 0.0f, 2.0f)) return true;
  if (parseGainCommand(command, "ykd", yawRateKd, 0.0f, 0.5f)) return true;
  if (parseGainCommand(command, "yhkp", yawHeadingKp, 0.0f, 10.0f)) return true;

  if (parseGainCommand(command, "rakp", rollAngleKp, 0.0f, 20.0f)) return true;
  if (parseGainCommand(command, "pakp", pitchAngleKp, 0.0f, 20.0f)) return true;

  if (parseGainCommand(command, "xyposkp", xyPositionKp, 0.0f, 5.0f)) return true;
  if (parseGainCommand(command, "xyvelkp", xyVelocityKp, 0.0f, 30.0f)) return true;
  if (parseGainCommand(command, "xyacckp", xyAccelerationKp, 0.0f, 4.0f)) return true;
  if (parseGainCommand(command, "zposkp", altitudePositionKp, 0.0f, 5.0f)) return true;
  if (parseGainCommand(command, "zvelkp", altitudeVelocityKp, 0.0f, 200.0f)) return true;
  if (parseGainCommand(command, "zvelki", altitudeVelocityKi, 0.0f, 100.0f)) return true;
  if (parseGainCommand(command, "ffblend", motionModelMotorBlend, 0.0f, 0.50f)) return true;
  if (parseGainCommand(command, "rateff", motionModelRateBlend, 0.0f, 0.80f)) return true;

  float bothGain = NAN;

  if (parseGainCommand(command, "kp", bothGain, 0.0f, 10.0f))
  {
    if (isfinite(bothGain))
    {
      rollRateKp = bothGain;
      pitchRateKp = bothGain;
      resetAttitudeControllers();
      runtimeLogLine("Applied KP to roll and pitch.");
      printPIDGains();
    }
    return true;
  }

  bothGain = NAN;
  if (parseGainCommand(command, "ki", bothGain, 0.0f, 5.0f))
  {
    if (isfinite(bothGain))
    {
      rollRateKi = bothGain;
      pitchRateKi = bothGain;
      resetAttitudeControllers();
      runtimeLogLine("Applied KI to roll and pitch.");
      printPIDGains();
    }
    return true;
  }

  bothGain = NAN;
  if (parseGainCommand(command, "kd", bothGain, 0.0f, 1.0f))
  {
    if (isfinite(bothGain))
    {
      rollRateKd = bothGain;
      pitchRateKd = bothGain;
      resetAttitudeControllers();
      runtimeLogLine("Applied KD to roll and pitch.");
      printPIDGains();
    }
    return true;
  }

  bothGain = NAN;
  if (parseGainCommand(command, "akp", bothGain, 0.0f, 20.0f))
  {
    if (isfinite(bothGain))
    {
      rollAngleKp = bothGain;
      pitchAngleKp = bothGain;
      resetAttitudeControllers();
      runtimeLogLine("Applied angle KP to roll and pitch.");
      printPIDGains();
    }
    return true;
  }

  return false;
}

void printFlowStatus()
{
  const uint32_t nowMs = millis();
  const uint32_t healthAgeMs =
    lastHealthyFlowMs == 0U ? UINT32_MAX :
      static_cast<uint32_t>(nowMs - lastHealthyFlowMs);
  const uint32_t measurementAgeMs =
    lastFlowMeasurementMs == 0U ? UINT32_MAX :
      static_cast<uint32_t>(nowMs - lastFlowMeasurementMs);

  runtimeLogf(
    "FLOWSTAT,COMMISSIONED:%d,MAP:%u,HOLD:%d,HEALTH:%d,HSTREAK:%u,HAGE:%lu,MEASOK:%d,MEASAGE:%lu,AXMASK:%X,MOTION:0x%02X,SQUAL:%u,SHUTTER:%u,NOMOTION:%lu,YAWDPS:%.2f,YAWNS:%.2f,YAWREJ:%lu,YAWDW:%lu,RAWVX:%.3f,RAWVY:%.3f,FILTVX:%.3f,FILTVY:%.3f,BADTIM:%lu,BADCOUNT:%lu,BADHEIGHT:%lu,BADQUAL:%lu,BADSPEED:%lu,ESTREJ:%lu",
    xyHoldCommissioned ? 1 : 0,
    static_cast<unsigned>(flowAxisMap),
    xyHoldActive ? 1 : 0,
    flowHealthFresh(nowMs) ? 1 : 0,
    static_cast<unsigned>(consecutiveHealthyFlowSamples),
    static_cast<unsigned long>(healthAgeMs),
    lastFlowSampleValid ? 1 : 0,
    static_cast<unsigned long>(measurementAgeMs),
    static_cast<unsigned>(lastFlowAxisAcceptedMask),
    static_cast<unsigned>(lastFlowMotion),
    static_cast<unsigned>(lastFlowSqual),
    static_cast<unsigned>(lastFlowShutter),
    static_cast<unsigned long>(flowNoMotionCount),
    lastFlowYawRateDps,
    lastFlowYawNoiseScale,
    static_cast<unsigned long>(flowYawRateRejectCount),
    static_cast<unsigned long>(flowYawRateDownweightCount),
    lastInstantFlowVelocityX_mps,
    lastInstantFlowVelocityY_mps,
    measuredBodyVelocityX_mps,
    measuredBodyVelocityY_mps,
    static_cast<unsigned long>(flowInvalidTimingCount),
    static_cast<unsigned long>(flowInvalidCountCount),
    static_cast<unsigned long>(flowInvalidHeightCount),
    static_cast<unsigned long>(flowInvalidQualityCount),
    static_cast<unsigned long>(flowInvalidSpeedCount),
    static_cast<unsigned long>(flowEstimatorRejectCount)
  );
}

void printStatus()
{
  const uint32_t nowMs = millis();

  runtimeLogf(
    "FLIGHT_EN:%d,ESKF:%d,EATT_EN:%d,EATT:%d,MODEL:%d,MVALID:%d,RPM_MASK:%X,RPM_ALL:%d,RPM_FZ:%.2f,RPM_QS:%.2f,XY_HOLD:%d,ALT_HOLD:%d,RANGE_OK:%d,RANGE_AGE:%lu,FLOW_GOOD:%d,RANGE_M:%.3f,EZ:%.3f,VZ:%.3f,X:%.3f,Y:%.3f,VX:%.3f,VY:%.3f,FVX:%.3f,FVY:%.3f,AX:%.3f,AY:%.3f,BASE:%d,FFCOLL:%.2f,YAW_RATE:%.2f",
    flightEnabled ? 1 : 0,
    eskfStateHealthy ? 1 : 0,
    eskfAttitudeControlEnabled ? 1 : 0,
    eskfAttitudeUsed ? 1 : 0,
    motionModelEnabled ? 1 : 0,
    motionModelOutput.valid ? 1 : 0,
    static_cast<unsigned>(rpmFreshMask),
    rpmTelemetryHealthy ? 1 : 0,
    rotorModelState.forceBodyN[2],
    eskfActuatorProcessNoiseScale,
    xyHoldActive ? 1 : 0,
    altitudeHoldActive ? 1 : 0,
    rangeControlUsable(nowMs) ? 1 : 0,
    static_cast<unsigned long>(rangeAgeMs(nowMs)),
    flowHealthFresh(nowMs) ? 1 : 0,
    filteredRangeMeters,
    altitudeForControl(),
    verticalVelocityMps,
    positionNavX_m,
    positionNavY_m,
    measuredBodyVelocityX_mps,
    measuredBodyVelocityY_mps,
    fusedBodyVelocityX_mps,
    fusedBodyVelocityY_mps,
    filteredLinearAccelBodyX_mps2,
    filteredLinearAccelBodyY_mps2,
    baseThrottleDshot,
    appliedModelCollectiveDshot,
    measuredYawRateDps
  );


  printFlowStatus();
}

void processSerialCommand(char *command)
{
  while (*command == ' ' || *command == '\t') command++;

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

  if (length == 0) return;

  if (strcasecmp(command, "flowstatus") == 0 ||
      strcasecmp(command, "flow") == 0)
  {
    printFlowStatus();
    return;
  }

  if (strncasecmp(command, "flowmap ", 8) == 0)
  {
    if (flightEnabled || enableRampActive || disableRampActive)
    {
      runtimeLogLine("FLOWMAP REJECTED: stop the motors first.");
      return;
    }

    const int requestedMap = atoi(command + 8);
    if (requestedMap < 0 || requestedMap > 7)
    {
      runtimeLogLine("FLOWMAP must be 0..7. Run the PMW bench diagnostic first.");
      return;
    }

    flowAxisMap = static_cast<FlowAxisMap>(requestedMap);
    xyHoldCommissioned = false;
    resetFlightNavigationState(millis());
    runtimeLogf(
      "FLOWMAP set to %d. XY commissioning interlock forced OFF.",
      requestedMap
    );
    return;
  }

  if (strcasecmp(command, "xycommission on") == 0)
  {
    if (flightEnabled || enableRampActive || disableRampActive)
    {
      runtimeLogLine("XYCOMMISSION ON REJECTED: stop the motors first.");
      return;
    }

    xyHoldCommissioned = true;
    resetFlightNavigationState(millis());
    runtimeLogLine(
      "XY HOLD COMMISSIONED: quality/settling gates enabled. Use a tethered test."
    );
    return;
  }

  if (strcasecmp(command, "xycommission off") == 0)
  {
    xyHoldCommissioned = false;
    deactivateXYHold();
    runtimeLogLine("XY HOLD COMMISSIONING INTERLOCK OFF.");
    return;
  }

  if (strcasecmp(command, "pid") == 0 ||
      strcasecmp(command, "gains") == 0)
  {
    printPIDGains();
    return;
  }

  if (strcasecmp(command, "outer") == 0)
  {
    printOuterGains();
    return;
  }

  if (strcasecmp(command, "model") == 0)
  {
    printModelSettings();
    return;
  }

  if (strcasecmp(command, "rpm") == 0 ||
      strcasecmp(command, "rpmstatus") == 0)
  {
    printRpmStatus();
    return;
  }

  if (strcasecmp(command, "mag") == 0 ||
      strcasecmp(command, "magstatus") == 0)
  {
    printMagStatus();
    return;
  }

  if (strcasecmp(command, "range") == 0 ||
      strcasecmp(command, "rangestatus") == 0)
  {
    printRangeStatus();
    return;
  }

  if (strcasecmp(command, "i2cdiag") == 0)
  {
    printI2cDiagnostics();
    return;
  }

  if (strcasecmp(command, "magreset") == 0)
  {
    resetMagnetometerWhileStopped();
    return;
  }

  if (strcasecmp(command, "mag on") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      runtimeLogLine("MAG ON REJECTED: disable the motors first.");
      return;
    }

    if (!magSensorInitialized)
    {
      runtimeLogLine("MAG ON REJECTED: QMC5883P is not initialized.");
      return;
    }

    magYawFusionEnabled = true;
    yawHeadingHoldRequested = false;
    yawHeadingHoldEnabled = false;
    resetFlightNavigationState(millis());
    runtimeLogLine("Magnetic ESKF yaw fusion enabled; local magnetic reference recapturing.");
    return;
  }

  if (strcasecmp(command, "mag off") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      runtimeLogLine("MAG OFF REJECTED: disable the motors first.");
      return;
    }

    magYawFusionEnabled = false;
    yawHeadingHoldRequested = false;
    yawHeadingHoldEnabled = false;
    resetFlightNavigationState(millis());
    runtimeLogLine("Magnetic yaw fusion disabled; gyro dead-reckoned yaw restored.");
    return;
  }

  if (strcasecmp(command, "magref") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      runtimeLogLine("MAGREF REJECTED: disable the motors first.");
      return;
    }

    resetFlightNavigationState(millis());
    runtimeLogLine("Navigation frame reset; next valid compass sample becomes yaw zero.");
    return;
  }

  if (strcasecmp(command, "yawhold on") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      runtimeLogLine("YAWHOLD ON REJECTED: disable the motors first.");
      return;
    }
    if (!magYawFusionEnabled || !magReferenceValid)
    {
      runtimeLogLine("YAWHOLD ON REJECTED: enable MAG and wait for a valid reference first.");
      return;
    }

    if (!magnetometerAcceptedFresh(millis()))
    {
      runtimeLogLine(
        "YAWHOLD ON REJECTED: magnetic fusion has no fresh accepted update."
      );
      return;
    }

    yawHeadingTargetCwDeg = yawAngleCwDeg;
    yawHeadingHoldRequested = true;
    yawHeadingHoldEnabled = true;
    resetYawController();
    runtimeLogLine(
      "Yaw heading hold requested and enabled; request will survive the next arm and activate after the startup ramp."
    );
    return;
  }

  if (strcasecmp(command, "yawhold off") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      runtimeLogLine("YAWHOLD OFF REJECTED: disable the motors first.");
      return;
    }

    yawHeadingHoldRequested = false;
    yawHeadingHoldEnabled = false;
    yawHeadingTargetCwDeg = yawAngleCwDeg;
    resetYawController();
    runtimeLogLine("Yaw heading hold disabled; centered stick commands zero yaw rate.");
    return;
  }

  if (strcasecmp(command, "model on") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      runtimeLogLine("MODEL ON REJECTED: disable the motors first.");
      return;
    }

    motionModelEnabled = true;
    resetMotionModelFeedforward();
    runtimeLogLine("Motion-model feed-forward enabled.");
    return;
  }

  if (strcasecmp(command, "model off") == 0)
  {
    motionModelEnabled = false;
    resetMotionModelFeedforward();
    runtimeLogLine("Motion-model feed-forward disabled; PID remains active.");
    return;
  }

  if (strcasecmp(command, "eskf") == 0)
  {
    printEskfStatus();
    return;
  }

  if (strcasecmp(command, "eatt") == 0)
  {
    runtimeLogf(
      "ESKF attitude feedback enabled: %d",
      eskfAttitudeControlEnabled ? 1 : 0
    );
    printEskfStatus();
    return;
  }

  if (strcasecmp(command, "eatt on") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      runtimeLogLine("EATT ON REJECTED: disable the motors first.");
      return;
    }

    if (!stateEstimator.healthy())
    {
      runtimeLogLine("EATT ON REJECTED: ESKF is not healthy.");
      return;
    }

    eskfAttitudeControlEnabled = true;
    resetAttitudeControllers();
    runtimeLogLine("ESKF attitude feedback enabled for the next flight.");
    return;
  }

  if (strcasecmp(command, "eatt off") == 0)
  {
    eskfAttitudeControlEnabled = false;
    eskfAttitudeUsed = false;
    resetAttitudeControllers();
    runtimeLogLine("ESKF attitude feedback disabled; complementary attitude active.");
    return;
  }

  if (strcasecmp(command, "pidreset") == 0)
  {
    resetAttitudeControllers();
    resetYawController();
    altitudeVelocityIntegral = 0.0f;
    runtimeLogLine("Controller states reset; gains unchanged.");
    return;
  }

  if (processPIDGainCommand(command))
  {
    return;
  }

  if (strcasecmp(command, "arm") == 0)
  {
    runtimeLogLine("Serial arm removed. Use remote enable with throttle centered.");
    return;
  }

  if (
    strcasecmp(command, "kill") == 0 ||
    strcasecmp(command, "stop") == 0 ||
    strcasecmp(command, "disarm") == 0
  )
  {
    serialStopLatch = true;
    clearFlightEnableState(millis());
    immediateStopFrames();

    runtimeLogLine(
      "SERIAL STOP: motors off. Toggle remote enable OFF before re-enabling."
    );
    return;
  }

  if (strcasecmp(command, "status") == 0)
  {
    printStatus();
    return;
  }

  if (strcasecmp(command, "navreset") == 0)
  {
    yawAngleCwDeg = 0.0f;
    resetEstimatorLocalFrame();
    targetNavX_m = positionNavX_m;
    targetNavY_m = positionNavY_m;

    if (filteredRangeMeters > 0.0f)
    {
      altitudeTargetM = altitudeForControl();
    }

    altitudeVelocityIntegral = 0.0f;
    resetMotionModelFeedforward();
    runtimeLogLine("ESKF navigation origin and active targets recaptured.");
    return;
  }

  if (strcasecmp(command, "level") == 0)
  {
    rollLevelTrimDeg += measuredRollAngleDeg;
    pitchLevelTrimDeg += measuredPitchAngleDeg;

    measuredRollAngleDeg = 0.0f;
    measuredPitchAngleDeg = 0.0f;
    complementaryRollAngleDeg = 0.0f;
    complementaryPitchAngleDeg = 0.0f;

    resetEstimatorLocalFrame();
    resetAttitudeControllers();
    runtimeLogLine("Current orientation stored as level; ESKF attitude reset.");
    return;
  }

  runtimeLogLine(
    "Commands: kill/stop/disarm, level, status, navreset, pid, outer, eskf, rpm, mag, range, "
    "mag on/mag off/magref/magreset, i2cdiag, yawhold on/yawhold off, eatt/eatt on/eatt off, "
    "model/model on/model off, ffblend/rateff, kp/ki/kd, rkp/rki/rkd, "
    "pkp/pki/pkd, ykp/yki/ykd/yhkp, akp/rakp/pakp, "
    "xyposkp/xyvelkp/xyacckp, zposkp/zvelkp/zvelki, pidreset"
  );
}

void processQueuedSerialCommand()
{
  if (!asyncDebugStarted) return;

  // UART RX is performed only by the low-priority Core-0 debug task. The
  // flight core consumes at most one completed command per pass, with a
  // zero-timeout queue receive and no HardwareSerial calls.
  if (asyncDebug.popCommand(serialBuffer, sizeof(serialBuffer)))
  {
    processSerialCommand(serialBuffer);
  }
}

// ================================================================
// ESP-NOW CALLBACK - PRESERVED
// ================================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int length
)
{
  (void)info;
  const uint32_t callbackStartUs = micros();

  if (length != static_cast<int>(sizeof(ControlData)))
  {
    return;
  }

  ControlData temporary;
  memcpy(&temporary, incomingData, sizeof(ControlData));

  if (temporary.magic != PACKET_MAGIC)
  {
    return;
  }

  const uint32_t packetRxUs = callbackStartUs;

  portENTER_CRITICAL(&espNowMux);

  if (previousEspNowRxUs != 0U)
  {
    const uint32_t intervalUs =
      static_cast<uint32_t>(packetRxUs - previousEspNowRxUs);
    lastEspNowIntervalUs = intervalUs;
    if (intervalUs < minEspNowIntervalUs) minEspNowIntervalUs = intervalUs;
    if (intervalUs > maxEspNowIntervalUs) maxEspNowIntervalUs = intervalUs;
  }
  previousEspNowRxUs = packetRxUs;

  memcpy(&receivedData, &temporary, sizeof(ControlData));
  packetReceived = true;
  lastPacketTimeMs = millis();

  // Callback execution time includes validation/copy and critical-section work.
  recordTiming(timingEspNowCallback, micros() - callbackStartUs);

  portEXIT_CRITICAL(&espNowMux);
}

// ================================================================
// COMPACT FLIGHT DEBUG
// ================================================================

void publishAsyncTelemetry(
  bool signalLost,
  bool safetyStop,
  bool tiltStop,
  bool imuValid,
  uint32_t nowMs
)
{
  AsyncFlightTelemetrySnapshot snapshot;
  snapshot.timestampMs = nowMs;

  snapshot.signalLost = signalLost;
  snapshot.imuValid = imuValid;
  snapshot.safetyStop = safetyStop;
  snapshot.tiltStop = tiltStop;
  snapshot.flightEnabled = flightEnabled;
  snapshot.eskfStateHealthy = eskfStateHealthy;
  snapshot.eskfAttitudeUsed = eskfAttitudeUsed;
  snapshot.motionModelValid = motionModelOutput.valid;
  snapshot.motionModelSaturated = motionModelOutput.saturated;
  snapshot.rpmFreshMask = rpmFreshMask;
  snapshot.rpmTelemetryHealthy = rpmTelemetryHealthy;
  snapshot.rotorForceZ_N = rotorModelState.forceBodyN[2];
  snapshot.eskfProcessNoiseScale = eskfActuatorProcessNoiseScale;
  snapshot.xyHoldActive = xyHoldActive;
  snapshot.altitudeHoldActive = altitudeHoldActive;
  snapshot.rangeControlUsable = rangeControlUsable(nowMs);
  snapshot.rangeAgeMs = rangeAgeMs(nowMs);
  snapshot.flowReady = flowHealthFresh(nowMs);
  snapshot.baseThrottleDshot = baseThrottleDshot;

  snapshot.rangeM = filteredRangeMeters;
  snapshot.altitudeM = altitudeForControl();
  snapshot.altitudeTargetM = altitudeTargetM;
  snapshot.verticalVelocityMps = verticalVelocityMps;
  snapshot.desiredVerticalVelocityMps = desiredVerticalVelocityMps;
  snapshot.altitudeCorrectionDshot = altitudeCorrectionDshot;

  snapshot.positionX_M = positionNavX_m;
  snapshot.targetX_M = targetNavX_m;
  snapshot.positionY_M = positionNavY_m;
  snapshot.targetY_M = targetNavY_m;
  snapshot.measuredVelocityX_Mps = measuredBodyVelocityX_mps;
  snapshot.targetVelocityX_Mps = controllerBodyVelocityTargetX_mps;
  snapshot.measuredVelocityY_Mps = measuredBodyVelocityY_mps;
  snapshot.targetVelocityY_Mps = controllerBodyVelocityTargetY_mps;
  snapshot.fusedVelocityX_Mps = fusedBodyVelocityX_mps;
  snapshot.fusedVelocityY_Mps = fusedBodyVelocityY_mps;
  snapshot.linearAccelX_Mps2 = filteredLinearAccelBodyX_mps2;
  snapshot.linearAccelY_Mps2 = filteredLinearAccelBodyY_mps2;

  snapshot.rollAngleDeg = measuredRollAngleDeg;
  snapshot.desiredRollAngleDeg = desiredRollAngleDeg;
  snapshot.pitchAngleDeg = measuredPitchAngleDeg;
  snapshot.desiredPitchAngleDeg = desiredPitchAngleDeg;
  snapshot.yawRateDps = measuredYawRateDps;
  snapshot.yawCorrection = yawCorrection;
  snapshot.innovationRatio = stateEstimator.lastInnovationRatio();
  snapshot.modelCollectiveDshot = appliedModelCollectiveDshot;
  for (int motor = 0; motor < 4; ++motor)
  {
    snapshot.modelMotorDeltaDshot[motor] =
      appliedModelMotorDeltaDshot[motor];
  }

  snapshot.motorCommand[0] = motor1Command;
  snapshot.motorCommand[1] = motor2Command;
  snapshot.motorCommand[2] = motor3Command;
  snapshot.motorCommand[3] = motor4Command;
  snapshot.controlOverrunCount = controlOverrunCount;
  snapshot.maximumControlDtUs = maximumObservedControlDtUs;
  snapshot.minimumControlDtUs = minimumObservedControlDtUs;

  // Per-window execution-time profiler. Averages/maxima are in microseconds.
  snapshot.timingCommandAvgUs = averageTimingUs(timingCommandProcessing);
  snapshot.timingCommandMaxUs = timingCommandProcessing.maxUs;
  snapshot.timingRxCopyAvgUs = averageTimingUs(timingEspNowCopy);
  snapshot.timingRxCopyMaxUs = timingEspNowCopy.maxUs;
  snapshot.timingControlAvgUs = averageTimingUs(timingControlBlock);
  snapshot.timingControlMaxUs = timingControlBlock.maxUs;
  snapshot.timingRotorAvgUs = averageTimingUs(timingRotorFeedback);
  snapshot.timingRotorMaxUs = timingRotorFeedback.maxUs;
  snapshot.timingAttitudeAvgUs = averageTimingUs(timingAttitude);
  snapshot.timingAttitudeMaxUs = timingAttitude.maxUs;
  snapshot.timingImuAvgUs = averageTimingUs(timingImuRead);
  snapshot.timingImuMaxUs = timingImuRead.maxUs;
  snapshot.timingEskfAvgUs = averageTimingUs(timingEskf);
  snapshot.timingEskfMaxUs = timingEskf.maxUs;
  snapshot.timingEnableAvgUs = averageTimingUs(timingEnableState);
  snapshot.timingEnableMaxUs = timingEnableState.maxUs;
  snapshot.timingFlightControllerAvgUs =
    averageTimingUs(timingFlightController);
  snapshot.timingFlightControllerMaxUs = timingFlightController.maxUs;
  snapshot.timingMotorPublishAvgUs = averageTimingUs(timingMotorPublish);
  snapshot.timingMotorPublishMaxUs = timingMotorPublish.maxUs;
  snapshot.timingRangeAvgUs = averageTimingUs(timingRangePoll);
  snapshot.timingRangeMaxUs = timingRangePoll.maxUs;
  snapshot.timingFlowAvgUs = averageTimingUs(timingFlowUpdate);
  snapshot.timingFlowMaxUs = timingFlowUpdate.maxUs;

  OpticalFlowTaskStats flowTaskStatsCopy;
  portENTER_CRITICAL(&flowTaskStatsMux);
  flowTaskStatsCopy = opticalFlowTaskStats;
  opticalFlowTaskStats = OpticalFlowTaskStats{};
  portEXIT_CRITICAL(&flowTaskStatsMux);

  snapshot.timingFlowIoAvgUs =
    flowTaskStatsCopy.ioSamples == 0U
      ? 0U
      : static_cast<uint32_t>(
          flowTaskStatsCopy.ioTotalUs / flowTaskStatsCopy.ioSamples
        );
  snapshot.timingFlowIoMaxUs = flowTaskStatsCopy.ioMaxUs;
  snapshot.flowTaskHz =
    flowTaskStatsCopy.intervalTotalUs == 0U
      ? 0.0f
      : 1000000.0f *
          static_cast<float>(flowTaskStatsCopy.intervalSamples) /
          static_cast<float>(flowTaskStatsCopy.intervalTotalUs);
  snapshot.flowSkippedSamples = skippedFlowSamplesWindow;
  snapshot.flowTaskCore = static_cast<int8_t>(opticalFlowTaskCore);
  skippedFlowSamplesWindow = 0;

  snapshot.timingLoopAvgUs = averageTimingUs(timingLoopActive);
  snapshot.timingLoopMaxUs = timingLoopActive.maxUs;
  snapshot.timingAsyncPublishAvgUs = averageTimingUs(timingAsyncPublish);
  snapshot.timingAsyncPublishMaxUs = timingAsyncPublish.maxUs;

  // ESP-NOW callback data are owned by the Wi-Fi task; copy/reset atomically.
  TimingAccumulator espNowCallbackCopy;
  uint32_t espNowIntervalCopy = 0;
  uint32_t espNowIntervalMinCopy = UINT32_MAX;
  uint32_t espNowIntervalMaxCopy = 0;

  portENTER_CRITICAL(&espNowMux);
  espNowCallbackCopy = timingEspNowCallback;
  espNowIntervalCopy = lastEspNowIntervalUs;
  espNowIntervalMinCopy = minEspNowIntervalUs;
  espNowIntervalMaxCopy = maxEspNowIntervalUs;
  clearTiming(timingEspNowCallback);
  minEspNowIntervalUs = UINT32_MAX;
  maxEspNowIntervalUs = 0;
  portEXIT_CRITICAL(&espNowMux);

  snapshot.espNowCallbackAvgUs = averageTimingUs(espNowCallbackCopy);
  snapshot.espNowCallbackMaxUs = espNowCallbackCopy.maxUs;
  snapshot.espNowIntervalUs = espNowIntervalCopy;
  snapshot.espNowIntervalMinUs =
    espNowIntervalMinCopy == UINT32_MAX ? 0 : espNowIntervalMinCopy;
  snapshot.espNowIntervalMaxUs = espNowIntervalMaxCopy;

  // The DShot service is a different FreeRTOS task. We can measure its actual
  // service rate from the snapshot sequence without touching the RMT task.
  static uint32_t previousRpmTimingSequence = 0;
  static uint32_t previousRpmTimingMs = 0;
  snapshot.dshotServiceHz = 0.0f;
  if (previousRpmTimingMs != 0U && nowMs != previousRpmTimingMs)
  {
    const uint32_t sequenceDelta =
      static_cast<uint32_t>(rpmSnapshot.sequence - previousRpmTimingSequence);
    const uint32_t elapsedMs =
      static_cast<uint32_t>(nowMs - previousRpmTimingMs);
    snapshot.dshotServiceHz =
      1000.0f * static_cast<float>(sequenceDelta) /
      static_cast<float>(elapsedMs);
  }
  previousRpmTimingSequence = rpmSnapshot.sequence;
  previousRpmTimingMs = nowMs;

  snapshot.rpmSequence = rpmSnapshot.sequence;
  snapshot.rotorTorqueX_Nm = rotorModelState.torqueBodyNm[0];
  snapshot.rotorTorqueY_Nm = rotorModelState.torqueBodyNm[1];

  for (int motor = 0; motor < 4; ++motor)
  {
    const BidirectionalDShot::MotorRpmSample &source =
      rpmSnapshot.motor[motor];
    AsyncRpmMotorSnapshot &destination = snapshot.rpmMotor[motor];

    destination.dshotCommand = source.dshotCommand;
    destination.rpmRaw = source.rpmRaw;
    destination.rpmFiltered = source.rpmFiltered;
    destination.omegaRadPerSec = source.omegaRadPerSec;
    destination.valid = source.valid;
    destination.lastPacketValid = source.lastPacketValid;
    destination.lost = source.lost;
    destination.packetQuality = source.packetQuality;
    destination.ageUs = source.ageUs;
    strncpy(
      destination.statusName,
      BidirectionalDShot::statusName(source.status),
      sizeof(destination.statusName) - 1
    );
    destination.statusName[sizeof(destination.statusName) - 1] = '\0';
    destination.measuredSource = rotorModelState.measured[motor];
    destination.omegaPerDshot = rotorModelState.omegaPerDshot[motor];
  }

  asyncDebug.publish(snapshot);
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(100);

  // Start the verified half-duplex motor service before the long sensor
  // calibration so powered Bluejay ESCs receive command-zero frames early.
  if (!bidirectionalDshot.begin())
  {
    Serial.println();
    Serial.print("BIDIRECTIONAL DSHOT INITIALIZATION FAILED: ");
    Serial.println(bidirectionalDshot.lastError());
    Serial.println("The original ESP32 needs all eight RMT channels free.");
    while (true) delay(1000);
  }
  bidirectionalDshot.stopAll();
  delay(900);

  Serial.println();
  Serial.println("ESP32 ESKF + MODEL-BASED QUADROTOR CONTROLLER");
  Serial.println("BIDIRECTIONAL DSHOT300 + BLUEJAY RPM FEEDBACK");
  Serial.println("REMOVE PROPELLERS FOR INITIAL TESTING");
  Serial.println();

  Serial.println("Motor pins:");
  Serial.println("M1 = GPIO26 (front-left, CCW)");
  Serial.println("M2 = GPIO14 (front-right, CW)");
  Serial.println("M3 = GPIO27 (rear-right, CCW)");
  Serial.println("M4 = GPIO25 (rear-left, CW)");
  Serial.println("Bluejay pole counts: 14 / 14 / 14 / 14");
  Serial.println("DShot/RPM service: 500 Hz on a dedicated FreeRTOS task.");
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

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
 

  if (!setupMPU())
  {
    Serial.println("MPU6050 not detected.");
    while (true) delay(1000);
  }

  if (!calibrateIMU())
  {
    while (true) delay(1000);
  }

  Serial.println("Initializing VL53L0X...");
  if (!initializeRangeFinder())
  {
    Serial.println("VL53L0X initialization failed.");
    while (true) delay(1000);
  }
  Serial.println("VL53L0X initialized.");
  Serial.print("VL53L0X mount offset X/Y [m]: ");
  Serial.print(RANGE_OFFSET_X_M, 3);
  Serial.print(" / ");
  Serial.println(RANGE_OFFSET_Y_M, 3);
  Serial.print("VL53L0X lens above landing gear [m]: ");
  Serial.println(RANGE_SENSOR_ABOVE_GEAR_M, 3);
  Serial.println("Raw sensor-to-floor range remains the altitude convention.");

  Serial.println("Initializing QMC5883P...");
  magYawFusionEnabled = false;
  yawHeadingHoldRequested = false;
  yawHeadingHoldEnabled = false;

  // Required-sensor startup policy: do not continue to the armable controller
  // until the QMC both accepts 0xCB and actually produces an XYZ measurement.
  magSensorInitialized = initializeMagnetometerReliableStartup();

  Serial.println("QMC5883P initialized and verified at 0x2C, continuous 100 Hz.");
  Serial.println("Mounted calibration: bias 48.0/-12.5/-281.0, scale 0.980290/1.020518/1.009972.");
  Serial.println("Mount mapping: body X=-magX, body Y=-magY, body Z=+magZ.");

  Serial.println("Initializing PMW3901...");
  if (!initializeOpticalFlow())
  {
    Serial.println("PMW3901 initialization failed.");
    while (true) delay(1000);
  }
  Serial.println("PMW3901 initialized.");
  Serial.println("PMW3901 commissioned map: MAP1 (+rawX->+bodyX, -rawY->bodyY).");
  Serial.print("PMW3901 roll/pitch rotation gains: ");
  Serial.print(FLOW_ROLL_COMP_GAIN, 2);
  Serial.print(" / ");
  Serial.println(FLOW_PITCH_COMP_GAIN, 2);
  Serial.print("PMW3901 physical offset X/Y [m]: ");
  Serial.print(PMW_OFFSET_X_M, 3);
  Serial.print(" / ");
  Serial.println(PMW_OFFSET_Y_M, 3);
  Serial.print("PMW3901 yaw flow trust soft/hard [deg/s]: ");
  Serial.print(FLOW_YAW_RATE_SOFT_DPS, 1);
  Serial.print(" / ");
  Serial.println(FLOW_YAW_RATE_HARD_DPS, 1);

  Serial.println("Sending DShot stop for five seconds.");
  const uint32_t stopStartMs = millis();

  while (static_cast<uint32_t>(millis() - stopStartMs) < 5000)
  {
    sendAllMotors(DSHOT_STOP);
    updateRangeSensor(micros(), millis());
    drainOpticalFlowDuringSetup(micros());
    delay(2);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Receiver MAC:");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW initialization failed.");

    while (true)
    {
      sendAllMotors(DSHOT_STOP);
      delay(2);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  previousControlUs = nowUs;
  previousSetupFlowDrainUs = nowUs;
  // Stagger the two 100 Hz sensor jobs by 5 ms to reduce peak Core-1 load.
  previousRangePollUs = nowUs - (RANGE_POLL_PERIOD_US / 2U);
  previousMagPollUs = nowUs - (MAG_POLL_PERIOD_US / 2U);
  previousDebugMs = nowMs;
  flightEnableMs = nowMs;

  portENTER_CRITICAL(&espNowMux);
  lastPacketTimeMs = nowMs;
  packetReceived = false;
  portEXIT_CRITICAL(&espNowMux);

  resetFlightNavigationState(nowMs);

  Serial.println();
  Serial.println("Flight enable:");
  Serial.println("  Use transmitter enable at centered throttle.");
  Serial.print("  Startup ramp DShot ");
  Serial.print(ENABLE_RAMP_START_DSHOT);
  Serial.print(" -> ");
  Serial.print(THROTTLE_CENTER_DSHOT);
  Serial.print(" in ");
  Serial.print(ENABLE_RAMP_DURATION_MS);
  Serial.println(" ms.");
  Serial.print("  Estimator/controller: ");
  Serial.print(CONTROL_FREQUENCY_HZ);
  Serial.println(" Hz; DShot/RPM service: 500 Hz.");
  Serial.println("  VL53L0X ready poll: 50 Hz (20 ms period) on the flight-loop core.");
  Serial.println("  QMC5883P: 100 Hz on the same I2C bus; ESKF quaternion tilt compensation.");
  Serial.println("  Commissioning mode: magnetic yaw fusion starts OFF.");
  Serial.println("  Yaw heading hold starts OFF; a manual yawhold request survives arming and activates after the startup ramp.");
  Serial.println("  Centered sticks hold heading and altitude; XY remains blocked until bench commissioning.");
  Serial.println("  Roll/pitch commands move the X/Y target; throttle moves altitude target; yaw release captures heading.");
  Serial.println("  PMW3901 SPI acquisition: 100 Hz on the opposite core; ESKF correction stays on the flight core.");
  Serial.println("  RPM stale timeout: 20 ms; smooth motor-model fallback afterward.");
  Serial.println("  XY hold: MAP1/rotation compensation commissioned; SAFE INTERLOCK still OFF until xycommission on.");
  Serial.println("  PMW health accepts coherent 0x30/0xB0 bursts; only B0 updates velocity.");
  Serial.println("  Altitude hold: automatic ESKF Z/VZ capture at 0.50 m for 300 ms.");
  Serial.print("  Motion-model feed-forward starts OFF; configured blend motor=");
  Serial.print(motionModelMotorBlend, 2);
  Serial.print(", rate=");
  Serial.println(motionModelRateBlend, 2);
  Serial.println("  Long range loss: base DShot 450 controlled descent.");
  Serial.println();

  Serial.println("Serial commands:");
  Serial.println("  stop / kill / disarm");
  Serial.println("  level / status / navreset / eskf / rpm / mag / range");
  Serial.println("  mag on / mag off / magref / magreset / i2cdiag");
  Serial.println("  yawhold on / yawhold off");
  Serial.println("  eatt / eatt on / eatt off");
  Serial.println("  pid / outer / model / model on / model off / pidreset");
  Serial.println("  ffblend 0..0.50 / rateff 0..0.80");
  Serial.println("  kp/ki/kd, rkp/rki/rkd, pkp/pki/pkd");
  Serial.println("  ykp/yki/ykd/yhkp, akp/rakp/pakp");
  Serial.println("  xyposkp/xyvelkp/xyacckp, zposkp/zvelkp/zvelki");
  Serial.println("  flow / flowstatus / flowmap 0..7 / xycommission on|off");
  Serial.println();

  printPIDGains();
  printOuterGains();
  printModelSettings();
  printEskfStatus();
  printRpmStatus();
  printMagStatus();

  // setup() is Arduino's loop task, so this selects the opposite processor.
  const BaseType_t loopCore = xPortGetCoreID();
  const BaseType_t backgroundCore = loopCore == 0 ? 1 : 0;

  if (!startOpticalFlowTask(backgroundCore))
  {
    Serial.println("PMW3901 OPPOSITE-CORE TASK INITIALIZATION FAILED.");
    while (true)
    {
      sendAllMotors(DSHOT_STOP);
      delay(2);
    }
  }

  if (!asyncDebug.begin(Serial, backgroundCore, 1, 12288))
  {
    Serial.println("ASYNC DEBUG TASK INITIALIZATION FAILED.");
    while (true)
    {
      sendAllMotors(DSHOT_STOP);
      delay(2);
    }
  }

  asyncDebugStarted = true;
  runtimeLogf(
    "ASYNC TASKS ACTIVE: loop core %d, PMW3901 SPI core %d, debug/UART TX+RX core %d.",
    static_cast<int>(loopCore),
    static_cast<int>(backgroundCore),
    static_cast<int>(backgroundCore)
  );
}

// ================================================================
// LOOP
// ================================================================

void loop()
{
  const uint32_t loopActiveStartUs = micros();

  uint32_t probeStartUs = micros();
  processQueuedSerialCommand();
  recordTiming(timingCommandProcessing, micros() - probeStartUs);

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  bool hasPacket = false;
  uint32_t packetTimeCopy = 0;

  probeStartUs = micros();
  portENTER_CRITICAL(&espNowMux);
  memcpy(&controlData, &receivedData, sizeof(ControlData));
  hasPacket = packetReceived;
  packetTimeCopy = lastPacketTimeMs;
  portEXIT_CRITICAL(&espNowMux);
  recordTiming(timingEspNowCopy, micros() - probeStartUs);

  const bool signalLost =
    !hasPacket ||
    static_cast<uint32_t>(nowMs - packetTimeCopy) > FAILSAFE_TIMEOUT_MS;

  if (
    static_cast<uint32_t>(nowUs - previousControlUs) >=
    CONTROL_PERIOD_US
  )
  {
    const uint32_t controlBlockStartUs = micros();

    const uint32_t rawControlDtUs =
      static_cast<uint32_t>(nowUs - previousControlUs);

    if (rawControlDtUs > maximumObservedControlDtUs)
    {
      maximumObservedControlDtUs = rawControlDtUs;
    }
    if (rawControlDtUs < minimumObservedControlDtUs)
    {
      minimumObservedControlDtUs = rawControlDtUs;
    }

    if (rawControlDtUs > CONTROL_PERIOD_US * 2U)
    {
      controlOverrunCount++;
    }

    float dt = static_cast<float>(rawControlDtUs) * 1.0e-6f;
    previousControlUs = nowUs;
    dt = clampFloat(dt, 0.005f, 0.020f);

    probeStartUs = micros();
    updateRotorFeedback(dt);
    recordTiming(timingRotorFeedback, micros() - probeStartUs);

    probeStartUs = micros();
    const bool imuValid = updateAttitude(
      dt,
      eskfActuatorProcessNoiseScale
    );
    recordTiming(timingAttitude, micros() - probeStartUs);

    const bool tiltStop = excessiveTiltDetected(nowMs);

    lastImuValidState = imuValid;
    lastTiltStopState = tiltStop;

    probeStartUs = micros();
    updateRemoteEnableState(
      signalLost,
      imuValid,
      tiltStop,
      nowMs
    );
    recordTiming(timingEnableState, micros() - probeStartUs);

    const bool safetyStop =
      signalLost ||
      !imuValid ||
      tiltStop ||
      controlData.kill ||
      serialStopLatch ||
      (!flightEnabled && !disableRampActive);

    lastSafetyStopState = safetyStop;

    if (safetyStop)
    {
      correctedRollStick = 0.0f;
      correctedPitchStick = 0.0f;
      correctedThrottleStick = 0.0f;
      correctedYawStick = 0.0f;

      desiredRollAngleDeg = 0.0f;
      desiredPitchAngleDeg = 0.0f;
      desiredYawRateDps = 0.0f;

      stopMotors();
    }
    else
    {
      probeStartUs = micros();
      updateFlightController(dt, nowMs);
      recordTiming(timingFlightController, micros() - probeStartUs);
    }

    // This measures only publishing the newest four commands to the DShot
    // task. It does NOT measure the RMT transaction inside BidirectionalDShot.
    probeStartUs = micros();
    sendMotorCommands();
    recordTiming(timingMotorPublish, micros() - probeStartUs);

    recordTiming(timingControlBlock, micros() - controlBlockStartUs);
  }

  // QMC5883P shares the same I2C bus and is sampled at 100 Hz. The read,
  // tilt compensation, and ESKF yaw measurement update all remain on this
  // flight-control core, so there are no cross-core estimator races.
  updateMagnetometer(nowUs, nowMs);
  serviceMagnetometerRecovery(nowMs);

  // VL53L0X remains on the flight-loop core because it shares the MPU6050 /
  // QMC5883P I2C bus. PMW3901 SPI acquisition remains on the opposite core.
  const bool rangePollDue =
    static_cast<uint32_t>(nowUs - previousRangePollUs) >=
      RANGE_POLL_PERIOD_US;
  probeStartUs = micros();
  updateRangeSensor(nowUs, nowMs);
  if (rangePollDue)
  {
    recordTiming(timingRangePoll, micros() - probeStartUs);
  }

  // ESKF state remains single-core owned. Only process a completed PMW sample
  // here; no SPI transaction is performed on the flight-control core.
  probeStartUs = micros();
  if (consumeOpticalFlowSample(nowMs))
  {
    recordTiming(timingFlowUpdate, micros() - probeStartUs);
  }

  if (
    static_cast<uint32_t>(nowMs - previousDebugMs) >=
    DEBUG_PERIOD_MS
  )
  {
    previousDebugMs = nowMs;

    const uint32_t publishStartUs = micros();
    publishAsyncTelemetry(
      signalLost,
      lastSafetyStopState,
      lastTiltStopState,
      lastImuValidState,
      nowMs
    );

    // The snapshot just queued contains the completed previous timing window.
    // Reset immediately; the cost of this publish becomes the first sample in
    // the next one-second window and is reported one second later.
    resetMainTimingWindow();
    recordTiming(timingAsyncPublish, micros() - publishStartUs);

    maximumObservedControlDtUs = 0;
    minimumObservedControlDtUs = UINT32_MAX;
  }

  // Active-loop time excludes this intentional idle delay.
  recordTiming(timingLoopActive, micros() - loopActiveStartUs);

  // Prevent loop() from spinning at 100% CPU and reduce timing jitter.
  delayMicroseconds(100);
}