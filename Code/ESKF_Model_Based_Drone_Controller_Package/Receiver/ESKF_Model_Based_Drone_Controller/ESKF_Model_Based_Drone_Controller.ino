/*
  ============================================================================
  FOOTNOTES / READ BEFORE TESTING
  ============================================================================

  1. REMOVE ALL PROPELLERS for the first power-up, sensor-validity test,
     motor-mixer test, and yaw-direction test. Then use a restrained/tethered
     test stand before attempting free flight.

  2. Architecture in this revision:
       IMU at 250 Hz -> 9-error-state ESKF prediction
       PMW3901 at 50 Hz -> asynchronous body-velocity update
       VL53L0X at 50 Hz -> asynchronous altitude/vertical-velocity update
       ESKF state -> X/Y/Z outer loops
       supplied inverse-dynamics model -> bounded rate and motor feed-forward
       preserved angle/rate PID -> final feedback stabilization and mixer

     The original MPU6050 signs, angle/rate gains, motor numbering, mixer
     polarities, ESP-NOW packet, ramps, and safety behavior are preserved.

  3. Required Arduino libraries:
       - DShotRMT
       - Bitcraze PMW3901
       - VL53L0X by Pololu

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
       yaw            -> desired yaw rate; the current transmitter sends zero.

  6. X/Y hold activates above 0.20 m after 10 consecutive valid flow samples.
     The current location is captured as the target. The target advances while
     a roll/pitch stick is held and stops when the stick is centered.

  7. Altitude hold activates at or above 0.50 m for 300 ms, after the startup
     throttle ramp has completed. It captures the current height and a hover
     throttle estimate limited to DShot 480..510. Altitude correction is
     limited to +/-40 DShot.

  8. Runtime range failure:
       - up to 500 ms: use the last valid measurement/control output;
       - longer than 500 ms: command base DShot 450 for a slow descent;
       - after 5 valid readings return: capture the recovered height as the
         new target instead of climbing back to the old target.

  9. Runtime optical-flow failure:
       - short loss: freeze position integration and decay velocity;
       - after 250 ms: disable X/Y hold;
       - after 10 good samples return: capture the current location and
         smoothly re-enable X/Y hold.
     While X/Y hold is unavailable, roll/pitch sticks fall back to the original
     direct angle command. Centered sticks therefore command level attitude.

 10. The VL53L0X and MPU6050 share I2C on SDA=22/SCL=21. The range sensor is
     polled for its data-ready register before calling the Pololu continuous
     read, so the 20 ms ranging cycle does not intentionally block the 250 Hz
     attitude loop waiting for a measurement.

 11. Yaw control is RATE DAMPING ONLY. The MPU6050 has no absolute heading
     reference, so yaw angle is dead-reckoned and will drift. The integrated
     gyro yaw is used only to rotate X/Y measurements into a local frame.

 12. The ESKF follows the predict -> asynchronous measurement update ->
     attitude-error injection/reset concept from koledickarlo/ESKF-ESP32. It
     is implemented with fixed-size arrays so this sketch remains a direct
     Arduino IDE build without an Eigen dependency. Optical-flow and ToF
     scale/noise still require validation on the actual airframe.

 13. The Python inverse-dynamics model is used as conservative two-degree-of-
     freedom feed-forward. It does not replace the tuned feedback PID. Because
     the ESCs provide no RPM telemetry, omega-to-DShot conversion is only a
     local hover approximation and is blended at a deliberately low default.

 14. Sensor offsets supplied for this airframe:
       PMW3901: X=0.00 m, Y=+0.04 m
       VL53L0X: X=0.00 m, Y=+0.07 m
       VL53L0X lens is about 0.03 m above the landing-gear bottom.
     The PMW3901 Y offset is used for yaw-rate lever-arm compensation. The
     altitude controller intentionally uses raw sensor-to-floor range, as
     requested; it does not subtract the 0.03 m landing-gear offset.

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
#include <DShotRMT.h>
#include <Bitcraze_PMW3901.h>
#include <VL53L0X.h>
#include <math.h>
#include "Eskf9.h"
#include "MotionModelFF.h"

// ================================================================
// MOTOR CONFIGURATION - PRESERVED
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
char serialBuffer[96];
size_t serialLength = 0;

// ================================================================
// I2C / MPU6050 CONFIGURATION - PRESERVED SIGNS
// ================================================================

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t VL53L0X_ADDR = 0x29;
static constexpr uint8_t VL53_RESULT_INTERRUPT_STATUS = 0x13;

static constexpr int SDA_PIN = 22;
static constexpr int SCL_PIN = 21;

static constexpr float GYRO_LSB_PER_DPS = 131.0f;
static constexpr bool IMU_UPSIDE_DOWN = true;

/*
  Existing tuned axis mapping; do not change without revalidating the
  original attitude controller.
*/
static constexpr float ROLL_GYRO_SIGN = -1.0f;
static constexpr float PITCH_GYRO_SIGN = +1.0f;
static constexpr float YAW_GYRO_SIGN = +1.0f;  // supplied: gzRaw is + clockwise

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
static constexpr float CONTROLLER_YAW_CW_TO_RH_SIGN = -1.0f;

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

static constexpr float COMPLEMENTARY_GYRO_WEIGHT = 0.985f;

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

static constexpr float ESKF_TOF_STD_M = 0.035f;
static constexpr float ESKF_RANGE_VELOCITY_STD_MPS = 0.20f;
static constexpr float ESKF_FLOW_VELOCITY_STD_MPS = 0.12f;
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
// PMW3901 CONFIGURATION
// ================================================================

static constexpr uint8_t FLOW_SCK_PIN = 18;
static constexpr uint8_t FLOW_MISO_PIN = 19;
static constexpr uint8_t FLOW_MOSI_PIN = 23;
static constexpr uint8_t FLOW_CS_PIN = 13;
static constexpr uint8_t FLOW_RST_PIN = 33;

static constexpr int8_t FLOW_X_DIRECTION = +1;
static constexpr int8_t FLOW_Y_DIRECTION = -1;

static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
static constexpr float RAD_TO_DEG_F = 180.0f / PI_F;

static constexpr float FLOW_EFFECTIVE_COUNTS = 30.0f;
static constexpr float FLOW_APERTURE_RAD = 4.2f * DEG_TO_RAD_F;
static constexpr float FLOW_RAD_PER_COUNT =
  FLOW_APERTURE_RAD / FLOW_EFFECTIVE_COUNTS;

static constexpr float FLOW_SCALE_X = 1.0f;
static constexpr float FLOW_SCALE_Y = 1.0f;

static constexpr float MIN_FLOW_RANGE_M = 0.080f;
static constexpr float MAX_FLOW_RANGE_M = 2.000f;
static constexpr int32_t FLOW_OUTLIER_LIMIT_COUNTS = 100;

static constexpr float PMW_OFFSET_X_M = 0.00f;
static constexpr float PMW_OFFSET_Y_M = 0.04f;

/*
  Rotational-flow compensation signs for this body convention.
  These are isolated constants so they can be reversed independently if a
  propeller-off hand-rotation test shows the compensation direction is wrong.
*/
static constexpr bool ENABLE_FLOW_GYRO_COMPENSATION = true;
static constexpr float FLOW_ROLL_COMP_SIGN = +1.0f;
static constexpr float FLOW_PITCH_COMP_SIGN = -1.0f;

Bitcraze_PMW3901 flow(FLOW_CS_PIN);

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

static constexpr float XY_HOLD_ACTIVATION_RANGE_M = 0.20f;
static constexpr uint8_t FLOW_RECOVERY_SAMPLES = 10;
static constexpr uint32_t FLOW_LOSS_DISABLE_MS = 250;
static constexpr uint32_t XY_HOLD_BLEND_MS = 300;
static constexpr float MAX_POSITION_TARGET_ERROR_M = 0.50f;
static constexpr float XY_POSITION_DEADBAND_M = 0.03f;
static constexpr float MAX_XY_CONTROLLER_ANGLE_DEG = 6.0f;
static constexpr float MAX_CONTROLLER_VELOCITY_MPS = 1.00f;
static constexpr float MAX_XY_ACCEL_ASSIST_ANGLE_DEG = 4.0f;

// Stronger X/Y hold gains. The accelerometer term acts only as fast damping;
// optical flow remains the long-term velocity and position reference.
float xyPositionKp = 0.35f;     // (m/s) per meter of position error
float xyVelocityKp = 12.00f;     // degrees per (m/s) of velocity error
float xyAccelerationKp = 0.00f; // degrees per (m/s^2), centered sticks only

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
uint8_t consecutiveValidFlowSamples = 0;
uint32_t lastValidFlowMs = 0;
bool lastFlowSampleValid = false;

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

static constexpr int HOVER_DSHOT_MIN = 480;
static constexpr int HOVER_DSHOT_MAX = 510;
static constexpr float MAX_ALTITUDE_CORRECTION_DSHOT = 40.0f;
static constexpr float ALTITUDE_INTEGRAL_LIMIT = 1.00f;

// Conservative starting gains; tune after validating range and velocity signs.
float altitudePositionKp = 1.00f;  // (m/s) per meter
float altitudeVelocityKp = 55.0f;  // DShot per (m/s)
float altitudeVelocityKi = 18.0f;  // DShot per integrated velocity error

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

float yawRateKp = 1.20f;
float yawRateKi = 0.02f;
float yawRateKd = 0.000f;

float yawRateIntegral = 0.0f;
float previousYawRateError = 0.0f;
float filteredYawDerivative = 0.0f;

static constexpr float YAW_INTEGRAL_LIMIT = 25.0f;
static constexpr float MAX_YAW_CORRECTION = 120.0f;

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

static constexpr uint32_t CONTROL_FREQUENCY_HZ = 250;
static constexpr uint32_t CONTROL_PERIOD_US =
  1000000UL / CONTROL_FREQUENCY_HZ;

static constexpr uint32_t FLOW_PERIOD_US = 20000;
static constexpr uint32_t RANGE_POLL_PERIOD_US = 4000;
static constexpr uint32_t DEBUG_PERIOD_MS = 1000;

uint32_t previousControlUs = 0;
uint32_t previousFlowUs = 0;
uint32_t previousRangePollUs = 0;
uint32_t previousDebugMs = 0;

uint32_t controlOverrunCount = 0;
uint32_t maximumObservedControlDtUs = 0;

bool lastImuValidState = false;
bool lastTiltStopState = false;
bool lastSafetyStopState = true;

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
  targetNavX_m = positionNavX_m;
  targetNavY_m = positionNavY_m;
  controllerBodyVelocityTargetX_mps = 0.0f;
  controllerBodyVelocityTargetY_mps = 0.0f;
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
  accumulatedFlowRollRad = 0.0f;
  accumulatedFlowPitchRad = 0.0f;
  accumulatedFlowYawCwRad = 0.0f;

  consecutiveValidFlowSamples = 0;
  lastValidFlowMs = nowMs;
  lastFlowSampleValid = false;

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

  for (int index = 0; index < 30; index++)
  {
    sendAllMotors(DSHOT_STOP);
    delayMicroseconds(500);
  }
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

  axRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  ayRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  azRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

  Wire.read();
  Wire.read();

  gxRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  gyRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  gzRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

  return true;
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

bool updateAttitude(float dt)
{
  if (!readMPURaw())
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

  // Like the reference ESKF architecture, lateral accelerometer integration
  // is suppressed in flight because optical flow is the reliable long-term
  // lateral observation.  Z specific force remains in the propagation.
  const bool airborneControlActive = flightEnabled || disableRampActive;

  const float eskfSpecificForceBody[3] = {
    airborneControlActive ? 0.0f : bodySpecificForceX_mps2,
    airborneControlActive ? 0.0f : bodySpecificForceY_mps2,
    bodySpecificForceZ_mps2
  };

  stateEstimator.predict(
    eskfSpecificForceBody,
    gyroBodyRadPerSecond,
    dt
  );

  static uint8_t gravityUpdateDivider = 0;
  gravityUpdateDivider++;
  if ((gravityUpdateDivider & 1U) == 0U)
  {
    stateEstimator.updateGravityDirection(
      measuredSpecificForceBody,
      airborneControlActive
        ? ESKF_GRAVITY_STD_IN_FLIGHT
        : ESKF_GRAVITY_STD_ON_GROUND
    );
  }

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
    yawAngleCwDeg = eskfYawCwDeg;
  }
  else
  {
    // Independent complementary fallback preserves the already-proven
    // attitude loop if an ESKF innovation or sign check is temporarily poor.
    measuredRollAngleDeg = complementaryRollAngleDeg;
    measuredPitchAngleDeg = complementaryPitchAngleDeg;
    yawAngleCwDeg = wrapAngle180(
      yawAngleCwDeg + measuredYawRateDps * dt
    );
  }

  // Synchronize gyro integration with the PMW3901 count interval.
  accumulatedFlowRollRad += measuredRollRateDps * DEG_TO_RAD_F * dt;
  accumulatedFlowPitchRad += measuredPitchRateDps * DEG_TO_RAD_F * dt;
  accumulatedFlowYawCwRad += measuredYawRateDps * DEG_TO_RAD_F * dt;

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
    return false;
  }

  if (Wire.requestFrom(VL53L0X_ADDR, static_cast<uint8_t>(1)) != 1)
  {
    return false;
  }

  ready = (Wire.read() & 0x07U) != 0U;
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

  bool dataReady = false;
  if (!readVL53DataReady(dataReady))
  {
    registerInvalidRangeSample();
    return;
  }

  if (!dataReady)
  {
    // Not an invalid sample; the current ranging cycle is simply unfinished.
    return;
  }

  const uint16_t rangeMillimeters =
    rangeSensor.readRangeContinuousMillimeters();

  const bool valid =
    !rangeSensor.timeoutOccurred() &&
    rangeMillimeters >= static_cast<uint16_t>(MIN_VALID_RANGE_M * 1000.0f) &&
    rangeMillimeters <= static_cast<uint16_t>(MAX_VALID_RANGE_M * 1000.0f);

  if (!valid)
  {
    registerInvalidRangeSample();
    return;
  }

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

// ================================================================
// PMW3901 UPDATE AND LOCAL POSITION ESTIMATE
// ================================================================

void updateOpticalFlow(uint32_t nowUs, uint32_t nowMs)
{
  if (static_cast<uint32_t>(nowUs - previousFlowUs) < FLOW_PERIOD_US)
  {
    return;
  }

  const float flowDt =
    static_cast<float>(nowUs - previousFlowUs) * 1.0e-6f;

  previousFlowUs = nowUs;

  int16_t rawDeltaX = 0;
  int16_t rawDeltaY = 0;
  flow.readMotionCount(&rawDeltaX, &rawDeltaY);

  const int32_t deltaXCounts =
    FLOW_X_DIRECTION * static_cast<int32_t>(rawDeltaX);

  const int32_t deltaYCounts =
    FLOW_Y_DIRECTION * static_cast<int32_t>(rawDeltaY);

  const float rollDeltaRad = accumulatedFlowRollRad;
  const float pitchDeltaRad = accumulatedFlowPitchRad;
  const float yawDeltaCwRad = accumulatedFlowYawCwRad;

  accumulatedFlowRollRad = 0.0f;
  accumulatedFlowPitchRad = 0.0f;
  accumulatedFlowYawCwRad = 0.0f;

  const bool timingValid = flowDt >= 0.010f && flowDt <= 0.100f;
  const bool countsValid =
    deltaXCounts > -FLOW_OUTLIER_LIMIT_COUNTS &&
    deltaXCounts <  FLOW_OUTLIER_LIMIT_COUNTS &&
    deltaYCounts > -FLOW_OUTLIER_LIMIT_COUNTS &&
    deltaYCounts <  FLOW_OUTLIER_LIMIT_COUNTS;

  const bool heightValid =
    rangeControlUsable(nowMs) &&
    filteredRangeMeters >= MIN_FLOW_RANGE_M &&
    filteredRangeMeters <= MAX_FLOW_RANGE_M;

  const bool sampleValid = timingValid && countsValid && heightValid;
  lastFlowSampleValid = sampleValid;

  if (!sampleValid)
  {
    consecutiveValidFlowSamples = 0;

    measuredBodyVelocityX_mps *= 0.85f;
    measuredBodyVelocityY_mps *= 0.85f;

    if (
      xyHoldActive &&
      static_cast<uint32_t>(nowMs - lastValidFlowMs) >
        FLOW_LOSS_DISABLE_MS
    )
    {
      deactivateXYHold();
      Serial.println("XY HOLD DISABLED: optical-flow timeout.");
    }

    return;
  }

  float deltaBodyX_m =
    static_cast<float>(deltaXCounts) *
    FLOW_RAD_PER_COUNT *
    filteredRangeMeters *
    FLOW_SCALE_X;

  float deltaBodyY_m =
    static_cast<float>(deltaYCounts) *
    FLOW_RAD_PER_COUNT *
    filteredRangeMeters *
    FLOW_SCALE_Y;

  if (ENABLE_FLOW_GYRO_COMPENSATION)
  {
    // Remove apparent floor motion created by roll/pitch camera rotation.
    deltaBodyX_m +=
      FLOW_ROLL_COMP_SIGN * filteredRangeMeters * rollDeltaRad;

    deltaBodyY_m +=
      FLOW_PITCH_COMP_SIGN * filteredRangeMeters * pitchDeltaRad;
  }

  // Remove rigid-body velocity at the PMW sensor caused by its lever arm.
  // Positive yawDeltaCwRad is clockwise viewed from above.
  deltaBodyX_m -= PMW_OFFSET_Y_M * yawDeltaCwRad;
  deltaBodyY_m += PMW_OFFSET_X_M * yawDeltaCwRad;

  const float instantBodyVelocityX = deltaBodyX_m / flowDt;
  const float instantBodyVelocityY = deltaBodyY_m / flowDt;

  static constexpr float FLOW_VELOCITY_FILTER_ALPHA = 0.30f;

  measuredBodyVelocityX_mps +=
    FLOW_VELOCITY_FILTER_ALPHA *
    (instantBodyVelocityX - measuredBodyVelocityX_mps);

  measuredBodyVelocityY_mps +=
    FLOW_VELOCITY_FILTER_ALPHA *
    (instantBodyVelocityY - measuredBodyVelocityY_mps);

  const float flowNoiseScale = 1.0f + 0.015f * fmaxf(
    fabsf(measuredRollAngleDeg),
    fabsf(measuredPitchAngleDeg)
  );

  const float flowVelocityStd = clampFloat(
    ESKF_FLOW_VELOCITY_STD_MPS * flowNoiseScale,
    ESKF_FLOW_VELOCITY_STD_MPS,
    0.35f
  );

  const bool estimatorAcceptedFlow = stateEstimator.updateBodyVelocity(
    measuredBodyVelocityX_mps,
    measuredBodyVelocityY_mps,
    flowVelocityStd,
    flowVelocityStd
  );

  if (estimatorAcceptedFlow)
  {
    eskfFlowVelocityReady = true;
    fusedVelocityInitialized = true;
    synchronizeEstimatorState();
    lastValidFlowMs = nowMs;

    if (consecutiveValidFlowSamples < FLOW_RECOVERY_SAMPLES)
    {
      consecutiveValidFlowSamples++;
    }
  }
  else
  {
    // A geometrically valid PMW sample that fails the Kalman innovation gate
    // is still an estimator outlier and must not keep XY hold alive.
    lastFlowSampleValid = false;
    consecutiveValidFlowSamples = 0;
  }
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

  Serial.print("XY HOLD ACTIVE at X=");
  Serial.print(positionNavX_m, 3);
  Serial.print(", Y=");
  Serial.println(positionNavY_m, 3);
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

  // Original direct-angle behavior, retained as fallback and blend source.
  const float manualRollAngleDeg =
    -correctedRollStick * MAX_ROLL_ANGLE_DEG;

  const float manualPitchAngleDeg =
    correctedPitchStick * MAX_PITCH_ANGLE_DEG;

  const bool flowRecovered =
    consecutiveValidFlowSamples >= FLOW_RECOVERY_SAMPLES;

  if (
    flightEnabled &&
    !disableRampActive &&
    !xyHoldActive &&
    flowRecovered &&
    eskfStateHealthy &&
    eskfFlowVelocityReady &&
    rangeControlUsable(nowMs) &&
    filteredRangeMeters >= XY_HOLD_ACTIVATION_RANGE_M
  )
  {
    activateXYHold(nowMs);
  }

  if (
    xyHoldActive &&
    (
      !rangeControlUsable(nowMs) ||
      !eskfStateHealthy ||
      !eskfFlowVelocityReady ||
      static_cast<uint32_t>(nowMs - lastValidFlowMs) >
        FLOW_LOSS_DISABLE_MS
    )
  )
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

  // Keep the stored target fixed. Limit only the error used by the
  // controller so a large displacement cannot demand an unsafe velocity.
  const float rawPositionErrorX = targetNavX_m - positionNavX_m;
  const float rawPositionErrorY = targetNavY_m - positionNavY_m;

  const float limitedPositionErrorX = clampFloat(
    rawPositionErrorX,
    -MAX_POSITION_TARGET_ERROR_M,
    MAX_POSITION_TARGET_ERROR_M
  );

  const float limitedPositionErrorY = clampFloat(
    rawPositionErrorY,
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

  const float controllerVelocityX = fusedVelocityInitialized
    ? fusedBodyVelocityX_mps
    : measuredBodyVelocityX_mps;

  const float controllerVelocityY = fusedVelocityInitialized
    ? fusedBodyVelocityY_mps
    : measuredBodyVelocityY_mps;

  const float bodyVelocityErrorX =
    controllerBodyVelocityTargetX_mps - controllerVelocityX;

  const float bodyVelocityErrorY =
    controllerBodyVelocityTargetY_mps - controllerVelocityY;

  // Acceleration damping is strongest with centered sticks. It produces an
  // immediate angle command before optical flow has accumulated velocity.
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

  const float holdRollAngleDeg = clampFloat(
    -xyVelocityKp * bodyVelocityErrorX + rollAccelerationAssistDeg,
    -MAX_XY_CONTROLLER_ANGLE_DEG,
    MAX_XY_CONTROLLER_ANGLE_DEG
  );

  const float holdPitchAngleDeg = clampFloat(
    xyVelocityKp * bodyVelocityErrorY + pitchAccelerationAssistDeg,
    -MAX_XY_CONTROLLER_ANGLE_DEG,
    MAX_XY_CONTROLLER_ANGLE_DEG
  );

  const float blend = clampFloat(
    static_cast<float>(nowMs - xyHoldBlendStartMs) /
      static_cast<float>(XY_HOLD_BLEND_MS),
    0.0f,
    1.0f
  );

  desiredRollAngleDeg =
    manualRollAngleDeg + blend * (holdRollAngleDeg - manualRollAngleDeg);

  desiredPitchAngleDeg =
    manualPitchAngleDeg + blend * (holdPitchAngleDeg - manualPitchAngleDeg);
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

  Serial.print("ALTITUDE HOLD ACTIVE at ");
  Serial.print(altitudeTargetM, 3);
  Serial.print(" m, hover DShot estimate ");
  Serial.println(learnedHoverDshot);
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

    Serial.print("RANGE RECOVERED: altitude target recaptured at ");
    Serial.print(altitudeTargetM, 3);
    Serial.println(" m.");
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

void calculateDesiredCommands(float dt, uint32_t nowMs)
{
  throttleTargetDshot = mapThrottleToDshot(controlData.throttle);

  correctedYawStick = applyCenteredStickDeadband(
    controlData.yaw,
    YAW_STICK_DEADBAND
  );

  desiredYawRateDps = correctedYawStick * MAX_YAW_RATE_DPS;

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

      Serial.println("DISABLE RAMP COMPLETE: motors stopped.");
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

      Serial.println("ENABLE RAMP COMPLETE: live throttle/altitude logic active.");
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
    hoverReferenceDshot
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

  Serial.print("REMOTE ENABLE OFF: ramping base DShot ");
  Serial.print(disableRampStartDshot);
  Serial.print(" -> ");
  Serial.print(DISABLE_RAMP_END_DSHOT);
  Serial.print(" over ");
  Serial.print(DISABLE_RAMP_DURATION_MS);
  Serial.println(" ms, then motor stop.");
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
    if (flightEnabled || enableRampActive || disableRampActive)
    {
      Serial.print("IMMEDIATE FLIGHT STOP:");

      if (controlData.kill) Serial.println("REMOTE_KILL");
      else if (signalLost) Serial.println("SIGNAL_LOST");
      else if (!imuValid) Serial.println("IMU_FAILURE");
      else if (tiltStop) Serial.println("EXCESSIVE_TILT");
      else Serial.println("SERIAL_STOP");
    }

    clearFlightEnableState(nowMs);
  }
  else if (
    enableFallingEdge &&
    flightEnabled &&
    !disableRampActive
  )
  {
    startDisableRamp(nowMs);
  }
  else if (enableRisingEdge && disableRampActive)
  {
    Serial.println(
      "ENABLE IGNORED DURING RAMP DOWN: leave switch OFF until motors stop, then enable again."
    );
  }
  else if (
    enableRisingEdge &&
    !flightEnabled &&
    !disableRampActive
  )
  {
    if (throttleCenteredForEnable(controlData.throttle))
    {
      flightEnabled = true;
      flightEnableMs = nowMs;

      disableRampActive = false;
      disableRampProgress = 0.0f;

      enableRampActive = true;
      enableRampStartMs = nowMs;
      enableRampProgress = 0.0f;

      throttleTargetDshot = THROTTLE_CENTER_DSHOT;
      baseThrottleDshot = ENABLE_RAMP_START_DSHOT;

      resetFlightNavigationState(nowMs);

      Serial.print("REMOTE ENABLE ACCEPTED: ramping base DShot ");
      Serial.print(ENABLE_RAMP_START_DSHOT);
      Serial.print(" -> ");
      Serial.print(THROTTLE_CENTER_DSHOT);
      Serial.print(" over ");
      Serial.print(ENABLE_RAMP_DURATION_MS);
      Serial.println(" ms.");
    }
    else
    {
      Serial.print("REMOTE ENABLE REJECTED: center throttle first. THR:");
      Serial.println(controlData.throttle, 3);
    }
  }

  previousRemoteEnable = remoteEnable;
}

// ================================================================
// LIVE TUNING / STATUS
// ================================================================

void printPIDGains()
{
  Serial.print("PID,RKP:"); Serial.print(rollRateKp, 5);
  Serial.print(",RKI:"); Serial.print(rollRateKi, 5);
  Serial.print(",RKD:"); Serial.print(rollRateKd, 5);
  Serial.print(",PKP:"); Serial.print(pitchRateKp, 5);
  Serial.print(",PKI:"); Serial.print(pitchRateKi, 5);
  Serial.print(",PKD:"); Serial.print(pitchRateKd, 5);
  Serial.print(",RAKP:"); Serial.print(rollAngleKp, 3);
  Serial.print(",PAKP:"); Serial.print(pitchAngleKp, 3);
  Serial.print(",YKP:"); Serial.print(yawRateKp, 4);
  Serial.print(",YKI:"); Serial.print(yawRateKi, 4);
  Serial.print(",YKD:"); Serial.println(yawRateKd, 5);
}

void printOuterGains()
{
  Serial.print("OUTER,XYPOSKP:"); Serial.print(xyPositionKp, 4);
  Serial.print(",XYVELKP:"); Serial.print(xyVelocityKp, 4);
  Serial.print(",XYACCKP:"); Serial.print(xyAccelerationKp, 4);
  Serial.print(",ZPOSKP:"); Serial.print(altitudePositionKp, 4);
  Serial.print(",ZVELKP:"); Serial.print(altitudeVelocityKp, 4);
  Serial.print(",ZVELKI:"); Serial.println(altitudeVelocityKi, 4);
}

void printModelSettings()
{
  Serial.print("MODEL,EN:"); Serial.print(motionModelEnabled ? 1 : 0);
  Serial.print(",VALID:"); Serial.print(motionModelOutput.valid ? 1 : 0);
  Serial.print(",SAT:"); Serial.print(motionModelOutput.saturated ? 1 : 0);
  Serial.print(",MOTOR_BLEND:"); Serial.print(motionModelMotorBlend, 3);
  Serial.print(",RATE_BLEND:"); Serial.print(motionModelRateBlend, 3);
  Serial.print(",COLL:"); Serial.print(appliedModelCollectiveDshot, 2);
  Serial.print(",D1:"); Serial.print(appliedModelMotorDeltaDshot[0], 2);
  Serial.print(",D2:"); Serial.print(appliedModelMotorDeltaDshot[1], 2);
  Serial.print(",D3:"); Serial.print(appliedModelMotorDeltaDshot[2], 2);
  Serial.print(",D4:"); Serial.println(appliedModelMotorDeltaDshot[3], 2);
}

void printEskfStatus()
{
  Serial.print("ESKF,HEALTHY:"); Serial.print(stateEstimator.healthy() ? 1 : 0);
  Serial.print(",ATT_EN:"); Serial.print(eskfAttitudeControlEnabled ? 1 : 0);
  Serial.print(",ATT_USED:"); Serial.print(eskfAttitudeUsed ? 1 : 0);
  Serial.print(",FLOW_READY:"); Serial.print(eskfFlowVelocityReady ? 1 : 0);
  Serial.print(",RANGE_READY:"); Serial.print(eskfRangeReady ? 1 : 0);
  Serial.print(",P:"); Serial.print(eskfPitchAngleDeg, 2);
  Serial.print(",R:"); Serial.print(eskfRollAngleDeg, 2);
  Serial.print(",Y:"); Serial.print(eskfYawCwDeg, 2);
  Serial.print(",X:"); Serial.print(stateEstimator.positionX(), 3);
  Serial.print(",Y_POS:"); Serial.print(stateEstimator.positionY(), 3);
  Serial.print(",Z:"); Serial.print(stateEstimator.positionZ(), 3);
  Serial.print(",VX:"); Serial.print(stateEstimator.velocityX(), 3);
  Serial.print(",VY:"); Serial.print(stateEstimator.velocityY(), 3);
  Serial.print(",VZ:"); Serial.print(stateEstimator.velocityZ(), 3);
  Serial.print(",NIS_SIGMA:"); Serial.print(stateEstimator.lastInnovationRatio(), 2);
  Serial.print(",ACC:"); Serial.print(stateEstimator.acceptedUpdates());
  Serial.print(",REJ:"); Serial.println(stateEstimator.rejectedUpdates());
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
    Serial.print("Invalid value for ");
    Serial.println(name);
    return true;
  }

  if (requested < minimum || requested > maximum)
  {
    Serial.print(name);
    Serial.print(" must be between ");
    Serial.print(minimum, 5);
    Serial.print(" and ");
    Serial.println(maximum, 5);
    return true;
  }

  gain = requested;
  resetAttitudeControllers();
  resetYawController();

  Serial.print(name);
  Serial.print(" set to ");
  Serial.println(gain, 5);
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
      Serial.println("Applied KP to roll and pitch.");
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
      Serial.println("Applied KI to roll and pitch.");
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
      Serial.println("Applied KD to roll and pitch.");
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
      Serial.println("Applied angle KP to roll and pitch.");
      printPIDGains();
    }
    return true;
  }

  return false;
}

void printStatus()
{
  const uint32_t nowMs = millis();

  Serial.print("FLIGHT_EN:"); Serial.print(flightEnabled ? 1 : 0);
  Serial.print(",ESKF:"); Serial.print(eskfStateHealthy ? 1 : 0);
  Serial.print(",EATT_EN:"); Serial.print(eskfAttitudeControlEnabled ? 1 : 0);
  Serial.print(",EATT:"); Serial.print(eskfAttitudeUsed ? 1 : 0);
  Serial.print(",MODEL:"); Serial.print(motionModelEnabled ? 1 : 0);
  Serial.print(",MVALID:"); Serial.print(motionModelOutput.valid ? 1 : 0);
  Serial.print(",XY_HOLD:"); Serial.print(xyHoldActive ? 1 : 0);
  Serial.print(",ALT_HOLD:"); Serial.print(altitudeHoldActive ? 1 : 0);
  Serial.print(",RANGE_OK:"); Serial.print(rangeControlUsable(nowMs) ? 1 : 0);
  Serial.print(",RANGE_AGE:"); Serial.print(rangeAgeMs(nowMs));
  Serial.print(",FLOW_GOOD:");
  Serial.print(consecutiveValidFlowSamples >= FLOW_RECOVERY_SAMPLES ? 1 : 0);
  Serial.print(",RANGE_M:"); Serial.print(filteredRangeMeters, 3);
  Serial.print(",EZ:"); Serial.print(altitudeForControl(), 3);
  Serial.print(",VZ:"); Serial.print(verticalVelocityMps, 3);
  Serial.print(",X:"); Serial.print(positionNavX_m, 3);
  Serial.print(",Y:"); Serial.print(positionNavY_m, 3);
  Serial.print(",VX:"); Serial.print(measuredBodyVelocityX_mps, 3);
  Serial.print(",VY:"); Serial.print(measuredBodyVelocityY_mps, 3);
  Serial.print(",FVX:"); Serial.print(fusedBodyVelocityX_mps, 3);
  Serial.print(",FVY:"); Serial.print(fusedBodyVelocityY_mps, 3);
  Serial.print(",AX:"); Serial.print(filteredLinearAccelBodyX_mps2, 3);
  Serial.print(",AY:"); Serial.print(filteredLinearAccelBodyY_mps2, 3);
  Serial.print(",BASE:"); Serial.print(baseThrottleDshot);
  Serial.print(",FFCOLL:"); Serial.print(appliedModelCollectiveDshot, 2);
  Serial.print(",YAW_RATE:"); Serial.println(measuredYawRateDps, 2);
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

  if (strcasecmp(command, "model on") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      Serial.println("MODEL ON REJECTED: disable the motors first.");
      return;
    }

    motionModelEnabled = true;
    resetMotionModelFeedforward();
    Serial.println("Motion-model feed-forward enabled.");
    return;
  }

  if (strcasecmp(command, "model off") == 0)
  {
    motionModelEnabled = false;
    resetMotionModelFeedforward();
    Serial.println("Motion-model feed-forward disabled; PID remains active.");
    return;
  }

  if (strcasecmp(command, "eskf") == 0)
  {
    printEskfStatus();
    return;
  }

  if (strcasecmp(command, "eatt") == 0)
  {
    Serial.print("ESKF attitude feedback enabled: ");
    Serial.println(eskfAttitudeControlEnabled ? 1 : 0);
    printEskfStatus();
    return;
  }

  if (strcasecmp(command, "eatt on") == 0)
  {
    if (flightEnabled || disableRampActive)
    {
      Serial.println("EATT ON REJECTED: disable the motors first.");
      return;
    }

    if (!stateEstimator.healthy())
    {
      Serial.println("EATT ON REJECTED: ESKF is not healthy.");
      return;
    }

    eskfAttitudeControlEnabled = true;
    resetAttitudeControllers();
    Serial.println("ESKF attitude feedback enabled for the next flight.");
    return;
  }

  if (strcasecmp(command, "eatt off") == 0)
  {
    eskfAttitudeControlEnabled = false;
    eskfAttitudeUsed = false;
    resetAttitudeControllers();
    Serial.println("ESKF attitude feedback disabled; complementary attitude active.");
    return;
  }

  if (strcasecmp(command, "pidreset") == 0)
  {
    resetAttitudeControllers();
    resetYawController();
    altitudeVelocityIntegral = 0.0f;
    Serial.println("Controller states reset; gains unchanged.");
    return;
  }

  if (processPIDGainCommand(command))
  {
    return;
  }

  if (strcasecmp(command, "arm") == 0)
  {
    Serial.println("Serial arm removed. Use remote enable with throttle centered.");
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

    Serial.println(
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
    Serial.println("ESKF navigation origin and active targets recaptured.");
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
    Serial.println("Current orientation stored as level; ESKF attitude reset.");
    return;
  }

  Serial.println(
    "Commands: kill/stop/disarm, level, status, navreset, pid, outer, eskf, "
    "eatt/eatt on/eatt off, "
    "model/model on/model off, ffblend/rateff, kp/ki/kd, rkp/rki/rkd, "
    "pkp/pki/pkd, ykp/yki/ykd, akp/rakp/pakp, "
    "xyposkp/xyvelkp/xyacckp, zposkp/zvelkp/zvelki, pidreset"
  );
}

void readSerialCommands()
{
  while (Serial.available() > 0)
  {
    const char character = static_cast<char>(Serial.read());

    if (character == '\n')
    {
      serialBuffer[serialLength] = '\0';
      processSerialCommand(serialBuffer);
      serialLength = 0;
    }
    else if (character != '\r')
    {
      if (serialLength < sizeof(serialBuffer) - 1)
      {
        serialBuffer[serialLength++] = character;
      }
      else
      {
        serialLength = 0;
        Serial.println("Serial command too long.");
      }
    }
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

  portENTER_CRITICAL(&espNowMux);
  memcpy(&receivedData, &temporary, sizeof(ControlData));
  packetReceived = true;
  lastPacketTimeMs = millis();
  portEXIT_CRITICAL(&espNowMux);
}

// ================================================================
// COMPACT FLIGHT DEBUG
// ================================================================

void printDebug(
  bool signalLost,
  bool safetyStop,
  bool tiltStop,
  bool imuValid,
  uint32_t nowMs
)
{
  Serial.print("LOST:"); Serial.print(signalLost ? 1 : 0);
  Serial.print(",IMU:"); Serial.print(imuValid ? 1 : 0);
  Serial.print(",STOP:"); Serial.print(safetyStop ? 1 : 0);
  Serial.print(",TILT:"); Serial.print(tiltStop ? 1 : 0);
  Serial.print(",EN:"); Serial.print(flightEnabled ? 1 : 0);
  Serial.print(",EOK:"); Serial.print(eskfStateHealthy ? 1 : 0);
  Serial.print(",EATT:"); Serial.print(eskfAttitudeUsed ? 1 : 0);
  Serial.print(",MFF:"); Serial.print(motionModelOutput.valid ? 1 : 0);
  Serial.print(",MSAT:"); Serial.print(motionModelOutput.saturated ? 1 : 0);
  Serial.print(",XY:"); Serial.print(xyHoldActive ? 1 : 0);
  Serial.print(",ALT:"); Serial.print(altitudeHoldActive ? 1 : 0);
  Serial.print(",RNGOK:"); Serial.print(rangeControlUsable(nowMs) ? 1 : 0);
  Serial.print(",RNGA:"); Serial.print(rangeAgeMs(nowMs));
  Serial.print(",FLW:");
  Serial.print(consecutiveValidFlowSamples >= FLOW_RECOVERY_SAMPLES ? 1 : 0);
  Serial.print(",BASE:"); Serial.print(baseThrottleDshot);

  Serial.print(",RNG:"); Serial.print(filteredRangeMeters, 3);
  Serial.print(",Z:"); Serial.print(altitudeForControl(), 3);
  Serial.print(",ZT:"); Serial.print(altitudeTargetM, 3);
  Serial.print(",VZ:"); Serial.print(verticalVelocityMps, 3);
  Serial.print(",VZT:"); Serial.print(desiredVerticalVelocityMps, 3);
  Serial.print(",ZC:"); Serial.print(altitudeCorrectionDshot, 1);

  Serial.print(",X:"); Serial.print(positionNavX_m, 3);
  Serial.print(",XT:"); Serial.print(targetNavX_m, 3);
  Serial.print(",Y:"); Serial.print(positionNavY_m, 3);
  Serial.print(",YT:"); Serial.print(targetNavY_m, 3);
  Serial.print(",VX:"); Serial.print(measuredBodyVelocityX_mps, 3);
  Serial.print(",VXT:"); Serial.print(controllerBodyVelocityTargetX_mps, 3);
  Serial.print(",VY:"); Serial.print(measuredBodyVelocityY_mps, 3);
  Serial.print(",VYT:"); Serial.print(controllerBodyVelocityTargetY_mps, 3);
  Serial.print(",FVX:"); Serial.print(fusedBodyVelocityX_mps, 3);
  Serial.print(",FVY:"); Serial.print(fusedBodyVelocityY_mps, 3);
  Serial.print(",AX:"); Serial.print(filteredLinearAccelBodyX_mps2, 2);
  Serial.print(",AY:"); Serial.print(filteredLinearAccelBodyY_mps2, 2);

  Serial.print(",RA:"); Serial.print(measuredRollAngleDeg, 2);
  Serial.print(",RAT:"); Serial.print(desiredRollAngleDeg, 2);
  Serial.print(",PA:"); Serial.print(measuredPitchAngleDeg, 2);
  Serial.print(",PAT:"); Serial.print(desiredPitchAngleDeg, 2);
  Serial.print(",YR:"); Serial.print(measuredYawRateDps, 2);
  Serial.print(",YC:"); Serial.print(yawCorrection, 1);
  Serial.print(",EIR:"); Serial.print(stateEstimator.lastInnovationRatio(), 2);
  Serial.print(",FFC:"); Serial.print(appliedModelCollectiveDshot, 1);
  Serial.print(",FFD1:"); Serial.print(appliedModelMotorDeltaDshot[0], 1);
  Serial.print(",FFD2:"); Serial.print(appliedModelMotorDeltaDshot[1], 1);
  Serial.print(",FFD3:"); Serial.print(appliedModelMotorDeltaDshot[2], 1);
  Serial.print(",FFD4:"); Serial.print(appliedModelMotorDeltaDshot[3], 1);

  Serial.print(",M1:"); Serial.print(motor1Command);
  Serial.print(",M2:"); Serial.print(motor2Command);
  Serial.print(",M3:"); Serial.print(motor3Command);
  Serial.print(",M4:"); Serial.print(motor4Command);
  Serial.print(",OVR:"); Serial.print(controlOverrunCount);
  Serial.print(",MAXDTUS:"); Serial.println(maximumObservedControlDtUs);
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 ESKF + MODEL-BASED QUADROTOR CONTROLLER");
  Serial.println("NORMAL DSHOT300 - NO BIDIRECTIONAL TELEMETRY");
  Serial.println("REMOVE PROPELLERS FOR INITIAL TESTING");
  Serial.println();

  Serial.println("Motor pins:");
  Serial.println("M1 = GPIO26 (front-left, CCW)");
  Serial.println("M2 = GPIO14 (front-right, CW)");
  Serial.println("M3 = GPIO27 (rear-right, CCW)");
  Serial.println("M4 = GPIO25 (rear-left, CW)");
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

  Serial.println("Initializing PMW3901...");
  if (!initializeOpticalFlow())
  {
    Serial.println("PMW3901 initialization failed.");
    while (true) delay(1000);
  }
  Serial.println("PMW3901 initialized.");

  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();

  Serial.println("Sending DShot stop for five seconds.");
  const uint32_t stopStartMs = millis();

  while (static_cast<uint32_t>(millis() - stopStartMs) < 5000)
  {
    sendAllMotors(DSHOT_STOP);
    updateRangeSensor(micros(), millis());
    updateOpticalFlow(micros(), millis());
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
  previousFlowUs = nowUs;
  previousRangePollUs = nowUs;
  previousDebugMs = nowMs;
  flightEnableMs = nowMs;
  lastValidFlowMs = nowMs;

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
  Serial.println("  Estimator: 250 Hz ESKF prediction, async flow/ToF updates.");
  Serial.println("  ESKF attitude feedback starts OFF; use eatt on after sign checks.");
  Serial.println("  XY hold: ESKF position/body velocity; max target 6 deg.");
  Serial.println("  Altitude hold: ESKF Z/VZ, activates at 0.50 m for 300 ms.");
  Serial.print("  Motion-model feed-forward starts OFF; configured blend motor=");
  Serial.print(motionModelMotorBlend, 2);
  Serial.print(", rate=");
  Serial.println(motionModelRateBlend, 2);
  Serial.println("  Long range loss: base DShot 450 controlled descent.");
  Serial.println();

  Serial.println("Serial commands:");
  Serial.println("  stop / kill / disarm");
  Serial.println("  level / status / navreset / eskf");
  Serial.println("  eatt / eatt on / eatt off");
  Serial.println("  pid / outer / model / model on / model off / pidreset");
  Serial.println("  ffblend 0..0.50 / rateff 0..0.80");
  Serial.println("  kp/ki/kd, rkp/rki/rkd, pkp/pki/pkd");
  Serial.println("  ykp/yki/ykd, akp/rakp/pakp");
  Serial.println("  xyposkp/xyvelkp/xyacckp, zposkp/zvelkp/zvelki");
  Serial.println();

  printPIDGains();
  printOuterGains();
  printModelSettings();
  printEskfStatus();
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
  memcpy(&controlData, &receivedData, sizeof(ControlData));
  hasPacket = packetReceived;
  packetTimeCopy = lastPacketTimeMs;
  portEXIT_CRITICAL(&espNowMux);

  const bool signalLost =
    !hasPacket ||
    static_cast<uint32_t>(nowMs - packetTimeCopy) > FAILSAFE_TIMEOUT_MS;

  if (
    static_cast<uint32_t>(nowUs - previousControlUs) >=
    CONTROL_PERIOD_US
  )
  {
    const uint32_t rawControlDtUs =
      static_cast<uint32_t>(nowUs - previousControlUs);

    if (rawControlDtUs > maximumObservedControlDtUs)
    {
      maximumObservedControlDtUs = rawControlDtUs;
    }

    if (rawControlDtUs > CONTROL_PERIOD_US * 2U)
    {
      controlOverrunCount++;
    }

    float dt = static_cast<float>(rawControlDtUs) * 1.0e-6f;
    previousControlUs = nowUs;
    dt = clampFloat(dt, 0.002f, 0.010f);

    const bool imuValid = updateAttitude(dt);
    const bool tiltStop = excessiveTiltDetected(nowMs);

    lastImuValidState = imuValid;
    lastTiltStopState = tiltStop;

    updateRemoteEnableState(
      signalLost,
      imuValid,
      tiltStop,
      nowMs
    );

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
      updateFlightController(dt, nowMs);
    }

    sendMotorCommands();
  }

  // Sensor reads are scheduled outside the time-critical attitude block.
  updateRangeSensor(nowUs, nowMs);
  updateOpticalFlow(nowUs, nowMs);

  if (
    static_cast<uint32_t>(nowMs - previousDebugMs) >=
    DEBUG_PERIOD_MS
  )
  {
    previousDebugMs = nowMs;

    printDebug(
      signalLost,
      lastSafetyStopState,
      lastTiltStopState,
      lastImuValidState,
      nowMs
    );
  }
}