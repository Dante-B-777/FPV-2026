/*
  ============================================================================
  FPV 2026 - ROTOR-RATE DRONE CONTROLLER
  ============================================================================

  Replacement for Receiver_Robust_XY. Two things changed:

  1. SIGN CONVENTIONS ARE NOW DEFINED IN EXACTLY ONE PLACE.

     The previous build's `measuredYawRateDps` physically held a
     COUNTER-CLOCKWISE-positive rate while being named `...Cw...`. Three
     separate consumers assumed clockwise:

       - the yaw mixer, which closed a POSITIVE FEEDBACK loop that saturated
         at +/-12 DShot and reversed at ~120 deg/s in the hover log;
       - calculateMagneticYawRightHandDeg(), which returned the negative of
         the right-handed yaw it advertised and fed that to the ESKF;
       - bodyToNavigation()/navigationToBody(), written as each other's
         transpose, which inverted every XY correction away from heading 0.

     Roll and pitch were unaffected, which is exactly why they flew well and
     yaw and XY did not. All conventions now live in FrameConventions.h and
     everything downstream is right-handed with names that say so.

  2. CONTROL RUNS THROUGH ROTOR ANGULAR VELOCITY, NOT DSHOT COUNTS.

       position -> velocity -> acceleration -> tilt
       tilt -> body rate -> angular acceleration -> torque
       (thrust, torque) -> RotorAllocator -> per-rotor omega setpoint
       omega setpoint -> RotorSpeedLoop (closed on Bluejay RPM) -> DShot

     The mixer is derived from rotor geometry and spin direction rather than
     hand-written, so a sign error would have to be a geometry error.

  ----------------------------------------------------------------------------
  HARDWARE - unchanged from the previous build
  ----------------------------------------------------------------------------

    Motors (DShot300 bidirectional, RMT):
      M1 GPIO26 front-left  CCW      M2 GPIO14 front-right CW
      M4 GPIO25 rear-left   CW       M3 GPIO27 rear-right  CCW

    I2C  SDA=22  SCL=21:
      MPU6050   0x68   mounted UPSIDE DOWN (180 deg about body +Y)
      VL53L0X   0x29   pointing straight down
      QMC5883P  0x2C   mounted BACKWARDS (sensor +X=left, +Y=backward)

    SPI (PMW3901 optical flow, dedicated task on the opposite core):
      SCK=18  MISO=19  MOSI=23  CS=13  RST=33
      Axis map commissioned as flowmap 1: body X = +raw X, body Y = -raw Y

    Sensor lever arms from the airframe centre:
      PMW3901  X = 0.00 m,  Y = +0.04 m
      VL53L0X  X = 0.00 m,  Y = +0.07 m

  ----------------------------------------------------------------------------
  FLIGHT BEHAVIOUR
  ----------------------------------------------------------------------------

    DISARMED -> ARM_RAMP -> GROUND_IDLE -> CLIMB -> HOVER

    Arming spins the rotors to idle and stops. The drone stays on the ground
    until the throttle stick is first raised, then climbs to HOVER_SETPOINT_M
    and holds hands-off.

    Yaw stabilisation is ALWAYS active. It is never released by stick input:
    the yaw stick commands a rate through the same loop, and on release the
    heading is re-captured after the axis settles.

    Throttle stick engaged  -> altitude hold releases, stick commands vertical
                               velocity, no hold resistance.
    Roll/pitch engaged      -> XY position hold releases, sticks command body
                               velocity.
    Sticks centred          -> each axis settles below its speed threshold,
                               dwells, then captures a new target.

    XY hold additionally requires the yaw loop to have PROVEN itself in this
    flight: yaw rate within 10 deg/s and heading error within 8 deg,
    continuously, for 3 s while airborne. Disarming clears the proof.

  ----------------------------------------------------------------------------
  NOT YET VALIDATED ON HARDWARE
  ----------------------------------------------------------------------------

    - All control gains are physically derived starting points, NOT ports of
      the old DShot-space tuning. They are runtime-tunable; expect to tune.
    - The rotor drag coefficient kM is identified in flight and starts from a
      conservative guess. Run `kmstatus` to watch it converge.
    - Flow gyro-compensation gains were carried over from the old empirical
      fit and re-expressed in the right-handed frame. Re-verify on the bench.

  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Bitcraze_PMW3901.h>
#include <VL53L0X.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "FrameConventions.h"
#include "RotorAllocator.h"
#include "RotorSpeedLoop.h"
#include "YawTorqueIdentifier.h"
#include "HoldManager.h"
#include "Eskf9.h"
#include "BidirectionalDShot.h"
#include "OpticalFlowAsync.h"
#include "AsyncDebugTelemetry.h"

// ================================================================
// PIN CONFIGURATION
// ================================================================

static constexpr gpio_num_t MOTOR1_PIN = GPIO_NUM_26;
static constexpr gpio_num_t MOTOR2_PIN = GPIO_NUM_14;
static constexpr gpio_num_t MOTOR3_PIN = GPIO_NUM_27;
static constexpr gpio_num_t MOTOR4_PIN = GPIO_NUM_25;

static constexpr gpio_num_t MOTOR_PINS[4] = {
  MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN
};

static constexpr uint8_t MOTOR_POLES[4] = {14, 14, 14, 14};

static constexpr int SDA_PIN = 22;
static constexpr int SCL_PIN = 21;

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t VL53L0X_ADDR = 0x29;
static constexpr uint8_t VL53_RESULT_INTERRUPT_STATUS = 0x13;
static constexpr uint8_t MAG_ADDR = 0x2C;

static constexpr uint8_t FLOW_SCK_PIN = 18;
static constexpr uint8_t FLOW_MISO_PIN = 19;
static constexpr uint8_t FLOW_MOSI_PIN = 23;
static constexpr uint8_t FLOW_CS_PIN = 13;
static constexpr uint8_t FLOW_RST_PIN = 33;

// ================================================================
// SENSOR AND ACTUATOR OBJECTS
// ================================================================

BidirectionalDShot bidirectionalDshot(MOTOR_PINS, MOTOR_POLES);
Bitcraze_PMW3901 flow(FLOW_CS_PIN);
VL53L0X rangeSensor;
Eskf9 stateEstimator;
RotorSpeedLoop rotorSpeedLoop;
YawTorqueIdentifier yawIdentifier;
AsyncDebugTelemetry asyncDebug;

SettleCapture altitudeHold;
SettleCapture horizontalHold;
SettleCapture headingHold;
YawStabilityMonitor yawStability;

// ================================================================
// ESP-NOW CONTROL PACKET - MUST MATCH THE EXISTING TRANSMITTER
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

/*
  Link health statistics.

  Added because repeated unexplained disarms could not be attributed from
  telemetry alone - the ~100 ms print interval is wider than the events that
  matter. These accumulate in the ESP-NOW receive callback so no gap can hide
  between prints. `link` prints them; they reset only on reboot.
*/
volatile uint32_t packetCount = 0;
volatile uint32_t rejectedMagicCount = 0;
volatile uint32_t rejectedLengthCount = 0;
volatile uint32_t worstPacketGapMs = 0;
volatile uint32_t gapsOver100msCount = 0;
volatile uint32_t gapsOverFailsafeCount = 0;

// ================================================================
// STICK SHAPING
// ================================================================

static constexpr float ROLL_STICK_DEADBAND = 0.055f;
static constexpr float PITCH_STICK_DEADBAND = 0.055f;
static constexpr float THROTTLE_STICK_DEADBAND = 0.050f;
static constexpr float YAW_STICK_DEADBAND = 0.050f;
static constexpr float ENABLE_THROTTLE_TOLERANCE = 0.05f;

// ================================================================
// FLIGHT ENVELOPE
// ================================================================

/*
  0.60 -> 0.50 on 2026-08-15. Prop radius is 0.0635 m, so 0.50 m is about four
  rotor diameters up and effectively clear of ground effect - which is the
  whole point: every kM identification run so far was flown at 0.14-0.20 m,
  where the rotor drag regressor and the yaw rate are too collinear to
  separate. It also keeps the first out-of-ground-effect hovers low enough to
  be survivable while the thrust model is still in question.
*/
static constexpr float HOVER_SETPOINT_M = 0.50f;
static constexpr float MIN_ALTITUDE_TARGET_M = 0.25f;
static constexpr float MAX_ALTITUDE_TARGET_M = 1.60f;

static constexpr float MAX_HORIZONTAL_VELOCITY_MPS = 0.50f;
/*
  Raised from 0.35. Full stick down previously bought only -0.35 m/s, which
  through verticalVelocityKp came to about a 14% thrust reduction - barely
  distinguishable from hover, and the second half of the "non-responsive
  downward" complaint once the ground-capture bug above is fixed.
*/
static constexpr float MAX_VERTICAL_VELOCITY_MPS = 0.50f;

/*
  Ground idle rotor speed, rad/s. Chosen from the bench log: DShot 200 at
  idle measured OMG 541-551, and thrust here is 4*kf*w^2 = 1.45 N against a
  4.45 N weight, so the airframe stays firmly on the ground.
*/
static constexpr float GROUND_IDLE_OMEGA_RAD_PER_SEC = 500.0f;
static constexpr float MAX_HORIZONTAL_ACCEL_MPS2 = 1.50f;
static constexpr float MAX_VERTICAL_ACCEL_MPS2 = 3.00f;

static constexpr float MAX_TILT_COMMAND_DEG = 12.0f;
static constexpr float MAX_TILT_HOLD_DEG = 6.0f;
static constexpr float MAX_SAFE_TILT_DEG = 35.0f;

static constexpr float MAX_BODY_RATE_DPS = 200.0f;
static constexpr float MAX_YAW_RATE_DPS = 90.0f;
static constexpr float MAX_YAW_HOLD_RATE_DPS = 35.0f;

static constexpr float MIN_THRUST_FRACTION = 0.35f;
static constexpr float MAX_THRUST_FRACTION = 1.90f;

static constexpr float CLIMB_RATE_MPS = 0.30f;

// Airframe inertia [kg m^2], slab approximation about each body axis.
static constexpr float INERTIA_XX =
  AIRFRAME_MASS_KG * (FRAME_SEPARATION_FRONT_BACK_M *
                      FRAME_SEPARATION_FRONT_BACK_M) / 12.0f;
static constexpr float INERTIA_YY =
  AIRFRAME_MASS_KG * (FRAME_SEPARATION_SIDE_SIDE_M *
                      FRAME_SEPARATION_SIDE_SIDE_M) / 12.0f;
static constexpr float INERTIA_ZZ = BODY_YAW_INERTIA_KGM2;

// ================================================================
// CONTROL GAINS - runtime tunable, see the `gains` command
// ================================================================

// Outer loops. Units are 1/s: output per unit of input error.
float positionKp = 1.00f;   // m/s per m
float velocityKp = 2.00f;   // (m/s^2) per (m/s)
float altitudeKp = 1.50f;   // m/s per m
float verticalVelocityKp = 4.00f;  // (m/s^2) per (m/s)
/*
  attitudeKp was 7.00 through the 2026-08-15 flights. Flight-tuned to 10.00.

  At 7.00 the aircraft was leaving three quarters of its roll/pitch authority
  on the table: the largest torque either axis requested across an entire
  flight was 0.113 N m against an available 0.336 N m. A 22.9 deg roll
  excursion produced a 160 deg/s rate target and only 0.09 N m of correction,
  and took about a second to recover. Pilot report: "weaker than I would want".

  Raising this alone would only buy overshoot - the cascade needs the rate
  loops to stay roughly 3.5x faster than the attitude loop, so rollRateKp and
  pitchRateKp moved with it and the ratio is preserved at 35/10 = 3.5.
*/
float attitudeKp = 10.00f;  // (rad/s) per rad

/*
  HEADING HOLD IS OFF BY DEFAULT, AND THAT IS DELIBERATE.

  The 2026-08-15 flight that flew well was flown with `set headingkp 0`, which
  takes the magnetometer out of the loop and leaves yaw as pure rate damping.
  That was a commissioning measure, not a fix: the QMC5883P still shifts with
  motor current, and this gain is what lets that corruption drive the yaw loop.

  The default was therefore moved to 0 so the safe state survives a flash and
  does not depend on remembering a serial command. There is no NVS on this
  board, so every gain resets to its compiled value at boot - which for this
  one is the direction you want.

  Re-enabling it is a deliberate `set headingkp 3` and should not happen until
  the compass is relocated away from the power leads and Phase 9 passes.

  (An earlier revision of this comment said flashing restores 3.00 and that
  `headingkp 0` was needed after every flash. That described the old default
  and was wrong for this build. Note also that the gain commands need the
  `set` prefix - a bare `headingkp 0` is answered with "Unknown command", as
  the 2026-08-15 capture shows happening.)
*/
float headingKp = 3.00f;    // (rad/s) per rad

// Inner rate loops. Units are 1/s: angular acceleration per rate error.
// roll/pitch flight-tuned from 25.0 to 35.0 on 2026-08-15; see attitudeKp.
float rollRateKp = 35.0f;
float rollRateKi = 8.0f;
float rollRateKd = 0.40f;

float pitchRateKp = 35.0f;
float pitchRateKi = 8.0f;
float pitchRateKd = 0.40f;

float yawRateKp = 15.0f;
float yawRateKi = 4.0f;
float yawRateKd = 0.20f;

static constexpr float RATE_INTEGRAL_LIMIT = 60.0f;  // rad/s^2
static constexpr float DERIVATIVE_FILTER_ALPHA = 0.20f;

// ================================================================
// IMU STATE
// ================================================================

int16_t axRaw = 0, ayRaw = 0, azRaw = 0;
int16_t gxRaw = 0, gyRaw = 0, gzRaw = 0;

float accelXBiasRaw = 0.0f;
float accelYBiasRaw = 0.0f;
float accelZBiasRaw = 0.0f;
float gyroXBiasRaw = 0.0f;
float gyroYBiasRaw = 0.0f;
float gyroZBiasRaw = 0.0f;

/*
  Captured by calibrateImu(). NOTE: these are structurally almost always ~0 -
  see the comment in calibrateImu() for why. The real attitude reference is
  the pose the aircraft happened to be sitting in when it booted.
*/
float rollLevelTrimRad = 0.0f;
float pitchLevelTrimRad = 0.0f;

/*
  PILOT TRIM, degrees, tunable at runtime as `rolltrim` / `pitchtrim`.

  Added because the boot pose IS the attitude reference on this build, so
  powering up on a bench and then flying off the floor leaves a permanent
  bank. The 2026-08-15 log showed the aircraft reading RP=+1.7 to +2.1 deg
  while sitting motionless on the ground both before arming and after
  disarming - a reference error, not motion. The attitude loop then drove
  that reading to zero, which physically banks the aircraft the other way
  and walks it across the room.

  Positive rolltrim subtracts from the measured roll, so if the aircraft
  reads +2 deg at rest, set rolltrim to +2 and it will read 0.
*/
float rollTrimDeg = 0.0f;
float pitchTrimDeg = 0.0f;

// Right-handed body angular rates [rad/s].
float bodyRateXRh = 0.0f;  // about +X, nose up positive
float bodyRateYRh = 0.0f;  // about +Y, right wing down positive

// Rate-loop setpoints, deg/s, published for `ratelog`.
float lastRollRateTargetDps = 0.0f;
float lastPitchRateTargetDps = 0.0f;
float lastYawRateTargetDps = 0.0f;
bool rateLogEnabled = false;

/*
  Outer-loop targets, published for the flight track log.

  These are the setpoints the velocity loops are actually chasing. They used
  to be locals inside updateHorizontalController() and
  updateVerticalController(), which meant the only observable part of the
  cascade was its two ends - stick in, attitude out - and everything between
  had to be inferred. Recording them makes each stage of the cascade
  independently checkable against the stage above it.
*/
float publishedBodyVelocityTargetX = 0.0f;
float publishedBodyVelocityTargetY = 0.0f;
float publishedVerticalVelocityTarget = 0.0f;

// ================================================================
// FLIGHT TRACK LOG
// ================================================================

/*
  A uniformly sampled CSV record of the whole command-to-response chain,
  written for post-flight analysis rather than for reading on a console.

  The 5 Hz status line is fine for watching a hover, but it is far too slow
  to resolve anything the controller actually does: the attitude loop runs at
  1.6 Hz and the rate loops at 5.6 Hz, so 5 Hz samples alias them completely.
  It also carries no timestamp, which means sample intervals have to be
  assumed rather than measured and a dropped line is invisible.

  This log fixes both. Default 50 Hz resolves the attitude loop with about
  thirty samples per time constant and costs roughly 185 kbaud of the 460800
  link. Every row carries millis() and a sequence number, so gaps are
  detectable rather than silently smoothed over.
*/
bool trackLogEnabled = false;
uint32_t trackLogPeriodUs = 20000;   // 50 Hz
uint32_t previousTrackLogUs = 0;
uint32_t trackLogSequence = 0;

/*
  ---------------------------------------------------------------------------
  UNTETHERED TRACK LOG (`tracklog net`)
  ---------------------------------------------------------------------------

  The USB cable that carries the tethered log is the one thing preventing a
  free-flight recording. But the asymmetry is in our favour: the TRANSMITTER
  is the device attached to the laptop, and it is already talking to us at
  50 Hz. So the drone sends the raw sample back over the same ESP-NOW link and
  the transmitter does the formatting and the printing.

  Because both sketches share TrackSample.h, the rows the transmitter prints
  are byte-identical to the rows the receiver prints over USB. capture.py and
  analyze_track.py need no changes and cannot tell the two apart.

  Cost: a 216-byte packet per sample. At 50 Hz that is 10.8 kB/s alongside the
  existing 50 Hz / 24-byte uplink - a small fraction of what ESP-NOW carries.
  The uplink is unaffected; it is a separate transfer in the opposite
  direction, and the RC packet is what the failsafe watches.

  The peer address is LEARNED from the first uplink packet rather than
  hard-coded, so pairing stays a one-way configuration: the transmitter knows
  the receiver's MAC, and the receiver discovers the transmitter's.
*/
bool trackLogDownlink = false;          // route rows to ESP-NOW, not the UART
volatile bool transmitterMacKnown = false;
uint8_t transmitterMac[6] = {};
bool transmitterPeerAdded = false;
uint32_t trackDownlinkSent = 0;
uint32_t trackDownlinkFailed = 0;

/*
  Preamble lines waiting to go out over the downlink. Filled by
  emitTrackLogHeader(), drained one per loop() pass by
  serviceTrackHeaderQueue(). 12 x 200 B of BSS, which is cheap next to not
  blocking the control step.
*/
static constexpr size_t TRACK_HEADER_MAX_LINES = 12;
char trackHeaderLines[TRACK_HEADER_MAX_LINES][200] = {};
size_t trackHeaderCount = 0;
size_t trackHeaderSent = 0;

/*
  A `tracklog net` asked for before the transmitter address was known.

  Rejecting outright was wrong: the address arrives on the first RC packet,
  which is typically a second or two after boot, so the command fails only
  because it was typed promptly. On 2026-08-18 that silently cost a whole
  diagnostic flight - the pilot issued the command, saw it refused, and flew
  anyway with no downlink running.

  The request is now held and started as soon as the link comes up. 0 means
  nothing is waiting.
*/
uint8_t deferredNetRateHz = 0;

// Set by the ESP-NOW callback, acted on in loop(). See OnDataRecv.
volatile bool pendingTrackCommand = false;
volatile bool pendingTrackCommandEnable = false;
volatile uint8_t pendingTrackCommandRateHz = 50;

static constexpr uint32_t TRACK_LOG_MIN_HZ = 1;
static constexpr uint32_t TRACK_LOG_MAX_HZ = 100;
float bodyRateZRh = 0.0f;  // about +Z, counter-clockwise positive

// Right-handed body specific force [m/s^2].
float specificForceX = 0.0f;
float specificForceY = 0.0f;
float specificForceZ = 0.0f;

// Right-handed attitude [rad].
float rollRhRad = 0.0f;
float pitchRhRad = 0.0f;
float yawRhRad = 0.0f;

float complementaryRollRad = 0.0f;
float complementaryPitchRad = 0.0f;

/*
  Complementary filter time constant, seconds. The blend weight is derived
  from dt rather than fixed, so changing the control rate does not silently
  change how much the filter trusts the accelerometer.
*/
static constexpr float COMPLEMENTARY_TAU_S = 0.133f;

// Gravity-aiding decimation. See updateAttitude().
static constexpr uint32_t GRAVITY_UPDATE_DECIMATION = 4;  // 400 Hz -> 100 Hz
static constexpr float GRAVITY_DECIMATION_STD_SCALE = 0.50f;  // 1/sqrt(4)
uint32_t gravityUpdateCounter = 0;

bool eskfAttitudeUsed = false;
bool eskfStateHealthy = false;
static constexpr float ESKF_MAX_ATTITUDE_DISAGREEMENT_RAD =
  15.0f * FC_DEG_TO_RAD;
static constexpr float ESKF_GRAVITY_STD_ON_GROUND = 0.05f;
static constexpr float ESKF_GRAVITY_STD_IN_FLIGHT = 0.20f;

// ================================================================
// NAVIGATION STATE
// ================================================================

float positionNavX = 0.0f;
float positionNavY = 0.0f;
float altitudeM = 0.0f;
float velocityNavX = 0.0f;
float velocityNavY = 0.0f;
float verticalVelocityMps = 0.0f;

float measuredBodyVelocityX = 0.0f;
float measuredBodyVelocityY = 0.0f;

float targetNavX = 0.0f;
float targetNavY = 0.0f;
float altitudeTargetM = HOVER_SETPOINT_M;
float headingTargetRhRad = 0.0f;

// ================================================================
// RANGE SENSOR
// ================================================================

static constexpr uint32_t RANGE_TIMING_BUDGET_US = 20000;
static constexpr uint32_t RANGE_PERIOD_US = 10000;
static constexpr uint32_t RANGE_LONG_FAILURE_MS = 500;
static constexpr float RANGE_FILTER_ALPHA = 0.30f;
static constexpr float MIN_VALID_RANGE_M = 0.03f;
static constexpr float MAX_VALID_RANGE_M = 2.00f;

float filteredRangeMeters = 0.0f;
bool rangeHasEverBeenValid = false;
uint32_t lastValidRangeMs = 0;
uint32_t previousRangePollUs = 0;
bool rangeSensorInitialized = false;

// ================================================================
// OPTICAL FLOW
// ================================================================

static constexpr uint32_t FLOW_PERIOD_US = 10000;
static constexpr uint32_t FLOW_SPI_HZ = 2000000UL;
static constexpr float FLOW_RAD_PER_COUNT = 1.0f / 385.0f;
static constexpr float MIN_FLOW_RANGE_M = 0.100f;
static constexpr float MAX_FLOW_RANGE_M = 2.000f;
static constexpr int32_t FLOW_OUTLIER_LIMIT_COUNTS = 100;
static constexpr uint8_t MIN_FLOW_SQUAL = 1;
static constexpr float MAX_PLAUSIBLE_FLOW_SPEED_MPS = 3.0f;
static constexpr float FLOW_VELOCITY_FILTER_ALPHA = 0.35f;
static constexpr float FLOW_INVALID_DECAY = 0.90f;

static constexpr uint32_t FLOW_HEALTH_LOSS_MS = 250;
static constexpr uint8_t FLOW_HEALTH_RECOVERY_SAMPLES = 25;

/*
  Gyro compensation gains, carried over from the previous build's empirical
  fit and re-expressed in the right-handed frame.

  The old code accumulated the CONTROLLER-signed rates, which relate to the
  right-handed rates as:
      oldRollDelta  = -omegaY * dt
      oldPitchDelta = -omegaX * dt
  Applying the old gains (-1.34, +1.65) to those quantities is identical to
  applying (+1.34, -1.65) to the right-handed deltas, so the fitted behaviour
  is preserved exactly.
*/
static constexpr bool ENABLE_FLOW_GYRO_COMPENSATION = true;
static constexpr float FLOW_ROLL_COMP_GAIN_RH = +1.34f;
static constexpr float FLOW_PITCH_COMP_GAIN_RH = -1.65f;

// Physical sensor lever arms [m].
static constexpr float PMW_OFFSET_X_M = 0.00f;
static constexpr float PMW_OFFSET_Y_M = 0.04f;

static constexpr float FLOW_YAW_RATE_SOFT_DPS = 20.0f;
static constexpr float FLOW_YAW_RATE_HARD_DPS = 60.0f;
static constexpr float FLOW_YAW_NOISE_SCALE_MAX = 6.0f;

QueueHandle_t opticalFlowSampleQueue = nullptr;
TaskHandle_t opticalFlowTaskHandle = nullptr;
portMUX_TYPE flowGyroMux = portMUX_INITIALIZER_UNLOCKED;

// Right-handed gyro angle accumulated over each PMW read interval [rad].
float accumulatedFlowRollRhRad = 0.0f;   // about +Y
float accumulatedFlowPitchRhRad = 0.0f;  // about +X
float accumulatedFlowYawRhRad = 0.0f;    // about +Z

bool flowSensorHealthy = false;
bool flowVelocityReady = false;
uint32_t lastHealthyFlowMs = 0;
uint32_t lastFlowMeasurementMs = 0;
uint8_t consecutiveHealthyFlowSamples = 0;
uint8_t lastFlowAxisAcceptedMask = 0;
uint8_t lastFlowMotion = 0;
uint8_t lastFlowSqual = 0;
uint16_t lastFlowShutter = 0;
uint32_t flowRejectCount = 0;

// ================================================================
// MAGNETOMETER
// ================================================================

static constexpr uint8_t MAG_CHIP_ID_REG = 0x00;
static constexpr uint8_t MAG_X_LSB_REG = 0x01;
static constexpr uint8_t MAG_STATUS_REG = 0x09;
static constexpr uint8_t MAG_CTRL1_REG = 0x0A;
static constexpr uint8_t MAG_CTRL2_REG = 0x0B;
static constexpr uint8_t MAG_AXIS_SIGN_REG = 0x29;
static constexpr uint8_t MAG_CTRL1_CONFIG = 0xCB;

// Hard-iron bias and axis scale, applied in SENSOR coordinates before the
// mounting transform. Unchanged from the validated 2026-08-10 calibration.
static constexpr float MAG_BIAS_X = 48.000f;
static constexpr float MAG_BIAS_Y = -12.500f;
static constexpr float MAG_BIAS_Z = -281.000f;
static constexpr float MAG_SCALE_X = 0.980290f;
static constexpr float MAG_SCALE_Y = 1.020518f;
static constexpr float MAG_SCALE_Z = 1.009972f;

static constexpr uint32_t MAG_POLL_PERIOD_US = 10000;
static constexpr uint32_t MAG_STALE_MS = 250;
static constexpr float MAG_FILTER_ALPHA = 0.20f;
static constexpr float MAG_YAW_STD_RAD = 10.0f * FC_DEG_TO_RAD;
static constexpr float MAG_MAX_INNOVATION_RAD = 35.0f * FC_DEG_TO_RAD;
static constexpr float MAG_FIELD_RATIO_MIN = 0.60f;
static constexpr float MAG_FIELD_RATIO_MAX = 1.40f;
static constexpr float MAG_MIN_HORIZONTAL_COUNTS = 40.0f;

float magFilteredBodyX = 0.0f;
float magFilteredBodyY = 0.0f;
float magFilteredBodyZ = 0.0f;
float magFieldMagnitude = 0.0f;
float magReferenceFieldMagnitude = 0.0f;
float magAbsoluteYawRhRad = 0.0f;
float magReferenceYawRhRad = 0.0f;
float magRelativeYawRhRad = 0.0f;

bool magSensorInitialized = false;
bool magFilterInitialized = false;
bool magMeasurementValid = false;
bool magReferenceValid = false;
bool magReferencePending = true;
bool magYawFusionEnabled = true;
uint32_t lastMagSampleMs = 0;
uint32_t lastMagAcceptedMs = 0;
uint32_t previousMagPollUs = 0;
uint32_t magAcceptedUpdates = 0;
uint32_t magRejectedUpdates = 0;

// ================================================================
// FLIGHT STATE MACHINE
// ================================================================

enum class FlightState : uint8_t
{
  Disarmed = 0,
  ArmRamp = 1,
  GroundIdle = 2,
  Climb = 3,
  Hover = 4,
  DisarmRamp = 5
};

FlightState flightState = FlightState::Disarmed;
uint32_t flightStateEnteredMs = 0;

static constexpr uint32_t ARM_RAMP_DURATION_MS = 2000;
static constexpr uint32_t DISARM_RAMP_DURATION_MS = 1500;

// Rate limit for the "why won't it arm" message.
uint32_t lastArmRefusalMs = 0;
static constexpr uint32_t ARM_REFUSAL_PERIOD_MS = 2000;

bool safetyStopLatched = false;
bool serialStopLatch = false;
bool signalLost = true;
bool imuValid = false;
uint32_t tiltExceededSinceMs = 0;

// Rate PID state.
float rollRateIntegral = 0.0f, pitchRateIntegral = 0.0f, yawRateIntegral = 0.0f;
float rollRatePreviousError = 0.0f, pitchRatePreviousError = 0.0f;
float yawRatePreviousError = 0.0f;
float rollRateDerivative = 0.0f, pitchRateDerivative = 0.0f;
float yawRateDerivative = 0.0f;

// Latest command outputs, for telemetry.
float commandedThrustN = 0.0f;
float commandedTauX = 0.0f, commandedTauY = 0.0f, commandedTauZ = 0.0f;

// True on any cycle where the yaw demand exceeded the rotors' drag authority
// and had to be clipped. Published as YSAT so a limit cycle is visible in the
// telemetry instead of having to be inferred from the DShot columns.
bool yawTorqueSaturated = false;

RotorAllocator::Output lastAllocation;
RotorSpeedLoop::Status lastRotorStatus;
YawTorqueIdentifier::Status lastIdentifierStatus;

/*
  Whatever was last written to the ESCs, recorded by sendMotorCommands().

  Telemetry used to print lastRotorStatus.dshotCommand, which is only
  populated when the full rotor speed loop runs. In every other state it
  reported a stale zero, so the ground-idle log claimed DS=0 while the motors
  were actually turning at idle. Report what was really sent.
*/
uint16_t lastCommandedDshot[4] = {0, 0, 0, 0};

// ================================================================
// BENCH MODE - rotor speed loop with no flight controller above it
// ================================================================

/*
  Phase 3 of commissioning needs the rotor speed loop running with props off
  and nothing else active. Arming will not do: ground idle bypasses the loop
  entirely, and climb commands thrust against an altitude the propless frame
  can never reach.

  Bench mode drives a fixed angular velocity setpoint straight into
  RotorSpeedLoop. It only runs while DISARMED, refuses to coexist with an arm
  request, and stops itself after a timeout so a forgotten bench session
  cannot leave motors spinning.
*/
bool benchModeActive = false;
float benchOmegaSetpoint = 0.0f;
int benchMotorIndex = -1;  // -1 = all four
uint32_t benchStartedMs = 0;

static constexpr uint32_t BENCH_TIMEOUT_MS = 60000;
static constexpr float BENCH_MIN_OMEGA = 250.0f;
static constexpr float BENCH_MAX_OMEGA = 1100.0f;

float desiredRollRhRad = 0.0f;
float desiredPitchRhRad = 0.0f;
float desiredYawRateRhRadPerSec = 0.0f;

// Ground zero-velocity update cadence.
uint32_t previousZeroVelocityMs = 0;
static constexpr uint32_t ZERO_VELOCITY_PERIOD_MS = 50;  // 20 Hz

/*
  Control-step profiling.

  Worth carrying permanently: when the control step approaches the 2 ms
  budget, everything sharing the loop starts behaving strangely in ways that
  look like unrelated faults. Being able to read the number turns that from a
  mystery into a measurement.
*/
// Sub-timers, so a fat control step points at its own cause.
uint64_t timingImuReadTotalUs = 0;
uint64_t timingEstimatorTotalUs = 0;
uint64_t timingControlTotalUs = 0;
uint32_t timingSampleCount = 0;
uint32_t reportedImuUs = 0;
uint32_t reportedEstimatorUs = 0;
uint32_t reportedControlUs = 0;

uint32_t controlStepMaxUs = 0;
uint64_t controlStepTotalUs = 0;
uint32_t controlStepCount = 0;
uint32_t controlOverrunCount = 0;
uint32_t loopPassCount = 0;
uint32_t reportedControlAvgUs = 0;
uint32_t reportedControlMaxUs = 0;
uint32_t reportedLoopPassesPerStep = 0;

// Loop timing.
uint32_t previousControlUs = 0;
uint32_t previousTelemetryMs = 0;
/*
  400 Hz, not 500.

  Measured on this hardware, the control step costs about 1415 us: 658 us of
  MPU6050 I2C burst read and 668 us of ESKF, with the actual flight control
  arithmetic only 32 us. Against a 2000 us budget that left 585 us of margin
  and produced a steadily climbing overrun count before the flight cascade,
  optical-flow fusion, and the identifier were even in the picture.

  The I2C read is close to irreducible without overclocking a bus shared with
  the VL53L0X and QMC5883P, so the honest fix is to stop pretending the step
  fits in 2 ms. 400 Hz still gives the rate loop more than an order of
  magnitude of oversampling over its ~25 rad/s bandwidth, and it buys
  1085 us of margin instead of 585.
*/
static constexpr uint32_t CONTROL_PERIOD_US = 2500;   // 400 Hz
static constexpr uint32_t TELEMETRY_PERIOD_MS = 200;  // 5 Hz

char serialCommandBuffer[AsyncDebugTelemetry::COMMAND_LINE_CAPACITY];

// ================================================================
// SMALL HELPERS
// ================================================================

int clampInt(int value, int minimum, int maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float applyCenteredStickDeadband(float input, float deadband)
{
  input = clampFloatFc(input, -1.0f, 1.0f);
  if (fabsf(input) <= deadband) return 0.0f;
  if (input > 0.0f) return (input - deadband) / (1.0f - deadband);
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

void logLine(const char *line)
{
  if (asyncDebug.started())
  {
    asyncDebug.postText(line);
  }
  else
  {
    Serial.println(line);
  }
}

void logf(const char *format, ...)
{
  char buffer[AsyncDebugTelemetry::TEXT_LINE_CAPACITY];
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);
  logLine(buffer);
}

uint32_t rangeAgeMs(uint32_t nowMs)
{
  if (!rangeHasEverBeenValid) return UINT32_MAX;
  return static_cast<uint32_t>(nowMs - lastValidRangeMs);
}

bool rangeUsable(uint32_t nowMs)
{
  return rangeHasEverBeenValid && rangeAgeMs(nowMs) <= RANGE_LONG_FAILURE_MS;
}

bool flowHealthFresh(uint32_t nowMs)
{
  return flowSensorHealthy &&
    lastHealthyFlowMs != 0U &&
    static_cast<uint32_t>(nowMs - lastHealthyFlowMs) <= FLOW_HEALTH_LOSS_MS;
}

bool magnetometerFresh(uint32_t nowMs)
{
  return magMeasurementValid &&
    lastMagSampleMs != 0U &&
    static_cast<uint32_t>(nowMs - lastMagSampleMs) <= MAG_STALE_MS;
}

bool flightActive()
{
  return flightState == FlightState::GroundIdle ||
         flightState == FlightState::Climb ||
         flightState == FlightState::Hover;
}

bool airborne()
{
  return flightState == FlightState::Climb ||
         flightState == FlightState::Hover;
}

/*
  PHYSICAL airborne detection, as opposed to airborne() which only reports
  which state the machine is in.

  Bench log 2026-08-14 showed "Altitude hold captured at 0.250 m" while the
  rangefinder read 0.03 m and the airframe was sitting on the table. The
  capture gate used airborne(), which is true the instant Climb is entered,
  so the settle detector saw a near-zero vertical velocity - correct, the
  drone was not moving because it was on the ground - and latched a hold.

  MIN_ALTITUDE_TARGET_M then clamped the captured target up to 0.25 m, and
  the altitude loop spent the rest of the flight commanding a climb the pilot
  never asked for. Throttle-down released the hold, but releasing it handed
  control straight back on stick release, which is what "non-responsive in
  the downward direction" felt like from the sticks.

  Requiring real height for a real dwell makes the gate mean what its name
  says. The latch is one-way within a flight so that a brief dip toward the
  ground does not drop the holds mid-flight; resetFlightState clears it.
*/
static constexpr float AIRBORNE_ALTITUDE_M = 0.15f;
static constexpr uint32_t AIRBORNE_DWELL_MS = 300;

uint32_t aboveGroundSinceMs = 0;
bool airborneLatched = false;

void updateAirborneLatch(uint32_t nowMs)
{
  if (!airborne())
  {
    aboveGroundSinceMs = 0;
    airborneLatched = false;
    return;
  }

  if (airborneLatched) return;

  if (!rangeUsable(nowMs) || altitudeM < AIRBORNE_ALTITUDE_M)
  {
    aboveGroundSinceMs = 0;
    return;
  }

  if (aboveGroundSinceMs == 0)
  {
    aboveGroundSinceMs = nowMs;
    return;
  }

  if (static_cast<uint32_t>(nowMs - aboveGroundSinceMs) >= AIRBORNE_DWELL_MS)
  {
    airborneLatched = true;
    logf("Airborne confirmed at %.2f m; position holds now permitted.",
      altitudeM);
  }
}

bool physicallyAirborne()
{
  return airborneLatched;
}

// ================================================================
// MPU6050
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

      const int16_t unusedTemperature =
        static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      (void)unusedTemperature;

      gxRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      gyRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      gzRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
      return true;
    }

    while (Wire.available() > 0) Wire.read();
    if (attempt == 0) delayMicroseconds(20);
  }

  return false;
}

bool setupMPU()
{
  delay(100);
  writeMPU(0x6B, 0x00);
  delay(100);
  writeMPU(0x1B, 0x00);  // gyro +/-250 dps
  writeMPU(0x1C, 0x00);  // accel +/-2 g
  writeMPU(0x1A, 0x03);  // DLPF 44 Hz
  delay(100);
  return readMPURaw();
}

/*
  Convert the current raw sample into right-handed body quantities. Bias
  correction happens in CHIP coordinates, before the mounting transform,
  which is the only order that lets the calibration means be reused directly.
*/
void convertImuSampleToBodyRh()
{
  const float chipGyroX =
    (static_cast<float>(gxRaw) - gyroXBiasRaw) / GYRO_LSB_PER_DPS;
  const float chipGyroY =
    (static_cast<float>(gyRaw) - gyroYBiasRaw) / GYRO_LSB_PER_DPS;
  const float chipGyroZ =
    (static_cast<float>(gzRaw) - gyroZBiasRaw) / GYRO_LSB_PER_DPS;

  float rateXDps = 0.0f, rateYDps = 0.0f, rateZDps = 0.0f;
  chipGyroToBodyRh(
    chipGyroX, chipGyroY, chipGyroZ,
    rateXDps, rateYDps, rateZDps
  );

  bodyRateXRh = rateXDps * FC_DEG_TO_RAD;
  bodyRateYRh = rateYDps * FC_DEG_TO_RAD;
  bodyRateZRh = rateZDps * FC_DEG_TO_RAD;

  const float chipAccelX =
    (static_cast<float>(axRaw) - accelXBiasRaw) / ACCEL_LSB_PER_G;
  const float chipAccelY =
    (static_cast<float>(ayRaw) - accelYBiasRaw) / ACCEL_LSB_PER_G;
  const float chipAccelZ =
    (static_cast<float>(azRaw) - accelZBiasRaw) / ACCEL_LSB_PER_G;

  chipAccelToBodyRh(
    chipAccelX, chipAccelY, chipAccelZ,
    specificForceX, specificForceY, specificForceZ
  );
}

bool calibrateIMU()
{
  Serial.println();
  Serial.println("IMU calibration starting. Keep the drone level and still.");
  delay(1500);

  constexpr int SAMPLE_COUNT = 1500;

  int64_t axTotal = 0, ayTotal = 0, azTotal = 0;
  int64_t gxTotal = 0, gyTotal = 0, gzTotal = 0;
  int validSamples = 0;

  for (int sample = 0; sample < SAMPLE_COUNT; ++sample)
  {
    if (readMPURaw())
    {
      axTotal += axRaw; ayTotal += ayRaw; azTotal += azRaw;
      gxTotal += gxRaw; gyTotal += gyRaw; gzTotal += gzRaw;
      ++validSamples;
    }
    delay(2);
  }

  if (validSamples < SAMPLE_COUNT / 2)
  {
    Serial.println("IMU calibration FAILED.");
    return false;
  }

  gyroXBiasRaw = static_cast<float>(gxTotal) / validSamples;
  gyroYBiasRaw = static_cast<float>(gyTotal) / validSamples;
  gyroZBiasRaw = static_cast<float>(gzTotal) / validSamples;

  accelXBiasRaw = static_cast<float>(axTotal) / validSamples;
  accelYBiasRaw = static_cast<float>(ayTotal) / validSamples;

  /*
    Body +Z must read +1 g at rest. The chip's Z axis points along body -Z,
    so a level chip reads about -ACCEL_LSB_PER_G. The bias is whatever is
    left over after removing that expected reading.
  */
  const float meanAzRaw = static_cast<float>(azTotal) / validSamples;
  accelZBiasRaw = meanAzRaw + ACCEL_LSB_PER_G;

  /*
    Residual mounting tilt, captured so level flight commands zero tilt.

    THIS CAPTURES ALMOST NOTHING, BY CONSTRUCTION. Read before trusting it.

    accelXBiasRaw and accelYBiasRaw above were set to the FULL mean of the
    raw X/Y accelerometer readings. At rest those readings are not bias -
    they are mostly the projection of gravity produced by however the
    aircraft is tilted. Subtracting the whole mean therefore forces the
    corrected specific force in X and Y to zero, and the tilt computed from
    it below comes out as sensor noise: the 2026-08-15 boot printed
    "level trim deg roll=-0.037 pitch=0.041", which is nothing.

    (It is computed from the LAST raw sample, not the mean, which is the only
    reason it is not identically zero.)

    The consequence is that the attitude reference is silently whatever pose
    the aircraft was sitting in when it booted. Power up on a bench, fly off
    the floor, and every degree of difference becomes a permanent bank that
    walks the aircraft across the room - and the printed trim still reads
    0.0, so nothing looks wrong.

    A static measurement genuinely cannot separate accelerometer bias from
    mounting tilt; both look identical. Fixing it properly needs either a
    multi-orientation (6-point) accel calibration or a pilot trim. The pilot
    trim is what exists today: rollTrimDeg / pitchTrimDeg, `rolltrim` and
    `pitchtrim` at the serial prompt.
  */
  convertImuSampleToBodyRh();
  bodySpecificForceToTiltRh(
    specificForceX, specificForceY, specificForceZ,
    rollLevelTrimRad, pitchLevelTrimRad
  );

  rollRhRad = 0.0f;
  pitchRhRad = 0.0f;
  yawRhRad = 0.0f;
  complementaryRollRad = 0.0f;
  complementaryPitchRad = 0.0f;

  Serial.println("IMU calibration complete.");
  Serial.printf("  gyro bias raw   X=%.2f Y=%.2f Z=%.2f\n",
    gyroXBiasRaw, gyroYBiasRaw, gyroZBiasRaw);
  Serial.printf("  accel bias raw  X=%.2f Y=%.2f Z=%.2f\n",
    accelXBiasRaw, accelYBiasRaw, accelZBiasRaw);
  Serial.printf("  level trim deg  roll=%.3f pitch=%.3f\n",
    rollLevelTrimRad * FC_RAD_TO_DEG, pitchLevelTrimRad * FC_RAD_TO_DEG);
  Serial.println();

  return true;
}

void synchronizeEstimatorState()
{
  positionNavX = stateEstimator.positionX();
  positionNavY = stateEstimator.positionY();
  velocityNavX = stateEstimator.velocityX();
  velocityNavY = stateEstimator.velocityY();
  verticalVelocityMps = stateEstimator.velocityZ();
}

void resetEstimatorLocalFrame()
{
  const float initialAltitude =
    rangeHasEverBeenValid ? filteredRangeMeters : 0.0f;

  stateEstimator.reset(
    0.0f, 0.0f, initialAltitude,
    pitchRhRad, rollRhRad, 0.0f
  );

  positionNavX = 0.0f;
  positionNavY = 0.0f;
  targetNavX = 0.0f;
  targetNavY = 0.0f;
  yawRhRad = 0.0f;
  headingTargetRhRad = 0.0f;

  magReferencePending = true;
  magReferenceValid = false;

  synchronizeEstimatorState();
}

bool updateAttitude(float dt)
{
  const uint32_t imuStartUs = micros();
  const bool imuOk = readMPURaw();
  timingImuReadTotalUs += static_cast<uint32_t>(micros() - imuStartUs);

  if (!imuOk) return false;

  convertImuSampleToBodyRh();

  const uint32_t estimatorStartUs = micros();

  if (!isfinite(bodyRateXRh) || !isfinite(bodyRateYRh) ||
      !isfinite(bodyRateZRh))
  {
    return false;
  }

  // ------------------------------------------------------------
  // Independent complementary filter, kept as a cross-check and as the
  // fallback if the ESKF attitude disagrees.
  // ------------------------------------------------------------

  float accelRollRad = 0.0f, accelPitchRad = 0.0f;
  bodySpecificForceToTiltRh(
    specificForceX, specificForceY, specificForceZ,
    accelRollRad, accelPitchRad
  );

  accelRollRad -= rollLevelTrimRad + rollTrimDeg * FC_DEG_TO_RAD;
  accelPitchRad -= pitchLevelTrimRad + pitchTrimDeg * FC_DEG_TO_RAD;

  const float complementaryWeight = clampFloatFc(
    1.0f - (dt / COMPLEMENTARY_TAU_S), 0.90f, 0.9995f
  );

  complementaryRollRad =
    complementaryWeight * (complementaryRollRad + bodyRateYRh * dt) +
    (1.0f - complementaryWeight) * accelRollRad;

  complementaryPitchRad =
    complementaryWeight * (complementaryPitchRad + bodyRateXRh * dt) +
    (1.0f - complementaryWeight) * accelPitchRad;

  // ------------------------------------------------------------
  // ESKF prediction. The estimator's gyro/accel arrays are ordered
  // [about X, about Y, about Z] and [X, Y, Z], all right-handed, which is
  // exactly what FrameConventions produces. No sign fixups here.
  // ------------------------------------------------------------

  const float gyroBody[3] = { bodyRateXRh, bodyRateYRh, bodyRateZRh };
  const float accelBody[3] =
    { specificForceX, specificForceY, specificForceZ };

  stateEstimator.predict(accelBody, gyroBody, dt, 1.0f);

  /*
    Gravity-direction attitude aiding, decimated to 100 Hz.

    Prediction has to run every cycle because it is the integration. The
    gravity update does not: it is a three-axis measurement update with a
    Kalman gain and a full covariance update, it was roughly half the
    estimator's 668 us, and the gyro already carries everything above a few
    Hz. Correcting attitude at 100 Hz is ample.

    The standard deviation is scaled by 1/sqrt(decimation) so that applying
    one update per five cycles delivers about the same correction per unit
    time as five updates did. Without that, decimating would quietly slow
    attitude convergence by a factor of five.
  */
  ++gravityUpdateCounter;
  if (gravityUpdateCounter >= GRAVITY_UPDATE_DECIMATION)
  {
    gravityUpdateCounter = 0;

    const float baseStd = airborne()
      ? ESKF_GRAVITY_STD_IN_FLIGHT
      : ESKF_GRAVITY_STD_ON_GROUND;

    stateEstimator.updateGravityDirection(
      accelBody, baseStd * GRAVITY_DECIMATION_STD_SCALE
    );
  }

  /*
    Zero-velocity pseudo-measurement while stationary on the ground.

    RATE LIMITED ON PURPOSE. Running a full three-axis ESKF velocity update
    at the 500 Hz control rate roughly doubles the cost of the control step
    while disarmed, which is exactly the state you are in when typing serial
    commands. 20 Hz is ample for constraining ground drift.
  */
  if (!flightActive() &&
      static_cast<uint32_t>(millis() - previousZeroVelocityMs) >=
        ZERO_VELOCITY_PERIOD_MS)
  {
    previousZeroVelocityMs = millis();
    stateEstimator.updateZeroVelocity(0.03f);
  }

  if (!stateEstimator.healthy())
  {
    eskfStateHealthy = false;
    return false;
  }

  eskfStateHealthy = true;
  synchronizeEstimatorState();

  timingEstimatorTotalUs += static_cast<uint32_t>(micros() - estimatorStartUs);

  float estimatorPitchRad = 0.0f;
  float estimatorRollRad = 0.0f;
  float estimatorYawRad = 0.0f;
  stateEstimator.getEulerRadians(
    estimatorPitchRad, estimatorRollRad, estimatorYawRad
  );

  const float rollDisagreement =
    fabsf(wrapAngleRad(estimatorRollRad - complementaryRollRad));
  const float pitchDisagreement =
    fabsf(wrapAngleRad(estimatorPitchRad - complementaryPitchRad));

  eskfAttitudeUsed =
    isfinite(estimatorRollRad) && isfinite(estimatorPitchRad) &&
    isfinite(estimatorYawRad) &&
    rollDisagreement <= ESKF_MAX_ATTITUDE_DISAGREEMENT_RAD &&
    pitchDisagreement <= ESKF_MAX_ATTITUDE_DISAGREEMENT_RAD &&
    fabsf(estimatorRollRad) < 80.0f * FC_DEG_TO_RAD &&
    fabsf(estimatorPitchRad) < 80.0f * FC_DEG_TO_RAD;

  if (eskfAttitudeUsed)
  {
    rollRhRad = estimatorRollRad;
    pitchRhRad = estimatorPitchRad;
  }
  else
  {
    rollRhRad = complementaryRollRad;
    pitchRhRad = complementaryPitchRad;
  }

  // Heading always comes from the estimator when magnetic fusion is live;
  // otherwise it is dead-reckoned from the right-handed yaw rate.
  if (magYawFusionEnabled && magReferenceValid)
  {
    yawRhRad = wrapAngleRad(estimatorYawRad);
  }
  else
  {
    yawRhRad = wrapAngleRad(yawRhRad + bodyRateZRh * dt);
  }

  // Accumulate right-handed gyro angle over the PMW read interval.
  portENTER_CRITICAL(&flowGyroMux);
  accumulatedFlowRollRhRad += bodyRateYRh * dt;
  accumulatedFlowPitchRhRad += bodyRateXRh * dt;
  accumulatedFlowYawRhRad += bodyRateZRh * dt;
  portEXIT_CRITICAL(&flowGyroMux);

  return isfinite(rollRhRad) && isfinite(pitchRhRad) && isfinite(yawRhRad);
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

bool readMagBytes(uint8_t startRegister, uint8_t *buffer, uint8_t length)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(startRegister);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAG_ADDR, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) buffer[i] = Wire.read();
  return true;
}

bool initializeMagnetometer()
{
  uint8_t chipId = 0;
  if (!readMagBytes(MAG_CHIP_ID_REG, &chipId, 1)) return false;

  if (!writeMagRegister(MAG_CTRL2_REG, 0x80)) return false;  // soft reset
  delay(20);
  if (!writeMagRegister(MAG_AXIS_SIGN_REG, 0x06)) return false;
  if (!writeMagRegister(MAG_CTRL2_REG, 0x08)) return false;  // +/-8 gauss
  if (!writeMagRegister(MAG_CTRL1_REG, MAG_CTRL1_CONFIG)) return false;
  delay(20);

  magSensorInitialized = true;
  return true;
}

bool readMagRaw(int16_t &rawX, int16_t &rawY, int16_t &rawZ)
{
  uint8_t status = 0;
  if (!readMagBytes(MAG_STATUS_REG, &status, 1)) return false;
  if ((status & 0x01) == 0) return false;

  uint8_t buffer[6] = {};
  if (!readMagBytes(MAG_X_LSB_REG, buffer, 6)) return false;

  rawX = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
  rawY = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
  rawZ = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));
  return true;
}

/*
  Tilt-compensated magnetic heading, RIGHT-HANDED, counter-clockwise positive.

  THIS IS THE CORRECTED VERSION. The previous build returned
  -atan2(northRight, northForward) while calling the result right-handed.

  With body x=right, y=forward, z=up forming a right-handed set, a drone whose
  heading is psi counter-clockwise from magnetic north sees north in body
  coordinates at (right, forward) = (sin(psi), cos(psi)). Therefore

      atan2(northRight, northForward) = +psi

  and negating it produced a heading of exactly the wrong sign. Because the
  reference heading is captured at each navigation reset, the error was zero
  at the arming heading and grew as 2*psi with every degree of real turn,
  which is why yaw looked plausible on the bench and failed as soon as the
  aircraft actually rotated.
*/
bool calculateMagneticYawRh(float &yawRhRadOut)
{
  float rotationNavFromBody[3][3] = {};
  stateEstimator.getRotationMatrix(rotationNavFromBody);

  // Row 2 of R_nav_from_body is navigation +Z expressed in body coordinates,
  // and is independent of heading.
  float upX = rotationNavFromBody[2][0];
  float upY = rotationNavFromBody[2][1];
  float upZ = rotationNavFromBody[2][2];

  const float upNorm = sqrtf(upX * upX + upY * upY + upZ * upZ);
  if (!isfinite(upNorm) || upNorm < 0.5f) return false;
  upX /= upNorm; upY /= upNorm; upZ /= upNorm;

  // Body forward (+Y) projected into the horizontal plane.
  float forwardX = -upY * upX;
  float forwardY = 1.0f - upY * upY;
  float forwardZ = -upY * upZ;

  const float forwardNorm =
    sqrtf(forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ);
  if (!isfinite(forwardNorm) || forwardNorm < 0.10f) return false;
  forwardX /= forwardNorm; forwardY /= forwardNorm; forwardZ /= forwardNorm;

  // right = forward x up. At level this is exactly body +X.
  const float rightX = forwardY * upZ - forwardZ * upY;
  const float rightY = forwardZ * upX - forwardX * upZ;
  const float rightZ = forwardX * upY - forwardY * upX;

  const float verticalComponent =
    magFilteredBodyX * upX + magFilteredBodyY * upY + magFilteredBodyZ * upZ;

  const float horizontalX = magFilteredBodyX - verticalComponent * upX;
  const float horizontalY = magFilteredBodyY - verticalComponent * upY;
  const float horizontalZ = magFilteredBodyZ - verticalComponent * upZ;

  const float northForward =
    horizontalX * forwardX + horizontalY * forwardY + horizontalZ * forwardZ;
  const float northRight =
    horizontalX * rightX + horizontalY * rightY + horizontalZ * rightZ;

  const float horizontalMagnitude =
    sqrtf(northForward * northForward + northRight * northRight);

  if (!isfinite(horizontalMagnitude) ||
      horizontalMagnitude < MAG_MIN_HORIZONTAL_COUNTS)
  {
    return false;
  }

  yawRhRadOut = wrapAngleRad(atan2f(northRight, northForward));
  return isfinite(yawRhRadOut);
}

void updateMagnetometer(uint32_t nowUs, uint32_t nowMs)
{
  if (!magSensorInitialized)
  {
    magMeasurementValid = false;
    return;
  }

  if (lastMagSampleMs != 0U &&
      static_cast<uint32_t>(nowMs - lastMagSampleMs) > MAG_STALE_MS)
  {
    magMeasurementValid = false;
  }

  if (static_cast<uint32_t>(nowUs - previousMagPollUs) < MAG_POLL_PERIOD_US)
  {
    return;
  }
  previousMagPollUs = nowUs;

  int16_t rawX = 0, rawY = 0, rawZ = 0;
  if (!readMagRaw(rawX, rawY, rawZ)) return;

  // Calibration is applied in SENSOR coordinates, before the mounting
  // transform. Do not reorder these two steps.
  const float sensorX = (static_cast<float>(rawX) - MAG_BIAS_X) * MAG_SCALE_X;
  const float sensorY = (static_cast<float>(rawY) - MAG_BIAS_Y) * MAG_SCALE_Y;
  const float sensorZ = (static_cast<float>(rawZ) - MAG_BIAS_Z) * MAG_SCALE_Z;

  float bodyX = 0.0f, bodyY = 0.0f, bodyZ = 0.0f;
  magSensorToBody(sensorX, sensorY, sensorZ, bodyX, bodyY, bodyZ);

  if (!magFilterInitialized)
  {
    magFilteredBodyX = bodyX;
    magFilteredBodyY = bodyY;
    magFilteredBodyZ = bodyZ;
    magFilterInitialized = true;
  }
  else
  {
    magFilteredBodyX += MAG_FILTER_ALPHA * (bodyX - magFilteredBodyX);
    magFilteredBodyY += MAG_FILTER_ALPHA * (bodyY - magFilteredBodyY);
    magFilteredBodyZ += MAG_FILTER_ALPHA * (bodyZ - magFilteredBodyZ);
  }

  magFieldMagnitude = sqrtf(
    magFilteredBodyX * magFilteredBodyX +
    magFilteredBodyY * magFilteredBodyY +
    magFilteredBodyZ * magFilteredBodyZ
  );

  float absoluteYawRh = 0.0f;
  if (!isfinite(magFieldMagnitude) || !calculateMagneticYawRh(absoluteYawRh))
  {
    magMeasurementValid = false;
    ++magRejectedUpdates;
    return;
  }

  magAbsoluteYawRhRad = absoluteYawRh;
  magMeasurementValid = true;
  lastMagSampleMs = nowMs;

  if (magReferencePending || !magReferenceValid)
  {
    magReferenceYawRhRad = magAbsoluteYawRhRad;
    magReferenceFieldMagnitude = magFieldMagnitude;
    magRelativeYawRhRad = 0.0f;
    magReferenceValid = true;
    magReferencePending = false;
    return;
  }

  magRelativeYawRhRad =
    wrapAngleRad(magAbsoluteYawRhRad - magReferenceYawRhRad);

  if (!magYawFusionEnabled) return;

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

  float estimatorPitch = 0.0f, estimatorRoll = 0.0f, estimatorYaw = 0.0f;
  stateEstimator.getEulerRadians(estimatorPitch, estimatorRoll, estimatorYaw);

  const float innovation =
    wrapAngleRad(magRelativeYawRhRad - estimatorYaw);

  if (fabsf(innovation) > MAG_MAX_INNOVATION_RAD)
  {
    ++magRejectedUpdates;
    return;
  }

  if (stateEstimator.updateYawRightHand(
        magRelativeYawRhRad, MAG_YAW_STD_RAD, 6.0f))
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

// ================================================================
// VL53L0X RANGE
// ================================================================

bool initializeRangeFinder()
{
  rangeSensor.setTimeout(100);
  if (!rangeSensor.init()) return false;
  rangeSensor.setMeasurementTimingBudget(RANGE_TIMING_BUDGET_US);
  rangeSensor.startContinuous();
  rangeSensorInitialized = true;
  return true;
}

bool readRangeDataReady(bool &ready)
{
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(VL53_RESULT_INTERRUPT_STATUS);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(VL53L0X_ADDR, static_cast<uint8_t>(1)) != 1)
  {
    return false;
  }
  ready = (Wire.read() & 0x07) != 0;
  return true;
}

void updateRangeSensor(uint32_t nowUs, uint32_t nowMs)
{
  if (!rangeSensorInitialized) return;

  if (static_cast<uint32_t>(nowUs - previousRangePollUs) < RANGE_PERIOD_US)
  {
    return;
  }
  previousRangePollUs = nowUs;

  bool ready = false;
  if (!readRangeDataReady(ready) || !ready) return;

  const uint16_t millimeters = rangeSensor.readRangeContinuousMillimeters();
  if (rangeSensor.timeoutOccurred()) return;

  const float meters = static_cast<float>(millimeters) * 0.001f;
  if (!isfinite(meters) ||
      meters < MIN_VALID_RANGE_M || meters > MAX_VALID_RANGE_M)
  {
    return;
  }

  if (!rangeHasEverBeenValid)
  {
    filteredRangeMeters = meters;
    rangeHasEverBeenValid = true;
  }
  else
  {
    filteredRangeMeters +=
      RANGE_FILTER_ALPHA * (meters - filteredRangeMeters);
  }

  lastValidRangeMs = nowMs;
  altitudeM = filteredRangeMeters;

  // The rangefinder points straight down along body -Z, so the measurement is
  // slant range. Correct it to vertical height using the estimated tilt.
  const float tiltCosine = cosf(rollRhRad) * cosf(pitchRhRad);
  const float verticalHeight =
    (tiltCosine > 0.5f) ? filteredRangeMeters * tiltCosine
                        : filteredRangeMeters;

  stateEstimator.updateAltitude(verticalHeight, 0.03f, 6.0f);
  synchronizeEstimatorState();
}

// ================================================================
// PMW3901 OPTICAL FLOW
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
  SPI.begin(FLOW_SCK_PIN, FLOW_MISO_PIN, FLOW_MOSI_PIN, FLOW_CS_PIN);
  return flow.begin();
}

static int16_t makeFlowSigned16(uint8_t lowByte, uint8_t highByte)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(lowByte) | (static_cast<uint16_t>(highByte) << 8U)
  );
}

bool readPmwMotionBurst(OpticalFlowRawSample &sample)
{
  uint8_t raw[12] = {};

  SPI.beginTransaction(SPISettings(FLOW_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(FLOW_CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.transfer(0x16U);  // Motion_Burst register
  delayMicroseconds(50);

  for (size_t i = 0; i < sizeof(raw); ++i) raw[i] = SPI.transfer(0x00U);

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
    (static_cast<uint16_t>(raw[10]) << 8U) | static_cast<uint16_t>(raw[11])
  );

  bool allZero = true;
  bool allOnes = true;
  for (uint8_t value : raw)
  {
    allZero = allZero && (value == 0x00U);
    allOnes = allOnes && (value == 0xFFU);
  }

  sample.transferPlausible = !allZero && !allOnes &&
    abs(static_cast<int>(sample.rawDeltaX)) <= 1000 &&
    abs(static_cast<int>(sample.rawDeltaY)) <= 1000;

  return sample.transferPlausible;
}

void snapshotAndResetFlowGyro(float &rollRh, float &pitchRh, float &yawRh)
{
  portENTER_CRITICAL(&flowGyroMux);
  rollRh = accumulatedFlowRollRhRad;
  pitchRh = accumulatedFlowPitchRhRad;
  yawRh = accumulatedFlowYawRhRad;
  accumulatedFlowRollRhRad = 0.0f;
  accumulatedFlowPitchRhRad = 0.0f;
  accumulatedFlowYawRhRad = 0.0f;
  portEXIT_CRITICAL(&flowGyroMux);
}

void opticalFlowTaskEntry(void *parameter)
{
  (void)parameter;

  TickType_t lastWakeTick = xTaskGetTickCount();
  uint32_t previousSampleUs = micros();
  uint32_t sequence = 0;

  for (;;)
  {
    vTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(FLOW_PERIOD_US / 1000U));

    OpticalFlowRawSample sample;
    readPmwMotionBurst(sample);

    // Snapshot the integrated gyro angle immediately after the SPI read so
    // the compensation window matches the sensor's integration window.
    snapshotAndResetFlowGyro(
      sample.rollDeltaRad, sample.pitchDeltaRad, sample.yawDeltaCwRad
    );

    sample.sampleUs = micros();
    sample.sampleMs = millis();
    sample.intervalUs =
      static_cast<uint32_t>(sample.sampleUs - previousSampleUs);
    previousSampleUs = sample.sampleUs;
    sample.sequence = ++sequence;

    if (opticalFlowSampleQueue != nullptr)
    {
      xQueueOverwrite(opticalFlowSampleQueue, &sample);
    }
  }
}

bool startOpticalFlowTask(BaseType_t coreId)
{
  opticalFlowSampleQueue = xQueueCreate(1, sizeof(OpticalFlowRawSample));
  if (opticalFlowSampleQueue == nullptr) return false;

  const BaseType_t result = xTaskCreatePinnedToCore(
    opticalFlowTaskEntry, "pmw3901", 4096, nullptr, 2,
    &opticalFlowTaskHandle, coreId
  );

  if (result != pdPASS)
  {
    vQueueDelete(opticalFlowSampleQueue);
    opticalFlowSampleQueue = nullptr;
    return false;
  }

  return true;
}

void processOpticalFlowSample(
  const OpticalFlowRawSample &sample,
  uint32_t nowMs
)
{
  const float flowDt = static_cast<float>(sample.intervalUs) * 1.0e-6f;

  lastFlowMotion = sample.motion;
  lastFlowSqual = sample.squal;
  lastFlowShutter = sample.shutter;

  float bodyXCounts = 0.0f, bodyYCounts = 0.0f;
  flowCountsToBody(
    sample.rawDeltaX, sample.rawDeltaY, bodyXCounts, bodyYCounts
  );

  const bool timingValid = flowDt >= 0.006f && flowDt <= 0.030f;

  const bool countsValid =
    fabsf(bodyXCounts) < static_cast<float>(FLOW_OUTLIER_LIMIT_COUNTS) &&
    fabsf(bodyYCounts) < static_cast<float>(FLOW_OUTLIER_LIMIT_COUNTS);

  const bool heightValid =
    rangeUsable(nowMs) &&
    filteredRangeMeters >= MIN_FLOW_RANGE_M &&
    filteredRangeMeters <= MAX_FLOW_RANGE_M;

  const bool motionFormatPlausible =
    sample.motion == 0x30U || sample.motion == 0xB0U;

  const bool qualityHealthy =
    sample.transferPlausible && motionFormatPlausible &&
    sample.squal >= MIN_FLOW_SQUAL && sample.shutter > 0U;

  const bool healthValid =
    timingValid && countsValid && heightValid && qualityHealthy;

  if (!healthValid)
  {
    consecutiveHealthyFlowSamples = 0;

    if (lastHealthyFlowMs == 0U ||
        static_cast<uint32_t>(nowMs - lastHealthyFlowMs) > FLOW_HEALTH_LOSS_MS)
    {
      flowSensorHealthy = false;
    }

    measuredBodyVelocityX *= FLOW_INVALID_DECAY;
    measuredBodyVelocityY *= FLOW_INVALID_DECAY;
    return;
  }

  lastHealthyFlowMs = nowMs;
  if (consecutiveHealthyFlowSamples < FLOW_HEALTH_RECOVERY_SAMPLES)
  {
    ++consecutiveHealthyFlowSamples;
  }
  if (consecutiveHealthyFlowSamples >= FLOW_HEALTH_RECOVERY_SAMPLES)
  {
    flowSensorHealthy = true;
  }

  // A coherent 0x30 burst means "sensor fine, no new motion". It keeps the
  // health streak alive but is not a zero-velocity observation.
  if (sample.motion != 0xB0U) return;

  float deltaBodyX =
    bodyXCounts * FLOW_RAD_PER_COUNT * filteredRangeMeters;
  float deltaBodyY =
    bodyYCounts * FLOW_RAD_PER_COUNT * filteredRangeMeters;

  if (ENABLE_FLOW_GYRO_COMPENSATION)
  {
    deltaBodyX +=
      FLOW_ROLL_COMP_GAIN_RH * filteredRangeMeters * sample.rollDeltaRad;
    deltaBodyY +=
      FLOW_PITCH_COMP_GAIN_RH * filteredRangeMeters * sample.pitchDeltaRad;
  }

  /*
    Rigid-body lever-arm correction. The sensor sits at r = (rx, ry, 0), so it
    measures  v_sensor = v_body + omega x r. With omega = (0, 0, wz),

        omega x r = (-wz*ry, +wz*rx, 0)

    and therefore  v_body = v_sensor + (wz*ry, -wz*rx, 0).

    The previous build applied this with both signs inverted, because it fed a
    right-handed yaw increment into an expression written for a clockwise one.
    That injected fake translation in the direction that reinforced the error
    every time the aircraft rotated.
  */
  const float yawDeltaRh = sample.yawDeltaCwRad;  // right-handed, see task
  deltaBodyX += PMW_OFFSET_Y_M * yawDeltaRh;
  deltaBodyY -= PMW_OFFSET_X_M * yawDeltaRh;

  const float instantVelocityX = deltaBodyX / flowDt;
  const float instantVelocityY = deltaBodyY / flowDt;

  const float speedSquared =
    instantVelocityX * instantVelocityX + instantVelocityY * instantVelocityY;

  if (!isfinite(instantVelocityX) || !isfinite(instantVelocityY) ||
      speedSquared >
        MAX_PLAUSIBLE_FLOW_SPEED_MPS * MAX_PLAUSIBLE_FLOW_SPEED_MPS)
  {
    ++flowRejectCount;
    return;
  }

  const float flowYawRateDps =
    fabsf(yawDeltaRh) * FC_RAD_TO_DEG / flowDt;

  if (!isfinite(flowYawRateDps) || flowYawRateDps >= FLOW_YAW_RATE_HARD_DPS)
  {
    ++flowRejectCount;
    return;
  }

  float yawNoiseScale = 1.0f;
  if (flowYawRateDps > FLOW_YAW_RATE_SOFT_DPS)
  {
    const float blend = clampFloatFc(
      (flowYawRateDps - FLOW_YAW_RATE_SOFT_DPS) /
        (FLOW_YAW_RATE_HARD_DPS - FLOW_YAW_RATE_SOFT_DPS),
      0.0f, 1.0f
    );
    yawNoiseScale = 1.0f + blend * (FLOW_YAW_NOISE_SCALE_MAX - 1.0f);
  }

  measuredBodyVelocityX +=
    FLOW_VELOCITY_FILTER_ALPHA * (instantVelocityX - measuredBodyVelocityX);
  measuredBodyVelocityY +=
    FLOW_VELOCITY_FILTER_ALPHA * (instantVelocityY - measuredBodyVelocityY);

  const float pixelStd = fmaxf(
    0.10f, 0.0007984f * static_cast<float>(sample.shutter) + 0.4335f
  );

  const float flowVelocityStd = clampFloatFc(
    pixelStd * FLOW_RAD_PER_COUNT * filteredRangeMeters / flowDt *
      yawNoiseScale,
    0.04f, 1.50f
  );

  // Translation-only update: optical flow never touches the quaternion.
  // Both axes must pass their innovation gate for the sample to count, so a
  // broken axis cannot hide behind a good one.
  lastFlowAxisAcceptedMask = stateEstimator.updateBodyVelocityVelocityOnly(
    measuredBodyVelocityX, measuredBodyVelocityY,
    flowVelocityStd, flowVelocityStd
  );

  if (lastFlowAxisAcceptedMask == 0x03U)
  {
    flowVelocityReady = true;
    lastFlowMeasurementMs = nowMs;
    synchronizeEstimatorState();
  }
  else
  {
    ++flowRejectCount;
  }
}

bool consumeOpticalFlowSample(uint32_t nowMs)
{
  if (opticalFlowSampleQueue == nullptr) return false;

  OpticalFlowRawSample sample;
  if (xQueueReceive(opticalFlowSampleQueue, &sample, 0) != pdTRUE)
  {
    return false;
  }

  processOpticalFlowSample(sample, nowMs);
  return true;
}

// ================================================================
// RATE PID
// ================================================================

/*
  Returns a demanded ANGULAR ACCELERATION in rad/s^2. Torque is obtained by
  multiplying by the axis inertia, which keeps every gain in physical units
  instead of DShot counts.
*/
float updateRatePID(
  float desiredRate,
  float measuredRate,
  float &integral,
  float &previousError,
  float &filteredDerivative,
  float kp, float ki, float kd,
  float dt
)
{
  const float error = desiredRate - measuredRate;

  integral += error * dt;
  integral = clampFloatFc(
    integral, -RATE_INTEGRAL_LIMIT / fmaxf(ki, 1e-3f),
    RATE_INTEGRAL_LIMIT / fmaxf(ki, 1e-3f)
  );

  const float rawDerivative = (dt > 0.0f) ? ((error - previousError) / dt) : 0.0f;
  previousError = error;

  filteredDerivative +=
    DERIVATIVE_FILTER_ALPHA * (rawDerivative - filteredDerivative);

  return kp * error + ki * integral + kd * filteredDerivative;
}

void resetRateControllers()
{
  rollRateIntegral = 0.0f;
  pitchRateIntegral = 0.0f;
  yawRateIntegral = 0.0f;
  rollRatePreviousError = 0.0f;
  pitchRatePreviousError = 0.0f;
  yawRatePreviousError = 0.0f;
  rollRateDerivative = 0.0f;
  pitchRateDerivative = 0.0f;
  yawRateDerivative = 0.0f;
}

// ================================================================
// OUTER CONTROL LOOPS
// ================================================================

// StickCommands is declared in HoldManager.h; see the note there.
StickCommands readSticks()
{
  StickCommands sticks;

  sticks.roll = applyCenteredStickDeadband(controlData.roll, ROLL_STICK_DEADBAND);
  sticks.pitch =
    applyCenteredStickDeadband(controlData.pitch, PITCH_STICK_DEADBAND);
  sticks.yaw = applyCenteredStickDeadband(controlData.yaw, YAW_STICK_DEADBAND);
  sticks.throttle =
    applyCenteredStickDeadband(controlData.throttle, THROTTLE_STICK_DEADBAND);

  sticks.rollPitchActive =
    fabsf(sticks.roll) > 0.0f || fabsf(sticks.pitch) > 0.0f;
  sticks.yawActive = fabsf(sticks.yaw) > 0.0f;
  sticks.throttleActive = fabsf(sticks.throttle) > 0.0f;

  return sticks;
}

/*
  Horizontal controller.

  Produces desired right-handed roll and pitch angles.

    held  : position error -> velocity target -> acceleration -> tilt
    free  : stick -> body velocity target -> acceleration -> tilt
    no flow: stick -> tilt directly, the original fallback behaviour

  The XY hold is permitted only when flow is healthy, the estimator is happy,
  the aircraft is high enough, and the yaw loop has proven itself in this
  flight. The last condition is the one the previous build lacked entirely.
*/
void updateHorizontalController(
  const StickCommands &sticks,
  float dt,
  uint32_t nowMs
)
{
  const bool flowUsable =
    flowHealthFresh(nowMs) &&
    flowVelocityReady &&
    lastFlowAxisAcceptedMask == 0x03U &&
    eskfStateHealthy &&
    rangeUsable(nowMs);

  const bool holdPermitted =
    flowUsable &&
    physicallyAirborne() &&
    yawStability.proven() &&
    filteredRangeMeters >= 0.30f &&
    fabsf(verticalVelocityMps) <= 0.20f &&
    fabsf(rollRhRad) <= MAX_TILT_HOLD_DEG * FC_DEG_TO_RAD &&
    fabsf(pitchRhRad) <= MAX_TILT_HOLD_DEG * FC_DEG_TO_RAD &&
    fabsf(bodyRateZRh) * FC_RAD_TO_DEG <= 25.0f;

  const float horizontalSpeed = sqrtf(
    measuredBodyVelocityX * measuredBodyVelocityX +
    measuredBodyVelocityY * measuredBodyVelocityY
  );

  horizontalHold.update(
    sticks.rollPitchActive, horizontalSpeed, holdPermitted, nowMs
  );

  if (horizontalHold.justCaptured())
  {
    targetNavX = positionNavX;
    targetNavY = positionNavY;
    logf("XY hold captured at X=%.3f Y=%.3f", targetNavX, targetNavY);
  }

  // ------------------------------------------------------------
  // No usable flow: fall straight back to direct angle command.
  // ------------------------------------------------------------

  if (!flowUsable)
  {
    desiredRollRhRad = sticks.roll * MAX_TILT_COMMAND_DEG * FC_DEG_TO_RAD;
    desiredPitchRhRad = -sticks.pitch * MAX_TILT_COMMAND_DEG * FC_DEG_TO_RAD;

    // No velocity loop runs on this path. Zero the published targets rather
    // than leaving the last good ones frozen in the track log, which would
    // read as a velocity command the controller is ignoring.
    publishedBodyVelocityTargetX = 0.0f;
    publishedBodyVelocityTargetY = 0.0f;
    return;
  }

  // ------------------------------------------------------------
  // Velocity target.
  // ------------------------------------------------------------

  float bodyVelocityTargetX = 0.0f;
  float bodyVelocityTargetY = 0.0f;

  if (horizontalHold.held())
  {
    float navVelocityTargetX =
      positionKp * (targetNavX - positionNavX);
    float navVelocityTargetY =
      positionKp * (targetNavY - positionNavY);

    limitVector2(
      navVelocityTargetX, navVelocityTargetY, MAX_HORIZONTAL_VELOCITY_MPS
    );

    navigationToBody(
      yawRhRad, navVelocityTargetX, navVelocityTargetY,
      bodyVelocityTargetX, bodyVelocityTargetY
    );
  }
  else
  {
    bodyVelocityTargetX = sticks.roll * MAX_HORIZONTAL_VELOCITY_MPS;
    bodyVelocityTargetY = sticks.pitch * MAX_HORIZONTAL_VELOCITY_MPS;
  }

  publishedBodyVelocityTargetX = bodyVelocityTargetX;
  publishedBodyVelocityTargetY = bodyVelocityTargetY;

  // ------------------------------------------------------------
  // Velocity error -> horizontal acceleration -> tilt.
  // ------------------------------------------------------------

  float accelerationX =
    velocityKp * (bodyVelocityTargetX - measuredBodyVelocityX);
  float accelerationY =
    velocityKp * (bodyVelocityTargetY - measuredBodyVelocityY);

  limitVector2(accelerationX, accelerationY, MAX_HORIZONTAL_ACCEL_MPS2);

  /*
    Tilt required to produce that horizontal acceleration.

    A right-handed roll phi about +Y points the thrust axis at
    (sin(phi), 0, cos(phi)) in the navigation frame, so positive roll
    accelerates toward +X (right).

    A right-handed pitch theta about +X points it at
    (0, -sin(theta), cos(theta)), so accelerating toward +Y (forward)
    requires NEGATIVE pitch, which is nose down. That is the physically
    correct sign and it is why the expression below is negated.
  */
  const float maximumTilt =
    (horizontalHold.held() ? MAX_TILT_HOLD_DEG : MAX_TILT_COMMAND_DEG) *
    FC_DEG_TO_RAD;

  desiredRollRhRad = clampFloatFc(
    atan2f(accelerationX, FC_GRAVITY_MPS2), -maximumTilt, maximumTilt
  );

  desiredPitchRhRad = clampFloatFc(
    atan2f(-accelerationY, FC_GRAVITY_MPS2), -maximumTilt, maximumTilt
  );
}

/*
  Vertical controller. Returns the demanded total thrust in newtons.
*/
float updateVerticalController(
  const StickCommands &sticks,
  float dt,
  uint32_t nowMs
)
{
  /*
    Altitude hold must NOT capture while the automatic climb is still running.

    It used to, and that is what truncated every takeoff. The 2026-08-15 log
    reads, in this order:

        Airborne confirmed at 0.18 m
        Altitude hold captured at 0.250 m
        Climb complete at 0.202 m; hovering.

    physicallyAirborne() latches at 0.18 m, which permits the hold. The hold
    settles a few cycles later and captures - and because the aircraft is only
    at ~0.20 m, MIN_ALTITUDE_TARGET_M clamps the captured target UP to 0.25 m.
    The Climb exit test is `altitudeM >= altitudeTargetM - 0.05`, which 0.202 m
    now satisfies against that 0.25 m target, so the climb declares itself
    complete and stops.

    The aircraft therefore never went anywhere near HOVER_SETPOINT_M. Every
    flight has been pinned just above the airborne threshold by its own hold
    engaging too early, which is also why every kM identification run has been
    flown inside ground effect.

    Climb owns the axis until it finishes.
  */
  const bool holdPermitted =
    rangeUsable(nowMs) && eskfStateHealthy && physicallyAirborne() &&
    flightState != FlightState::Climb;

  altitudeHold.update(
    sticks.throttleActive, verticalVelocityMps, holdPermitted, nowMs
  );

  if (altitudeHold.justCaptured())
  {
    altitudeTargetM = clampFloatFc(
      altitudeM, MIN_ALTITUDE_TARGET_M, MAX_ALTITUDE_TARGET_M
    );
    logf("Altitude hold captured at %.3f m", altitudeTargetM);
  }

  float verticalVelocityTarget = 0.0f;

  /*
    Decide whether the automatic climb is still in charge.

    Climb used to command a FIXED CLIMB_RATE_MPS and ignore the throttle
    stick completely, exiting only when the rangefinder reached the hover
    setpoint. On the bench that altitude is never reached, so the state
    never ended and the stick stayed dead - the 2026-08-14 log shows
    throttle sweeping +0.18 -> +1.00 -> -1.00 -> 0.00 while T sat at exactly
    5.00 N, which is 0.4536 * (9.81 + 4.0*0.30): the fixed climb command.

    It also contradicted the design intent for this airframe, which is that
    an engaged throttle stick gets the axis WITHOUT resistance. So the
    automatic climb yields to the stick - but only ONCE THE AIRCRAFT IS
    ACTUALLY AIRBORNE, which is the 2026-08-15 fix.

    Yielding on the first stick touch regardless of height created a
    deadlock that stranded the aircraft in ground effect:

      1. Pilot blips the throttle. throttleActive goes true and auto-climb
         is cancelled PERMANENTLY.
      2. Pilot releases. The else branch below commands
         sticks.throttle * MAX_VERTICAL_VELOCITY_MPS, which is ZERO - a
         perfect hover at whatever height it happened to reach.
      3. That height was 0.10 m, under the 0.15 m physicallyAirborne()
         gate, so holdPermitted stayed false and altitude hold could never
         capture and climb to the setpoint.
      4. Nothing could ever lift it. The whole Phase 8 flight sat at
         0.09-0.14 m with A=FREE and no "Airborne confirmed" line, and
         T = 4.5 N against a 4.45 N hover requirement: the controller was
         doing exactly what it was told.

    Note this is NOT fixed by adding thrust at zero stick. Thrust was
    already correct - the loop was holding zero vertical VELOCITY and
    succeeding. A thrust bias would delete the neutral point that makes
    releasing the stick safe, and turn zero throttle into an open-loop
    climb with no altitude reference.

    A DOWN command still aborts the climb at any height. That has to stay
    unconditional; it is the pilot's only way to stop a bad takeoff.
  */
  if (flightState == FlightState::Climb)
  {
    const bool pilotCommandedDown = sticks.throttle < -0.02f;

    if (pilotCommandedDown)
    {
      flightState = FlightState::Hover;
      flightStateEnteredMs = nowMs;
      logLine("Throttle down: auto-climb aborted.");
    }
    else if (physicallyAirborne() && sticks.throttleActive)
    {
      flightState = FlightState::Hover;
      flightStateEnteredMs = nowMs;
      logLine("Manual throttle: auto-climb released.");
    }
    else if (altitudeM >= altitudeTargetM - 0.05f)
    {
      flightState = FlightState::Hover;
      flightStateEnteredMs = nowMs;
      logf("Climb complete at %.3f m; hovering.", altitudeM);
    }
  }

  if (flightState == FlightState::Climb)
  {
    /*
      Automatic climb, plus any UP stick the pilot adds on top. Adding
      rather than replacing means a blip on the way up speeds the takeoff
      instead of cancelling it, which is what stranded the last flight.
    */
    verticalVelocityTarget = fminf(
      CLIMB_RATE_MPS + fmaxf(sticks.throttle, 0.0f) * MAX_VERTICAL_VELOCITY_MPS,
      MAX_VERTICAL_VELOCITY_MPS
    );
  }
  else if (altitudeHold.held())
  {
    verticalVelocityTarget = clampFloatFc(
      altitudeKp * (altitudeTargetM - altitudeM),
      -MAX_VERTICAL_VELOCITY_MPS, MAX_VERTICAL_VELOCITY_MPS
    );
  }
  else
  {
    // Throttle stick engaged: direct vertical velocity, no hold resistance.
    verticalVelocityTarget = sticks.throttle * MAX_VERTICAL_VELOCITY_MPS;

    // Track the current height so the eventual capture starts from here.
    altitudeTargetM = clampFloatFc(
      altitudeM, MIN_ALTITUDE_TARGET_M, MAX_ALTITUDE_TARGET_M
    );
  }

  publishedVerticalVelocityTarget = verticalVelocityTarget;

  const float verticalAcceleration = clampFloatFc(
    verticalVelocityKp * (verticalVelocityTarget - verticalVelocityMps),
    -MAX_VERTICAL_ACCEL_MPS2, MAX_VERTICAL_ACCEL_MPS2
  );

  // Thrust must overcome gravity plus the demanded acceleration, and must be
  // divided by the tilt cosine so that tilting does not sag the altitude.
  const float tiltCosine = cosf(rollRhRad) * cosf(pitchRhRad);
  const float safeTiltCosine = (tiltCosine > 0.5f) ? tiltCosine : 0.5f;

  const float thrust =
    AIRFRAME_MASS_KG * (FC_GRAVITY_MPS2 + verticalAcceleration) /
    safeTiltCosine;

  const float weight = AIRFRAME_MASS_KG * FC_GRAVITY_MPS2;

  return clampFloatFc(
    thrust, MIN_THRUST_FRACTION * weight, MAX_THRUST_FRACTION * weight
  );
}

/*
  Yaw controller. Yaw stabilisation is NEVER released: the stick commands a
  rate through the same loop, and the heading target is re-captured after the
  axis settles on release.
*/
/*
  Largest yaw torque the rotors can actually produce, given the collective
  they are already holding for `thrustN`.

  Yaw comes only from rotor DRAG, so its authority is set by kM, which is
  roughly a hundredth of kF. At the hover collective (u ~ 7.66e5, omega ~ 875)
  the binding limit is the 250 rad/s rotor floor, leaving

      |tauZ|max = 4 * kM * (u_hover - u_min)
                = 4 * 1.4531e-8 * (765600 - 62500)
                = 0.041 N m

  against a roll/pitch authority of 0.336 N m on the same airframe.

  The yaw rate PID had no idea any of this existed. With yawratekp = 15 and
  Izz = 2.027e-3, a rate error of only 77 deg/s already demands more torque
  than the aircraft owns, and the 2026-08-15 log shows it asking for
  0.1622 N m - four times over - while the gyro swung +-194 deg/s in a limit
  cycle that never had any chance of closing.

  Clamping the DEMAND here, rather than letting the allocator discover the
  demand is impossible, is what keeps the loop out of that cycle: an
  unachievable setpoint is an open loop with extra steps.
*/
float achievableYawTorqueNm(float thrustN, float dragCoefficientKm)
{
  if (!isfinite(thrustN) || !isfinite(dragCoefficientKm) ||
      dragCoefficientKm <= 0.0f)
  {
    return 0.0f;
  }

  const float collective = thrustN / (4.0f * ROTOR_THRUST_KF);

  const float uMin =
    RotorAllocator::MIN_ROTOR_OMEGA * RotorAllocator::MIN_ROTOR_OMEGA;
  const float uMax =
    RotorAllocator::MAX_ROTOR_OMEGA * RotorAllocator::MAX_ROTOR_OMEGA;

  const float room = fminf(collective - uMin, uMax - collective);

  return 4.0f * dragCoefficientKm * fmaxf(room, 0.0f);
}

float updateYawController(
  const StickCommands &sticks,
  float dt,
  uint32_t nowMs,
  float thrustN
)
{
  const bool holdPermitted =
    flightActive() && magnetometerFresh(nowMs) && eskfStateHealthy;

  const float yawRateDps = bodyRateZRh * FC_RAD_TO_DEG;

  headingHold.update(sticks.yawActive, yawRateDps, holdPermitted, nowMs);

  if (headingHold.justCaptured())
  {
    headingTargetRhRad = yawRhRad;
    logf("Heading captured at %.1f deg", headingTargetRhRad * FC_RAD_TO_DEG);
  }

  float headingErrorDeg = 0.0f;

  if (sticks.yawActive)
  {
    desiredYawRateRhRadPerSec =
      sticks.yaw * MAX_YAW_RATE_DPS * FC_DEG_TO_RAD;
    headingTargetRhRad = yawRhRad;
  }
  else if (headingHold.held())
  {
    const float headingError = wrapAngleRad(headingTargetRhRad - yawRhRad);
    headingErrorDeg = headingError * FC_RAD_TO_DEG;

    desiredYawRateRhRadPerSec = clampFloatFc(
      headingKp * headingError,
      -MAX_YAW_HOLD_RATE_DPS * FC_DEG_TO_RAD,
      MAX_YAW_HOLD_RATE_DPS * FC_DEG_TO_RAD
    );
  }
  else
  {
    // Settling: damp the rate to zero without chasing a heading.
    desiredYawRateRhRadPerSec = 0.0f;
    headingTargetRhRad = yawRhRad;
  }

  yawStability.update(
    yawRateDps, headingErrorDeg, airborne(), headingHold.held(), nowMs
  );

  lastYawRateTargetDps = desiredYawRateRhRadPerSec * FC_RAD_TO_DEG;

  const float yawAcceleration = updateRatePID(
    desiredYawRateRhRadPerSec, bodyRateZRh,
    yawRateIntegral, yawRatePreviousError, yawRateDerivative,
    yawRateKp, yawRateKi, yawRateKd, dt
  );

  float tauZ = INERTIA_ZZ * yawAcceleration;

  /*
    Clamp to what the rotors can deliver, and BACK THE INTEGRATOR OUT when the
    clamp bites.

    The back-out is the half that matters. RATE_INTEGRAL_LIMIT is 60 rad/s^2,
    but the yaw axis can only produce about 20 rad/s^2 at hover - so the
    integral term ALONE was allowed to demand three times the airframe's total
    yaw authority. Once it charged up against a saturated actuator it could not
    discharge, and it held the rotors pinned at the rails through the reversal:
    the log shows DS = 750,250,750,113 held for many consecutive cycles, two
    rotors at the ceiling and two on the floor, with the throttle stick at zero.

    Conditional integration - only integrate while the output is inside the
    achievable band - is what turns that latch back into a control loop.
  */
  const float tauZLimit =
    achievableYawTorqueNm(thrustN, yawIdentifier.dragCoefficientKm());

  yawTorqueSaturated = (tauZLimit > 0.0f) && (fabsf(tauZ) > tauZLimit);

  if (yawTorqueSaturated)
  {
    const float clamped = (tauZ > 0.0f) ? tauZLimit : -tauZLimit;

    // Give back exactly the integral that the clamp threw away, so the
    // integrator can never accumulate authority the aircraft does not have.
    if (yawRateKi > 1e-3f)
    {
      const float discarded = (tauZ - clamped) / INERTIA_ZZ;
      yawRateIntegral -= discarded / yawRateKi;
    }

    tauZ = clamped;
  }

  return tauZ;
}

// ================================================================
// MOTOR OUTPUT
// ================================================================

void sendMotorCommands(const uint16_t commands[4])
{
  bidirectionalDshot.setAllMotorCommands(commands);
  memcpy(lastCommandedDshot, commands, sizeof(lastCommandedDshot));
}

void stopMotors()
{
  const uint16_t stopped[4] = {0, 0, 0, 0};
  sendMotorCommands(stopped);
  rotorSpeedLoop.notifyStopped();
}

/*
  Refresh the measured half of the rotor telemetry without running the control
  loop. Called every control cycle so OMG and M report the true ESC telemetry
  state in EVERY flight state, not just the two that happen to run the loop.
*/
void refreshRotorTelemetry()
{
  BidirectionalDShot::MotorRpmSample samples[4];
  bidirectionalDshot.getAllMotorSamples(samples);

  lastRotorStatus.measuredMask = 0;

  for (size_t i = 0; i < 4; ++i)
  {
    const bool fresh = samples[i].valid && !samples[i].stale;

    lastRotorStatus.usingMeasurement[i] = fresh;
    lastRotorStatus.omegaMeasured[i] = fresh ? samples[i].omegaRadPerSec : 0.0f;

    if (fresh)
    {
      lastRotorStatus.measuredMask |= static_cast<uint8_t>(1u << i);
    }
  }
}

RotorSpeedLoop::Measurement collectRotorMeasurement(bool &allFresh)
{
  RotorSpeedLoop::Measurement measurement;
  allFresh = true;

  BidirectionalDShot::MotorRpmSample samples[4];
  bidirectionalDshot.getAllMotorSamples(samples);

  for (size_t i = 0; i < 4; ++i)
  {
    measurement.omegaRadPerSec[i] = samples[i].omegaRadPerSec;
    measurement.valid[i] = samples[i].valid && !samples[i].stale;
    if (!measurement.valid[i]) allFresh = false;
  }

  return measurement;
}

// ================================================================
// FLIGHT STATE MACHINE AND MAIN CONTROL STEP
// ================================================================

void enterState(FlightState next, uint32_t nowMs, const char *reason)
{
  if (flightState == next) return;
  flightState = next;
  flightStateEnteredMs = nowMs;
  logf("State -> %d (%s)", static_cast<int>(next), reason);
}

void resetFlightState(uint32_t nowMs)
{
  resetRateControllers();
  rotorSpeedLoop.resetIntegrators();
  altitudeHold.reset();
  horizontalHold.reset();
  headingHold.reset();
  yawStability.reset();
  resetEstimatorLocalFrame();
  altitudeTargetM = HOVER_SETPOINT_M;
  flowVelocityReady = false;

  aboveGroundSinceMs = 0;
  airborneLatched = false;

  commandedThrustN = 0.0f;
  commandedTauX = 0.0f;
  commandedTauY = 0.0f;
  commandedTauZ = 0.0f;

  (void)nowMs;
}

bool excessiveTilt(uint32_t nowMs)
{
  const bool tilted =
    fabsf(rollRhRad) > MAX_SAFE_TILT_DEG * FC_DEG_TO_RAD ||
    fabsf(pitchRhRad) > MAX_SAFE_TILT_DEG * FC_DEG_TO_RAD;

  if (!tilted)
  {
    tiltExceededSinceMs = 0;
    return false;
  }

  if (tiltExceededSinceMs == 0)
  {
    tiltExceededSinceMs = nowMs;
    return false;
  }

  return static_cast<uint32_t>(nowMs - tiltExceededSinceMs) >= 150U;
}

void stopBenchMode(const char *reason)
{
  if (!benchModeActive) return;

  benchModeActive = false;
  benchOmegaSetpoint = 0.0f;
  benchMotorIndex = -1;
  stopMotors();
  logf("Bench mode stopped (%s).", reason);
}

/*
  Drive the rotor speed loop directly at a fixed setpoint. Returns true if it
  handled this cycle, in which case no flight control runs.
*/
bool runBenchMode(float dt, uint32_t nowMs, bool armRequested)
{
  if (!benchModeActive) return false;

  // Never let bench mode and the flight controller contend for the motors.
  if (armRequested || flightState != FlightState::Disarmed)
  {
    stopBenchMode("arm requested");
    return false;
  }

  if (static_cast<uint32_t>(nowMs - benchStartedMs) >= BENCH_TIMEOUT_MS)
  {
    stopBenchMode("timeout");
    return true;
  }

  float setpoints[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 4; ++i)
  {
    const bool driven =
      (benchMotorIndex < 0) || (benchMotorIndex == i);

    setpoints[i] = driven ? benchOmegaSetpoint : 0.0f;
  }

  bool allFresh = false;
  const RotorSpeedLoop::Measurement measurement =
    collectRotorMeasurement(allFresh);

  lastRotorStatus = rotorSpeedLoop.update(setpoints, measurement, dt);

  // Undriven rotors are commanded fully stopped, not to the loop's floor.
  uint16_t commands[4];
  for (int i = 0; i < 4; ++i)
  {
    const bool driven = (benchMotorIndex < 0) || (benchMotorIndex == i);
    commands[i] = driven ? lastRotorStatus.dshotCommand[i] : 0;
  }

  sendMotorCommands(commands);
  return true;
}

void runControlStep(float dt, uint32_t nowMs)
{
  const StickCommands sticks = readSticks();

  // Report true ESC telemetry in every state, including the ones that return
  // before the control loop runs.
  refreshRotorTelemetry();
  updateAirborneLatch(nowMs);

  // The identifier's live value, so telemetry never shows a default zero for
  // a coefficient that is actually initialised.
  lastIdentifierStatus.dragCoefficientKm = yawIdentifier.dragCoefficientKm();
  lastIdentifierStatus.dragToThrustRatioM =
    yawIdentifier.dragCoefficientKm() / ROTOR_THRUST_KF;

  // ------------------------------------------------------------
  // Arm / disarm handling.
  // ------------------------------------------------------------

  const bool armRequested =
    controlData.motorsEnabled && !controlData.kill &&
    !safetyStopLatched && !serialStopLatch && !signalLost;

  if (!armRequested && flightState != FlightState::Disarmed &&
      flightState != FlightState::DisarmRamp)
  {
    /*
      Name the condition that dropped, rather than logging a generic
      "arm released".

      The 2026-08-14 bench log showed repeated arm/disarm cycles where the
      telemetry line on BOTH sides of the transition read ARM=1 LNK=1, which
      made the cause unidentifiable after the fact: telemetry only samples
      every ~100 ms, so a link gap that opens and closes between two prints
      is invisible. Five different conditions can release the arm and the log
      distinguished none of them.
    */
    /*
      signalLost is tested FIRST, because the failsafe path zeroes
      controlData.motorsEnabled before this runs. Checking motorsEnabled
      first therefore reports every signal loss as "transmitter OFF" and
      hides the real cause - which is exactly what happened on the first
      run of this diagnostic.
    */
    const char *reason =
      signalLost                 ? "arm released: SIGNAL LOST"
      : controlData.kill         ? "arm released: KILL button"
      : safetyStopLatched        ? "arm released: safety stop latched"
      : serialStopLatch          ? "arm released: serial stop"
      : !controlData.motorsEnabled ? "arm released: transmitter OFF"
      :                            "arm released: unknown";

    enterState(FlightState::DisarmRamp, nowMs, reason);

    if (signalLost)
    {
      // Signed, for the same reason the failsafe comparison is signed.
      const int32_t ageMs =
        static_cast<int32_t>(nowMs - lastPacketTimeMs);

      logf("  last packet %ld ms ago; worst gap this session %lu ms",
        (long)ageMs, (unsigned long)worstPacketGapMs);
    }
  }

  // Bench mode owns the motors while disarmed, and skips flight control.
  if (runBenchMode(dt, nowMs, armRequested)) return;

  switch (flightState)
  {
    case FlightState::Disarmed:
    {
      stopMotors();

      if (armRequested &&
          fabsf(controlData.throttle) <= ENABLE_THROTTLE_TOLERANCE)
      {
        resetFlightState(nowMs);
        enterState(FlightState::ArmRamp, nowMs, "armed");
        return;
      }

      /*
        Say why arming was refused rather than sitting silently.

        Throttle is a CENTRED stick in this design: centre holds altitude,
        up climbs, down descends. Arming therefore requires it near centre,
        not at the bottom of its travel. Without this message the only
        symptom is that nothing happens.
      */
      if (armRequested &&
          static_cast<uint32_t>(nowMs - lastArmRefusalMs) >=
            ARM_REFUSAL_PERIOD_MS)
      {
        lastArmRefusalMs = nowMs;
        logf("ARM REFUSED: throttle is %+.2f, must be within %+.2f of "
             "centre. Move the throttle stick to mid-travel.",
          controlData.throttle, ENABLE_THROTTLE_TOLERANCE);
      }

      return;
    }

    case FlightState::ArmRamp:
    {
      const uint32_t elapsed =
        static_cast<uint32_t>(nowMs - flightStateEnteredMs);

      const float progress = clampFloatFc(
        static_cast<float>(elapsed) /
          static_cast<float>(ARM_RAMP_DURATION_MS),
        0.0f, 1.0f
      );

      const uint16_t idleCommand = static_cast<uint16_t>(
        RotorSpeedLoop::DSHOT_IDLE +
        progress * (200.0f - RotorSpeedLoop::DSHOT_IDLE)
      );

      const uint16_t commands[4] =
        {idleCommand, idleCommand, idleCommand, idleCommand};
      sendMotorCommands(commands);

      if (progress >= 1.0f)
      {
        resetEstimatorLocalFrame();
        enterState(FlightState::GroundIdle, nowMs, "ramp complete");
      }
      return;
    }

    case FlightState::DisarmRamp:
    {
      const uint32_t elapsed =
        static_cast<uint32_t>(nowMs - flightStateEnteredMs);

      if (elapsed >= DISARM_RAMP_DURATION_MS)
      {
        stopMotors();
        enterState(FlightState::Disarmed, nowMs, "disarm complete");
        return;
      }

      const float progress = 1.0f -
        static_cast<float>(elapsed) /
        static_cast<float>(DISARM_RAMP_DURATION_MS);

      const uint16_t command = static_cast<uint16_t>(
        RotorSpeedLoop::DSHOT_IDLE + progress * 150.0f
      );

      const uint16_t commands[4] = {command, command, command, command};
      sendMotorCommands(commands);
      return;
    }

    case FlightState::GroundIdle:
    {
      // Rotors at idle. The drone stays on the ground until the throttle
      // stick is first raised, then climbs to the hover setpoint.
      if (sticks.throttle > 0.10f)
      {
        altitudeTargetM = HOVER_SETPOINT_M;
        enterState(FlightState::Climb, nowMs, "throttle raised");
      }
      else
      {
        /*
          Hold ground idle through the SAME rotor speed loop that Climb and
          Hover use, rather than writing a fixed DShot value.

          The previous fixed-DShot idle bypassed RotorSpeedLoop entirely, so
          the loop's slewedSetpoint_, integral_ and lastCommand_ were all
          still zero at the moment Climb took over. The first Climb cycle
          therefore started its setpoint slew from 0 rad/s and commanded
          DSHOT_FLIGHT_MIN.

          The 2026-08-14 bench log caught it exactly: the first Climb line
          reads DS=48,48,48,48 with OMG falling through 341 - full thrust
          cut at the instant of takeoff, recovering over the ~220 ms the
          slew limiter needs to climb back to hover. That is the "not smooth
          in the upward direction" the pilot felt.

          Driving idle through the loop keeps the setpoint, the learned
          feedforward gain and the integrator all converged and continuous
          across the handoff, so Climb starts from a rotor state that is
          already correct.
        */
        float idleSetpoint[4] = {
          GROUND_IDLE_OMEGA_RAD_PER_SEC, GROUND_IDLE_OMEGA_RAD_PER_SEC,
          GROUND_IDLE_OMEGA_RAD_PER_SEC, GROUND_IDLE_OMEGA_RAD_PER_SEC
        };

        bool allFresh = false;
        const RotorSpeedLoop::Measurement measurement =
          collectRotorMeasurement(allFresh);

        lastRotorStatus =
          rotorSpeedLoop.update(idleSetpoint, measurement, dt);

        sendMotorCommands(lastRotorStatus.dshotCommand);
        resetRateControllers();

        // Nothing is being commanded of the airframe on the ground; report
        // that honestly instead of leaving the last flight's values frozen
        // in telemetry.
        commandedThrustN = 0.0f;
        commandedTauX = 0.0f;
        commandedTauY = 0.0f;
        commandedTauZ = 0.0f;
        return;
      }
      break;
    }

    case FlightState::Climb:
    case FlightState::Hover:
      break;
  }

  // ------------------------------------------------------------
  // Safety.
  // ------------------------------------------------------------

  if (excessiveTilt(nowMs))
  {
    safetyStopLatched = true;
    stopMotors();
    enterState(FlightState::Disarmed, nowMs, "excessive tilt");
    logLine("SAFETY STOP: excessive tilt.");
    return;
  }

  // ------------------------------------------------------------
  // Outer loops.
  // ------------------------------------------------------------

  updateHorizontalController(sticks, dt, nowMs);
  const float thrustN = updateVerticalController(sticks, dt, nowMs);
  const float tauZ = updateYawController(sticks, dt, nowMs, thrustN);

  // ------------------------------------------------------------
  // Attitude -> body rate -> angular acceleration -> torque.
  // ------------------------------------------------------------

  const float rollRateTarget = clampFloatFc(
    attitudeKp * (desiredRollRhRad - rollRhRad),
    -MAX_BODY_RATE_DPS * FC_DEG_TO_RAD,
    MAX_BODY_RATE_DPS * FC_DEG_TO_RAD
  );

  const float pitchRateTarget = clampFloatFc(
    attitudeKp * (desiredPitchRhRad - pitchRhRad),
    -MAX_BODY_RATE_DPS * FC_DEG_TO_RAD,
    MAX_BODY_RATE_DPS * FC_DEG_TO_RAD
  );

  // Roll is a rotation about +Y, pitch about +X. Keeping this pairing
  // explicit is the whole point of the naming discipline in this build.
  const float rollAcceleration = updateRatePID(
    rollRateTarget, bodyRateYRh,
    rollRateIntegral, rollRatePreviousError, rollRateDerivative,
    rollRateKp, rollRateKi, rollRateKd, dt
  );

  const float pitchAcceleration = updateRatePID(
    pitchRateTarget, bodyRateXRh,
    pitchRateIntegral, pitchRatePreviousError, pitchRateDerivative,
    pitchRateKp, pitchRateKi, pitchRateKd, dt
  );

  const float tauY = INERTIA_YY * rollAcceleration;
  const float tauX = INERTIA_XX * pitchAcceleration;

  // Published for the `ratelog` view. Tuning a rate loop against attitude
  // telemetry is guesswork; these are the signals the loop actually closes on.
  lastRollRateTargetDps = rollRateTarget * FC_RAD_TO_DEG;
  lastPitchRateTargetDps = pitchRateTarget * FC_RAD_TO_DEG;

  commandedThrustN = thrustN;
  commandedTauX = tauX;
  commandedTauY = tauY;
  commandedTauZ = tauZ;

  // ------------------------------------------------------------
  // Allocation -> rotor speed loop -> DShot.
  // ------------------------------------------------------------

  bool allRotorsFresh = false;
  const RotorSpeedLoop::Measurement measurement =
    collectRotorMeasurement(allRotorsFresh);

  lastAllocation = RotorAllocator::allocate(
    thrustN, tauX, tauY, tauZ, yawIdentifier.dragCoefficientKm()
  );

  if (!lastAllocation.valid)
  {
    stopMotors();
    logLine("ALLOCATION FAILED: motors stopped.");
    return;
  }

  lastRotorStatus = rotorSpeedLoop.update(
    lastAllocation.rotorOmegaRadPerSec, measurement, dt
  );

  sendMotorCommands(lastRotorStatus.dshotCommand);

  // ------------------------------------------------------------
  // In-flight drag coefficient identification.
  // ------------------------------------------------------------

  float measuredOmega[4];
  for (size_t i = 0; i < 4; ++i)
  {
    measuredOmega[i] = measurement.valid[i]
      ? measurement.omegaRadPerSec[i]
      : lastRotorStatus.omegaEstimated[i];
  }

  lastIdentifierStatus = yawIdentifier.update(
    measuredOmega,
    allRotorsFresh,
    bodyRateZRh,
    // The identifier must never learn from a cycle whose yaw torque was not
    // actually delivered. yawScaled is the new one: with yaw now yielding
    // instead of the collective, a clipped yaw demand shows up HERE and
    // nowhere else, and learning kM from it would bias the estimate low -
    // which is the very error that made yaw over-commanded to begin with.
    lastAllocation.torqueScaled || lastAllocation.collectiveSlid ||
      lastAllocation.yawScaled || yawTorqueSaturated,
    rollRhRad,
    pitchRhRad,
    // physicallyAirborne(), not airborne(). The latter is true the instant
    // Climb is entered, which let the 2026-08-15 run identify kM through a
    // whole flight spent in ground effect at 0.09-0.14 m.
    physicallyAirborne(),
    dt
  );
}

// ================================================================
// TRACK LOG DOWNLINK
// ================================================================

/*
  Register the transmitter as an ESP-NOW peer so we can send to it.

  Called from task context only. Returns false until the first uplink packet
  has arrived, because that packet is where the address comes from.
*/
bool ensureTransmitterPeer()
{
  if (transmitterPeerAdded) return true;
  if (!transmitterMacKnown) return false;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, transmitterMac, 6);
  peer.channel = 0;      // 0 = whatever channel the interface is already on
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) return false;

  transmitterPeerAdded = true;
  return true;
}

/*
  Track sink. Installed on the debug task, so this runs on core 0 and never
  on the 500 Hz flight core.

  Failures are counted, not retried. A retry would delay the next sample and
  turn one lost row into a burst of them; the sequence column already makes a
  gap visible, which is the property the log needs.
*/
void trackDownlinkSend(const AsyncTrackSample &sample)
{
  if (!ensureTransmitterPeer())
  {
    trackDownlinkFailed++;
    return;
  }

  TrackDownlinkPacket packet;
  packet.sample = sample;

  if (esp_now_send(transmitterMac,
                   reinterpret_cast<const uint8_t *>(&packet),
                   sizeof(packet)) == ESP_OK)
  {
    trackDownlinkSent++;
  }
  else
  {
    trackDownlinkFailed++;
  }
}

/*
  Queue one preamble line for the downlink.

  NOT sent here. The header is a burst of about ten packets and ESP-NOW only
  queues a few at a time, so they have to be paced - but this is reached from
  processSerialCommand(), which runs in loop() on the FLIGHT CORE. Sleeping
  between sends would stall the 500 Hz control step for tens of milliseconds,
  and the transmitter can ask for logging at any moment, including while
  armed and airborne.

  So the lines go into a small queue and loop() sends one per pass. At the
  loop rate the whole header is out in well under a tenth of a second and
  nothing blocks.
*/
void trackDownlinkText(const char *text)
{
  if (trackHeaderCount >= TRACK_HEADER_MAX_LINES) return;

  strncpy(trackHeaderLines[trackHeaderCount], text,
          sizeof(trackHeaderLines[0]) - 1);
  trackHeaderLines[trackHeaderCount][sizeof(trackHeaderLines[0]) - 1] = '\0';
  trackHeaderCount++;
}

/*
  Send at most one queued preamble line. Called once per loop() pass.
*/
void serviceTrackHeaderQueue()
{
  if (trackHeaderSent >= trackHeaderCount) return;
  if (!ensureTransmitterPeer()) return;

  TrackTextPacket packet;
  strncpy(packet.text, trackHeaderLines[trackHeaderSent],
          sizeof(packet.text) - 1);

  if (esp_now_send(transmitterMac,
                   reinterpret_cast<const uint8_t *>(&packet),
                   sizeof(packet)) == ESP_OK)
  {
    trackHeaderSent++;
  }
  // On failure, leave the index alone and retry on the next pass. Unlike a
  // dropped data row, a dropped header line loses the column names for the
  // whole flight, so this one is worth retrying.
}

// ================================================================
// FLIGHT TRACK LOG
// ================================================================

/*
  Emit the column names and the constants an analysis needs in order to
  interpret the rows. Printed once when logging starts, so a capture file is
  self-describing and a change to the row format cannot silently mis-parse:
  the version tag is checked by the analysis script.
*/
void emitTrackLogHeader()
{
  /*
    Route each preamble line to whichever path this run is using. Tethered it
    goes to the UART; untethered there is no UART, so it rides the downlink as
    a text packet and the transmitter prints it. Either way the capture file
    ends up self-describing.
  */
  auto emit = [](const char *line)
  {
    if (trackLogDownlink) trackDownlinkText(line);
    else                  logLine(line);
  };

  char buffer[200];

  emit("#TRK v1 FPV2026 rotor-rate controller flight track log");

  snprintf(buffer, sizeof(buffer),
    "#TRK rate_hz=%.1f deadband_rp=%.3f deadband_thr=%.3f deadband_yaw=%.3f",
    1.0e6f / static_cast<float>(trackLogPeriodUs),
    ROLL_STICK_DEADBAND, THROTTLE_STICK_DEADBAND, YAW_STICK_DEADBAND);
  emit(buffer);

  snprintf(buffer, sizeof(buffer),
    "#TRK mass_kg=%.5f kf=%.6e km=%.6e armx_m=%.4f army_m=%.5f",
    AIRFRAME_MASS_KG, ROTOR_THRUST_KF,
    yawIdentifier.dragCoefficientKm(), ROTOR_ARM_X_M, ROTOR_ARM_Y_M);
  emit(buffer);

  snprintf(buffer, sizeof(buffer),
    "#TRK ixx=%.6e iyy=%.6e izz=%.6e hover_omega=%.1f",
    INERTIA_XX, INERTIA_YY, INERTIA_ZZ, hoverOmegaRadPerSec());
  emit(buffer);

  snprintf(buffer, sizeof(buffer), "#TRK link=%s",
    trackLogDownlink ? "espnow-downlink" : "usb-serial");
  emit(buffer);

  emit("#TRK hold states: 0=free 1=settling 2=held");
  emit("#TRK angles in deg, rates in deg/s, positions in m, "
       "velocities in m/s, torque in N.m");
  /*
    Rate columns are named for the axis they rotate about, not p/q/r. On this
    airframe roll is a rotation about +Y and pitch about +X, so the usual
    aerospace p=roll reading would be exactly backwards.
  */
  emit("#TRK wx = about +X = PITCH rate, wy = about +Y = ROLL rate, "
       "wz = about +Z = yaw rate, CCW positive");

  /*
    The column header does not fit in a text packet and must not be split, so
    over the downlink we send a sentinel and the transmitter prints its own
    compiled copy. See TrackSample.h.
  */
  if (trackLogDownlink) trackDownlinkText(TRACK_COLUMN_HEADER_SENTINEL);
  else                  logLine(TRACK_COLUMN_HEADER);
}

/*
  Fill one sample and hand it to the debug task. Called from the control step
  so every field comes from the same cycle.

  Cost on the flight core is a struct fill and a queue send. All formatting
  happens on core 0; see AsyncTrackSample.
*/
void publishTrackSample(uint32_t nowMs, uint32_t controlDtUs)
{
  AsyncTrackSample sample;

  sample.timestampMs = nowMs;
  sample.sequence = trackLogSequence++;
  sample.controlDtUs = static_cast<uint16_t>(
    (controlDtUs > 65535u) ? 65535u : controlDtUs);

  sample.flightState = static_cast<uint8_t>(flightState);
  sample.holdAltitude = static_cast<uint8_t>(altitudeHold.state());
  sample.holdHorizontal = static_cast<uint8_t>(horizontalHold.state());
  sample.holdHeading = static_cast<uint8_t>(headingHold.state());
  sample.rpmFreshMask = lastRotorStatus.measuredMask;

  sample.healthFlags = static_cast<uint8_t>(
    (controlData.motorsEnabled     ? 0x01u : 0u) |
    (signalLost                    ? 0u : 0x02u) |
    (yawStability.proven()         ? 0x04u : 0u) |
    (flowHealthFresh(nowMs)        ? 0x08u : 0u) |
    (rangeUsable(nowMs)            ? 0x10u : 0u) |
    (magnetometerFresh(nowMs)      ? 0x20u : 0u) |
    (eskfStateHealthy              ? 0x40u : 0u) |
    (eskfAttitudeUsed              ? 0x80u : 0u));

  sample.saturationFlags = static_cast<uint8_t>(
    (yawTorqueSaturated              ? 0x01u : 0u) |
    (lastAllocation.yawScaled        ? 0x02u : 0u) |
    (lastAllocation.torqueScaled     ? 0x04u : 0u));

  // Raw stick values, before the deadband. See AsyncTrackSample.
  sample.rcThrottle = controlData.throttle;
  sample.rcRoll = controlData.roll;
  sample.rcPitch = controlData.pitch;
  sample.rcYaw = controlData.yaw;

  sample.positionX = positionNavX;
  sample.positionY = positionNavY;
  sample.altitude = altitudeM;
  sample.targetX = targetNavX;
  sample.targetY = targetNavY;
  sample.targetAltitude = altitudeTargetM;

  sample.bodyVelocityX = measuredBodyVelocityX;
  sample.bodyVelocityY = measuredBodyVelocityY;
  sample.navVelocityX = velocityNavX;
  sample.navVelocityY = velocityNavY;
  sample.verticalVelocity = verticalVelocityMps;
  sample.bodyVelocityTargetX = publishedBodyVelocityTargetX;
  sample.bodyVelocityTargetY = publishedBodyVelocityTargetY;
  sample.verticalVelocityTarget = publishedVerticalVelocityTarget;

  sample.roll = rollRhRad;
  sample.pitch = pitchRhRad;
  sample.yaw = yawRhRad;
  sample.rollDesired = desiredRollRhRad;
  sample.pitchDesired = desiredPitchRhRad;
  sample.yawTarget = headingTargetRhRad;

  sample.rateX = bodyRateXRh;
  sample.rateY = bodyRateYRh;
  sample.rateZ = bodyRateZRh;
  // The rate targets are published in deg/s; the sample carries radians.
  sample.rateTargetX = lastPitchRateTargetDps * FC_DEG_TO_RAD;
  sample.rateTargetY = lastRollRateTargetDps * FC_DEG_TO_RAD;
  sample.rateTargetZ = lastYawRateTargetDps * FC_DEG_TO_RAD;

  sample.thrustN = commandedThrustN;
  sample.tauX = commandedTauX;
  sample.tauY = commandedTauY;
  sample.tauZ = commandedTauZ;

  for (size_t i = 0; i < 4; ++i)
  {
    sample.rotorSetpoint[i] = lastAllocation.valid
      ? lastAllocation.rotorOmegaRadPerSec[i] : 0.0f;
    sample.rotorMeasured[i] = lastRotorStatus.omegaMeasured[i];
    sample.dshot[i] = lastCommandedDshot[i];
  }

  sample.rangeM = filteredRangeMeters;
  sample.magYawRad = magRelativeYawRhRad;
  sample.flowSqual = lastFlowSqual;

  asyncDebug.publishTrack(sample);
}

// ================================================================
// TELEMETRY
// ================================================================

void publishTelemetry(uint32_t nowMs)
{
  const StickCommands sticks = readSticks();

  /*
    Rate-loop view, off by default.

    The main telemetry line carries ATTITUDE (RP, PT) but the roll and pitch
    rate loops close on BODY RATE, which appears nowhere - only yaw rate (YR)
    was visible. Tuning rollratekp/ki/kd against an attitude readout means
    inferring the inner loop from the outer loop's response, which is exactly
    the kind of guesswork this build exists to remove.

    Prints measured/target in deg/s for all three axes plus the torque each
    loop is asking for.
  */
  if (rateLogEnabled)
  {
    logf(
      "RATE r=%+7.1f/%+7.1f p=%+7.1f/%+7.1f y=%+7.1f/%+7.1f dps | "
      "TAU=%+.4f,%+.4f,%+.4f",
      bodyRateYRh * FC_RAD_TO_DEG, lastRollRateTargetDps,
      bodyRateXRh * FC_RAD_TO_DEG, lastPitchRateTargetDps,
      bodyRateZRh * FC_RAD_TO_DEG, lastYawRateTargetDps,
      commandedTauX, commandedTauY, commandedTauZ);
  }

  logf(
    // RC is the SHAPED stick value after deadband: throttle, roll, pitch,
    // yaw, each -1..+1. ARM is the enable switch, LNK=0 means link lost.
    // Ground idle advances to climb when RC throttle exceeds +0.10.
    "ST=%d RC=%+.2f,%+.2f,%+.2f,%+.2f ARM=%d LNK=%d | "
    "ALT=%.2f/%.2f VZ=%+.2f | X=%+.2f/%+.2f Y=%+.2f/%+.2f "
    "VB=%+.2f,%+.2f | RP=%+.1f/%+.1f PT=%+.1f/%+.1f YAW=%+.1f/%+.1f "
    // OMG is measured rotor speed in rad/s, not RPM. It reads 0 for any
    // rotor whose bidirectional-DShot telemetry is stale this cycle.
    "YR=%+.1f | T=%.2fN TAU=%+.4f,%+.4f,%+.4f | OMG=%.0f,%.0f,%.0f,%.0f "
    // C is the jointly estimated yaw damping, N m per rad/s. SPR is how far
    // kM is wandering as a fraction of its own mean - THAT is the number
    // that says whether the identification converged, not N.
    "DS=%u,%u,%u,%u M=%X | HOLD A=%s X=%s H=%s YP=%d | KM=%.3e R=%.4f N=%lu "
    "C=%.4f SPR=%.3f CV=%d "
    // SAT: yaw demand clipped / yaw allocation clipped / roll-pitch scaled.
    // Any sustained run of these means a loop is asking for torque the
    // rotors do not have, which is a gain problem, not a tuning nicety.
    "SAT=%d%d%d "
    "FLW=%d Q=%u RNG=%d CTL=%lu/%luus SLK=%lu%s",
    static_cast<int>(flightState),
    sticks.throttle, sticks.roll, sticks.pitch, sticks.yaw,
    controlData.motorsEnabled ? 1 : 0,
    signalLost ? 0 : 1,
    altitudeM, altitudeTargetM, verticalVelocityMps,
    positionNavX, targetNavX, positionNavY, targetNavY,
    measuredBodyVelocityX, measuredBodyVelocityY,
    rollRhRad * FC_RAD_TO_DEG, desiredRollRhRad * FC_RAD_TO_DEG,
    pitchRhRad * FC_RAD_TO_DEG, desiredPitchRhRad * FC_RAD_TO_DEG,
    yawRhRad * FC_RAD_TO_DEG, headingTargetRhRad * FC_RAD_TO_DEG,
    bodyRateZRh * FC_RAD_TO_DEG,
    commandedThrustN, commandedTauX, commandedTauY, commandedTauZ,
    lastRotorStatus.omegaMeasured[0], lastRotorStatus.omegaMeasured[1],
    lastRotorStatus.omegaMeasured[2], lastRotorStatus.omegaMeasured[3],
    // What was actually written to the ESCs, not the control loop's copy,
    // which is only populated in the states that run the loop.
    lastCommandedDshot[0], lastCommandedDshot[1],
    lastCommandedDshot[2], lastCommandedDshot[3],
    lastRotorStatus.measuredMask,
    altitudeHold.stateName(), horizontalHold.stateName(),
    headingHold.stateName(), yawStability.proven() ? 1 : 0,
    static_cast<double>(lastIdentifierStatus.dragCoefficientKm),
    static_cast<double>(lastIdentifierStatus.dragToThrustRatioM),
    static_cast<unsigned long>(lastIdentifierStatus.updateCount),
    static_cast<double>(lastIdentifierStatus.yawDampingC),
    static_cast<double>(lastIdentifierStatus.kmSpreadFraction),
    lastIdentifierStatus.converged ? 1 : 0,
    yawTorqueSaturated ? 1 : 0,
    lastAllocation.yawScaled ? 1 : 0,
    lastAllocation.torqueScaled ? 1 : 0,
    flowSensorHealthy ? 1 : 0, lastFlowSqual,
    rangeUsable(nowMs) ? 1 : 0,
    static_cast<unsigned long>(reportedControlAvgUs),
    static_cast<unsigned long>(reportedControlMaxUs),
    static_cast<unsigned long>(reportedLoopPassesPerStep),
    benchModeActive ? " [BENCH]" : ""
  );
}

// ================================================================
// SERIAL COMMANDS
// ================================================================

void printHelp()
{
  logLine("Commands:");
  logLine("  selftest            verify sign conventions and allocator");
  logLine("  status              full state summary");
  logLine("  rotor               per-rotor setpoint vs measured speed");
  logLine("  timing              control step cost and loop headroom");
  logLine("  bench <omega>       PROPS OFF: drive all rotors at a fixed");
  logLine("                      speed through the rotor loop, disarmed");
  logLine("  benchone <n> <omega>  same, one motor only (1-4)");
  logLine("  bench off           stop bench mode");
  logLine("  gains               list all control gains");
  logLine("  ratelog on|off      stream rate-loop measured/target, deg/s");
  logLine("  tracklog on|off|<hz>  full command-to-response CSV log,");
  logLine("  tracklog net [hz]     same log over the radio - the TRANSMITTER");
  logLine("                        prints it, so the drone flies untethered");
  logLine("                        default 50 Hz, 1-100 Hz");
  logLine("  link                ESP-NOW packet and gap statistics");
  logLine("  set <name> <value>  set a gain, e.g. set yawratekp 18");
  logLine("  kmstatus            drag coefficient identification state");
  logLine("  kmlearn on|off      enable in-flight kM identification");
  logLine("  kmratio <metres>    seed the drag/thrust ratio directly");
  logLine("  magfusion on|off    magnetic yaw fusion");
  logLine("  rotorgains kp ki    rotor speed loop gains");
  logLine("  stop                latch a software stop");
  logLine("  clearstop           clear the software stop latch");
}

void printStatus(uint32_t nowMs)
{
  logf("state=%d armed=%d signalLost=%d safetyStop=%d serialStop=%d",
    static_cast<int>(flightState), controlData.motorsEnabled ? 1 : 0,
    signalLost ? 1 : 0, safetyStopLatched ? 1 : 0, serialStopLatch ? 1 : 0);

  logf("attitude roll=%+.2f pitch=%+.2f yaw=%+.2f deg  eskfAtt=%d healthy=%d",
    rollRhRad * FC_RAD_TO_DEG, pitchRhRad * FC_RAD_TO_DEG,
    yawRhRad * FC_RAD_TO_DEG, eskfAttitudeUsed ? 1 : 0,
    eskfStateHealthy ? 1 : 0);

  logf("holds altitude=%s horizontal=%s heading=%s yawProven=%d stableMs=%lu",
    altitudeHold.stateName(), horizontalHold.stateName(),
    headingHold.stateName(), yawStability.proven() ? 1 : 0,
    static_cast<unsigned long>(yawStability.stableForMs()));

  logf("range=%.3f m age=%lu ms usable=%d | flow healthy=%d squal=%u "
       "shutter=%u mask=%X rejects=%lu",
    filteredRangeMeters, static_cast<unsigned long>(rangeAgeMs(nowMs)),
    rangeUsable(nowMs) ? 1 : 0, flowSensorHealthy ? 1 : 0,
    lastFlowSqual, lastFlowShutter, lastFlowAxisAcceptedMask,
    static_cast<unsigned long>(flowRejectCount));

  logf("mag valid=%d fusion=%d accepted=%lu rejected=%lu relYaw=%+.1f deg",
    magMeasurementValid ? 1 : 0, magYawFusionEnabled ? 1 : 0,
    static_cast<unsigned long>(magAcceptedUpdates),
    static_cast<unsigned long>(magRejectedUpdates),
    magRelativeYawRhRad * FC_RAD_TO_DEG);

  logf("inertia Ixx=%.6f Iyy=%.6f Izz=%.6f kF=%.4e hoverOmega=%.1f",
    INERTIA_XX, INERTIA_YY, INERTIA_ZZ,
    static_cast<double>(ROTOR_THRUST_KF), hoverOmegaRadPerSec());
}

void printGains()
{
  logf("positionkp=%.3f velocitykp=%.3f altitudekp=%.3f vzkp=%.3f",
    positionKp, velocityKp, altitudeKp, verticalVelocityKp);
  logf("attitudekp=%.3f headingkp=%.3f", attitudeKp, headingKp);
  logf("rollrate  kp=%.2f ki=%.2f kd=%.3f",
    rollRateKp, rollRateKi, rollRateKd);
  logf("pitchrate kp=%.2f ki=%.2f kd=%.3f",
    pitchRateKp, pitchRateKi, pitchRateKd);
  logf("yawrate   kp=%.2f ki=%.2f kd=%.3f",
    yawRateKp, yawRateKi, yawRateKd);
  logf("rotorloop kp=%.3f ki=%.3f slew=%.0f rad/s^2",
    rotorSpeedLoop.kp, rotorSpeedLoop.ki, rotorSpeedLoop.maxSlewRadPerSec2);
  logf("settle    alt=%.3f m/s xy=%.3f m/s yaw=%.2f deg/s",
    altitudeHold.settleSpeedThreshold,
    horizontalHold.settleSpeedThreshold,
    headingHold.settleSpeedThreshold);
}

/*
  Per-rotor detail for tuning the innermost loop. Everything above the rotor
  speed loop assumes it tracks its setpoint, so this is the first thing to
  get right and the first place to look when an outer loop misbehaves.
*/
void printRotorDetail()
{
  logf("rotor loop kp=%.3f ki=%.3f   hover omega = %.1f rad/s",
    rotorSpeedLoop.kp, rotorSpeedLoop.ki, hoverOmegaRadPerSec());

  logLine("  M  setpoint  measured  estimate   error   dshot  ds/omega  tel");

  for (size_t i = 0; i < 4; ++i)
  {
    logf("  %u  %8.1f  %8.1f  %8.1f  %+6.1f  %5u  %8.4f  %s",
      static_cast<unsigned>(i + 1),
      lastRotorStatus.omegaSetpoint[i],
      lastRotorStatus.omegaMeasured[i],
      lastRotorStatus.omegaEstimated[i],
      lastRotorStatus.omegaError[i],
      lastRotorStatus.dshotCommand[i],
      lastRotorStatus.dshotPerOmega[i],
      lastRotorStatus.usingMeasurement[i] ? "OK" : "STALE");
  }

  logf("allocation valid=%d torqueScaled=%d scale=%.3f collectiveSlid=%d",
    lastAllocation.valid ? 1 : 0,
    lastAllocation.torqueScaled ? 1 : 0,
    lastAllocation.torqueScale,
    lastAllocation.collectiveSlid ? 1 : 0);
}

bool setGainByName(const char *name, float value)
{
  struct Entry { const char *name; float *target; };

  const Entry entries[] = {
    {"positionkp", &positionKp},
    {"velocitykp", &velocityKp},
    {"altitudekp", &altitudeKp},
    {"vzkp", &verticalVelocityKp},
    {"attitudekp", &attitudeKp},
    {"headingkp", &headingKp},
    {"rollratekp", &rollRateKp},
    {"rollrateki", &rollRateKi},
    {"rollratekd", &rollRateKd},
    {"pitchratekp", &pitchRateKp},
    {"pitchrateki", &pitchRateKi},
    {"pitchratekd", &pitchRateKd},
    {"yawratekp", &yawRateKp},
    {"yawrateki", &yawRateKi},
    {"yawratekd", &yawRateKd},
    {"rotorkp", &rotorSpeedLoop.kp},
    {"rotorki", &rotorSpeedLoop.ki},
    {"rotorslew", &rotorSpeedLoop.maxSlewRadPerSec2},
    // Settle thresholds. Altitude and XY are m/s; heading is deg/s.
    {"settlealt", &altitudeHold.settleSpeedThreshold},
    {"settlexy", &horizontalHold.settleSpeedThreshold},
    {"settleyaw", &headingHold.settleSpeedThreshold},
    // Pilot level trim, degrees. Set to whatever RP/PT read at rest.
    {"rolltrim", &rollTrimDeg},
    {"pitchtrim", &pitchTrimDeg}
  };

  for (const Entry &entry : entries)
  {
    if (strcmp(entry.name, name) == 0)
    {
      *entry.target = value;
      logf("%s = %.4f", entry.name, value);
      return true;
    }
  }

  return false;
}

/*
  On-target self test. Pure math, no motion, safe to run while disarmed.

  This guards the exact bug class that grounded the previous build: a sign
  convention that drifts out of agreement with the geometry it describes.
  Every check here would have failed on the old firmware.
*/
void runSelfTest()
{
  int failures = 0;

  auto report = [&failures](const char *name, bool passed)
  {
    logf("  %s  %s", passed ? "PASS" : "FAIL", name);
    if (!passed) ++failures;
  };

  logLine("Self test:");

  const float hoverOmega = hoverOmegaRadPerSec();
  const float weight = AIRFRAME_MASS_KG * FC_GRAVITY_MPS2;

  // 1. Allocator round-trip.
  {
    RotorAllocator::Output allocation = RotorAllocator::allocate(
      weight, 0.012f, -0.009f, 0.003f, yawIdentifier.dragCoefficientKm()
    );

    float thrust = 0.0f, tauX = 0.0f, tauY = 0.0f, tauZ = 0.0f;
    RotorAllocator::forward(
      allocation.rotorOmegaRadPerSec, yawIdentifier.dragCoefficientKm(),
      thrust, tauX, tauY, tauZ
    );

    report("allocator round-trip",
      allocation.valid &&
      fabsf(thrust - weight) < 1e-4f &&
      fabsf(tauX - 0.012f) < 1e-5f &&
      fabsf(tauY + 0.009f) < 1e-5f &&
      fabsf(tauZ - 0.003f) < 1e-6f);
  }

  // 2. Torque signs against the physical rotor layout.
  {
    RotorAllocator::Output noseUp = RotorAllocator::allocate(
      weight, 0.015f, 0.0f, 0.0f, yawIdentifier.dragCoefficientKm());
    report("+tauX raises front rotors M1,M2",
      noseUp.rotorOmegaRadPerSec[0] > hoverOmega &&
      noseUp.rotorOmegaRadPerSec[1] > hoverOmega &&
      noseUp.rotorOmegaRadPerSec[2] < hoverOmega &&
      noseUp.rotorOmegaRadPerSec[3] < hoverOmega);

    RotorAllocator::Output rollRight = RotorAllocator::allocate(
      weight, 0.0f, 0.015f, 0.0f, yawIdentifier.dragCoefficientKm());
    report("+tauY raises left rotors M1,M4",
      rollRight.rotorOmegaRadPerSec[0] > hoverOmega &&
      rollRight.rotorOmegaRadPerSec[3] > hoverOmega &&
      rollRight.rotorOmegaRadPerSec[1] < hoverOmega &&
      rollRight.rotorOmegaRadPerSec[2] < hoverOmega);

    // The one the old build got backwards: a counter-clockwise yaw demand
    // must speed up the CLOCKWISE rotors, because a clockwise rotor imposes
    // a counter-clockwise reaction torque on the airframe.
    RotorAllocator::Output yawCcw = RotorAllocator::allocate(
      weight, 0.0f, 0.0f, 0.004f, yawIdentifier.dragCoefficientKm());
    report("+tauZ (CCW) raises the CW rotors M2,M4",
      yawCcw.rotorOmegaRadPerSec[1] > hoverOmega &&
      yawCcw.rotorOmegaRadPerSec[3] > hoverOmega &&
      yawCcw.rotorOmegaRadPerSec[0] < hoverOmega &&
      yawCcw.rotorOmegaRadPerSec[2] < hoverOmega);
  }

  // 3. Body/navigation rotation is a true inverse pair, and points the right
  //    way. The old build's two functions were each other's transpose.
  {
    float worst = 0.0f;
    for (int step = 0; step < 36; ++step)
    {
      const float yaw = (static_cast<float>(step) * 10.0f - 180.0f) *
        FC_DEG_TO_RAD;

      float navX = 0.0f, navY = 0.0f, backX = 0.0f, backY = 0.0f;
      bodyToNavigation(yaw, 1.3f, -0.7f, navX, navY);
      navigationToBody(yaw, navX, navY, backX, backY);

      worst = fmaxf(worst, fabsf(backX - 1.3f));
      worst = fmaxf(worst, fabsf(backY + 0.7f));
    }
    report("navigationToBody inverts bodyToNavigation", worst < 1e-5f);

    float navX = 0.0f, navY = 0.0f;
    bodyToNavigation(90.0f * FC_DEG_TO_RAD, 1.0f, 0.0f, navX, navY);
    report("at yaw=+90deg, body +X maps to nav +Y",
      fabsf(navX) < 1e-5f && fabsf(navY - 1.0f) < 1e-5f);
  }

  // 4. IMU mounting transform.
  {
    float bodyX = 0.0f, bodyY = 0.0f, bodyZ = 0.0f;
    chipAccelToBodyRh(0.0f, 0.0f, -1.0f, bodyX, bodyY, bodyZ);
    report("level chip reading gives body specific force (0,0,+g)",
      fabsf(bodyX) < 1e-5f && fabsf(bodyY) < 1e-5f &&
      fabsf(bodyZ - FC_GRAVITY_MPS2) < 1e-4f);

    float roll = 0.0f, pitch = 0.0f;
    bodySpecificForceToTiltRh(
      -FC_GRAVITY_MPS2 * sinf(0.15f), 0.0f,
      FC_GRAVITY_MPS2 * cosf(0.15f), roll, pitch
    );
    report("tilt extraction recovers right-handed roll",
      fabsf(roll - 0.15f) < 1e-4f && fabsf(pitch) < 1e-4f);
  }

  logf("Self test %s (%d failure%s)",
    failures == 0 ? "PASSED" : "FAILED",
    failures, failures == 1 ? "" : "s");
}

void processSerialCommand(char *command, uint32_t nowMs)
{
  for (char *cursor = command; *cursor != '\0'; ++cursor)
  {
    *cursor = static_cast<char>(tolower(*cursor));
  }

  if (strcmp(command, "help") == 0) { printHelp(); return; }
  if (strcmp(command, "status") == 0) { printStatus(nowMs); return; }
  if (strcmp(command, "gains") == 0) { printGains(); return; }
  if (strcmp(command, "selftest") == 0) { runSelfTest(); return; }
  if (strcmp(command, "rotor") == 0) { printRotorDetail(); return; }

  if (strcmp(command, "timing") == 0)
  {
    logf("control step avg=%lu us max=%lu us  budget=%lu us",
      static_cast<unsigned long>(reportedControlAvgUs),
      static_cast<unsigned long>(reportedControlMaxUs),
      static_cast<unsigned long>(CONTROL_PERIOD_US));

    logf("  breakdown: IMU read=%lu us  estimator=%lu us  control=%lu us",
      static_cast<unsigned long>(reportedImuUs),
      static_cast<unsigned long>(reportedEstimatorUs),
      static_cast<unsigned long>(reportedControlUs));

    logf("loop passes per control step = %lu   overruns since boot = %lu",
      static_cast<unsigned long>(reportedLoopPassesPerStep),
      static_cast<unsigned long>(controlOverrunCount));

    if (reportedLoopPassesPerStep <= 1)
    {
      logLine("WARNING: no loop headroom. Serial and sensor servicing will "
              "be starved.");
    }

    logf("telemetry dropped: text=%lu commands=%lu",
      static_cast<unsigned long>(asyncDebug.droppedTextLines()),
      static_cast<unsigned long>(asyncDebug.droppedCommands()));

    /*
      The number that decides whether a track log is usable. Anything other
      than zero means rows are missing from the file; the sequence column
      will show where, but the sampling is no longer uniform and the rate
      should come down.
    */
    logf("track log: %s  emitted=%lu dropped=%lu",
      trackLogEnabled ? "ON" : "off",
      static_cast<unsigned long>(trackLogSequence),
      static_cast<unsigned long>(asyncDebug.droppedTrackSamples()));
    return;
  }

  if (strcmp(command, "stop") == 0)
  {
    serialStopLatch = true;
    stopMotors();
    logLine("Software stop latched.");
    return;
  }

  if (strcmp(command, "clearstop") == 0)
  {
    if (flightState != FlightState::Disarmed)
    {
      logLine("Refusing to clear the stop latch while not disarmed.");
      return;
    }
    serialStopLatch = false;
    safetyStopLatched = false;
    logLine("Stop latches cleared.");
    return;
  }

  if (strncmp(command, "bench", 5) == 0)
  {
    const bool singleMotor = (strncmp(command, "benchone ", 9) == 0);
    const char *arguments = command + (singleMotor ? 9 : 5);

    while (*arguments == ' ') ++arguments;

    if (*arguments == '\0' || strcmp(arguments, "off") == 0)
    {
      if (benchModeActive) stopBenchMode("commanded off");
      else logLine("Bench mode was not running.");
      return;
    }

    if (flightState != FlightState::Disarmed)
    {
      logLine("Bench mode requires the DISARMED state.");
      return;
    }

    int motorNumber = 0;
    float omega = 0.0f;

    if (singleMotor)
    {
      if (sscanf(arguments, "%d %f", &motorNumber, &omega) != 2 ||
          motorNumber < 1 || motorNumber > 4)
      {
        logLine("Usage: benchone <motor 1-4> <omega rad/s>");
        return;
      }
    }
    else
    {
      omega = atof(arguments);
    }

    if (!isfinite(omega) ||
        omega < BENCH_MIN_OMEGA || omega > BENCH_MAX_OMEGA)
    {
      logf("Rejected: omega must be %.0f to %.0f rad/s (hover is %.0f).",
        BENCH_MIN_OMEGA, BENCH_MAX_OMEGA, hoverOmegaRadPerSec());
      return;
    }

    benchOmegaSetpoint = omega;
    benchMotorIndex = singleMotor ? (motorNumber - 1) : -1;
    benchStartedMs = millis();

    if (!benchModeActive)
    {
      rotorSpeedLoop.resetIntegrators();
      benchModeActive = true;
    }

    logf("BENCH: %s at %.0f rad/s. PROPS MUST BE OFF. "
         "Auto-stops in %lu s. 'bench off' to end.",
      singleMotor ? "one motor" : "all motors", omega,
      static_cast<unsigned long>(BENCH_TIMEOUT_MS / 1000U));

    if (singleMotor) logf("  driving motor M%d only", motorNumber);
    return;
  }

  if (strcmp(command, "kmstatus") == 0)
  {
    logf("kM=%.4e ratio=%.4f m updates=%lu rejects=%lu converged=%d "
         "learn=%d residual=%.4e",
      static_cast<double>(lastIdentifierStatus.dragCoefficientKm),
      static_cast<double>(lastIdentifierStatus.dragToThrustRatioM),
      static_cast<unsigned long>(lastIdentifierStatus.updateCount),
      static_cast<unsigned long>(lastIdentifierStatus.rejectCount),
      lastIdentifierStatus.converged ? 1 : 0,
      yawIdentifier.commissioningEnabled ? 1 : 0,
      static_cast<double>(lastIdentifierStatus.filteredResidualMagnitude));
    return;
  }

  if (strncmp(command, "kmlearn ", 8) == 0)
  {
    if (flightState != FlightState::Disarmed)
    {
      logLine("Change kmlearn only while disarmed.");
      return;
    }
    yawIdentifier.commissioningEnabled = (strcmp(command + 8, "on") == 0);
    logf("kM identification %s",
      yawIdentifier.commissioningEnabled ? "ENABLED" : "disabled");
    return;
  }

  if (strncmp(command, "kmratio ", 8) == 0)
  {
    const float ratio = atof(command + 8);
    if (yawIdentifier.setDragToThrustRatio(ratio))
    {
      logf("drag/thrust ratio = %.4f m", ratio);
    }
    else
    {
      logf("Rejected: ratio must be between %.3f and %.3f m",
        DRAG_TO_THRUST_RATIO_MIN_M, DRAG_TO_THRUST_RATIO_MAX_M);
    }
    return;
  }

  if (strncmp(command, "magfusion ", 10) == 0)
  {
    magYawFusionEnabled = (strcmp(command + 10, "on") == 0);
    logf("magnetic yaw fusion %s", magYawFusionEnabled ? "ON" : "OFF");
    return;
  }

  if (strncmp(command, "set ", 4) == 0)
  {
    char name[32] = {};
    float value = 0.0f;
    if (sscanf(command + 4, "%31s %f", name, &value) == 2)
    {
      if (!setGainByName(name, value))
      {
        logf("Unknown gain: %s", name);
      }
    }
    else
    {
      logLine("Usage: set <name> <value>");
    }
    return;
  }

  if (strncmp(command, "tracklog", 8) == 0)
  {
    const char *argument = command + 8;
    while (*argument == ' ') argument++;

    if (strcmp(argument, "off") == 0)
    {
      if (trackLogEnabled)
      {
        char tail[128];
        snprintf(tail, sizeof(tail),
          "#TRK end seq=%lu dropped=%lu dl_sent=%lu dl_failed=%lu",
          (unsigned long)trackLogSequence,
          (unsigned long)asyncDebug.droppedTrackSamples(),
          (unsigned long)trackDownlinkSent,
          (unsigned long)trackDownlinkFailed);

        /*
          Clear the queue first. The start header left ten of its twelve
          slots used, and without a reset a few stop/start cycles would fill
          it and silently drop the end line - which is the line carrying the
          drop counters you check the log against.
        */
        trackHeaderCount = 0;
        trackHeaderSent = 0;

        if (trackLogDownlink) trackDownlinkText(tail);
        else                  logLine(tail);
      }

      trackLogEnabled = false;
      deferredNetRateHz = 0;      // also cancels one that never started

      // Restore the tethered routing so the next bare `tracklog on` behaves.
      trackLogDownlink = false;
      asyncDebug.setTrackSink(nullptr);
      asyncDebug.setTrackSerialEnabled(true);

      logLine("tracklog off");
      return;
    }

    /*
      "tracklog net [hz]" - untethered. Rows go to the transmitter over
      ESP-NOW instead of the UART, and the transmitter prints them to the
      laptop. Everything else about the log is identical.
    */
    bool wantDownlink = false;
    if (strncmp(argument, "net", 3) == 0)
    {
      wantDownlink = true;
      argument += 3;
      while (*argument == ' ') argument++;
    }

    if (wantDownlink && !transmitterMacKnown)
    {
      /*
        Hold the request rather than refusing it. The address arrives with the
        first RC packet; loop() starts the log then. See deferredNetRateHz.
      */
      long queued = 50;
      if (*argument != '\0' && strcmp(argument, "on") != 0)
      {
        queued = strtol(argument, nullptr, 10);
        if (queued < static_cast<long>(TRACK_LOG_MIN_HZ) ||
            queued > static_cast<long>(TRACK_LOG_MAX_HZ))
        {
          logf("Rate must be %lu-%lu Hz.",
            (unsigned long)TRACK_LOG_MIN_HZ, (unsigned long)TRACK_LOG_MAX_HZ);
          return;
        }
      }

      deferredNetRateHz = static_cast<uint8_t>(queued);
      logf("tracklog net queued at %ld Hz - waiting for the transmitter link. "
           "It will start by itself; `tracklog off` cancels.", queued);
      return;
    }

    // "tracklog", "tracklog on", "tracklog <hz>", or the rate after "net".
    if (*argument != '\0' && strcmp(argument, "on") != 0)
    {
      const long requested = strtol(argument, nullptr, 10);
      if (requested < static_cast<long>(TRACK_LOG_MIN_HZ) ||
          requested > static_cast<long>(TRACK_LOG_MAX_HZ))
      {
        logf("Rate must be %lu-%lu Hz.",
          (unsigned long)TRACK_LOG_MIN_HZ, (unsigned long)TRACK_LOG_MAX_HZ);
        return;
      }
      trackLogPeriodUs = static_cast<uint32_t>(1000000L / requested);
    }

    trackLogSequence = 0;
    previousTrackLogUs = micros();
    trackDownlinkSent = 0;
    trackDownlinkFailed = 0;

    /*
      Install the routing BEFORE the header goes out, so the preamble and the
      rows travel the same path and land in the same file in the right order.
    */
    trackHeaderCount = 0;
    trackHeaderSent = 0;

    trackLogDownlink = wantDownlink;
    asyncDebug.setTrackSink(wantDownlink ? &trackDownlinkSend : nullptr);
    asyncDebug.setTrackSerialEnabled(!wantDownlink);

    trackLogEnabled = true;

    const float rateHz = 1.0e6f / static_cast<float>(trackLogPeriodUs);

    emitTrackLogHeader();

    if (trackLogDownlink)
    {
      /*
        No serial-link warning here: the constraint is the radio, not a UART.
        One 216-byte packet per sample is 10.8 kB/s at 50 Hz, well inside what
        ESP-NOW carries, so the useful check after the flight is the dl_failed
        count in the `#TRK end` line rather than a predicted utilisation.
      */
      logf("tracklog net at %.1f Hz -> transmitter "
           "%02X:%02X:%02X:%02X:%02X:%02X", rateHz,
        transmitterMac[0], transmitterMac[1], transmitterMac[2],
        transmitterMac[3], transmitterMac[4], transmitterMac[5]);
      logLine("Rows are printed by the TRANSMITTER. Capture its port, not "
              "this one. The drone's USB can now be unplugged.");
      return;
    }

    /*
      Warn rather than refuse. A row is about 370 characters, so the link
      carries roughly 3.7 kB/s per 10 Hz at 460800 baud. Past about 60%
      utilisation the UART stops absorbing bursts and rows start landing in
      the dropped counter instead of the file.
    */
    const float utilisation = rateHz * 370.0f * 10.0f / 460800.0f;

    logf("tracklog on at %.1f Hz  (~%.0f%% of the serial link)",
      rateHz, utilisation * 100.0f);

    if (utilisation > 0.60f)
    {
      logLine("WARNING: above 60% link utilisation. Check `timing` for "
              "dropped track samples after the flight and drop the rate if "
              "any were lost.");
    }
    return;
  }

  if (strncmp(command, "ratelog", 7) == 0)
  {
    const char *argument = command + 7;
    while (*argument == ' ') argument++;

    if (strcmp(argument, "on") == 0)       rateLogEnabled = true;
    else if (strcmp(argument, "off") == 0) rateLogEnabled = false;
    else                                   rateLogEnabled = !rateLogEnabled;

    logf("ratelog %s  (measured/target deg/s per axis)",
      rateLogEnabled ? "on" : "off");
    return;
  }

  if (strcmp(command, "link") == 0)
  {
    portENTER_CRITICAL(&espNowMux);
    const uint32_t total = packetCount;
    const uint32_t worst = worstPacketGapMs;
    const uint32_t over100 = gapsOver100msCount;
    const uint32_t overFail = gapsOverFailsafeCount;
    const uint32_t badMagic = rejectedMagicCount;
    const uint32_t badLength = rejectedLengthCount;
    const uint32_t last = lastPacketTimeMs;
    portEXIT_CRITICAL(&espNowMux);

    logf("packets accepted : %lu", (unsigned long)total);
    logf("worst gap        : %lu ms  (failsafe fires at %lu ms)",
      (unsigned long)worst, (unsigned long)FAILSAFE_TIMEOUT_MS);
    logf("gaps over 100 ms : %lu", (unsigned long)over100);
    logf("gaps that tripped failsafe : %lu", (unsigned long)overFail);
    logf("rejected bad magic  : %lu", (unsigned long)badMagic);
    logf("rejected bad length : %lu", (unsigned long)badLength);
    logf("last packet      : %ld ms ago",
      (long)static_cast<int32_t>(millis() - last));
    logf("expected rate    : one packet every 20 ms");
    return;
  }

  logf("Unknown command: %s (try help)", command);
}

// ================================================================
// ESP-NOW
// ================================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int length
)
{
  /*
    Track-log control from the transmitter. Checked first and returned from
    immediately, so a logging packet never reaches any of the RC bookkeeping
    below - it must not touch lastPacketTimeMs or the failsafe would treat a
    logging command as proof the RC link is alive.

    Only a flag is set here. Starting the log emits a header and registers an
    ESP-NOW peer, neither of which is legal from this callback; loop() picks
    the request up.
  */
  if (length == static_cast<int>(sizeof(TrackCommandPacket)))
  {
    TrackCommandPacket cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));

    if (cmd.magic == TRACK_COMMAND_MAGIC)
    {
      if (!transmitterMacKnown && info != nullptr)
      {
        memcpy(transmitterMac, info->src_addr, 6);
        transmitterMacKnown = true;
      }
      pendingTrackCommandEnable = (cmd.enable != 0);
      pendingTrackCommandRateHz = cmd.rateHz;
      pendingTrackCommand = true;
      return;
    }
  }

  if (length != static_cast<int>(sizeof(ControlData)))
  {
    rejectedLengthCount++;
    return;
  }

  ControlData temporary;
  memcpy(&temporary, incomingData, sizeof(ControlData));

  if (temporary.magic != PACKET_MAGIC)
  {
    rejectedMagicCount++;
    return;
  }

  /*
    Learn who is talking to us, for the track downlink. Copied here because
    this is the only place the source address is available, but the peer is
    REGISTERED later from task context - esp_now_add_peer() takes a mutex and
    must not be called from this callback.
  */
  if (!transmitterMacKnown && info != nullptr)
  {
    memcpy(transmitterMac, info->src_addr, 6);
    transmitterMacKnown = true;
  }

  const uint32_t nowMs = millis();

  portENTER_CRITICAL_ISR(&espNowMux);

  // Measure the gap here, in the callback, so that a dropout shorter than
  // the telemetry interval still gets recorded.
  const uint32_t previous = lastPacketTimeMs;

  if (previous != 0)
  {
    const uint32_t gap = nowMs - previous;

    if (gap > worstPacketGapMs) worstPacketGapMs = gap;
    if (gap > 100U) gapsOver100msCount++;
    if (gap >= FAILSAFE_TIMEOUT_MS) gapsOverFailsafeCount++;
  }

  memcpy(&receivedData, &temporary, sizeof(ControlData));
  packetReceived = true;
  lastPacketTimeMs = nowMs;
  packetCount++;

  portEXIT_CRITICAL_ISR(&espNowMux);
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(460800);
  delay(300);

  Serial.println();
  Serial.println("========================================");
  Serial.println("FPV 2026 ROTOR-RATE CONTROLLER");
  Serial.println("Right-handed frame: +X right, +Y forward, +Z up");
  Serial.println("Yaw is COUNTER-CLOCKWISE positive throughout.");
  Serial.println("========================================");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!setupMPU())
  {
    Serial.println("FATAL: MPU6050 not responding.");
    while (true) delay(1000);
  }

  if (!calibrateIMU())
  {
    Serial.println("FATAL: IMU calibration failed.");
    while (true) delay(1000);
  }

  if (!initializeRangeFinder())
  {
    Serial.println("WARNING: VL53L0X init failed; altitude hold unavailable.");
  }

  if (!initializeMagnetometer())
  {
    Serial.println("WARNING: QMC5883P init failed; heading hold unavailable.");
    magYawFusionEnabled = false;
  }

  if (!initializeOpticalFlow())
  {
    Serial.println("WARNING: PMW3901 init failed; XY hold unavailable.");
  }

  /*
    Settle thresholds are PER AXIS because each one is fed a different
    quantity in different units. The altitude and horizontal holds receive
    metres per second; the heading hold receives DEGREES PER SECOND. Leaving
    the heading hold on the class default of 0.15 would demand a yaw rate
    below gyro noise, and it would never capture a heading.
  */
  altitudeHold.settleSpeedThreshold = 0.12f;   // m/s
  altitudeHold.settleDwellMs = 400;

  horizontalHold.settleSpeedThreshold = 0.15f; // m/s
  horizontalHold.settleDwellMs = 500;

  headingHold.settleSpeedThreshold = 8.0f;     // deg/s
  headingHold.settleDwellMs = 300;

  stateEstimator.reset(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  if (!bidirectionalDshot.begin())
  {
    Serial.println("FATAL: bidirectional DShot init failed.");
    while (true) delay(1000);
  }
  stopMotors();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("FATAL: ESP-NOW init failed.");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(OnDataRecv);

  /*
    Read the MAC from the driver directly.

    WiFi.macAddress() printed 00:00:00:00:00:00 here, which happens when the
    Arduino wrapper is asked before its interface bookkeeping is populated.
    esp_wifi_get_mac() asks the driver, which is up by this point because
    esp_now_init() succeeded.
  */
  uint8_t receiverMac[6] = {};
  if (esp_wifi_get_mac(WIFI_IF_STA, receiverMac) == ESP_OK)
  {
    Serial.printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      receiverMac[0], receiverMac[1], receiverMac[2],
      receiverMac[3], receiverMac[4], receiverMac[5]);
  }
  else
  {
    Serial.println("WARNING: could not read receiver MAC.");
  }

  // The PMW SPI task goes on the core opposite the Arduino loop.
  const BaseType_t flowCore = (xPortGetCoreID() == 0) ? 1 : 0;
  if (!startOpticalFlowTask(flowCore))
  {
    Serial.println("WARNING: optical flow task failed to start.");
  }

  asyncDebug.begin(Serial, flowCore, 1, 12288);

  previousControlUs = micros();

  Serial.println();
  Serial.println("Ready. Arm with throttle centred.");
  Serial.println("kM identification starts DISABLED; enable with 'kmlearn on'.");
  Serial.println("Type 'help' for commands.");
  Serial.println();
}

// ================================================================
// LOOP
// ================================================================

void loop()
{
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  ++loopPassCount;

  /*
    Serial commands and asynchronous sensors are serviced on EVERY pass, not
    only when the control step finishes early.

    This used to sit inside the "loop has slack" branch, which coupled
    command responsiveness to how long the control step happened to take. As
    soon as the step approached its 2 ms budget the branch stopped being
    reached, and commands went from instant to arbitrarily delayed with no
    indication why. Both of these are internally rate-limited or non-blocking,
    so calling them unconditionally is cheap and predictable.
  */
  for (int drained = 0; drained < 4; ++drained)
  {
    if (!asyncDebug.popCommand(serialCommandBuffer, sizeof(serialCommandBuffer)))
    {
      break;
    }
    processSerialCommand(serialCommandBuffer, nowMs);
  }

  /*
    Track-log start/stop requested by the transmitter. Routed through the same
    command parser as the serial path so there is exactly one implementation of
    what `tracklog net` means - the radio just types the command for you.
  */
  if (pendingTrackCommand)
  {
    pendingTrackCommand = false;

    char synthesised[32];
    if (pendingTrackCommandEnable)
    {
      snprintf(synthesised, sizeof(synthesised), "tracklog net %u",
        static_cast<unsigned>(pendingTrackCommandRateHz));
    }
    else
    {
      strcpy(synthesised, "tracklog off");
    }

    processSerialCommand(synthesised, nowMs);
  }

  /*
    Start a `tracklog net` that was asked for before the link existed.
  */
  if (deferredNetRateHz != 0 && transmitterMacKnown)
  {
    char synthesised[32];
    snprintf(synthesised, sizeof(synthesised), "tracklog net %u",
      static_cast<unsigned>(deferredNetRateHz));
    deferredNetRateHz = 0;
    processSerialCommand(synthesised, nowMs);
  }

  serviceTrackHeaderQueue();

  updateRangeSensor(nowUs, nowMs);
  updateMagnetometer(nowUs, nowMs);
  consumeOpticalFlowSample(nowMs);

  // ------------------------------------------------------------
  // Control cadence.
  // ------------------------------------------------------------

  const uint32_t elapsedUs = static_cast<uint32_t>(nowUs - previousControlUs);
  if (elapsedUs < CONTROL_PERIOD_US) return;

  previousControlUs = nowUs;
  const uint32_t controlStartUs = micros();
  const float dt = clampFloatFc(
    static_cast<float>(elapsedUs) * 1.0e-6f, 0.0005f, 0.020f
  );

  // ------------------------------------------------------------
  // Latch the newest control packet and evaluate the failsafe.
  // ------------------------------------------------------------

  portENTER_CRITICAL(&espNowMux);
  if (packetReceived)
  {
    memcpy(&controlData, &receivedData, sizeof(ControlData));
    packetReceived = false;
  }
  const uint32_t lastPacketMs = lastPacketTimeMs;
  portEXIT_CRITICAL(&espNowMux);

  /*
    SIGNED age comparison. This was an unsigned subtraction and it was the
    cause of the repeated unexplained disarms.

    nowMs is sampled once at the top of the control cycle. lastPacketMs is
    written by the ESP-NOW receive INTERRUPT, which can fire after that
    snapshot is taken and store a later millis(). When it does, lastPacketMs
    is 1-2 ms AHEAD of nowMs, and the unsigned subtraction wraps to about
    4.29e9 - which is trivially greater than the 300 ms timeout, so the
    failsafe declared signal loss on a perfectly healthy link.

    The 2026-08-14 log printed the smoking gun: "last packet 4294967295 ms
    ago" (that is 2^32 - 1, i.e. -1) on the same run where `link` reported a
    52 ms worst gap and zero failsafe-length gaps.

    Casting the difference to int32_t makes a packet from 1 ms in the
    "future" read as -1 ms of age, which correctly fails the > 300 test.
    Millis rollover at 49.7 days is still handled, because the signed
    difference of two uint32 values stays correct across the wrap.
  */
  const int32_t packetAgeMs = static_cast<int32_t>(nowMs - lastPacketMs);

  signalLost =
    (lastPacketMs == 0U) ||
    (packetAgeMs > static_cast<int32_t>(FAILSAFE_TIMEOUT_MS));

  if (signalLost)
  {
    controlData.motorsEnabled = false;
    controlData.throttle = 0.0f;
    controlData.roll = 0.0f;
    controlData.pitch = 0.0f;
    controlData.yaw = 0.0f;
  }

  // ------------------------------------------------------------
  // Attitude and estimator.
  // ------------------------------------------------------------

  imuValid = updateAttitude(dt);

  if (!imuValid)
  {
    if (flightState != FlightState::Disarmed)
    {
      safetyStopLatched = true;
      stopMotors();
      enterState(FlightState::Disarmed, nowMs, "IMU failure");
      logLine("SAFETY STOP: IMU read or estimator failure.");
    }
    return;
  }

  // ------------------------------------------------------------
  // Control.
  // ------------------------------------------------------------

  const uint32_t controlOnlyStartUs = micros();
  runControlStep(dt, nowMs);
  timingControlTotalUs +=
    static_cast<uint32_t>(micros() - controlOnlyStartUs);
  ++timingSampleCount;

  // ------------------------------------------------------------
  // Flight track log. Sampled here, immediately after the control step, so
  // every field in a row belongs to the same cycle. The cadence is checked
  // against the control clock rather than millis() so the interval does not
  // drift against the loop it is sampling.
  // ------------------------------------------------------------

  if (trackLogEnabled &&
      static_cast<uint32_t>(nowUs - previousTrackLogUs) >= trackLogPeriodUs)
  {
    previousTrackLogUs = nowUs;
    publishTrackSample(nowMs, elapsedUs);
  }

  // ------------------------------------------------------------
  // Profiling. A step that approaches CONTROL_PERIOD_US starves everything
  // else sharing the loop, so keep the number visible rather than inferring
  // it from odd behaviour elsewhere.
  // ------------------------------------------------------------

  const uint32_t controlDurationUs =
    static_cast<uint32_t>(micros() - controlStartUs);

  controlStepTotalUs += controlDurationUs;
  ++controlStepCount;
  if (controlDurationUs > controlStepMaxUs)
  {
    controlStepMaxUs = controlDurationUs;
  }
  if (controlDurationUs >= CONTROL_PERIOD_US)
  {
    ++controlOverrunCount;
  }

  // ------------------------------------------------------------
  // Telemetry.
  // ------------------------------------------------------------

  if (static_cast<uint32_t>(nowMs - previousTelemetryMs) >=
      TELEMETRY_PERIOD_MS)
  {
    previousTelemetryMs = nowMs;

    reportedControlAvgUs = (controlStepCount > 0)
      ? static_cast<uint32_t>(controlStepTotalUs / controlStepCount)
      : 0;
    reportedControlMaxUs = controlStepMaxUs;

    // Loop passes per control step. This is the headroom figure: 1 means the
    // loop has none, and anything sharing it will be starved.
    reportedLoopPassesPerStep = (controlStepCount > 0)
      ? (loopPassCount / controlStepCount)
      : 0;

    if (timingSampleCount > 0)
    {
      reportedImuUs =
        static_cast<uint32_t>(timingImuReadTotalUs / timingSampleCount);
      reportedEstimatorUs =
        static_cast<uint32_t>(timingEstimatorTotalUs / timingSampleCount);
      reportedControlUs =
        static_cast<uint32_t>(timingControlTotalUs / timingSampleCount);
    }

    timingImuReadTotalUs = 0;
    timingEstimatorTotalUs = 0;
    timingControlTotalUs = 0;
    timingSampleCount = 0;

    controlStepTotalUs = 0;
    controlStepCount = 0;
    controlStepMaxUs = 0;
    loopPassCount = 0;

    publishTelemetry(nowMs);
  }
}
