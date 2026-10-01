#pragma once

#include <Arduino.h>

/*
  ============================================================================
  FLIGHT TRACK SAMPLE - SHARED BETWEEN RECEIVER AND TRANSMITTER
  ============================================================================

  This header is compiled into BOTH sketches. The receiver fills the struct on
  the flight core; the transmitter receives it over the ESP-NOW downlink and
  formats it. Keeping the struct and the formatter in one file is what
  guarantees the untethered CSV is byte-identical to the tethered CSV, so
  capture.py and analyze_track.py cannot tell the two apart.

  If you edit this file, copy it to BOTH sketch folders. The Arduino build
  system compiles each sketch directory independently and will not follow a
  relative path out of the sketch folder.

  ---------------------------------------------------------------------------
  WHY THIS IS A POD STRUCT AND NOT A FORMATTED STRING

  Formatting ~45 floats costs several hundred microseconds on this part. The
  control step already averages 1276 us against a 2500 us budget and peaks at
  2025 us, so doing that work on the flight core would put the loop over
  budget on its worst cycles. The flight core therefore fills this struct and
  hands it to a queue - a copy of about 200 bytes - and every snprintf runs on
  core 0 in the debug task, which has nothing else time-critical to do.

  In downlink mode the struct is handed to ESP-NOW instead, and the snprintf
  runs on the transmitter, which is not flying anything.

  Angles are stored in RADIANS and converted to degrees at print time, so the
  flight core never pays for a unit conversion either.
*/
struct AsyncTrackSample
{
  uint32_t timestampMs = 0;
  uint32_t sequence = 0;      // gaps in this prove the log lost samples
  uint16_t controlDtUs = 0;   // measured, not assumed

  uint8_t flightState = 0;
  uint8_t holdAltitude = 0;   // 0 free, 1 settling, 2 held
  uint8_t holdHorizontal = 0;
  uint8_t holdHeading = 0;
  uint8_t rpmFreshMask = 0;

  // bit0 armed, 1 link up, 2 yaw proven, 3 flow healthy, 4 range usable,
  // 5 magnetometer fresh, 6 estimator healthy, 7 ESKF attitude in use
  uint8_t healthFlags = 0;

  // bit0 yaw demand clipped, 1 yaw allocation clipped, 2 roll/pitch scaled
  uint8_t saturationFlags = 0;

  // Pilot input as received, BEFORE the deadband. The deadband is a known
  // constant, so the analysis can reapply it; the raw value cannot be
  // recovered from the shaped one.
  float rcThrottle = 0.0f, rcRoll = 0.0f, rcPitch = 0.0f, rcYaw = 0.0f;

  // Position: estimate and the target the controller is holding.
  float positionX = 0.0f, positionY = 0.0f, altitude = 0.0f;
  float targetX = 0.0f, targetY = 0.0f, targetAltitude = 0.0f;

  // Velocity: optical-flow body measurement, estimator navigation state, and
  // the targets the outer loops asked for.
  float bodyVelocityX = 0.0f, bodyVelocityY = 0.0f;
  float navVelocityX = 0.0f, navVelocityY = 0.0f;
  float verticalVelocity = 0.0f;
  float bodyVelocityTargetX = 0.0f, bodyVelocityTargetY = 0.0f;
  float verticalVelocityTarget = 0.0f;

  // Attitude [rad]: measured and demanded.
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  float rollDesired = 0.0f, pitchDesired = 0.0f, yawTarget = 0.0f;

  // Body rates [rad/s]: measured and demanded. rateX is about +X (pitch
  // rate), rateY about +Y (roll rate), rateZ about +Z (yaw rate).
  float rateX = 0.0f, rateY = 0.0f, rateZ = 0.0f;
  float rateTargetX = 0.0f, rateTargetY = 0.0f, rateTargetZ = 0.0f;

  // Commanded wrench.
  float thrustN = 0.0f, tauX = 0.0f, tauY = 0.0f, tauZ = 0.0f;

  // Allocation and actuators.
  float rotorSetpoint[4] = {};   // rad/s out of the allocator
  float rotorMeasured[4] = {};   // rad/s from ESC telemetry
  uint16_t dshot[4] = {};

  // Raw-ish sensor context.
  float rangeM = 0.0f;
  float magYawRad = 0.0f;
  uint8_t flowSqual = 0;
};

/*
  ============================================================================
  DOWNLINK PACKET
  ============================================================================

  The struct above is about 200 bytes, comfortably inside the 250-byte ESP-NOW
  payload limit, so one sample is one packet and there is no fragmentation to
  get wrong. The magic word lets the transmitter tell a track packet apart from
  anything else that might arrive, and lets it reject a stale build: bump
  TRACK_PACKET_VERSION whenever a field is added to AsyncTrackSample, and a
  transmitter running old firmware will say so instead of silently emitting
  misaligned columns.
*/
static constexpr uint32_t TRACK_PACKET_MAGIC = 0x54524B31UL;  // "TRK1"
static constexpr uint8_t  TRACK_PACKET_VERSION = 1;

struct TrackDownlinkPacket
{
  uint32_t magic = TRACK_PACKET_MAGIC;
  uint8_t version = TRACK_PACKET_VERSION;
  uint8_t reserved[3] = {};
  AsyncTrackSample sample;
};

/*
  The "#TRK ..." preamble carries airframe constants the analysis needs - mass,
  kF, the identified kM, the inertias - which only the drone knows. Untethered
  there is no UART to print them on, so they ride the same downlink as a
  handful of short text packets sent just before the rows start.

  Low rate and low volume: about ten packets, once per tracklog start.
*/
static constexpr uint32_t TRACK_TEXT_MAGIC = 0x54524B54UL;  // "TRKT"

struct TrackTextPacket
{
  uint32_t magic = TRACK_TEXT_MAGIC;
  char text[200] = {};
};

/*
  The column header is ~430 characters - longer than a text packet, and it
  must arrive as ONE line or the CSV header is corrupt. It is not sent at all:
  the receiver sends this sentinel and the transmitter prints its own copy of
  TRACK_COLUMN_HEADER. Both sketches compile that constant from this file, so
  the line the transmitter prints is by construction the header for the rows
  the receiver is about to send.
*/
static const char TRACK_COLUMN_HEADER_SENTINEL[] = "#TRKCOLS?";

/*
  ============================================================================
  UPLINK COMMAND PACKET
  ============================================================================

  Starts and stops the downlink log from the TRANSMITTER, so the whole session
  runs without ever touching the drone's USB port. You type at the transmitter,
  which is the device on your desk.

  Deliberately a SEPARATE packet type rather than two more fields in
  ControlData. The RC packet is the one the failsafe watches and the one whose
  size the receiver validates; changing it to carry a logging convenience would
  put a non-safety feature inside the safety path. The receiver dispatches on
  length and magic, so this packet cannot be mistaken for an RC packet and a
  malformed one cannot disturb the control loop.
*/
static constexpr uint32_t TRACK_COMMAND_MAGIC = 0x54524B43UL;  // "TRKC"

struct TrackCommandPacket
{
  uint32_t magic = TRACK_COMMAND_MAGIC;
  uint8_t enable = 0;    // 1 start downlink logging, 0 stop
  uint8_t rateHz = 50;   // ignored when stopping
  uint8_t reserved[2] = {};
};

/*
  Column names. Emitted once when logging starts so the analysis never has to
  hard-code a column order. The "#TRKCOLS," prefix and the "TRK," row prefix
  let both be extracted from a capture file that also holds ordinary telemetry,
  events and command output.
*/
static const char TRACK_COLUMN_HEADER[] =
  "#TRKCOLS,"
  "t_ms,seq,dt_us,state,hold_alt,hold_xy,hold_hdg,"
  "armed,link,yaw_proven,flow_ok,range_ok,mag_ok,eskf_ok,eskf_att,"
  "sat_yaw_demand,sat_yaw_alloc,sat_rollpitch,rpm_mask,"
  "rc_thr,rc_roll,rc_pitch,rc_yaw,"
  "px,py,alt,px_t,py_t,alt_t,"
  "vbx,vby,vnx,vny,vz,vbx_t,vby_t,vz_t,"
  "roll,pitch,yaw,roll_d,pitch_d,yaw_t,"
  "wx,wy,wz,wx_t,wy_t,wz_t,"
  "thrust,tau_x,tau_y,tau_z,"
  "w1_sp,w2_sp,w3_sp,w4_sp,w1,w2,w3,w4,d1,d2,d3,d4,"
  "range,mag_yaw,flow_squal";

/*
  One CSV row of the command-to-response chain. Returns the length written.

  A row is 367 characters at typical magnitudes; the 640-byte buffer the
  callers use covers the worst case with room to spare.
*/
inline int formatTrackRow(char *line, size_t capacity, const AsyncTrackSample &s)
{
  static constexpr float DEG = 57.2957795f;

  int n = 0;

  n += snprintf(line + n, capacity - n,
    "TRK,%lu,%lu,%u,%u,%u,%u,%u,",
    static_cast<unsigned long>(s.timestampMs),
    static_cast<unsigned long>(s.sequence),
    static_cast<unsigned>(s.controlDtUs),
    static_cast<unsigned>(s.flightState),
    static_cast<unsigned>(s.holdAltitude),
    static_cast<unsigned>(s.holdHorizontal),
    static_cast<unsigned>(s.holdHeading));

  // Health flags expanded to individual columns: a packed byte saves a few
  // characters and costs every reader a bit-mask lookup.
  n += snprintf(line + n, capacity - n,
    "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%X,",
    static_cast<unsigned>((s.healthFlags >> 0) & 1u),
    static_cast<unsigned>((s.healthFlags >> 1) & 1u),
    static_cast<unsigned>((s.healthFlags >> 2) & 1u),
    static_cast<unsigned>((s.healthFlags >> 3) & 1u),
    static_cast<unsigned>((s.healthFlags >> 4) & 1u),
    static_cast<unsigned>((s.healthFlags >> 5) & 1u),
    static_cast<unsigned>((s.healthFlags >> 6) & 1u),
    static_cast<unsigned>((s.healthFlags >> 7) & 1u),
    static_cast<unsigned>((s.saturationFlags >> 0) & 1u),
    static_cast<unsigned>((s.saturationFlags >> 1) & 1u),
    static_cast<unsigned>((s.saturationFlags >> 2) & 1u),
    static_cast<unsigned>(s.rpmFreshMask));

  n += snprintf(line + n, capacity - n,
    "%.3f,%.3f,%.3f,%.3f,",
    s.rcThrottle, s.rcRoll, s.rcPitch, s.rcYaw);

  n += snprintf(line + n, capacity - n,
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
    s.positionX, s.positionY, s.altitude,
    s.targetX, s.targetY, s.targetAltitude);

  n += snprintf(line + n, capacity - n,
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
    s.bodyVelocityX, s.bodyVelocityY,
    s.navVelocityX, s.navVelocityY, s.verticalVelocity,
    s.bodyVelocityTargetX, s.bodyVelocityTargetY, s.verticalVelocityTarget);

  n += snprintf(line + n, capacity - n,
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",
    s.roll * DEG, s.pitch * DEG, s.yaw * DEG,
    s.rollDesired * DEG, s.pitchDesired * DEG, s.yawTarget * DEG);

  n += snprintf(line + n, capacity - n,
    "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,",
    s.rateX * DEG, s.rateY * DEG, s.rateZ * DEG,
    s.rateTargetX * DEG, s.rateTargetY * DEG, s.rateTargetZ * DEG);

  n += snprintf(line + n, capacity - n,
    "%.3f,%.5f,%.5f,%.5f,",
    s.thrustN, s.tauX, s.tauY, s.tauZ);

  n += snprintf(line + n, capacity - n,
    "%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%u,%u,%u,",
    s.rotorSetpoint[0], s.rotorSetpoint[1],
    s.rotorSetpoint[2], s.rotorSetpoint[3],
    s.rotorMeasured[0], s.rotorMeasured[1],
    s.rotorMeasured[2], s.rotorMeasured[3],
    static_cast<unsigned>(s.dshot[0]), static_cast<unsigned>(s.dshot[1]),
    static_cast<unsigned>(s.dshot[2]), static_cast<unsigned>(s.dshot[3]));

  n += snprintf(line + n, capacity - n,
    "%.3f,%.2f,%u",
    s.rangeM, s.magYawRad * DEG, static_cast<unsigned>(s.flowSqual));

  return n;
}
