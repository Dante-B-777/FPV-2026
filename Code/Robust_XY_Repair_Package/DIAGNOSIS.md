# Full XY-Hold Failure Diagnosis

## Bottom line

The XY controller is **executing**, but it is not a trustworthy position-hold
system in the current build. Three independent problems are stacked together:

1. XY hold is allowed to arm while the aircraft is still in the takeoff ramp
   and climbing rapidly.
2. The position-return authority was tuned almost to zero, while the noisy
   optical-flow velocity term still has enough authority to hit the attitude
   limit.
3. The PMW3901 path declares flow valid without using the sensor's quality
   information, without proving the mounted axis transform, and without
   requiring both estimator axes to accept the measurement.

The severe yaw oscillation is a fourth blocking fault. A navigation-frame
position controller cannot hold a fixed X/Y point while the yaw frame is
rotating back and forth at roughly 120 deg/s.

Do not free-fly the old XY build. Gain changes alone cannot repair this.

## Evidence from the supplied hover log

`analyze_telemetry.py latest_hover_log.txt` reports:

```text
telemetry_rows=28
xy_active_rows=22

First XY-active sample:
  ALT=0
  Z=0.272 m
  VZ=+0.427 m/s
  YR=-7.41 deg/s

While XY was active:
  max |X-XT| = 1.162 m
  max |Y-YT| = 1.516 m
  max |FVX|  = 0.766 m/s
  max |FVY|  = 0.758 m/s
  max |YR|   = 119.73 deg/s
  max |YC|   = 12.0, the configured yaw limit
```

The first XY-active sample occurs before altitude hold is active and while the
vehicle is climbing at 0.427 m/s. Later, the fixed target remains near
X=0.123 m, Y=0.026 m while the estimated position moves more than a meter away.
The yaw controller repeatedly saturates in opposite directions.

## Root cause 1: XY hold arms during takeoff

The old command sequence calls `updateHorizontalController()` before the
startup-ramp branch finishes. The old arming check requires `flightEnabled`,
valid flow, valid range, and enough height, but it does **not** require:

- `!enableRampActive`;
- altitude hold already active;
- low vertical speed;
- low horizontal speed;
- low yaw rate;
- a stable dwell period.

The log proves the consequence: XY hold captures a target at approximately
0.27 m while the aircraft is still rising quickly. During takeoff the floor
image changes scale rapidly, vibration rises, attitude changes, and yaw is not
settled. That is the worst possible time to establish the navigation origin.

## Root cause 2: position restoration was reduced to about 0.19 degrees

The old values are:

```text
xyPositionKp                 = 0.10  1/s
xyVelocityKp                 = 4.00  deg per (m/s)
MAX_POSITION_TARGET_ERROR_M  = 0.50 m
XY_POSITION_DEADBAND_M       = 0.03 m
```

The largest position error that reaches the P term is therefore:

```text
0.50 - 0.03 = 0.47 m
```

The largest position-only velocity request is:

```text
0.47 m * 0.10 1/s = 0.047 m/s
```

The largest position-only attitude request is:

```text
0.047 m/s * 4 deg/(m/s) = 0.188 deg
```

Therefore, even a one-meter position error produces less than two-tenths of a
degree of restoring tilt. That is too small to overcome normal frame trim,
propeller mismatch, airflow, and estimator noise on this airframe.

At the same time, a measured velocity error of only 0.75 m/s produces the full
3-degree limit:

```text
0.75 m/s * 4 deg/(m/s) = 3.0 deg
```

The log reaches approximately 0.76 m/s fused velocity. The old tuning therefore
makes the **desired signal weak** and the **noisy signal dominant**.

## Root cause 3: `FLW=1` is not a real flow-quality test

The old PMW sample contains only:

- raw delta X/Y;
- sample time and interval;
- integrated roll, pitch, and yaw increments.

The old validity check only tests:

- interval between 6 and 30 ms;
- absolute raw count below 100;
- a valid range measurement.

It does not use:

- the PMW3901 motion-burst marker;
- surface-quality `SQUAL`;
- shutter/exposure;
- raw-data minimum, maximum, and sum;
- transfer coherence;
- a physical speed limit.

At 0.60 m height and 10 ms sample time, the old scale converts 100 counts into
approximately 14.7 m/s. A threshold that accepts almost 15 m/s on a small hover
test does not reject realistic bad data.

The robust replacement reads the full motion burst and requires:

```text
motion == 0xB0
SQUAL >= 1
shutter > 0
coherent/plausible burst fields
valid sample timing and height
finite speed below a physical limit
both X and Y estimator innovations accepted
```

## Root cause 4: the mounted PMW axis transform has never been proven

The old mapping is:

```text
body X = +raw X
body Y = -raw Y
```

A common flow-deck mapping in the Bitcraze/Espressif implementation is:

```text
body X = -raw Y
body Y = -raw X
```

Neither mapping should be copied blindly. The correct transform depends on the
actual sensor PCB rotation and lens orientation on this drone. There are eight
possible signed/swapped 2-D mappings. The diagnostic sketch exposes all eight.

A wrong axis or sign creates one of two failures:

- the controller corrects the wrong direction immediately; or
- roll/pitch rotational compensation creates fake translation with positive
  feedback, producing an oscillation that grows with time.

## Root cause 5: partial ESKF acceptance is treated as a good 2-D sample

The old `updateBodyVelocity()` processes X and Y as separate scalar Kalman
updates and returns `acceptedAny`. This means:

```text
X accepted, Y rejected -> function returns true
X rejected, Y accepted -> function returns true
```

The main controller then marks flow ready, increments its valid-sample count,
and keeps XY hold alive. A broken axis can therefore hide behind the good axis.

The old flow update also has attitude-error Jacobian terms, so uncommissioned
flow geometry can push the quaternion. The robust ESKF adds a commissioning
method that:

- updates translational velocity only;
- returns an X/Y acceptance bit mask;
- requires mask `0x03`, meaning both axes accepted.

## Root cause 6: yaw instability makes navigation-frame hold impossible

The latest log reaches approximately:

```text
YR = +118.5 deg/s, YC = -12.0
YR = -119.7 deg/s, YC = +12.0
```

The yaw controller is saturating and reversing. The X/Y controller rotates
body velocity and target commands through `yawAngleCwDeg`. When yaw is wrong or
oscillating, the same physical floor motion is projected into changing
navigation directions. Even a perfect XY P controller cannot hold a fixed
world-frame point under that condition.

The robust build therefore starts magnetic fusion and heading hold **off**.
Yaw must pass an independent mixer/sign/rate test before navigation-frame XY
hold is commissioned.

## Root cause 7: RMT flush timeouts are real but secondary

The log contains repeated `rmt_tx_wait_all_done(...): flush timeout` messages.
However, the same log reports roughly 498-501 Hz DShot service, fresh RPM mask
`F`, and near-100% packet quality. That makes the RMT warning a separate
reliability issue, not the primary explanation for the growing XY position
error. Fix it later, after attitude/yaw/flow commissioning, without changing
several control layers at once.

## Systematic replacement architecture

The robust build rearranges the loop as follows:

```text
PMW3901 full motion burst
  -> transfer/motion/SQUAL/shutter/timing/height/speed gates
  -> bench-verified signed/swapped body-axis mapping
  -> gyro and lever-arm compensation
  -> direct filtered body velocity for the fast velocity loop
  -> translation-only ESKF velocity correction, both axes required

ESKF position
  -> position P in 1/s
  -> bounded desired velocity
  -> body-frame velocity error
  -> acceleration P in 1/s
  -> bounded horizontal acceleration
  -> atan2(acceleration, gravity)
  -> filtered and slew-limited roll/pitch request
  -> existing angle/rate PID and validated mixer
```

XY activation is now a state transition, not a single Boolean expression. It
requires all of the following continuously for one second:

- explicit `xycommission on` interlock;
- startup ramp complete;
- altitude hold active;
- at least 0.5 s of valid two-axis flow;
- range at least 0.30 m;
- low vertical and horizontal speed;
- low yaw rate;
- low roll/pitch angle.

It aborts on stale flow, bad estimator state, excessive vertical speed, high
yaw rate, or excessive tilt.

## What is and is not proven

The source package has been checked for internal file consistency, balanced
braces, command names, include names, and reproducible log calculations. It has
**not** been compiled against your exact Arduino-ESP32 installation and has not
been flown on the hardware. The PMW axis map and compensation signs cannot be
proved from source code or a normal hover log; they must be measured on the
physical assembly before enabling XY hold.
