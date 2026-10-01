# Rotor-Rate Controller Package

Replacement for `Robust_XY_Repair_Package`. Fixes the yaw and XY failures at
their root and moves control onto per-rotor angular velocity.

## The bug, in one paragraph

`measuredYawRateDps` in the previous build physically held a
**counter-clockwise-positive** rate while being named `...Cw...`. The IMU is
mounted upside down, so the MPU6050's `gz` reads clockwise-positive; applying
`YAW_GYRO_SIGN = -1` made the stored value counter-clockwise. Roll and pitch
were unaffected, which is exactly why they flew well. Three consumers assumed
clockwise and each broke in a different way.

| # | Location | Consequence |
|---|----------|-------------|
| 1 | Yaw mixer `m1,m3 += yawMix` | Positive feedback. Saturated at ±12 DShot and reversed at ~120 °/s. |
| 2 | `calculateMagneticYawRightHandDeg()` | Returned `-atan2(...)`, the negative of the right-handed yaw it advertised, then fed it to the ESKF as a heading measurement. |
| 3 | `bodyToNavigation()` / `navigationToBody()` | Written as each other's transpose. Invisible at heading 0; fully cross-coupled at 90°; runaway at 180°. |
| 4 | Flow lever-arm `ω × r` | Applied with both signs inverted, injecting fake translation that reinforced the error during rotation. |
| 5 | **Transmitter** `#define YAW_COMMAND 0.0f` | Yaw was a compile-time constant zero, read from no pin. The receiver was handed 0.0 on every packet. This alone would have prevented yaw control even with items 1–4 fixed. |

Item 5 was found last and is the most consequential for the *symptom*: items
1–4 made yaw control wrong, item 5 meant no yaw command was ever sent at all.
Both had to be fixed. See `Transmitter_RotorRate/`.

The hover log corroborates all of it: `YR=+118.5` paired with `YC=-12.0` and
`YR=-119.7` with `YC=+12.0` is the signature of sign-inverted feedback pinned
against its limit, and XY position walked past 1.5 m from a target fixed at
X=0.123, Y=0.026 and never returned.

`DIAGNOSIS.md` in the old package correctly identified the gating, quality,
and gain problems and fixed them. It did not find the sign layer, and no gain
change could have.

## What changed

**Conventions live in exactly one file.** `FrameConventions.h` defines the
body frame (+X right, +Y forward, +Z up), declares every rotation
right-handed, and derives the IMU transform. Comparing the six previously
validated IMU signs shows they are all consistent with a single rigid
180° rotation about body +Y — so the roll and pitch behaviour that already
worked is preserved exactly, and only the yaw naming and its consumers change.

**The mixer is derived, not written down.** `RotorAllocator` builds the
allocation matrix from rotor positions and spin directions by summing
`r × F`. A sign error would now have to be a geometry error.

**Control runs on rotor angular velocity.**

```
position → velocity → acceleration → tilt
tilt → body rate → angular acceleration → torque
(thrust, torque) → RotorAllocator → per-rotor ω setpoint
ω setpoint → RotorSpeedLoop (closed on Bluejay RPM) → DShot
```

**`kM` is identified in flight.** The prop drag-torque coefficient was never
characterised for this airframe; the old model side-stepped yaw torque
entirely. `YawTorqueIdentifier` runs normalised LMS on
`Izz·αz + dHz/dt = -kM·D`, gated behind a commissioning flag, fresh RPM on all
four rotors, sufficient excitation, an unsaturated allocator, and near-level
attitude. It is hard-clamped to a physical band, so a failure degrades to
"yaw authority is somewhat off", never to a sign flip.

## Files

```
Transmitter_RotorRate/
  Transmitter_RotorRate.ino  radio: four real axes, NVS calibration

Receiver_RotorRate/
  Receiver_RotorRate.ino     main sketch: sensors, cascade, state machine
  FrameConventions.h         THE FIX - all axes, signs, mounting transforms
  RotorAllocator.h           wrench → per-rotor ω, with saturation policy
  RotorSpeedLoop.h           per-rotor ω PI closed on measured RPM
  YawTorqueIdentifier.h      in-flight kM identification
  HoldManager.h              settle-then-capture, yaw stability proof
  Eskf9.h                    unchanged from the previous build
  BidirectionalDShot.h/.cpp  unchanged
  AsyncDebugTelemetry.h/.cpp unchanged
  OpticalFlowAsync.h         unchanged

Validation/
  verify_signs.js            independent numeric check of every convention
```

## Verification status

**Compiles clean** against your installed toolchain: arduino-cli with
esp32:esp32 core 3.3.10, FQBN `esp32:esp32:esp32`. 76% flash, 15% RAM.

**Math verified numerically.** `node Validation/verify_signs.js` reimplements
the conventions and the allocator from scratch in JavaScript — building the
wrench by summing cross products rather than reusing the closed-form
inversion — so a shared typo cannot pass both. All 18 checks pass, including
three that reproduce the old bugs and confirm they were real.

```bash
node Validation/verify_signs.js
```

The same checks run on the target with the `selftest` serial command.

**Not verified:** nothing has been flown. Gains are physically derived
starting points, not ports of the old DShot-space tuning. Expect to tune.

## Commissioning order

Do these in order. Do not skip to step 4.

### 1. Bench, props removed

```
selftest
status
```

`selftest` must report PASSED. `status` should show a plausible range, flow
squal, and magnetometer validity.

### 2. Verify yaw direction on a stand, props removed

Arm, raise throttle slightly, and hand-rotate the airframe **clockwise** seen
from above. Telemetry `YR` must go **negative** (right-handed convention), and
`TAU` z must go **positive** to oppose it. If `YR` has the wrong sign, stop —
the IMU has been remounted and `IMU_CHIP_TO_BODY_*` needs rechecking.

### 3. Verify yaw torque direction, props ON, tethered

Command a small yaw stick input. The airframe must rotate the way the stick
asks. Confirm `M2`/`M4` DShot rise for a counter-clockwise (positive yaw
stick) demand. Watch for any repeat of the old `±limit` reversal — there
should be none.

### 4. Identify kM

With the airframe tethered and hovering steadily:

```
kmlearn on
kmstatus
```

Watch `ratio` converge. It starts at 0.010 m and should settle somewhere in
0.005–0.020 m for 2.5″ props. Once `converged=1`, note the value and seed it
at boot with `kmratio <value>` so future flights start converged.

### 5. Altitude and heading hold

Hover hands-off. Both should capture after the settle dwell — telemetry shows
`HOLD A=HELD` and `H=HELD`. Push each axis and confirm it returns.

### 6. XY hold

XY hold arms itself, but only after the yaw loop has proven itself in this
flight: yaw rate within 10 °/s and heading error within 8°, continuously for
3 s while airborne. Telemetry `YP=1` means proven. Disarming clears it, so
every flight re-proves it.

The flow axis map is baked in as the commissioned **flowmap 1**
(body X = +raw X, body Y = −raw Y) in `FrameConventions.h`.

## Flight track logging

`tracklog` records the whole command-to-response chain as CSV, at a rate that
can actually resolve the loops. The 5 Hz status line cannot: the attitude loop
runs at 1.6 Hz and the rate loops at 5.6 Hz, so 5 Hz aliases both, and it
carries no timestamp.

```
tracklog on          start at 50 Hz
tracklog 25          start at 25 Hz
tracklog off         stop, and print the sample count and drop count
```

Each row carries `millis()`, a sequence number, and 68 columns: raw RC input,
position/velocity/attitude/body-rate each paired with the target the
controller was chasing, the commanded wrench, per-rotor setpoint, measured
speed and DShot output, plus sensor health. A `#TRKCOLS` header is printed
when logging starts, so the analysis never hard-codes a column order.

A row is 367 characters, so at 460800 baud:

| rate | link utilisation |
|---|---|
| 25 Hz | 20% |
| 50 Hz | 40% (default) |
| 75 Hz | 60% |
| 100 Hz | 80% — expect drops |

**Always check `timing` after a run.** It reports `track log: emitted=N
dropped=M`. Any non-zero `dropped` means rows are missing; the sequence
column shows where, but the sampling is no longer uniform. Lower the rate and
re-fly before using the data for anything that matters.

### Untethered: `tracklog net`

The USB cable is the only thing keeping the log tethered, and the asymmetry is
in our favour - the TRANSMITTER is the device on the desk. `tracklog net` sends
each sample back over the same ESP-NOW link and the transmitter prints it.

Type it at the **transmitter**, so the drone's USB is never connected:

```
tracklog        start the relay at 50 Hz
tracklog 25     start it at 25 Hz
tracklog off    stop, and report the row count
```

Both sketches compile `TrackSample.h`, so the rows are byte-identical to the
tethered ones - point `capture.py` at the transmitter's port and everything
downstream is unchanged.

The sample is a 216-byte packet, one per row, which is 10.8 kB/s at 50 Hz
alongside the existing 50 Hz uplink. `ControlData` was deliberately left
untouched: logging travels as its own packet type, dispatched on length and
magic, so nothing was added to the packet the failsafe watches.

After the flight, `tracklog off` prints

```
#TRK end seq=N dropped=M dl_sent=S dl_failed=F
```

`dropped` is the UART/queue counter and `dl_failed` the radio one. **Both must
be zero** before the log is used for a time-domain result.

> `TrackSample.h` exists in both sketch folders because the Arduino build will
> not follow a path out of the sketch directory. Edit one, copy to the other,
> and bump `TRACK_PACKET_VERSION` if the struct changes - a mismatched pair is
> then reported rather than silently mis-parsed.

Capture and analyse:

```bash
python capture.py COM5 --tag circle
python analyze_track.py logs/circle_<timestamp>.log
```

`analyze_track.py` validates the log first — sequence continuity, sample
interval jitter, control-step overruns, saturation flags, flow health — then
writes five figures: an overview of the full chain against time, the ground
track with a fitted circle and the stick command beside it, commanded vs
achieved for all six degrees of freedom, and a per-axis stack showing all five
cascade stages so you can see which stage introduces an error. `--airborne`
restricts to Climb and Hover; `--span T0 T1` selects a window.

### One flight, one command

`record_run.py` is the whole loop with nothing to remember. It opens the
transmitter, starts the track log, stops it on Ctrl+C, and writes a numbered
run file.

```bash
python record_run.py
```

```
run 7  -  transmitter on COM5 @ 921600
waiting 2.5s for the board to boot...

RECORDING.  Fly.  Ctrl+C when you have landed and disarmed.

     5s   248 rows
    10s   498 rows
^C
stopping...
#TRK end seq=1139 dropped=0 dl_sent=0 dl_failed=0
  1139 samples
  sequence continuous, no corrupt rows

==============================================
  RUN 7
  runs/run_007.txt
==============================================
```

Two files per flight: `runs/run_007.txt` with the stick commands, and
`runs/run_007_raw.log` with everything the port produced. The raw log is
written first and never deleted, so a failure downstream cannot cost the
flight - `analyze_track.py` reads it directly.

Numbering is the highest `run_NNN` present plus one. Deleting a run leaves its
gap rather than causing the next flight to overwrite a later file.

`--drone` if the drone is on USB instead (bench, props off), `--rate 25` if
the previous run reported drops, `--note "circle, breezy"` to put a line at
the top of the file.

### Stick commands on their own

`extract_sticks.py` pulls the four transmitter channels out of a track log and
writes nothing else. The full log is 68 columns and unreadable in a text
editor; this is the four you want when the question is what the pilot asked
for and when.

```bash
python extract_sticks.py logs/circle_20260818_170745.log
```

Writes `logs/circle_20260818_170745_sticks.csv`:

```
t_s,t_ms,seq,thr,roll,pitch,yaw
23.131,41047,558,+0.2480,+0.0010,-0.0000,+0.0000
```

`t_s` is seconds from the first sample, `t_ms` the board's raw `millis()` for
lining up against another log, and `seq` the sample number - a gap there is a
dropped row, and the script says so rather than leaving it to be noticed
later. Channels are normalised to [-1, 1] after deadband, exactly as
transmitted.

`--airborne` keeps only Climb and Hover, `--span T0 T1` selects a window,
`--state` adds armed/state/link so a stick input the aircraft ignored while
disarmed is distinguishable from one it acted on, and `--tsv` switches the
separator. Standard library only, so it runs in the field with no environment
set up.

## Video

`render_flight.py` turns a track log into an animated panel for video: ground
track, the two gimbals as the pilot moved them, position and attitude against
their targets, per-rotor model demand vs measured speed, and ESKF state vs the
raw sensor.

```bash
# beside the flight footage, synchronised on the arm event
python render_flight.py logs/final.log --video DJI_0042.MP4 --sync 12.4     --fit 1920x1080

# transparent PNG sequence for an editor
python render_flight.py logs/final.log --overlay
```

Video is written with OpenCV; ffmpeg is not required. `make_test_log.py`
generates a synthetic log in the firmware's exact format so the whole render
pipeline can be exercised before flying.

Full procedure, including the choreography and the sync method, is in
`VIDEO_FLIGHT_PROCEDURE.md`.

## Tuning entry points

Everything is runtime-tunable over serial; `gains` lists current values, and
`tracklog` above records what each change actually did.

```
set attitudekp 8.0
set yawratekp 18
set rotorkp 0.2
```

Suggested order: rotor speed loop first (it is the innermost and everything
above assumes it tracks), then rate loops, then attitude, then the outer
velocity and position loops.

## Carried-over values that still need re-verification

- **Flow gyro compensation** `FLOW_ROLL_COMP_GAIN_RH = +1.34`,
  `FLOW_PITCH_COMP_GAIN_RH = -1.65`. These are the old empirical fit
  re-expressed in the right-handed frame — algebraically identical to what
  the old code applied, but the fit itself absorbed scale error and was never
  independently validated.
- **Magnetometer calibration** unchanged from 2026-08-10. Recalibrate if
  anything magnetic moved.
- **Rotor inertia** is an estimate, not a measurement. It affects only the
  gyroscopic term in the kM identifier.
